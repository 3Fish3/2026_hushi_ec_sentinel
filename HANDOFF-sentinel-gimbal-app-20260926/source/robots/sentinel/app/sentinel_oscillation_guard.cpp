// M2-B 独立振荡守护器实现: O(1) 固定内存, 无 IO / 无堆 / 无 FFT / 无变长扫描.
//
// 结构与数据流:
//   step(sample):
//     1) 校验配置 (非法/未配置 => 不判安全, 保持 unconfigured/observe_only).
//     2) 校验输入有限性 (NaN/Inf) 与时间 (dt, now 单调/重复, boot).
//     3) 硬条件 (机械区间/边距, 速度, 必需电流/热, 反馈, RC, 权限) => 当拍锁存.
//     4) 残差 r = adopted_target - angle; 高通去慢分量 -> 带内能量; 死区外反向计数;
//        相邻完整窗包络比较 (衰减/等幅/增长); 跨秒 exposure 累积.
//     5) 动态估计仅在无硬故障时推进; suspect/warning/fault 分级.
//   reset_latched(auth, gen): 仅锁存态 + 配置有效 => disarmed; 需二者皆新才 rearm.
//
// 所有 float 比较前已确认有限; 预算采用有界/饱和语义, 不随无效拍清零.
#include "sentinel_oscillation_guard.hpp"

#include <cmath>
#include <limits>

namespace ec::sentinel::app::oscillation {

namespace {
constexpr float inf = std::numeric_limits<float>::infinity();

[[nodiscard]] bool finite(float v) noexcept { return std::isfinite(v); }

// 饱和 +1.
void sat_inc(std::uint32_t& c) noexcept {
    if (c != 0xFFFF'FFFFu) { ++c; }
}

} // namespace

const char* axis_name(Axis axis) noexcept {
    switch (axis) {
    case Axis::fire_yaw: return "fire_yaw";
    case Axis::pitch: return "pitch";
    }
    return "unknown";
}
const char* mode_name(GuardMode mode) noexcept {
    switch (mode) {
    case GuardMode::unconfigured: return "unconfigured";
    case GuardMode::observe_only: return "observe_only";
    case GuardMode::synthetic_test: return "synthetic_test";
    case GuardMode::approved: return "approved";
    }
    return "unknown";
}
const char* state_name(GuardState state) noexcept {
    switch (state) {
    case GuardState::unconfigured: return "unconfigured";
    case GuardState::observe_only: return "observe_only";
    case GuardState::normal: return "normal";
    case GuardState::suspect: return "suspect";
    case GuardState::warning: return "warning";
    case GuardState::fault_latched: return "fault_latched";
    case GuardState::disarmed: return "disarmed";
    }
    return "unknown";
}
const char* stop_request_name(StopRequest r) noexcept {
    switch (r) {
    case StopRequest::none: return "none";
    case StopRequest::requested: return "requested";
    }
    return "unknown";
}
const char* stop_confirmation_name(StopConfirmation c) noexcept {
    switch (c) {
    case StopConfirmation::unknown: return "unknown";
    case StopConfirmation::requested_local: return "requested_local";
    case StopConfirmation::confirmed: return "confirmed";
    }
    return "unknown";
}

// ---- 配置有效性 ----
bool SentinelOscillationGuard::config_valid() const noexcept {
    const GuardianLimits& l = limits_;
    if (l.mode == GuardMode::unconfigured) { return false; }
    // 阈值有限性.
    if (!finite(l.mechanical_min_rad) || !finite(l.mechanical_max_rad) ||
        !finite(l.margin_rad)) {
        return false;
    }
    if (!finite(l.max_speed_rad_s) || !finite(l.residual_deadband_rad) ||
        !finite(l.residual_energy_budget) || !finite(l.reversal_count_budget) ||
        !finite(l.window_ms) || !finite(l.exposure_budget) || !finite(l.decay_ratio) ||
        !finite(l.growth_ratio) || !finite(l.suspect_exit_ms) || !finite(l.max_dt_s)) {
        return false;
    }
    // mode / axis 合法性 (不得是越界枚举值).
    if (l.mode != GuardMode::unconfigured && l.mode != GuardMode::observe_only &&
        l.mode != GuardMode::synthetic_test && l.mode != GuardMode::approved) {
        return false;
    }
    if (l.axis != Axis::fire_yaw && l.axis != Axis::pitch) { return false; }
    // 所有预算/尺度/时间常数必须有限 (上面已 coverage 一部分; 这里补漏).
    if (!finite(l.energy_tau_s) || !finite(l.hazard_level_scale) ||
        !finite(l.nominal_dt_s) || !finite(l.latch_budget)) {
        return false;
    }

    // 阈值序.
    if (l.has_mechanical_range && !(l.mechanical_min_rad < l.mechanical_max_rad)) { return false; }
    if (l.margin_rad < 0.0f) { return false; }
    // margin 不得耗尽机械区间 (两端都要有正宽).
    if (l.has_mechanical_range &&
        (l.mechanical_min_rad + l.margin_rad) >= (l.mechanical_max_rad - l.margin_rad)) {
        return false;
    }
    if (l.max_dt_s <= 0.0f || l.window_ms <= 0.0f) { return false; }
    if (l.energy_tau_s <= 0.0f || l.hazard_level_scale <= 0.0f) { return false; }
    if (l.nominal_dt_s <= 0.0f) { return false; }
    // 残差死区不得为负 (负死区无意义).
    if (l.residual_deadband_rad < 0.0f) { return false; }
    // 需要限速时 max_speed 必须为正 (0 会让任何运动都算超速/或永不算, 隐藏危险).
    if (l.has_max_speed && !(l.max_speed_rad_s > 0.0f)) { return false; }
    // 恢复时长必须为正.
    if (!(l.suspect_exit_ms > 0.0f)) { return false; }
    // 名义 dt 不得大于最大允许 dt (否则常规采样即被判坏时序).
    if (l.nominal_dt_s > l.max_dt_s) { return false; }
    // 预算必须为正 (零/负预算无意义且隐藏危险).
    if (!(l.residual_energy_budget > 0.0f) || !(l.reversal_count_budget > 0.0f) ||
        !(l.exposure_budget > 0.0f) || !(l.latch_budget > 0.0f)) {
        return false;
    }
    // latch 门须 >= warning 门 (否则锁定永不触发).
    if (l.latch_budget < l.exposure_budget) { return false; }
    // 衰减比 <1 < 增长比.
    if (!(l.decay_ratio > 0.0f && l.decay_ratio < 1.0f)) { return false; }
    if (!(l.growth_ratio > 1.0f)) { return false; }
    // optional 阈值若 present 必须有限; required 必须同时 present (配置一致).
    if (l.current_limit_a.present && !finite(l.current_limit_a.value)) { return false; }
    if (l.temperature_limit_c.present && !finite(l.temperature_limit_c.value)) { return false; }
    // 电流阈值 present 时必须为正 (非正限流无意义/隐藏危险).
    if (l.current_limit_a.present && !(l.current_limit_a.value > 0.0f)) { return false; }
    // 温度阈值 present 必须落在物理有效范围 (高于绝对零 -273.15C, 且为有限值).
    if (l.temperature_limit_c.present && !(l.temperature_limit_c.value > -273.15f)) { return false; }
    if (l.current_limit_a.required && !l.current_limit_a.present) { return false; }
    if (l.temperature_limit_c.required && !l.temperature_limit_c.present) { return false; }
    // 带内能量频段配置有效性 + Nyquist: fs>0, 0<hp<lp, lp<fs/2.
    if (!finite(l.sample_rate_hz) || !finite(l.band_highpass_hz) || !finite(l.band_lowpass_hz)) {
        return false;
    }
    if (!(l.sample_rate_hz > 0.0f)) { return false; }
    if (!(l.band_highpass_hz > 0.0f)) { return false; }
    if (!(l.band_lowpass_hz > l.band_highpass_hz)) { return false; }
    if (!(l.band_lowpass_hz < l.sample_rate_hz * 0.5f)) { return false; } // Nyquist
    // approved 模式必须来自真实标定机械区间 (不得无 range 就 approved).
    if (l.mode == GuardMode::approved && !l.has_mechanical_range) { return false; }
    return true;
}

void SentinelOscillationGuard::configure(const GuardianLimits& limits) noexcept {
    // 轴一旦绑定, 禁止改绑 (两轴隔离对象不得跨轴重配置绕过).
    if (have_axis_ && limits.axis != axis_) {
        configured_ = false; // 拒绝该配置, 保持原轴/原状态 (不授予安全结论)
        return;
    }
    limits_ = limits;
    axis_ = limits.axis;
    have_axis_ = true;
    configured_ = config_valid();
    // 不动 latched_ / 预算: 重新配置不清锁存也不清危险预算.
    if (!configured_) {
        // 非法配置: 不授予任何安全结论. 若尚未锁存, 降到 unconfigured/observe_only.
        if (!latched_) {
            state_ = (limits.mode == GuardMode::observe_only) ? GuardState::observe_only
                                                              : GuardState::unconfigured;
        }
    }
}

void SentinelOscillationGuard::enter_state(GuardState s) noexcept {
    if (state_ != s) {
        state_ = s;
        state_elapsed_ms_ = 0.0f;
        recover_accum_ms_ = 0.0f;
        bump_event();
    }
}

void SentinelOscillationGuard::bump_event() noexcept { sat_inc(event_count_); }

GuardianReport SentinelOscillationGuard::make_report(GuardState before, bool transitioned,
                                                     std::uint32_t reasons) const noexcept {
    GuardianReport rep{};
    rep.axis = axis_;
    rep.mode = limits_.mode;
    rep.state = state_;
    rep.state_before = before;
    rep.transitioned = transitioned;
    rep.reason_bits = reasons;
    rep.latched = latched_;
    // Pitch 永远 hardware_ready=false 且 blocked (无独立防坠/停止确认).
    // Fire 仅 approved + 有效配置才可能 ready; 其它模式一律 blocked (不判安全).
    if (axis_ == Axis::pitch) {
        rep.hardware_ready = false;
        rep.blocked = true;
    } else if (limits_.mode == GuardMode::approved && configured_) {
        rep.hardware_ready = true;
        rep.blocked = false;
    } else {
        rep.hardware_ready = false;
        rep.blocked = true;
    }
    // 饱和转换: 避免 float->u32 越界 (UB); 上限取 u32 的较大但安全值.
    constexpr float kElapsedCapMs = 3.0e9f;
    float el = state_elapsed_ms_;
    if (!(el > 0.0f)) { el = 0.0f; }        // 含 NaN 保护
    if (el > kElapsedCapMs) { el = kElapsedCapMs; }
    rep.elapsed_ms = static_cast<std::uint32_t>(el);
    rep.exposure = exposure_;
    rep.band_energy = band_energy_;
    rep.residual_energy_raw = residual_energy_raw_;
    rep.reversal_count = reversal_count_;
    rep.last_envelope = last_envelope_;
    rep.prev_envelope = prev_envelope_;
    rep.stop_request = stop_request_;
    rep.stop_confirmation = stop_confirmation_;
    rep.event_count = event_count_;
    rep.guard_version = limits_.version;
    rep.config_hash = limits_.config_hash;
    return rep;
}

GuardianReport SentinelOscillationGuard::status() const noexcept {
    return make_report(state_, false, reason_bits_);
}

GuardianReport SentinelOscillationGuard::step(const GuardianSample& sample) noexcept {
    const GuardState before = state_;
    if (!have_axis_) { axis_ = sample.axis; have_axis_ = true; }
    std::uint32_t reasons = 0;
    const float claimed_dt = sample.dt_s; // 声称 dt (仅用于一致性校验)
    // 实际经过时间: 有可信前值 (clock.now_us 差分) 时用真实 elapsed_s 做曝光/滤波/窗累计;
    // 只有首次无前值时才用已验证的 claimed_dt. 不得用 sample_rate_hz 假设 dt.
    float actual_s = claimed_dt;

    // ---- 0) 轴不匹配: 样本轴与已绑定配置轴不同 => 不判安全 (硬配置故障, 当拍锁存). ----
    if (have_axis_ && sample.axis != axis_) {
        Axis saved = axis_;
        reasons |= reason_hard_config;
        latched_ = true;
        state_ = GuardState::fault_latched;
        reason_bits_ = reasons;
        state_elapsed_ms_ = 0.0f;
        recover_accum_ms_ = 0.0f;
        stop_request_ = StopRequest::requested;
        stop_confirmation_ = StopConfirmation::unknown;
        bump_event();
        GuardianReport rep = make_report(before, true, reason_bits_);
        rep.axis = saved; // 报告绑定轴 (配置轴), 避免被错误样本轴污染
        return rep;
    }

    // ---- 1) 配置未就绪: 不判安全 ----
    if (!configured_) {
        // 若曾锁存, 保持锁存且不得抹掉锁存原因 (reason_bits_ 保留); 否则保持 unconfigured/observe_only.
        if (!latched_) { reason_bits_ = reason_hard_config; }
        GuardianReport rep = make_report(before, false, reason_bits_);
        rep.axis = axis_;
        return rep;
    }

    // ---- 2) 输入有限性 (只对"已观测" float 拒 NaN/Inf) ----
    // 未观测 (has_*=false) 的数值即便 NaN 也忽略, 不得当 0 也不得触发 nonfinite.
    bool nonfinite = !finite(claimed_dt);
    if (sample.feedback_valid) {
        if (!finite(sample.angle_rad) || !finite(sample.velocity_rad_s)) { nonfinite = true; }
    }
    // 目标 request/adopted 各按自身 valid 各自核 finite (valid=false 的字段即便 NaN 也忽略).
    if (sample.target_valid && !finite(sample.request_target_rad)) { nonfinite = true; }
    if (sample.target_adopted && !finite(sample.adopted_target_rad)) { nonfinite = true; }
    if (sample.has_output && !finite(sample.output_normalized)) { nonfinite = true; }
    if (sample.has_current && !finite(sample.current_a)) { nonfinite = true; }
    if (sample.has_temperature && !finite(sample.temperature_c)) { nonfinite = true; }

    // ---- 3) 时间校验 (dt, now 单调/重复, boot, dt与now差) ----
    bool time_bad = false;
    bool restart = sample.restart; // 显式重启观测优先
    if (nonfinite) {
        time_bad = true;
    } else if (claimed_dt <= 0.0f || claimed_dt > limits_.max_dt_s) {
        time_bad = true;
    }
    // boot identity: 只有本拍与历史都"可确知"时才比较; 不可确知 => 不假设唯一/相同.
    if (!restart && have_boot_ && sample.boot_identity_available && have_boot_known_ &&
        sample.boot_id != boot_id_) {
        restart = true; // 明确的 boot 变化 (系统重启, 与 u32 count wrap 区分)
    }
    if (!time_bad && have_last_now_) {
        std::uint32_t diff = sample.now_us - last_now_us_; // u32 无符号差分 (自然回绕)
        if (sample.now_us == last_now_us_) { time_bad = true; } // 时钟未前进却 dt>0
        float implied_s = static_cast<float>(diff) * 1e-6f;
        // dt 与 now 差分必须一致 (双向): 相对误差 + 合理量化误差 (u32 µs 量化 1us + 浮点).
        // 容差 = rel*max(dt,implied) + 量化(2us). rel 取 0.5 (宽但对 1000x 低估必拒).
        float big = claimed_dt > implied_s ? claimed_dt : implied_s;
        float tol = 0.5f * big + 2e-6f;
        float diff_abs = claimed_dt > implied_s ? (claimed_dt - implied_s) : (implied_s - claimed_dt);
        if (diff_abs > tol) { time_bad = true; }
        // 真实经过时间超过 max_dt => 坏时序 (含异常大跨度).
        if (implied_s > limits_.max_dt_s) { time_bad = true; }
        // clock 差分已可信 => 曝光/滤波/窗累计直接用实际 elapsed_s; claimed dt 仅校验.
        if (!time_bad) { actual_s = implied_s; }
    }
    const float dt = actual_s; // 下游 (leak/能量/曝光/窗) 一律用真实 elapsed_s (首次无前值用 validated dt)
    // 逐拍 Nyquist 保护: 实际 dt 决定有效采样率 fs_eff=1/dt; 若本拍对带通 LP 截止已不满足
    // 奈奎斯特 (band_lowpass_hz >= fs_eff/2, 即 dt > 0.5/band_lowpass_hz) 则有 alias 风险 => 时序故障.
    // sample_rate_hz 仅参考, 不得掩盖实际低采样; 滤波系数本身用真实 dt.
    if (!time_bad && limits_.band_lowpass_hz > 0.0f) {
        const float dt_nyq = 0.5f / limits_.band_lowpass_hz; // 允许的最大 dt (Nyquist)
        if (dt > dt_nyq * 1.0001f + 1e-9f) { time_bad = true; }
    }

    // ---- 4) 硬条件 ----
    if (nonfinite) { reasons |= reason_hard_nonfinite; }
    if (time_bad) { reasons |= reason_hard_time; }
    if (restart) { reasons |= reason_hard_restart; }

    // 反馈/权限/RC (仅当时间有效时评估, 否则也已 time_bad; 但仍记录各自原因).
    if (!sample.feedback_valid) { reasons |= reason_hard_feedback; }
    if (!sample.rc_valid) { reasons |= reason_hard_rc; }
    if (!sample.executing || !sample.authority_ok) { reasons |= reason_hard_authority; }
    if (sample.manual_stop) { reasons |= reason_manual_stop; } // 独立原因, 不作为故障位锁存依据
    // 停止提交传输失败: accepted != confirmed; 传输失败不算已停.
    if (sample.report_stop_transport && sample.stop_tx_state == StopTxState::failed) {
        reasons |= reason_tx_failure;
    }

    // 位置硬限 (有限才比): 当前角度 + 请求目标 + 已采纳目标 都必须落在机械区间内.
    if (limits_.has_mechanical_range) {
        const float lo = limits_.mechanical_min_rad + limits_.margin_rad;
        const float hi = limits_.mechanical_max_rad - limits_.margin_rad;
        // 只有反馈有效时才用 angle 判位置 (feedback_valid=false 时 angle 无意义, 只报反馈失效).
        if (sample.feedback_valid && finite(sample.angle_rad) &&
            (sample.angle_rad < lo || sample.angle_rad > hi)) {
            reasons |= reason_hard_position_limit;
        }
        if (sample.target_valid && finite(sample.request_target_rad) &&
            (sample.request_target_rad < lo || sample.request_target_rad > hi)) {
            reasons |= reason_hard_position_limit;
        }
        if (sample.target_adopted && finite(sample.adopted_target_rad) &&
            (sample.adopted_target_rad < lo || sample.adopted_target_rad > hi)) {
            reasons |= reason_hard_position_limit;
        }
    }
    // 速度硬限.
    if (limits_.has_max_speed && sample.feedback_valid && finite(sample.velocity_rad_s)) {
        if (std::fabs(sample.velocity_rad_s) > limits_.max_speed_rad_s) {
            reasons |= reason_hard_velocity;
        }
    }
    // 必需电流.
    if (limits_.current_limit_a.required) {
        if (!sample.has_current) {
            reasons |= reason_hard_current;
        } else if (limits_.current_limit_a.present &&
                   std::fabs(sample.current_a) > limits_.current_limit_a.value) {
            reasons |= reason_hard_current;
        }
    }
    // 必需温度.
    if (limits_.temperature_limit_c.required) {
        if (!sample.has_temperature) {
            reasons |= reason_hard_thermal;
        } else if (limits_.temperature_limit_c.present &&
                   sample.temperature_c > limits_.temperature_limit_c.value) {
            reasons |= reason_hard_thermal;
        }
    }

    // Pitch 恒 HARDWARE_BLOCKED 标记.
    if (sample.axis == Axis::pitch) { reasons |= reason_pitch_no_fall_arrester; }

    // 硬故障位集合 (不含 manual_stop / osc 位 / pitch 标记).
    constexpr std::uint32_t hard_mask =
        reason_hard_position_limit | reason_hard_velocity | reason_hard_current |
        reason_hard_thermal | reason_hard_nonfinite | reason_hard_feedback |
        reason_hard_rc | reason_hard_authority | reason_hard_time | reason_hard_restart |
        reason_hard_config | reason_tx_failure;
    const bool hard_fault = (reasons & hard_mask) != 0;

    // ---- 时间推进: 只有时间有效时才前进 last_now / boot ----
    if (!time_bad && !nonfinite) {
        if (sample.boot_identity_available) {
            boot_id_ = sample.boot_id;
            have_boot_ = true;
            have_boot_known_ = true;
        }
        last_now_us_ = sample.now_us;
        have_last_now_ = true;
    }

    // 记录本拍是否干净 (完整新鲜有效 + 无硬故障), 供 reset 恢复条件核验.
    const bool sample_fresh_valid = sample.feedback_valid && !time_bad && !nonfinite &&
                                    sample.boot_identity_available;
    last_hard_fault_ = hard_fault;
    last_sample_clean_ = sample_fresh_valid && !hard_fault;
    // 现场恢复判据 (供 reset): 物理安全 (位置/速度/电流/热/非有限/反馈/RC/时间/重启/配置/传输)
    // 且反馈与时钟新鲜、RC 可确认; **不**要求 executing/authority (锁存撤输出后本就无执行无授权),
    // manual_stop 允许 (现场本应手动停). 不含 authority 位.
    constexpr std::uint32_t physical_mask =
        reason_hard_position_limit | reason_hard_velocity | reason_hard_current |
        reason_hard_thermal | reason_hard_nonfinite | reason_hard_feedback |
        reason_hard_rc | reason_hard_time | reason_hard_restart |
        reason_hard_config | reason_tx_failure;
    const bool physical_safe = (reasons & physical_mask) == 0;
    // 反馈/时钟新鲜 + RC 可确认 (这些已含在 physical_mask 里; 再显式保证新鲜).
    reset_recovery_ok_ = physical_safe && sample.feedback_valid && sample.rc_valid &&
                         !time_bad && !nonfinite;

    // ---- 锁存态: 保持, 仍累加可记录预算 (不清), 不自动解锁 ----
    if (latched_) {
        reason_bits_ |= reasons;
        // 无效拍不推进虚构时间; 有效拍可累加暴露(饱和).
        if (!time_bad && !nonfinite) {
            const float level = 1.0f;
            exposure_ += level * dt;
            if (exposure_ > 1e9f) { exposure_ = 1e9f; }
            state_elapsed_ms_ += dt * 1000.0f;
        }
        stop_request_ = StopRequest::requested;
        // 传输成功(ok)仅代表本地接受; 失败=unknown; 绝不自动 confirmed (无 device 确认).
        if (sample.report_stop_transport && sample.stop_tx_state == StopTxState::ok) {
            stop_confirmation_ = StopConfirmation::requested_local;
        } else {
            stop_confirmation_ = StopConfirmation::unknown;
        }
        GuardianReport rep = make_report(before, false, reason_bits_);
        rep.axis = axis_;
        return rep;
    }

    // ---- observe_only: 只观测不锁存 (含硬条件也仅记录, 不 latch) ----
    if (limits_.mode == GuardMode::observe_only) {
        reason_bits_ = reasons;
        GuardianReport rep = make_report(before, false, reasons);
        rep.axis = axis_;
        return rep;
    }

    // ---- 硬故障当拍锁存 (无宽限) ----
    // 注意: disarmed 也走此分支 (rearm 必须在 same-pass 硬检查之后, 不得先回 normal).
    if (hard_fault) {
        latched_ = true;
        reason_bits_ = reasons;
        state_ = GuardState::fault_latched;
        state_elapsed_ms_ = 0.0f;
        recover_accum_ms_ = 0.0f;
        bump_event();
        stop_request_ = StopRequest::requested;
        if (sample.report_stop_transport && sample.stop_tx_state == StopTxState::ok) {
            stop_confirmation_ = StopConfirmation::requested_local;
        } else {
            stop_confirmation_ = StopConfirmation::unknown;
        }
        GuardianReport rep = make_report(before, true, reason_bits_);
        rep.axis = axis_;
        return rep;
    }

    // ---- 动态振荡估计 (配置有效且无硬故障; synthetic_test/approved) ----
    if (time_bad || nonfinite) {
        // 已在上方 time_bad 分支处理; 不会到这里.
        GuardianReport rep = make_report(before, false, reasons);
        rep.axis = axis_;
        return rep;
    }

    // 残差 r = adopted_target - angle (目标稳定时反映非指令运动).
    const float residual = sample.adopted_target_rad - sample.angle_rad;
    const float deadband = limits_.residual_deadband_rad;
    const float absr = std::fabs(residual);
    const bool residual_outside = absr > deadband;

    // 泄漏时间常数 (s): 衰减振铃缓降, 等幅饱和; 决定跨秒预算的"记忆".
    const float tau = (limits_.energy_tau_s > 1e-6f) ? limits_.energy_tau_s : 1.0f;
    const float leak = std::exp(-dt / tau);

    // ---- 危险等级 level ∈ [0,1]: (|r|-deadband)/hazard_scale 归一 ----
    float level = 0.0f;
    if (residual_outside) {
        const float scale = (limits_.hazard_level_scale > 1e-6f) ? limits_.hazard_level_scale : 1.0f;
        level = (absr - deadband) / scale;
        if (level > 1.0f) { level = 1.0f; }
    }

    // ---- 原始残差能量 (leaky, 保留诊断): raw <- raw*leak + r^2*dt (仅越死区计入) ----
    residual_energy_raw_ = residual_energy_raw_ * leak;
    if (residual_outside) { residual_energy_raw_ += residual * residual * dt; }
    if (residual_energy_raw_ > 1e12f) { residual_energy_raw_ = 1e12f; }
    if (residual_energy_raw_ < 0.0f) { residual_energy_raw_ = 0.0f; }

    // ---- 带内能量: 固定成本一阶 HP + 一阶 LP 组带通, E <- E*leak + band_signal^2*dt ----
    // 一阶 HP: y = a*(y_prev + x - x_prev), a = RC_h/(RC_h+dt), RC_h=1/(2*pi*f_hp) — 去 DC/慢漂.
    // 一阶 LP: y_lp += alpha*(y_hp - y_lp), alpha = dt/(RC_l+dt), RC_l=1/(2*pi*f_lp) — 抑高频/带外.
    // 频段与真实采样率绑定; band_lowpass_hz<Nyquist 已由 config 保证. 恒定复杂度, 无堆.
    {
        const float two_pi = 6.28318530718f;
        const float fhp = (limits_.band_highpass_hz > 1e-6f) ? limits_.band_highpass_hz : 1e-6f;
        const float flp = (limits_.band_lowpass_hz > fhp) ? limits_.band_lowpass_hz : (fhp * 2.0f);
        const float rc_h = 1.0f / (two_pi * fhp);
        const float rc_l = 1.0f / (two_pi * flp);
        const float a_hp = rc_h / (rc_h + dt);
        const float alpha_lp = dt / (rc_l + dt);
        // HP.
        float hp_out;
        if (!hp_have_prev_) {
            hp_x_prev_ = residual; hp_y_prev_ = 0.0f; hp_have_prev_ = true;
            hp_out = 0.0f; // 第一个样本无历史差分, 输出 0 (正确: DC 不产生带内能量)
        } else {
            hp_out = a_hp * (hp_y_prev_ + residual - hp_x_prev_);
            hp_x_prev_ = residual; hp_y_prev_ = hp_out;
        }
        // LP.
        if (!lp_have_) { lp_y_ = hp_out; lp_have_ = true; }
        else { lp_y_ += alpha_lp * (hp_out - lp_y_); }
        const float band_signal = lp_y_;
        band_energy_ = band_energy_ * leak;
        band_energy_ += band_signal * band_signal * dt;
        if (band_energy_ > 1e12f) { band_energy_ = 1e12f; }
        if (band_energy_ < 0.0f) { band_energy_ = 0.0f; }
    }

    // ---- 幅值加权反向计数 (leaky): 每次反向按 level 加权累加, 按时间泄漏 ----
    // 衰减环的后期幅值小, level 低, 贡献小 => 不会像等幅环那样饱和到高值.
    // 对外 uint32 仅作证据 (饱和取整), 不单独判安全.
    reversal_accum_ *= leak;

    {
        bool reversal = false;
        if (residual_outside) {
            const bool pos = residual > 0.0f;
            if (have_last_residual_ && pos != last_residual_sign_positive_) { reversal = true; }
            last_residual_sign_positive_ = pos;
            have_last_residual_ = true;
        }
        // 残差回到死区内时保留上次符号 (不重置): 否则每次穿越死区的零交叉都丢失反向证据,
        // 使真实正弦振荡永远检测不到 sign flip. 衰减/收敛由 rev_activity 泄漏与衰减趋势处理.
        // Fire 输出不断反向 (即便位置 RMS 很小) 也提供补充危险证据.
        float out_contrib = 0.0f;
        if (sample.has_output && finite(sample.output_normalized)) {
            const bool pos = sample.output_normalized > 0.0f;
            if (have_output_prev_ && pos != output_prev_positive_) {
                out_contrib = std::fabs(sample.output_normalized);
                if (!residual_outside) { reversal = true; }
            }
            output_prev_positive_ = pos;
            have_output_prev_ = true;
        } else {
            have_output_prev_ = false;
        }

        if (reversal || out_contrib > 0.0f) {
            // 权重: 残差反向按其 level; 输出反向按其归一幅度 (0..1).
            float w = level;
            if (out_contrib > w) { w = out_contrib; }
            if (w <= 0.0f) { w = 0.01f; } // 有反向但残差在死区内: 给存在性基础权重
            reversal_accum_ += w;
            rev_activity_ += w; // 振荡活跃度 (泄漏, 判"是否真在振荡")
        }
        if (reversal_accum_ > 1e9f) { reversal_accum_ = 1e9f; }
        reversal_count_ = static_cast<std::uint32_t>(reversal_accum_);
    }
    rev_activity_ *= leak; // 没有新反向则衰减
    if (rev_activity_ > 1e9f) { rev_activity_ = 1e9f; }
    if (rev_activity_ < 0.0f) { rev_activity_ = 0.0f; }

    // ---- 完整窗包络: 达 window_ms 输出 RMS 包络, 比较相邻窗判衰减/增长 ----
    envelope_sum_sq_ += residual * residual * dt;
    window_accum_ms_ += dt * 1000.0f;
    bool window_growing = false;
    bool window_decaying = false;
    bool window_complete = false;
    if (window_accum_ms_ >= limits_.window_ms) {
        const float denom = (window_accum_ms_ >= 1e-6f) ? window_accum_ms_ : 1.0f;
        const float envelope = std::sqrt(envelope_sum_sq_ * 1000.0f / denom);
        prev_envelope_ = last_envelope_;
        last_envelope_ = envelope;
        envelope_sum_sq_ = 0.0f;
        window_accum_ms_ = 0.0f;
        window_complete = true;
        // 有界近期包络环 (最近 N 个完整窗), 用于判"近期趋势"而非永久峰值依赖.
        env_ring_[env_ring_pos_] = envelope;
        env_ring_pos_ = static_cast<std::uint8_t>((env_ring_pos_ + 1) % 8);
        if (env_ring_fill_ < 8) { env_ring_fill_ = static_cast<std::uint8_t>(env_ring_fill_ + 1); }
        // 近期峰值: 只在有界环内取最大 (峰值随旧样本移出而自然下降, 不会永久滞留).
        float local_peak = 0.0f;
        for (std::uint8_t k = 0; k < env_ring_fill_; ++k) {
            if (env_ring_[k] > local_peak) { local_peak = env_ring_[k]; }
        }
        peak_envelope_ = local_peak;
        // 相邻窗增长.
        if (prev_envelope_ > 1e-6f) {
            if (limits_.growth_ratio > 0.0f &&
                last_envelope_ > prev_envelope_ * limits_.growth_ratio) {
                window_growing = true;
            }
        }
        // 近期"连续非下降"检测: 若最近若干窗整体不下降(含微升/持平), 则即使历史峰值高也不得判衰减.
        // 具体: 统计最近 min(fill,4) 窗中, 当前窗是否低于其中较早窗 (真下降趋势才允许 decay).
        bool recent_non_decaying = true;   // 假定近期非下降
        if (env_ring_fill_ >= 3) {
            // 取最近 3 窗 (从旧到新): 若单调不增(每步都下降)才算"真衰减趋势".
            float e0 = env_ring_[static_cast<std::uint8_t>((env_ring_pos_ + 5) % 8)]; // 3 窗前
            float e1 = env_ring_[static_cast<std::uint8_t>((env_ring_pos_ + 6) % 8)]; // 2 窗前
            float e2 = env_ring_[static_cast<std::uint8_t>((env_ring_pos_ + 7) % 8)]; // 1 窗前
            // 真衰减: 持续下降 (e0>=e1>=e2 且总降幅明显), 否则视为非下降(维持).
            const float eps = 1e-6f;
            bool monotone_down = (e0 >= e1 - eps) && (e1 >= e2 - eps) && (e0 > e2 + eps);
            recent_non_decaying = !monotone_down;
        }
        // 衰减: 当前包络显著低于近期峰值 且 近期确实在下降 (非维持) => 才判衰减趋势.
        if (limits_.decay_ratio > 0.0f && peak_envelope_ > 1e-6f &&
            last_envelope_ < peak_envelope_ * limits_.decay_ratio &&
            !recent_non_decaying) {
            window_decaying = true;
        }
    }
    // 记录是否处于衰减趋势 (供 osc 锁定预算门控; 衰减环即便幅度曾大也不应锁定).
    // 每个完整窗刷新: 仅"当前窗相对上一窗衰减"才算衰减; 稳定/增长都视为非衰减.
    if (window_complete) { decaying_trend_ = window_decaying && !window_growing; }

    // ---- 跨秒危险暴露 (leaky): exposure <- exposure*leak + level*dt ----
    // 等幅高 level 饱和高值; 衰减环 level 降低 => exposure 回落.
    exposure_ = exposure_ * leak;
    exposure_ += level * dt;
    if (exposure_ > 1e9f) { exposure_ = 1e9f; }
    if (exposure_ < 0.0f) { exposure_ = 0.0f; }

    // 振荡锁定暴露 (跨秒累积, 无泄漏但有"活跃门"): 只在"近期确有反向振荡活跃"时累积 sev*dt.
    // 非活跃(无反向/残差回死区/衰减趋势) => 复位为 0 (证明正在恢复); 持续振荡 => 单调增长最终锁定;
    // 增长 => sev 更大更快达阈. 常值下垂/单次阶跃无反向 => 不累积.
    {
        constexpr float kActive = 0.05f; // rev_activity 高于此视为在振荡
        // 近期确有反向活跃 (rev_activity 为泄漏的"recent reversals") 且不在衰减趋势 => 正在振荡.
        const bool active = (rev_activity_ > kActive) && !decaying_trend_;
        if (active) {
            float sev = level;
            if (sev < 1e-6f) { sev = (rev_activity_ < 1.0f) ? rev_activity_ : 1.0f; } // 输出反向但残差死区内
            osc_exposure_ += sev * dt;
            if (osc_exposure_ > 1e9f) { osc_exposure_ = 1e9f; }
        } else {
            osc_exposure_ = 0.0f; // 不活跃 => 清零 (衰减/消除即恢复)
        }
        if (osc_exposure_ < 0.0f) { osc_exposure_ = 0.0f; }
    }

    // ---- 状态分级 ----
    const bool over_energy = limits_.residual_energy_budget > 0.0f &&
                             band_energy_ > limits_.residual_energy_budget;
    const bool over_reversal = limits_.reversal_count_budget > 0.0f &&
                               reversal_accum_ > limits_.reversal_count_budget;
    // warning 门: 振荡暴露超 exposure_budget; 锁定门: 超 latch_budget (更高) => 最终 fault_latched.
    const bool warn_osc = limits_.exposure_budget > 0.0f &&
                          osc_exposure_ > limits_.exposure_budget;
    // 锁定阈值: latch_budget (配置有效时必 >0 且 >= exposure_budget).
    const float latch_threshold = limits_.latch_budget;
    const bool latch_osc = latch_threshold > 0.0f && osc_exposure_ > latch_threshold;
    const bool growing = window_growing;

    GuardState new_state = state_;
    std::uint32_t osc_reasons = 0;

    // 硬检查 (hard_fault) 已在上面 same-pass 处理: 到这里的拍都无硬故障.
    bool rearmed = false;
    if (state_ == GuardState::disarmed) {
        // rearm: 需新授权 (authorization_id) + 新目标 (target_revision) 双双更新,
        // 且本拍为完整新鲜有效干净样本 (无硬故障, 已在上面通过).
        const bool new_auth = have_reset_ids_ && sample.authorization_id != reset_authority_;
        const bool new_target = have_reset_ids_ && sample.target_revision != reset_generation_;
        if (new_auth && new_target && sample.target_adopted && sample.target_valid &&
            last_sample_clean_) {
            rearmed = true;
            new_state = GuardState::normal; // 进入块内做振荡门检查; 干净则保持 normal
            have_reset_ids_ = false; // rearm 完成
        } else {
            new_state = GuardState::disarmed;
            osc_reasons |= reason_boot_or_reset;
        }
    }

    if (new_state != GuardState::disarmed) {
        if (latch_osc) {
            // 跨秒持续振荡预算耗尽 => 最终 fault_latched (停止, 不自动恢复).
            latched_ = true;
            state_ = GuardState::fault_latched;
            state_elapsed_ms_ = 0.0f;
            recover_accum_ms_ = 0.0f;
            bump_event();
            osc_reasons |= reason_osc_exposure;
            if (over_energy) { osc_reasons |= reason_osc_energy; }
            if (over_reversal) { osc_reasons |= reason_osc_reversal; }
            if (growing) { osc_reasons |= reason_osc_growth; }
            reason_bits_ = reasons | osc_reasons;
            stop_request_ = StopRequest::requested;
            stop_confirmation_ = (sample.report_stop_transport &&
                                  sample.stop_tx_state == StopTxState::ok)
                                     ? StopConfirmation::requested_local
                                     : StopConfirmation::unknown;
            GuardianReport rep = make_report(before, true, reason_bits_);
            rep.axis = axis_;
            return rep;
        } else if (warn_osc || growing) {
            // 危险但未达锁定预算 => warning (阻断/请求停止, 不虚构回滚).
            // 注意: band_energy 单超预算不作为独立 warning 触发 (一次性大阶跃不要误报);
            // 能量只在与反向振荡共现时作为佐证原因记录.
            new_state = GuardState::warning;
            if (warn_osc) { osc_reasons |= reason_osc_exposure; }
            if (over_energy && warn_osc) { osc_reasons |= reason_osc_energy; }
            if (growing) { osc_reasons |= reason_osc_growth; }
            recover_accum_ms_ = 0.0f;
        } else if (rearmed) {
            new_state = GuardState::normal; // 新授权+新目标的干净 rearm: 直接 normal
            recover_accum_ms_ = 0.0f;
        } else if (level > 0.0f) {
            // 轻微静态/残余: suspect 短暂观察.
            new_state = GuardState::suspect;
            osc_reasons |= reason_osc_suspect;
            recover_accum_ms_ = 0.0f;
        } else if (state_ == GuardState::suspect || state_ == GuardState::warning ||
                   state_ == GuardState::disarmed) {
            // 恢复: 持续干净 suspect_exit_ms 才回 normal.
            recover_accum_ms_ += dt * 1000.0f;
            if (recover_accum_ms_ >= limits_.suspect_exit_ms) {
                new_state = GuardState::normal;
            } else {
                new_state = state_;
            }
        } else {
            new_state = GuardState::normal;
        }
    }

    reasons |= osc_reasons;
    reason_bits_ = reasons;
    const bool transitioned = (new_state != before);
    if (transitioned) { enter_state(new_state); }
    if (new_state == GuardState::warning || new_state == GuardState::suspect) {
        state_elapsed_ms_ += dt * 1000.0f;
    }
    if (new_state == GuardState::warning) {
        stop_request_ = StopRequest::requested;
        stop_confirmation_ = StopConfirmation::unknown; // 本地请求不等于已停
    } else if (new_state == GuardState::normal) {
        stop_request_ = StopRequest::none;
    }

    GuardianReport rep = make_report(before, transitioned, reasons);
    rep.axis = axis_;
    return rep;
}

ResetResult SentinelOscillationGuard::reset_latched(std::uint64_t authorization_id,
                                                    std::uint32_t target_revision,
                                                    bool locally_confirmed,
                                                    std::uint32_t now_us) noexcept {
    if (!configured_) { return ResetResult::config_invalid; }
    if (!latched_) { return ResetResult::not_latched; }
    // 恢复条件: 必须显式本地确认 + 当前存在完整新鲜有效且无硬危险的干净样本.
    // 硬危险未恢复 (未观测/无效/仍然危险) 不得解锁.
    // 恢复条件: 显式本地确认 + 物理安全恢复判据 (不含 executing/authority; manual_stop 允许).
    if (!locally_confirmed || !reset_recovery_ok_) {
        return ResetResult::config_invalid; // 恢复条件未满足 => 拒绝 (不改状态)
    }
    // 新鲜性: 最近样本必须"现在仍然新鲜". 只在 last_now_us_ 已记录时核.
    // **u32 无符号差分**: 合法计时回绕 (如 0xFFFFFF00 -> 0x00000100, 真实 512us) 必须能通过,
    // 不得用 now<last 简单拒; 真实少量回退会形成巨大无符号 diff 而超出时龄上限被拒.
    if (!have_last_now_) { return ResetResult::config_invalid; }
    const std::uint32_t age_us = now_us - last_now_us_; // u32 无符号差分 (自然回绕, 含 reset 跨 wrap)
    const float age_ms = static_cast<float>(age_us) * 1e-3f;
    // 允许的样本时龄: 几倍最大 dt (至少 50ms). 超过即视为"无新样本".
    float stale_ms = limits_.max_dt_s * 1000.0f * 2.0f;
    if (stale_ms < 50.0f) { stale_ms = 50.0f; }
    if (age_ms > stale_ms) { return ResetResult::config_invalid; }
    latched_ = false;
    state_ = GuardState::disarmed;
    // 记录复位时的标识; 只有新 (authorization_id, target_revision) 双双更新才 rearm.
    reset_authority_ = authorization_id;
    reset_generation_ = target_revision;
    have_reset_ids_ = true;
    state_elapsed_ms_ = 0.0f;
    recover_accum_ms_ = 0.0f;
    // 不清 exposure_/band_energy_ 危险预算 (保守); reason 保留 boot_or_reset 标记.
    reason_bits_ = reason_boot_or_reset;
    stop_request_ = StopRequest::none;
    stop_confirmation_ = StopConfirmation::unknown;
    bump_event();
    return ResetResult::accepted;
}

} // namespace ec::sentinel::app::oscillation
