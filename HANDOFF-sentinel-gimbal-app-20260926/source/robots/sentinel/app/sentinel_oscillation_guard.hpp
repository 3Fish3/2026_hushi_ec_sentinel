#pragma once
// M2-B 独立振荡守护器 (离线纯算法 + 主机单测).
//
// 定位: 每轴 (仅 fire_yaw / pitch) 一个 SentinelOscillationGuard 纯对象, 无 IO / 无堆 /
// 无 FFT / 无变长扫描 / O(1) 固定内存. 只做"阻断/锁存判定", 不授予任何运动/调参许可,
// 不向原手动遥控路径写目标或输出. 生产默认 UNCONFIGURED / NOT_SAFE_FOR_HARDWARE:
// 守护器在未标定配置下不给出"安全通过"结论, 只能保持 OBSERVE_ONLY/UNCONFIGURED.
//
// 安全约定 (由测试冻结):
//  - 硬条件 (机械区间/边距/速度/必需电流热/NaN/Inf/反馈心跳/RC/权限/时序) 先于一切动态
//    估计, 当拍即锁存, 无"几秒宽限".
//  - 旧 manual_stop 是独立原因: 既不因它清除, 也不由它清除已锁存故障.
//  - 新授权 / 新正常样本不能自动解锁锁存. reset_latched 仅为显式本地确认
//    (给 C 测试/未来现场流程), 恢复后只能到 DISARMED; 只接受新的
//    (authority_binding + target_generation) 二者皆更新后才回 NORMAL. 生产不接远端 clear.
//  - configure / 改阈值 / boot change 都不清除锁存, 也不清除已累积危险预算.
//    真正重新对象构造才回到默认 UNCONFIGURED, 不自动执行.
//  - 所有 float 输入 (含 optional) 先拒 NaN/Inf; missing 输出不得当 0.
//  - u32 now_us 用无符号差分; 真实 wrap 用差分阈值; boot identity 变化视为 restart,
//    与计数回绕区分. 无效 dt 不推进虚构时间也不清旧预算.
//  - 两轴独立对象完全隔离.
#include <cstddef>
#include <cstdint>

namespace ec::sentinel::app::oscillation {

// 一个轴的标识 (仅这两个轴受支持).
enum class Axis : std::uint8_t { fire_yaw = 0, pitch = 1 };
inline constexpr std::size_t axis_count = 2;

// 配置模式. 只有 approved 且阈值经工程师确认才可进入实机在线保护;
// 生产默认 unconfigured. synthetic_test 仅测试夹具, 不代表实机安全.
enum class GuardMode : std::uint8_t {
    unconfigured = 0, // 未配置: 无硬阈值, 不判安全
    observe_only,     // 只观测: 不锁存, 不授予许可
    synthetic_test,   // SYNTHETIC_TEST_ONLY: 阈值仅测试夹具
    approved,         // 经批准的实机阈值 (本阶段无真实值, 不出现)
};

// 状态机. 明确区分不可实机启用的未标定态与在线态.
enum class GuardState : std::uint8_t {
    unconfigured = 0, // 无阈值 / 无配置, 未标定
    observe_only,     // 只观测不锁存
    normal,           // 在线且当前无异常振荡
    suspect,          // 疑似非指令振荡, 受限观察
    warning,          // 持续/等幅达预算, 请求安全停止/阻断, 未锁存
    fault_latched,    // 硬条件/危险/失控, 锁存; 只能显式现场复位到 disarmed
    disarmed,         // 已复位但未重新授权/重建目标, 不发许可
};

// 故障 / 事件原因位. 位叠加; 0 = 无原因.
enum ReasonBit : std::uint32_t {
    reason_none = 0,
    // --- 硬条件 (当拍锁存) ---
    reason_hard_position_limit = 1u << 0, // 关节角越机械区间+边距
    reason_hard_velocity = 1u << 1,       // 速度越危险上限
    reason_hard_current = 1u << 2,        // 必需电流缺失或越界
    reason_hard_thermal = 1u << 3,        // 必需温度缺失或越界
    reason_hard_nonfinite = 1u << 4,      // 输入 NaN/Inf
    reason_hard_feedback = 1u << 5,       // 反馈无效/失鲜
    reason_hard_rc = 1u << 6,             // RC 无效
    reason_hard_authority = 1u << 7,      // 执行资格/授权否定 (未 executing / authority_ok=false)
    reason_hard_time = 1u << 8,           // 时序无效 (dt<=0/过大/重复/非有限)
    reason_hard_restart = 1u << 9,        // boot identity 变化 (重启)
    reason_hard_config = 1u << 10,        // 配置非法 (NaN/Inf/阈值序/模式不匹配)
    // --- 动态振荡 ---
    reason_osc_suspect = 1u << 11,        // 进入/维持 suspect
    reason_osc_energy = 1u << 12,         // 带内残差能量超预算
    reason_osc_reversal = 1u << 13,       // 死区外反向计数超预算
    reason_osc_growth = 1u << 14,         // 包络增长 (相邻完整窗)
    reason_osc_exposure = 1u << 15,       // 跨秒 exposure 预算耗尽
    // --- 其它 ---
    reason_manual_stop = 1u << 16,        // 旧人工停止原因 (独立, 不清锁存)
    reason_boot_or_reset = 1u << 17,      // 显式复位/重启未 rearm
    reason_pitch_no_fall_arrester = 1u << 18, // Pitch 无独立防坠 => HARDWARE_BLOCKED
    reason_tx_failure = 1u << 19,             // CAN 停止提交失败 (accepted≠confirmed)
    reason_no_fresh_sample = 1u << 20,        // 缺完整新鲜有效样本 (不得据此判安全)
    // --- M2.1 调参资格拒绝原因 (Fail-Closed 语义) ---
    reason_tune_not_configured = 1u << 21,    // Guardian 未配置 => 拒调参
    reason_tune_not_approved = 1u << 22,      // mode 非 approved / 未通过批准 => 拒调参
    reason_tune_latched = 1u << 23,           // 处于锁存/抑制态 => 拒调参
    reason_tune_pitch_device_not_ready = 1u << 24,       // Pitch 装置未就绪
    reason_tune_pitch_stop_policy = 1u << 25,            // Pitch 无批准的安全停止策略
    reason_tune_pitch_feedback = 1u << 26,               // Pitch 反馈不健康
    reason_tune_pitch_rc = 1u << 27,                     // Pitch RC 不健康
    reason_tune_pitch_stop_pending = 1u << 28,           // Pitch 停止请求待确认(新鲜反馈未证实已停)
};

// 停止请求是否发出 (本地语义, 不等于设备已停).
enum class StopRequest : std::uint8_t { none = 0, requested };

// 停止提交传输状态 (独立观测): 本地 accepted 不等于设备 confirmed.
enum class StopTxState : std::uint8_t {
    unknown = 0, // 未观测/未知
    ok,          // 传输层接受 (command accepted, 非 stopped)
    failed,      // 传输失败 (可能未送达)
};

// 停止确认: 只能由明确可信的新鲜反馈或外部确认变成 confirmed.
enum class StopConfirmation : std::uint8_t {
    unknown = 0,     // 无确认 / 无反馈: 不得当作已停
    requested_local, // 本对象仅发起本地请求, 未提交
    confirmed,       // 明确可信新鲜反馈/外部确认 (本阶段算法不产生)
};

// 单轴一个 tick 的完整输入 (全部 float 字段若非观测则 observed=false).
struct GuardianSample {
    Axis axis{Axis::fire_yaw};

    // 时间: 真实 u32 微秒时刻 + 该拍真实 dt (秒). dt 必须 > 0 且有限且 <= max_dt_s.
    std::uint32_t now_us{0};
    float dt_s{0.0f};
    // boot identity: 每次系统启动不同; 与 u32 wrap 区分.
    // boot_identity_available=false 表示本拍无法确知 boot (不得假设唯一/相同).
    std::uint32_t boot_id{0};
    bool boot_identity_available{false};
    // 显式重启观测 (例如启动原因/单调 seq 归零); 与 u32 count wrap 区分.
    bool restart{false};

    // 目标: request (遥控/来源原样) 与 adopted (实际采用). 不可用用 valid=false.
    float request_target_rad{0.0f};
    float adopted_target_rad{0.0f};
    bool target_valid{false};
    bool target_adopted{false};
    // 目标代次 + 授权标识. 与 C 的真实来源对齐:
    //   authorization_id 对应 authority().token()->Grant.generation (uint64, 不得截断);
    //   target_revision 对应 app.target(domain)->DomainTarget accepted_at 代次 (uint32).
    // rearm 需二者皆更新 (新授权 + 新目标), 不是任意 bool 重新 true.
    std::uint64_t authorization_id{0};
    std::uint32_t target_revision{0};

    // 关节反馈.
    float angle_rad{0.0f};
    float velocity_rad_s{0.0f};
    bool feedback_valid{false};

    // 执行 / 权限 / 旧语义.
    bool executing{false};
    bool rc_valid{false};
    bool authority_ok{false};
    bool manual_stop{false};

    // 可选观测: 输出归一化 / 电流 / 温度. 未观测用 observed=false (不得当 0).
    // 注意: has_*=false 时对应数值即使为 NaN 也视为未观测, 忽略; 只有 has_*=true 才要求有限.
    bool has_output{false};
    float output_normalized{0.0f};
    bool has_current{false};
    float current_a{0.0f};
    bool has_temperature{false};
    float temperature_c{0.0f};

    // 停止提交传输观测 (独立于 feedback): accepted 不等于 device stopped/confirmed.
    StopTxState stop_tx_state{StopTxState::unknown};
    bool report_stop_transport{false}; // 本拍是否有传输层回报可解读
};

// 逐轴安全包络与预算. 全部阈值须有可追溯来源; 本阶段仅测试夹具用 synthetic_test.
struct GuardianLimits {
    Axis axis{Axis::fire_yaw};   // 绑定轴 (configure 时确定; pitch 恒 blocked)
    GuardMode mode{GuardMode::unconfigured};
    std::uint32_t version{0};
    std::uint32_t config_hash{0}; // 可追溯配置指纹

    // ---- 阈值来源/审批标识与单位 (任务书 §77 必查) ----
    // 固定大小字段 (非动态分配, 适合裸机). 空串 = 无凭据. 仅 approved 模式强制要求:
    //  deployment 侧 approved 阈值必须携带来源 id + 审批 id + 显式现场批准位, 否则拒绝装载.
    // synthetic_test 夹具可携带 synthetic 元数据 (明确标注非实车批准), 不冒充实车批准.
    static constexpr std::size_t id_capacity = 32;
    char source_id[id_capacity]{};    // 阈值来源 (标定/文档/责任人) 标识
    char approval_id[id_capacity]{};  // 审批单/评审记录 标识
    char unit_notes[id_capacity]{};   // 单位说明 (如 "rad, rad/s, s, Hz")
    bool on_site_approved{false};     // 是否经现场人工批准 (缺失 => 不得部署为 approved)

    // 是否携带完整审批凭据. 要求:
    //  - source_id / approval_id / unit_notes 三个定长字段均非空;
    //  - 每个定长字段都必须在容量内 NUL 终止 (否则是溢出/未终止的坏数据, 不得当凭据);
    //  - 显式现场批准位 on_site_approved 为真.
    // 注意: 这里只校验"字段存在且格式合法", 不证明阈值内容正确; config_hash 是外部提供的
    // 审计标识 (audit id), 本结构不据其自证数据完整性 (真实完整性需外部 canonical hash 校验).
    [[nodiscard]] bool has_provenance() const noexcept {
        auto cstr_ok = [](const char (&s)[id_capacity]) noexcept {
            // 必须在容量内出现 NUL 终止符 (禁止越界访问: 只读到 id_capacity-1).
            for (std::size_t i = 0; i < id_capacity; ++i) {
                if (s[i] == '\0') { return i > 0; } // 空串 => 不满足
            }
            return false; // 无终止 => 不满足
        };
        return cstr_ok(source_id) && cstr_ok(approval_id) && cstr_ok(unit_notes) &&
            on_site_approved;
    }

    // 机械区间 (必须来自真实标定, 不得用协议 PMAX 或 commissioning_unbounded).
    bool has_mechanical_range{false};
    float mechanical_min_rad{0.0f};
    float mechanical_max_rad{0.0f};
    float margin_rad{0.0f}; // 边距 (>=0)

    bool has_max_speed{false};
    float max_speed_rad_s{0.0f};

    // 可选电流/热阈值: 只有 required 时才强制; required 缺失才阻断, optional 缺失不阻断.
    struct OptionalFloat {
        bool present{false};
        bool required{false};
        float value{0.0f};

        // 逐字段相等 (C++20). 供 GuardianLimits 同版本幂等性判定使用.
        bool operator==(const OptionalFloat&) const = default;
    };
    OptionalFloat current_limit_a{};
    OptionalFloat temperature_limit_c{};

    // 残差带与振荡预算 (真实采样率相关).
    float residual_deadband_rad{0.0f};    // 残差死区 (rad)
    float residual_energy_budget{0.0f};   // 带内泄漏能量预算上限 (rad^2 * s)
    float energy_tau_s{0.0f};             // 带内能量泄漏时间常数 (s): 衰减环可回落
    float reversal_count_budget{0.0f};    // 死区外反向计数预算 (泄漏计数)
    float window_ms{0.0f};                // 完整窗时长 (ms), 用于包络比较
    float exposure_budget{0.0f};          // 跨秒危险暴露预算 (level*s) -> warning 门
    float latch_budget{0.0f};             // 跨秒振荡锁定预算 (>warning, 最终 fault_latched)
    float decay_ratio{0.0f};              // 包络衰减阈值 (如 0.7: 下降>=30% 视为衰减)
    float growth_ratio{0.0f};             // 包络增长阈值 (如 1.3: 增>=30% 视为增长)
    float suspect_exit_ms{0.0f};          // 持续恢复该毫秒数后可回 normal
    float hazard_level_scale{0.0f};       // 残差/死区比到 level(0..1) 的归一尺度, 越小越敏感
    float max_dt_s{0.0f};                 // 有效 dt 上限
    float nominal_dt_s{0.0f};             // 标称采样周期 (集成核对用, 本算法用真实 dt)

    // 带内能量频段配置 (真实采样率/传感器带宽匹配; SYNTHETIC_TEST_ONLY).
    // 固定成本一阶 HP + 一阶 LP 组带通; band_lowpass_hz 必须 < Nyquist (sample_rate_hz/2).
    float sample_rate_hz{0.0f};           // 实际采样率 (Hz), 须 >0
    float band_highpass_hz{0.0f};         // 可检测频带下沿 HP 截止 (Hz), 须 >0
    float band_lowpass_hz{0.0f};          // 可检测频带上沿 LP 截止 (Hz), 须 >HP 且 < Nyquist

    // 逐字段相等 (C++20 default). 包含定长 char 数组 (逐元素比较, 不是 memcmp, 无 padding 风险).
    // 供 configure_guardian 的同版本幂等性判定: 同 version 下须全字段一致才算"同一份不可变配置".
    bool operator==(const GuardianLimits&) const = default;
};

// 守护器报告: 每步/查询返回. sizeof 固定, 无指针/无变长.
struct GuardianReport {
    Axis axis{Axis::fire_yaw};
    GuardMode mode{GuardMode::unconfigured};
    GuardState state{GuardState::unconfigured};
    GuardState state_before{GuardState::unconfigured}; // 本步跃迁前状态
    bool transitioned{false}; // 本步是否发生跃迁 (幂等: 同事件仅一次)
    std::uint32_t reason_bits{0};

    bool latched{false};        // 处于 fault_latched 锁存
    bool hardware_ready{false}; // pitch 永远 false (无防坠); fire 仅 approved 才可能 true
    bool blocked{false};        // pitch: 恒 HARDWARE_BLOCKED; 其它: 无安全结论也 true

    std::uint32_t elapsed_ms{0};    // 自进入当前异常态累计真实毫秒
    float exposure{0.0f};           // 跨秒危险暴露累积 (level*s)
    float band_energy{0.0f};        // 带内残差能量 (HP×LP 滤波后 band_signal^2 dt 泄漏积分)
    float residual_energy_raw{0.0f}; // 原始残差 r^2 泄漏积分 (旧 band_energy 语义保留, 供诊断)
    std::uint32_t reversal_count{0}; // 死区外反向计数
    float last_envelope{0.0f};      // 最近完整窗包络幅值 (证据)
    float prev_envelope{0.0f};      // 上一完整窗包络幅值 (证据)

    StopRequest stop_request{StopRequest::none};
    StopConfirmation stop_confirmation{StopConfirmation::unknown};

    std::uint32_t event_count{0}; // 有界/饱和事件跃迁计数器

    std::uint32_t guard_version{0};
    std::uint32_t config_hash{0};

    [[nodiscard]] bool inhibit() const noexcept {
        return state == GuardState::fault_latched || state == GuardState::warning ||
               state == GuardState::disarmed;
    }
};

// M2.1-A 调参资格 (Fail-Closed). 只有 configured && approved && 未锁存/未抑制 且 (Pitch)
// 通过统一安全门 才 can_tune=true. 固件/状态查询据此清晰暴露 configured/mode/approved/
// config_hash/version/reason/can_tune. 默认全 false = 100% 拒调参.
struct TuneEligibility {
    bool configured{false};      // guardian 是否已装载有效配置
    GuardMode mode{GuardMode::unconfigured};
    bool approved{false};        // mode==approved 且配置有效
    std::uint32_t guard_version{0};
    std::uint32_t config_hash{0};
    std::uint32_t reason_bits{0}; // 拒绝原因位 (reason_tune_* / reason_pitch_no_fall_arrester)
    bool can_tune{false};         // 最终裁决: 唯一权威
};

// M2.1-B Pitch 统一防坠安全门状态. 四阶段停止确认 + 各互锁单项可见性.
// 部署默认 pitch_fall_arrest_verified=false => can_tune 恒 false (HARDWARE_BLOCKED).
struct PitchSafetyStatus {
    // 互锁输入项.
    bool pitch_fall_arrest_verified{false}; // 经人工核验的独立防坠 (部署默认 false)
    bool device_ready{false};               // 装置就绪
    bool approved_stop_policy{false};       // 批准的安全停止策略
    bool guardian_approved{false};          // Guardian 已批准且有效
    bool feedback_healthy{false};           // 反馈健康
    bool rc_healthy{false};                 // RC 健康
    bool fault_latched{false};              // 故障锁存
    // 四阶段停止确认 (不得混同: 本地发送 != 电调接受 != 电机已停 != 物理支撑已核验).
    bool stop_requested{false};             // 阶段1: 本地已发起停止请求
    bool transport_accepted{false};         // 阶段2: 传输层已接受 (不等于电机已停)
    bool motor_feedback_confirmed{false};   // 阶段3: 新鲜反馈确认电机已停
    bool physical_support_verified{false};  // 阶段4: 物理防坠支撑经人工核验
    bool stop_confirmed{false};             // 综合: 停止是否真正落实 (=motor_feedback_confirmed)
    std::uint32_t reason_bits{0};           // 拒绝原因位
    bool can_tune{false};                   // 最终裁决 (全互锁满足才 true)
};

// 显式本地复位结果 (生产不接远端协议; 仅 C 测试/未来现场窄 API).
enum class ResetResult : std::uint8_t {
    accepted = 0,    // 复位到 disarmed, 已清锁存
    not_latched,     // 当前并非锁存态, 未发生复位
    config_invalid,  // 配置非法, 拒绝复位
};

class SentinelOscillationGuard {
public:
    SentinelOscillationGuard() noexcept = default;

    // 配置阈值. 任何配置都不清除已锁存故障, 也不清除已累积危险预算.
    void configure(const GuardianLimits& limits) noexcept;

    // 每 tick 推进. 真实 dt 驱动 budget; 硬条件当拍锁存.
    [[nodiscard]] GuardianReport step(const GuardianSample& sample) noexcept;

    // 只读最近状态 (不推进).
    [[nodiscard]] GuardianReport status() const noexcept;

    // 显式本地确认复位: 需 (1) 配置有效 (2) 当前锁存 (3) locallly_confirmed=true (现场明确确认)
    // (4) 当前存在完整新鲜有效且无硬危险的样本 (硬危险未恢复不得解锁).
    // 仅到 disarmed; 不清预算; rearm 仍需新 (authorization_id, target_revision) 双双更新,
    // 且 rearm 在下一拍通过 same-pass 硬检查后才回 normal.
    // now_us: 调用方当前时刻 (u32 同 time 域). 用于核"最近样本是否新鲜"——长时间无新样本
    // (now_us 早已远超 last_now_us_) 不得复位. 不设默认, 强制调用方显式提供当前时间.
    [[nodiscard]] ResetResult reset_latched(std::uint64_t authorization_id,
                                            std::uint32_t target_revision,
                                            bool locally_confirmed,
                                            std::uint32_t now_us) noexcept;

    // 供 C 在集成层核对: 最近一拍是否为完整 (feedback valid & finite) 且无硬故障的干净样本.
    [[nodiscard]] bool last_sample_clean() const noexcept { return last_sample_clean_; }

private:
    [[nodiscard]] bool config_valid() const noexcept;
    [[nodiscard]] GuardianReport make_report(GuardState before, bool transitioned,
                                             std::uint32_t reasons) const noexcept;
    void enter_state(GuardState s) noexcept;
    void bump_event() noexcept;

    GuardianLimits limits_{};
    Axis axis_{Axis::fire_yaw};   // 本对象绑定轴 (configure/step 确定; pitch 恒 blocked)
    bool have_axis_{false};
    GuardState state_{GuardState::unconfigured};
    bool configured_{false};
    bool latched_{false};

    // 时序/boot.
    std::uint32_t boot_id_{0};
    bool have_boot_{false};
    bool have_boot_known_{false};
    std::uint32_t last_now_us_{0};
    bool have_last_now_{false};

    // 预算/证据 (跨秒持久).
    float exposure_{0.0f};
    float band_energy_{0.0f};       // 带内 (滤波后) 能量
    float residual_energy_raw_{0.0f}; // 原始 r^2 能量
    // 一阶 HP + LP 滤波状态 (固定成本).
    float hp_x_prev_{0.0f};          // 上一输入 (原始残差)
    float hp_y_prev_{0.0f};          // 上一 HP 输出
    bool  hp_have_prev_{false};
    float lp_y_{0.0f};               // 当前 LP 输出 (带通信号)
    bool  lp_have_{false};
    float reversal_accum_{0.0f};
    std::uint32_t reversal_count_{0};
    float envelope_sum_sq_{0.0f};
    std::uint32_t envelope_samples_{0};
    float last_envelope_{0.0f};
    float prev_envelope_{0.0f};
    float peak_envelope_{0.0f};   // 近期包络峰值 (判相对衰减; 有界近期, 非永久)
    float env_ring_[8]{0.0f};     // 近期包络环 (有界趋势, 判连续非下降)
    std::uint8_t env_ring_pos_{0};
    std::uint8_t env_ring_fill_{0};
    float window_accum_ms_{0.0f};
    float state_elapsed_ms_{0.0f};
    float recover_accum_ms_{0.0f};
    float osc_exposure_{0.0f};   // 仅在振荡活跃(近期有反向)时累积的跨秒暴露 (锁定用)
    bool decaying_trend_{false}; // 当前是否处于包络衰减趋势 (衰减环不应最终锁存)
    float rev_activity_{0.0f};   // 近期反向活跃度 (泄漏), 用于判"是否真在振荡"
    float last_residual_{0.0f};
    bool have_last_residual_{false};
    bool last_residual_sign_positive_{false};
    bool have_output_prev_{false};
    bool output_prev_positive_{false};

    // rearm: 复位时的 (authorization_id, target_revision); 二者皆新才回 normal.
    std::uint64_t reset_authority_{0};
    std::uint32_t reset_generation_{0};
    bool have_reset_ids_{false};

    // 最近一拍: 完整新鲜有效且无硬故障 (供 reset 恢复条件核验).
    bool last_sample_clean_{false};      // 完整新鲜且无任何硬故障 (含 authority; 供诊断)
    bool reset_recovery_ok_{false};      // 物理安全恢复可复位判据 (不含 executing/authority)
    bool last_hard_fault_{false};

    std::uint32_t event_count_{0};
    std::uint32_t reason_bits_{0};
    StopRequest stop_request_{StopRequest::none};
    StopConfirmation stop_confirmation_{StopConfirmation::unknown};
};

[[nodiscard]] const char* axis_name(Axis axis) noexcept;
[[nodiscard]] const char* mode_name(GuardMode mode) noexcept;
[[nodiscard]] const char* state_name(GuardState state) noexcept;
[[nodiscard]] const char* stop_request_name(StopRequest r) noexcept;
[[nodiscard]] const char* stop_confirmation_name(StopConfirmation c) noexcept;

} // namespace ec::sentinel::app::oscillation
