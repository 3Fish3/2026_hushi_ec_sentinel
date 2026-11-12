#pragma once
// 正式 APP 业务协议 (N5, v3). 复用既有 0x7e 分帧 (UartFrame: 版本/类型/序号/长度/CRC16),
// 这里定义其上的版本化业务消息与会话处理. 不使用板级观测布局, 也不提供任意内存/CAN 写入.
//  - 会话身份: hello 分配会话; 身份包含启动身份 (boot_id), 重启后即使序号相同也不是同一身份;
//  - 会话老化: 会话链路按 host_link_ttl 对真实收包时刻老化, 到期即撤销该来源的授权与目标;
//  - 业务序号: 每个会话内单调递增; 同一序号重传返回上次的结果码, 不用"重放"冒充成功;
//  - 请求资格: 目标/对齐/参数写入必须来自当前持权的来源会话 (或当前无人持权), 只读请求不受限;
//  - 各域源时效: targets 的每个域携带源年龄, 接收端据此设定期限, 不因整包刷新相互续命;
//  - 代次: 目标包携带 config/reference 代次, 不匹配按 epoch_mismatch 拒绝;
//  - 时间: 一律使用 MCU 单调毫秒时钟; 收包时间不会被当作任何传感器的采样时刻;
//  - 接受/拒绝与实际执行状态分开: 目标回包只表示是否接受, 执行状态由 status 消息报告;
//  - 应答关联: 响应帧序号回显请求帧序号, 业务层再核对 session/seq, 两者都必须匹配.
#include "sentinel_app.hpp"
#include "sentinel_config.hpp"
#include "sentinel_execution.hpp"
#include "sentinel_params.hpp"
#include "sentinel_perception.hpp"
#include "sentinel_experiment_wire.hpp"
#include <array>
#include <cstdint>
#include <span>

namespace ec::sentinel::app {
inline constexpr std::uint8_t protocol_version = 3;
// status 页 0 为高频反馈, 页 1 为全域执行, 页 2 为原始请求与命令预测.
enum class StatusPage : std::uint8_t { fast = 0, domains = 1, requests = 2 };

// 请求/响应消息类型. 与板级观测 (0x81) 和采集 (0x30/0xb0) 明确分开.
enum class MessageType : std::uint8_t {
    hello = 0x40,
    authorize = 0x41,
    targets = 0x42,
    align = 0x43,
    param_read = 0x44,
    param_write = 0x45,
    status = 0x46,
    diagnostics = 0x47,
    permit = 0x48,
    pitch_identification = 0x49,
    experiment = 0x60,
    hello_ack = 0xc0,
    ack = 0xc1,
    targets_ack = 0xc2,
    param_value = 0xc4,
    status_snapshot = 0xc6,
    diagnostics_page = 0xc7,
    pitch_identification_status = 0xc9,
    experiment_snapshot = 0xca,
};

enum class PitchIdentificationOp : std::uint8_t {
    query = 0,
    start = 1,
    cancel = 2,
    reset = 3,
};

enum class ResultCode : std::uint8_t {
    accepted = 0,
    rejected = 1,
    bad_version = 2,
    bad_length = 3,
    no_session = 4,
    replay = 5,
    epoch_mismatch = 6,
    not_permitted = 7,
    unknown_request = 8,
    unknown_param = 9,
    invalid_value = 10,
    reference_in_use = 11,
    no_target = 12,
    align_failed = 13,
    unsupported = 14, // 该参数/组合尚未实现实际生效路径, 不谎报 effective
};

// 对齐类型: 机械关节参考与惯性/odom 对齐偏移是两件不同的事, 不能共用一个数值.
enum class AlignKind : std::uint8_t {
    joint_reference = 0, // 当前机械位置对应的关节角 (建立/重建位置参考)
    inertial_alignment = 1, // MCU 惯性参考与上位机 odom 的偏移 (只对主 yaw 有意义)
};

inline constexpr std::size_t protocol_payload_max = 240;
// 参数读回按页返回: 一页最多容纳的条目数, 由载荷上限与单条目 16 字节决定.
inline constexpr std::size_t param_page_max = (protocol_payload_max - 12) / 16;
inline constexpr std::size_t identity_length = 16;

// 响应载荷自带的业务类型 (第 2 字节). 外层帧类型必须与它一致: 错误路径回短 ACK 时
// 不能再借"请求的预期成功类型"当外层类型, 否则接收方会按成功布局解码本次错误.
[[nodiscard]] inline std::uint8_t declared_response_type(
    std::span<const std::uint8_t> response) noexcept {
    return response.size() > 1 ? response[1] : 0;
}

// 逐字段小端编码/解码, 不传输 C 结构体布局.
struct PayloadWriter {
    std::span<std::uint8_t> out{};
    std::size_t used{};
    bool overflow{};
    void u8(std::uint8_t value) noexcept;
    void u16(std::uint16_t value) noexcept;
    void u32(std::uint32_t value) noexcept;
    void f32(float value) noexcept;
    void bytes(std::span<const std::uint8_t> value) noexcept;
    [[nodiscard]] std::size_t size() const noexcept { return used; }
};

struct PayloadReader {
    std::span<const std::uint8_t> in{};
    std::size_t used{};
    bool underflow{};
    [[nodiscard]] std::uint8_t u8() noexcept;
    [[nodiscard]] std::uint16_t u16() noexcept;
    [[nodiscard]] std::uint32_t u32() noexcept;
    [[nodiscard]] float f32() noexcept;
    [[nodiscard]] std::size_t remaining() const noexcept { return in.size() - used; }
    [[nodiscard]] bool ok() const noexcept { return !underflow; }
    // 精确长度检查: 尾随垃圾与截断一样都是格式错误, 不能只检查"读到的字段够用".
    [[nodiscard]] bool exact() const noexcept { return !underflow && used == in.size(); }
};

// 会话处理器: 主机后端与板级固件共用同一份请求语义.
// 只处理业务消息; 分帧、CRC 与帧序号回显由持有 UartFrameDecoder 的一方负责.
class SentinelHostSession {
public:
    // host_link_ttl: 会话链路的有效期 (真实收包时刻老化); boot_id: 启动身份, 0 表示不可用.
    SentinelHostSession(SentinelApp& app, ParamStore& params, SentinelExecutor* executor,
        SentinelPerception* perception, Lifetime host_link_ttl = {}, std::uint32_t boot_id = 0,
        const excitation::TrustedDeploymentSafetyConfig* deployment = nullptr,
        const excitation::TrustedDeploymentRoot* root = nullptr) noexcept
        : experiment_(deployment && root && root->boot_id == boot_id
              ? excitation::Wire(*deployment, *root) : excitation::Wire{}),
          app_(&app), params_(&params), executor_(executor), perception_(perception),
          link_ttl_(host_link_ttl), boot_id_(boot_id) {
        // 构造时就把“启动会话身份”登记给应用: 应用在此之前不能上报 0 ("不提供身份"),
        // 否则首次握手会被核心仲裁当成“会话被替换”而吞掉新会话的第一个请求边沿.
        epoch_ = compute_epoch(0);
        session_id_ = epoch_;
        app_->set_host_session(false, false, epoch_);
    }

    // 会话身份代次 = 启动身份 + 会话序号 (重启后即使序号相同也不是同一身份).
    // boot_id 不可用时只能保证"同一启动内不同会话不同", 由 hello_ack 如实上报.
    [[nodiscard]] std::uint32_t compute_epoch(std::uint32_t counter) const noexcept {
        if (boot_id_ == 0) { return boot_session_epoch + counter; }
        const auto value = boot_id_ ^ (counter * 0x9E3779B9u);
        return value != 0 ? value : boot_session_epoch;
    }

    // 处理一条请求载荷, 写出响应载荷; 返回响应长度 (0 = 无法处理, 不产生响应).
    [[nodiscard]] std::size_t handle(std::span<const std::uint8_t> request, Tick now,
        std::span<std::uint8_t> response) noexcept;
    [[nodiscard]] std::uint32_t session_id() const noexcept { return session_id_; }
    [[nodiscard]] std::uint32_t replay_rejects() const noexcept { return replay_rejects_; }
    [[nodiscard]] std::uint32_t duplicate_requests() const noexcept { return duplicate_requests_; }
    [[nodiscard]] std::uint32_t accepted_commands() const noexcept { return accepted_commands_; }
    [[nodiscard]] std::uint32_t expired_sessions() const noexcept { return expired_sessions_; }
    [[nodiscard]] std::uint32_t boot_id() const noexcept { return boot_id_; }
    // 启动身份是否可用: 为假时跨重启的会话/序号隔离不成立, 必须如实上报而不是假装唯一.
    [[nodiscard]] bool boot_identity_available() const noexcept { return boot_id_ != 0; }
    // 本会话当前授予的来源 (host/maintenance); 未授权过时为 none.
    [[nodiscard]] Source role_source() const noexcept { return role_source_; }
    // 激励实验控制器 (供周期层绑定: 同一实例驱动状态机与记录).
    [[nodiscard]] excitation::Controller& experiment_controller() noexcept { return experiment_.controller_for_cycle(); }

    // 会话链路状态: 每个控制周期调用一次.
    //  - alive: 会话存在且距最后一次收包未超过 host_link_ttl;
    //  - request: 该会话是否仍持有显式授权请求 (电平);
    //  - epoch: 会话身份代次, 用于让"换会话"立即失效旧授权;
    //  - expired: 本次调用刚好发生到期 (调用方可用它记录一次事件).
    struct LinkState {
        bool alive{};
        bool request{};
        std::uint32_t epoch{};
        bool expired{};
    };
    [[nodiscard]] LinkState link_state(Tick now) noexcept;

    // 参数生效事务: 由 apply_param 落到运行对象并从运行对象读回实际值.
    [[nodiscard]] ParamWriteResult write_param(std::uint16_t id, float value, bool idle) noexcept;
    // 帧层拒绝计数由持有分帧解码器的一方填入 (主机后端或板级 UART), 只用于上报.
    void set_frame_rejects(std::uint32_t count) noexcept { frame_rejects_ = count; }
    void set_tx_diagnostics(const std::array<std::uint32_t, 10>& values) noexcept { tx_diagnostics_ = values; }

private:
    [[nodiscard]] std::size_t ack(MessageType type, std::uint32_t session, std::uint32_t seq,
        ResultCode result, std::uint8_t reason, std::span<std::uint8_t> response) noexcept;
    // 会话与序号校验结果.
    //  - proceed=false 时调用方必须直接回 code 并放弃执行任何副作用;
    //  - 同一序号重传在 handle() 里用缓存的完整响应重发, 不会走到这里; 这里只处理
    //    真正重放 (序号倒退) 与幂等请求的同序号重复.
    struct SessionCheck {
        ResultCode code{ResultCode::accepted};
        bool proceed{};
    };
    [[nodiscard]] SessionCheck check_session(std::uint32_t session, std::uint32_t seq,
        bool non_idempotent, Tick now) noexcept;
    // 按当前时刻判定本会话是否已超时并执行到期撤销; 返回本次调用是否刚好到期.
    // check_session 与 link_state 共用它, 因此"请求先到还是老化先跑"不再改变结果.
    [[nodiscard]] bool expire_if_stale(Tick now) noexcept;
    // 只有新握手或业务被接受才续会话; 身份/序号校验和缓存回放不续任何期限.
    void accept_activity(Tick now) noexcept { last_request_at_ = now; }
    [[nodiscard]] bool domain_idle(Domain domain) const noexcept;
    [[nodiscard]] bool all_domains_idle() const noexcept;
    // 请求是否来自当前持权的来源 (执行类操作的必要条件).
    [[nodiscard]] bool requester_holds() const noexcept;
    // 请求是否有资格修改参考/配置: 当前无人持权, 或请求方就是持权来源.
    [[nodiscard]] bool requester_may_configure() const noexcept;
    [[nodiscard]] std::size_t handle_hello(std::span<std::uint8_t> response, Tick now,
        std::span<const std::uint8_t> payload) noexcept;
    [[nodiscard]] std::size_t handle_authorize(std::span<std::uint8_t> response,
        std::span<const std::uint8_t> payload, Tick now) noexcept;
    [[nodiscard]] std::size_t handle_targets(std::span<std::uint8_t> response,
        std::span<const std::uint8_t> payload, Tick now) noexcept;
    [[nodiscard]] std::size_t handle_align(std::span<std::uint8_t> response,
        std::span<const std::uint8_t> payload, Tick now) noexcept;
    [[nodiscard]] std::size_t handle_param_read(std::span<std::uint8_t> response,
        std::span<const std::uint8_t> payload, Tick now) noexcept;
    [[nodiscard]] std::size_t handle_param_write(std::span<std::uint8_t> response,
        std::span<const std::uint8_t> payload, Tick now) noexcept;
    [[nodiscard]] std::size_t handle_permit(std::span<std::uint8_t> response,
        std::span<const std::uint8_t> payload, Tick now) noexcept;
    [[nodiscard]] std::size_t handle_status(std::span<std::uint8_t> response,
        std::span<const std::uint8_t> payload, Tick now) noexcept;
    [[nodiscard]] std::size_t handle_diagnostics(std::span<std::uint8_t> response,
        std::span<const std::uint8_t> payload, Tick now) noexcept;
    [[nodiscard]] std::size_t handle_pitch_identification(std::span<std::uint8_t> response,
        std::span<const std::uint8_t> payload, Tick now) noexcept;
    [[nodiscard]] std::size_t handle_experiment(std::span<std::uint8_t> response,
        std::span<const std::uint8_t> payload, Tick now) noexcept;
    // 维护调试操作批准: 需维护持权 + 本机已选择调试维护 + 遥控在线.
    [[nodiscard]] bool operator_approval_present() const noexcept;
    // 非幂等请求 (会改变状态): 同序号重传必须回放原完整响应, 不重复执行.
    [[nodiscard]] static bool non_idempotent_request(MessageType type) noexcept;
    [[nodiscard]] static bool mutation_request(MessageType type,
        std::span<const std::uint8_t> payload = {}) noexcept {
        if (type == MessageType::authorize || type == MessageType::align || type == MessageType::param_write) {
            return true;
        }
        if (type == MessageType::pitch_identification) {
            return payload.size() > 10 && payload[10] != 0;
        }
        if (type == MessageType::experiment) {
            return payload.size() > 10 && payload[10] != 0;
        }
        return false;
    }
    // 一个待确认变更槽与最后消息槽分开. 目标/只读流推进序号不覆盖变更结果;
    // 新变更覆盖旧槽. 完整请求逐字节比较, 不把散列碰撞当成同一请求.
    struct ResultCache {
        std::array<std::uint8_t, protocol_payload_max> request{}, response{};
        std::size_t request_size{}, response_size{};
        std::uint32_t seq{};
        [[nodiscard]] bool matches(std::span<const std::uint8_t> payload) const noexcept;
        void store(std::uint32_t sequence, std::span<const std::uint8_t> payload,
            std::span<const std::uint8_t> result) noexcept;
    };

    // 激励实验会话处理器: 必须声明在 app_ 之前以匹配构造初始化顺序 (-Wreorder).
    excitation::Wire experiment_{};
    SentinelApp* app_{};
    ParamStore* params_{};
    SentinelExecutor* executor_{};
    SentinelPerception* perception_{};
    Lifetime link_ttl_{};
    std::uint32_t boot_id_{};
    std::uint32_t session_id_{};
    std::uint32_t session_counter_{};
    std::uint32_t client_nonce_{};
    std::uint32_t epoch_{};
    std::uint32_t last_seq_{};
    Source role_source_{Source::none};
    Tick last_request_at_{};
    bool has_session_{};
    bool has_seq_{};
    // 本请求是否已通过当前会话/序号校验进入处理阶段: 只有它为真才允许写结果缓存,
    // 否则错误 session/长度的请求会用自己的错误结果覆盖合法请求的缓存.
    bool request_validated_{};
    bool request_{};
    bool link_expired_{};
    ResultCache last_result_{}, mutation_result_{};
    std::uint32_t replay_rejects_{};
    std::uint32_t duplicate_requests_{};
    std::uint32_t accepted_commands_{};
    std::uint32_t expired_sessions_{};
    std::uint32_t frame_rejects_{};
    std::array<std::uint32_t, 10> tx_diagnostics_{};
};
} // 命名空间 ec::sentinel::app
