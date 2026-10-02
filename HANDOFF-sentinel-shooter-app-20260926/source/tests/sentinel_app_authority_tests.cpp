// O1 来源/权限/目标/恢复状态机契约:
//  - 首启许可: 尚无有效遥控运行状态时, Jetson/维护会话不能独立启动驱动;
//  - 人工整车停止只由人工触发并锁存, 失联不解除, 恢复只执行新接受的目标;
//  - 本地/Jetson/互斥维护三种来源的选择、释放与回收;
//  - 整车原子目标: 缺省域撤旧目标、整包拒绝不续期、有效零值 ≠ 无目标;
//  - 参考/配置代次与来源切换作废旧目标; 执行器重新上电不复活旧目标;
//  - A09 故障影响集合与"自动故障不切人工停止".
#include "sentinel_app.hpp"
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <initializer_list>
#include <utility>

namespace {
void check(bool condition, const char* expression, int line) {
    if (!condition) {
        std::fprintf(stderr, "line %d: %s\n", line, expression);
        std::_Exit(3);
    }
}
#define CHECK(...) check(static_cast<bool>((__VA_ARGS__)), #__VA_ARGS__, __LINE__)

using ec::Tick;
using ec::sentinel::app::AcceptResult;
using ec::sentinel::app::AppInputs;
using ec::sentinel::app::ConfigAudience;
using ec::sentinel::app::Domain;
using ec::sentinel::app::DomainSetpoint;
using ec::sentinel::app::Epochs;
using ec::sentinel::app::FaultState;
using ec::sentinel::app::Reason;
using ec::sentinel::app::SentinelApp;
using ec::sentinel::app::Source;
using ec::sentinel::app::StatusSnapshot;
using ec::sentinel::app::TargetSnapshot;
using ec::control::ControlMode;
using ec::control::StopAction;
using ec::drivers::Dr16Switch;

constexpr std::uint32_t index(Domain domain) { return ec::sentinel::app::domain_index(domain); }

// 测试台: 只组装应用对象, 不接触任何设备或总线.
struct Rig {
    SentinelApp app{ec::sentinel::app::synthetic_config(), ConfigAudience::offline_test};
    AppInputs inputs{};
    Rig() { inputs.epochs = Epochs{1, 0}; inputs.link.locally_ready = true; }
    void step(std::uint32_t ms) {
        // 本文件关注故障依赖, 因此默认给出新鲜发射许可; 许可门禁本身在感知契约里核对.
        inputs.aux.launch_permit_valid = true;
        inputs.aux.launch_permit = true;
        inputs.aux.launch_sampled_at = Tick{ms};
        inputs.aux.launch_ttl = ec::Lifetime{1000};
        app.update(Tick{ms}, inputs);
    }
    [[nodiscard]] const StatusSnapshot& snap() const { return app.snapshot(); }
    [[nodiscard]] const ec::sentinel::app::DomainExecution& exec(Domain domain) const {
        return snap().executions[index(domain)];
    }
};

TargetSnapshot packet(std::initializer_list<std::pair<Domain, DomainSetpoint>> items) {
    TargetSnapshot out{};
    for (const auto& [domain, setpoint] : items) { out.domains[index(domain)] = setpoint; }
    return out;
}

DomainSetpoint velocity(float value) { return DomainSetpoint{ControlMode::velocity, value}; }
DomainSetpoint position(float value) { return DomainSetpoint{ControlMode::position, value}; }

// 建立"遥控运行档": 左拨杆本地 + 右拨杆运行 + 遥控在线.
void go_local(Rig& rig, std::uint32_t ms) {
    rig.inputs.link.remote_online = true;
    rig.inputs.link.switch_left = Dr16Switch::up;
    rig.inputs.link.switch_right = Dr16Switch::mid;
    rig.step(ms);
}

void first_boot_contracts() {
    Rig rig;
    // Jetson 已选来源与会话都就绪, 但还没有有效遥控运行状态 → 不能取得来源.
    rig.inputs.link.switch_left = Dr16Switch::down;
    rig.inputs.link.host_link_alive = true;
    rig.inputs.link.host_request = true;
    rig.step(10);
    CHECK(!rig.app.authority().running_state_reached());
    CHECK(rig.snap().permission.source == Source::none);
    CHECK(!rig.snap().permission.authority);
    CHECK(rig.exec(Domain::chassis_lf).reason == Reason::awaiting_remote);
    // 维护同样不能绕过首启许可.
    Rig maintenance;
    maintenance.inputs.link.switch_left = Dr16Switch::mid;
    maintenance.inputs.link.maintenance_link_alive = true;
    maintenance.inputs.link.maintenance_request = true;
    maintenance.step(10);
    CHECK(maintenance.snap().permission.source == Source::none);
    CHECK(maintenance.exec(Domain::chassis_lf).reason == Reason::awaiting_remote);
    // 无来源时的目标请求被拒绝且不改变任何状态.
    const auto result = maintenance.app.accept_targets(
        packet({{Domain::chassis_lf, velocity(1.0f)}}), Tick{10});
    CHECK(result == AcceptResult::no_authority);
    CHECK(!maintenance.app.target(Domain::chassis_lf).has_value());
}

void remote_and_manual_stop_contracts() {
    Rig rig;
    go_local(rig, 10);
    CHECK(rig.snap().permission.source == Source::remote);
    CHECK(rig.snap().permission.authority);
    CHECK(rig.app.authority().running_state_reached());
    // 有效零值目标是"执行", 与"无目标"不同 (正常零 twist 持续闭环).
    CHECK(rig.app.accept_targets(packet({{Domain::chassis_lf, velocity(0.0f)},
                                      {Domain::main_yaw, position(0.5f)}}),
              Tick{10}) == AcceptResult::accepted);
    rig.step(11);
    CHECK(rig.exec(Domain::chassis_lf).execute);
    CHECK(rig.exec(Domain::chassis_lf).adopted_target == 0.0f);
    CHECK(rig.exec(Domain::main_yaw).execute);
    // 未置有效位的域是"无目标", 并给出该机构约定的撤输出动作.
    CHECK(rig.exec(Domain::fire_yaw).reason == Reason::no_target);
    CHECK(rig.exec(Domain::fire_yaw).stop == StopAction::zero_output);
    CHECK(rig.exec(Domain::pitch).stop == StopAction::disable_output);

    // 人工整车停止: 右拨杆下位. 目标作废, 九 DJI 零输出 / pitch disable.
    rig.inputs.link.switch_right = Dr16Switch::down;
    rig.step(12);
    CHECK(rig.snap().permission.manual_stop);
    CHECK(rig.snap().permission.source == Source::none);
    CHECK(!rig.snap().permission.authority);
    CHECK(!rig.app.target(Domain::chassis_lf).has_value());
    CHECK(!rig.app.target(Domain::main_yaw).has_value());
    CHECK(rig.exec(Domain::chassis_lf).reason == Reason::manual_stop);
    CHECK(rig.exec(Domain::chassis_lf).stop == StopAction::zero_output);
    CHECK(rig.exec(Domain::pitch).stop == StopAction::disable_output);
    CHECK(!rig.exec(Domain::chassis_lf).execute);
    // 停止态目标请求被拒绝.
    CHECK(rig.app.accept_targets(packet({{Domain::chassis_lf, velocity(1.0f)}}), Tick{12}) ==
        AcceptResult::manual_stop);

    // 遥控失联不解除已生效的人工停止.
    rig.inputs.link.remote_online = false;
    rig.step(13);
    CHECK(rig.snap().permission.manual_stop);
    CHECK(rig.exec(Domain::chassis_lf).reason == Reason::manual_stop);

    // 拨回运行挡 = 显式恢复资格; 但只执行恢复后新接受的目标.
    rig.inputs.link.remote_online = true;
    rig.inputs.link.switch_right = Dr16Switch::mid;
    rig.step(200);
    CHECK(!rig.snap().permission.manual_stop);
    CHECK(rig.snap().permission.source == Source::remote);
    CHECK(!rig.app.target(Domain::chassis_lf).has_value());
    CHECK(rig.exec(Domain::chassis_lf).reason == Reason::no_target);
    CHECK(rig.app.accept_targets(packet({{Domain::chassis_lf, velocity(2.0f)}}), Tick{200}) ==
        AcceptResult::accepted);
    rig.step(201);
    CHECK(rig.exec(Domain::chassis_lf).execute);
    CHECK(rig.exec(Domain::chassis_lf).adopted_target == 2.0f);
}

void remote_loss_contracts() {
    Rig rig;
    go_local(rig, 10);
    CHECK(rig.app.accept_targets(packet({{Domain::chassis_lf, velocity(1.0f)}}), Tick{10}) ==
        AcceptResult::accepted);
    // 原本本地控制时失联: 撤销授权, 不自动转给 Jetson (即使 Jetson 会话与请求都在).
    rig.inputs.link.host_link_alive = true;
    rig.inputs.link.host_request = true;
    rig.inputs.link.remote_online = false;
    rig.step(11);
    CHECK(rig.snap().permission.source == Source::none);
    CHECK(!rig.snap().permission.authority);
    CHECK(!rig.app.target(Domain::chassis_lf).has_value());
    CHECK(rig.exec(Domain::chassis_lf).reason == Reason::no_authority);
}

void host_contracts() {
    Rig rig;
    go_local(rig, 10);
    // 显式切到 Jetson: 会话存活且有请求才接入; 目标在上位机持有期间接受.
    rig.inputs.link.switch_left = Dr16Switch::down;
    rig.inputs.link.host_link_alive = true;
    rig.inputs.link.host_request = true;
    rig.step(20);
    CHECK(rig.snap().permission.source == Source::host);
    CHECK(rig.app.accept_targets(packet({{Domain::main_yaw, position(-0.25f)}}), Tick{20}) ==
        AcceptResult::accepted);
    // 已取得 Jetson 控制后遥控失联不撤销该执行.
    rig.inputs.link.remote_online = false;
    rig.step(21);
    CHECK(rig.snap().permission.source == Source::host);
    CHECK(rig.exec(Domain::main_yaw).execute);
    // 遥控接管优先: 拨回本地并在线 → 立即换来源并作废旧目标.
    rig.inputs.link.remote_online = true;
    rig.inputs.link.switch_left = Dr16Switch::up;
    rig.step(22);
    CHECK(rig.snap().permission.source == Source::remote);
    CHECK(!rig.app.target(Domain::main_yaw).has_value());
    // 切回 Jetson 但会话未恢复: 不接入.
    rig.inputs.link.switch_left = Dr16Switch::down;
    rig.inputs.link.host_link_alive = false;
    rig.step(23);
    CHECK(rig.snap().permission.source == Source::none);
    // 会话恢复后重新接入 (请求电平本身不再产生边沿, 这里由会话恢复产生).
    rig.inputs.link.host_link_alive = true;
    rig.step(24);
    CHECK(rig.snap().permission.source == Source::host);
    // 释放请求 → 撤销; 再次请求需要新边沿.
    rig.inputs.link.host_request = false;
    rig.step(25);
    CHECK(rig.snap().permission.source == Source::none);
    rig.inputs.link.host_request = true;
    rig.step(26);
    CHECK(rig.snap().permission.source == Source::host);
}

void maintenance_contracts() {
    Rig rig;
    go_local(rig, 10);
    // 左拨杆待机位 → 释放正常来源.
    rig.inputs.link.switch_left = Dr16Switch::mid;
    rig.step(20);
    CHECK(rig.snap().permission.source == Source::none);
    CHECK(rig.exec(Domain::chassis_lf).reason == Reason::standby);
    // 无来源时维护会话接入.
    rig.inputs.link.maintenance_link_alive = true;
    rig.inputs.link.maintenance_request = true;
    rig.step(21);
    CHECK(rig.snap().permission.source == Source::maintenance);
    CHECK(rig.app.accept_targets(packet({{Domain::friction_left, velocity(3.0f)}}), Tick{21}) ==
        AcceptResult::accepted);
    // 普通 Jetson 请求不能抢正在执行的维护来源.
    rig.inputs.link.host_link_alive = true;
    rig.inputs.link.host_request = true;
    rig.step(22);
    CHECK(rig.snap().permission.source == Source::maintenance);
    // 现场左拨杆显式切到本地 → 撤销维护权与旧目标.
    rig.inputs.link.remote_online = true;
    rig.inputs.link.switch_left = Dr16Switch::up;
    rig.step(23);
    CHECK(rig.snap().permission.source == Source::remote);
    CHECK(!rig.app.target(Domain::friction_left).has_value());
    // 人工停止挡覆盖维护: 停止态可建立会话但不得驱动.
    rig.inputs.link.switch_right = Dr16Switch::down;
    rig.step(24);
    CHECK(rig.snap().permission.manual_stop);
    CHECK(rig.snap().permission.source == Source::none);
    CHECK(rig.exec(Domain::friction_left).reason == Reason::manual_stop);
    CHECK(rig.app.accept_targets(packet({{Domain::friction_left, velocity(3.0f)}}), Tick{24}) ==
        AcceptResult::manual_stop);
    // 解锁后不能靠旧维护请求电平自动重新接入: 人工选源事件已撤销它的本次资格.
    rig.inputs.link.switch_right = Dr16Switch::mid;
    rig.inputs.link.switch_left = Dr16Switch::mid;
    rig.step(25);
    CHECK(rig.snap().permission.source == Source::none);
    // 新的显式维护申请 (请求先退出再提出) 才能重新接入.
    rig.inputs.link.maintenance_request = false;
    rig.step(26);
    rig.inputs.link.maintenance_request = true;
    rig.step(27);
    CHECK(rig.snap().permission.source == Source::maintenance);
}

void maintenance_host_takeover_contract() {
    // F02: 维护持权期间, 新鲜显式选到 Jetson 位必须撤销维护及其目标;
    // 稳定拨杆位置与普通 host 请求不能抢维护.
    Rig rig;
    go_local(rig, 10);
    rig.inputs.link.switch_left = Dr16Switch::mid;
    rig.step(20);
    CHECK(rig.snap().permission.source == Source::none);
    rig.inputs.link.maintenance_link_alive = true;
    rig.inputs.link.maintenance_request = true;
    rig.step(21);
    CHECK(rig.snap().permission.source == Source::maintenance);
    CHECK(rig.app.accept_targets(packet({{Domain::chassis_lf, velocity(1.0f)}}), Tick{21}) ==
        AcceptResult::accepted);
    // 稳定待在待机位 + 普通 Jetson 请求: 不能抢维护.
    rig.inputs.link.host_link_alive = true;
    rig.inputs.link.host_request = true;
    rig.step(22);
    CHECK(rig.snap().permission.source == Source::maintenance);
    CHECK(rig.app.authority().maintenance_holds());
    // 新鲜显式选到 Jetson 位: 撤销维护与旧目标; 已有有效 Jetson 请求则当即接权.
    rig.inputs.link.switch_left = Dr16Switch::down;
    rig.step(23);
    CHECK(!rig.app.authority().maintenance_holds());
    CHECK(!rig.app.target(Domain::chassis_lf).has_value());
    CHECK(rig.snap().permission.source == Source::host);
    CHECK(rig.snap().permission.authority);
}

// A02: 人工显式选到 Jetson 位后, 即使 host 尚未就绪, 旧维护请求也不能在同一周期重新获得资格.
void maintenance_not_rearmed_by_selection_contract() {
    Rig rig;
    go_local(rig, 10);
    // 待机位 + 维护申请 → 维护持权.
    rig.inputs.link.switch_left = Dr16Switch::mid;
    rig.step(20);
    rig.inputs.link.maintenance_link_alive = true;
    rig.inputs.link.maintenance_request = true;
    rig.step(21);
    CHECK(rig.snap().permission.source == Source::maintenance);
    CHECK(rig.app.authority().maintenance_holds());
    // 人工显式选到 Jetson 位, 但 host 链路/请求都未就绪.
    rig.inputs.link.switch_left = Dr16Switch::down;
    rig.inputs.link.host_link_alive = false;
    rig.inputs.link.host_request = false;
    rig.step(22);
    // 新来源未就绪时保持无来源等待, 不能回落维护.
    CHECK(rig.snap().permission.source != Source::maintenance);
    CHECK(!rig.app.authority().maintenance_holds());
    // 旧维护请求电平继续保持, 稳定位置也不能把它重新签回来.
    rig.step(23);
    CHECK(rig.snap().permission.source != Source::maintenance);
    rig.step(24);
    CHECK(rig.snap().permission.source != Source::maintenance);
    // 新的显式维护申请 (请求先退出再提出) 才能重新接入.
    rig.inputs.link.switch_left = Dr16Switch::mid;
    rig.inputs.link.maintenance_request = false;
    rig.step(25);
    rig.inputs.link.maintenance_request = true;
    rig.step(26);
    CHECK(rig.snap().permission.source == Source::maintenance);
}

// A01: 维护会话被替换 (身份代次变化) 时, 重新授权必须换新 generation, 不能继承旧 token.
void maintenance_session_epoch_contract() {
    Rig rig;
    go_local(rig, 10);
    rig.inputs.link.switch_left = Dr16Switch::mid;
    rig.step(20);
    rig.inputs.link.maintenance_link_alive = true;
    rig.inputs.link.maintenance_request = true;
    rig.inputs.link.maintenance_session_epoch = 7;
    rig.step(21);
    CHECK(rig.snap().permission.source == Source::maintenance);
    const auto generation_first = rig.snap().permission.generation;
    // 会话被替换并重新授权: 同一批内两个电平都是真, 但身份代次变了.
    rig.inputs.link.maintenance_session_epoch = 8;
    rig.step(22);
    CHECK(rig.snap().permission.source == Source::maintenance);
    CHECK(rig.snap().permission.generation != generation_first);
}

// 维护会话接入/替换不得牵连已合法持权的遥控来源.
void maintenance_session_does_not_disturb_remote_contract() {
    Rig rig;
    go_local(rig, 10);
    CHECK(rig.snap().permission.source == Source::remote);
    rig.inputs.link.maintenance_link_alive = true;
    rig.inputs.link.maintenance_request = true;
    rig.inputs.link.maintenance_session_epoch = 5;
    rig.step(11);
    CHECK(rig.snap().permission.source == Source::remote);
    rig.inputs.link.maintenance_session_epoch = 6;
    rig.step(12);
    CHECK(rig.snap().permission.source == Source::remote);
    CHECK(rig.snap().permission.authority);
}

// R2: 人工撤销维护后, 新的维护会话身份 (hello+authorize) 必须重建资格,
// 不能继承被撤销那一次申请的封禁; 但同一周期的人工选源事件仍优先.
void maintenance_new_session_rebuilds_qualification_contract() {
    Rig rig;
    go_local(rig, 10);
    // 旧维护申请接入.
    rig.inputs.link.switch_left = Dr16Switch::mid;
    rig.step(20);
    rig.inputs.link.maintenance_link_alive = true;
    rig.inputs.link.maintenance_request = true;
    rig.inputs.link.maintenance_session_epoch = 7;
    rig.step(21);
    CHECK(rig.snap().permission.source == Source::maintenance);
    // 人工切到 host (host 未就绪): 撤销维护并封禁本次申请.
    rig.inputs.link.switch_left = Dr16Switch::down;
    rig.inputs.link.host_link_alive = false;
    rig.inputs.link.host_request = false;
    rig.step(22);
    CHECK(rig.snap().permission.source != Source::maintenance);
    // 对照: 同一身份旧请求持续为真, 稳定位置也不得重新接入.
    rig.step(23);
    CHECK(rig.snap().permission.source != Source::maintenance);
    // 新会话的显式维护申请 (epoch 7→8, 请求电平保持为真): 必须重建资格.
    rig.inputs.link.maintenance_session_epoch = 8;
    rig.step(24);
    CHECK(rig.snap().permission.source == Source::maintenance);
    const auto generation = rig.snap().permission.generation;
    CHECK(generation != 0);
    // 同一周期同时发生人工选源与新身份申请: 人工决定优先, 该次申请被视为已撤销.
    rig.inputs.link.switch_left = Dr16Switch::up;
    rig.inputs.link.maintenance_session_epoch = 9;
    rig.step(25);
    CHECK(rig.snap().permission.source == Source::remote);
    rig.inputs.link.switch_left = Dr16Switch::mid;
    rig.step(26);
    CHECK(rig.snap().permission.source != Source::maintenance);
    // 再次出现新身份申请才能重建.
    rig.inputs.link.maintenance_session_epoch = 10;
    rig.step(27);
    CHECK(rig.snap().permission.source == Source::maintenance);
}

void atomic_target_contracts() {
    Rig rig;
    go_local(rig, 10);
    CHECK(rig.app.accept_targets(packet({{Domain::chassis_lf, velocity(1.0f)},
                                      {Domain::main_yaw, position(0.5f)},
                                      {Domain::pitch, position(-0.2f)}}),
              Tick{10}) == AcceptResult::accepted);
    rig.step(10);
    CHECK(rig.app.target(Domain::chassis_lf).has_value());
    CHECK(rig.app.target(Domain::main_yaw).has_value());
    CHECK(rig.app.target(Domain::pitch).has_value());
    CHECK(rig.exec(Domain::pitch).execute);
    // 缺省域撤销旧目标: 第二包只含一个域.
    CHECK(rig.app.accept_targets(packet({{Domain::chassis_lf, velocity(2.0f)}}), Tick{11}) ==
        AcceptResult::accepted);
    CHECK(!rig.app.target(Domain::main_yaw).has_value());
    CHECK(!rig.app.target(Domain::pitch).has_value());
    CHECK(rig.app.target(Domain::chassis_lf)->value == 2.0f);
    // 关节越限整包拒绝: 已有目标保持不变, 且不部分执行.
    CHECK(rig.app.accept_targets(packet({{Domain::chassis_lf, velocity(9.0f)},
                                      {Domain::pitch, position(3.0f)}}),
              Tick{12}) == AcceptResult::limit_violation);
    CHECK(rig.app.target(Domain::chassis_lf)->value == 2.0f);
    // 非有限值与显式 none 模式同样整包拒绝.
    CHECK(rig.app.accept_targets(packet({{Domain::chassis_lf, velocity(std::nanf(""))}}), Tick{13}) ==
        AcceptResult::invalid_target);
    CHECK(rig.app.accept_targets(packet({{Domain::chassis_lf, DomainSetpoint{ControlMode::none, 1.0f}}}),
              Tick{13}) == AcceptResult::invalid_target);
    CHECK(rig.app.target(Domain::chassis_lf)->value == 2.0f);
    // 目标过期: 各域独立时效, 过期后是撤输出动作而不是继续执行.
    rig.step(40);
    CHECK(rig.exec(Domain::chassis_lf).reason == Reason::stale_target);
    CHECK(rig.exec(Domain::chassis_lf).stop == StopAction::zero_output);
    CHECK(!rig.exec(Domain::chassis_lf).execute);
}

void epoch_and_switch_contracts() {
    Rig rig;
    go_local(rig, 10);
    CHECK(rig.app.accept_targets(packet({{Domain::chassis_lf, velocity(1.0f)}}), Tick{10}) ==
        AcceptResult::accepted);
    // 参考代次变化: 旧参考目标失效 (参考重置不是真实转动).
    rig.inputs.epochs.reference = 7;
    rig.step(11);
    CHECK(!rig.app.target(Domain::chassis_lf).has_value());
    CHECK(rig.app.reference_epoch() == 7);
    // 配置代次变化同样作废.
    CHECK(rig.app.accept_targets(packet({{Domain::chassis_lf, velocity(1.0f)}}), Tick{12}) ==
        AcceptResult::accepted);
    rig.inputs.epochs.config = 9;
    rig.step(13);
    CHECK(!rig.app.target(Domain::chassis_lf).has_value());
    CHECK(rig.app.config_epoch() == 9);
    // 来源切换作废旧来源目标, 切回也不复活.
    CHECK(rig.app.accept_targets(packet({{Domain::chassis_lf, velocity(3.0f)}}), Tick{14}) ==
        AcceptResult::accepted);
    rig.inputs.link.switch_left = Dr16Switch::down;
    rig.inputs.link.host_link_alive = true;
    rig.inputs.link.host_request = true;
    rig.step(15);
    CHECK(rig.snap().permission.source == Source::host);
    CHECK(!rig.app.target(Domain::chassis_lf).has_value());
    rig.inputs.link.switch_left = Dr16Switch::up;
    rig.step(16);
    CHECK(rig.snap().permission.source == Source::remote);
    CHECK(!rig.app.target(Domain::chassis_lf).has_value());
    CHECK(rig.exec(Domain::chassis_lf).reason == Reason::no_target);
}

void fault_dependency_contracts() {
    { // 底盘: 任一底盘轮故障 → 四轮撤输出, 不牵连发射机构.
        Rig rig;
        go_local(rig, 10);
        rig.inputs.faults.feedback_lost[index(Domain::chassis_rf)] = true;
        rig.step(11);
        for (const auto domain : {Domain::chassis_lf, Domain::chassis_rf, Domain::chassis_lb,
                 Domain::chassis_rb}) {
            CHECK(rig.exec(domain).reason == Reason::fault_dependency);
            CHECK(rig.exec(domain).stop == StopAction::zero_output);
            CHECK(!rig.exec(domain).execute);
        }
        CHECK(rig.snap().domains[index(Domain::chassis_rf)].fault == FaultState::feedback_lost);
        CHECK(rig.snap().domains[index(Domain::chassis_lf)].fault == FaultState::dependency_fault);
        CHECK(rig.exec(Domain::trigger).reason == Reason::no_target);
        CHECK(!rig.snap().permission.manual_stop);
    }
    { // 主 yaw 参考失效 → 该轴撤输出并禁止正向拨弹; 反转与其它域不受影响.
        Rig rig;
        go_local(rig, 10);
        CHECK(rig.app.accept_targets(packet({{Domain::trigger, velocity(8.0f)},
                                          {Domain::fire_yaw, position(0.1f)}}),
                  Tick{10}) == AcceptResult::accepted);
        rig.inputs.faults.reference_lost[index(Domain::main_yaw)] = true;
        rig.step(11);
        CHECK(rig.exec(Domain::main_yaw).reason == Reason::fault_dependency);
        CHECK(rig.exec(Domain::main_yaw).stop == StopAction::zero_output);
        CHECK(rig.exec(Domain::trigger).reason == Reason::fault_dependency); // 正向拨弹被禁
        CHECK(rig.exec(Domain::fire_yaw).execute);                            // 其它轴不受牵连
        // 人工反转不受正向拨弹许可限制 (拨弹目标被改为负向).
        CHECK(rig.app.accept_targets(packet({{Domain::trigger, velocity(-8.0f)}}), Tick{12}) ==
            AcceptResult::accepted);
        rig.step(12);
        CHECK(rig.exec(Domain::trigger).execute);
    }
    { // pitch 反馈失效 → 请求 disable 并禁止正向拨弹.
        Rig rig;
        go_local(rig, 10);
        rig.inputs.faults.feedback_lost[index(Domain::pitch)] = true;
        rig.step(11);
        CHECK(rig.exec(Domain::pitch).stop == StopAction::disable_output);
        CHECK(!rig.exec(Domain::pitch).execute);
        CHECK(rig.snap().domains[index(Domain::pitch)].fault == FaultState::feedback_lost);
    }
    { // 摩擦轮任一故障 → 两摩擦轮与拨弹撤输出, 底盘/云台无关.
        Rig rig;
        go_local(rig, 10);
        rig.inputs.faults.device_fault[index(Domain::friction_right)] = true;
        rig.step(11);
        CHECK(rig.exec(Domain::friction_left).reason == Reason::fault_dependency);
        CHECK(rig.exec(Domain::friction_right).reason == Reason::fault_dependency);
        CHECK(rig.exec(Domain::trigger).reason == Reason::fault_dependency);
        CHECK(rig.exec(Domain::chassis_lf).reason == Reason::no_target);
        CHECK(rig.exec(Domain::main_yaw).reason == Reason::no_target);
        CHECK(rig.exec(Domain::trigger).stop == StopAction::zero_output);
    }
    { // 拨弹自身故障 → 仅拨弹撤输出, 摩擦轮仍可按独立请求运行.
        Rig rig;
        go_local(rig, 10);
        rig.inputs.faults.device_fault[index(Domain::trigger)] = true;
        rig.step(11);
        CHECK(rig.exec(Domain::trigger).reason == Reason::fault_dependency);
        CHECK(rig.exec(Domain::friction_left).reason == Reason::no_target);
        CHECK(rig.exec(Domain::friction_right).reason == Reason::no_target);
    }
    { // 主 yaw 仅惯性失效: G07 降级保持, 不是设备故障, 不影响拨弹资格.
        Rig rig;
        go_local(rig, 10);
        CHECK(rig.app.accept_targets(packet({{Domain::trigger, velocity(8.0f)}}), Tick{10}) ==
            AcceptResult::accepted);
        rig.inputs.faults.main_yaw_inertial_only = true;
        rig.step(11);
        CHECK(rig.snap().domains[index(Domain::main_yaw)].fault == FaultState::degraded);
        CHECK(rig.exec(Domain::main_yaw).reason != Reason::fault_dependency);
        CHECK(rig.exec(Domain::trigger).execute);
        CHECK(!rig.snap().permission.manual_stop); // 自动降级不等于人工停止
    }
}

void recovery_contracts() {
    Rig rig;
    go_local(rig, 10);
    CHECK(rig.app.accept_targets(packet({{Domain::chassis_lf, velocity(3.0f)},
                                      {Domain::chassis_rf, velocity(3.0f)},
                                      {Domain::chassis_lb, velocity(3.0f)},
                                      {Domain::chassis_rb, velocity(3.0f)}}),
              Tick{10}) == AcceptResult::accepted);
    rig.step(11);
    CHECK(rig.exec(Domain::chassis_lf).execute);
    // 反馈失效: 旧目标立即作废.
    rig.inputs.faults.feedback_lost[index(Domain::chassis_rb)] = true;
    rig.step(12);
    CHECK(!rig.app.target(Domain::chassis_lf).has_value());
    // 反馈刚恢复: 不追旧目标, 也不自动重新使能, 等待新目标.
    rig.inputs.faults.feedback_lost[index(Domain::chassis_rb)] = false;
    rig.step(13);
    CHECK(!rig.app.target(Domain::chassis_lf).has_value());
    CHECK(rig.exec(Domain::chassis_lf).reason == Reason::no_target);
    CHECK(!rig.exec(Domain::chassis_lf).execute);
    // 新目标重新建立后才执行.
    CHECK(rig.app.accept_targets(packet({{Domain::chassis_lf, velocity(1.0f)},
                                      {Domain::chassis_rf, velocity(1.0f)},
                                      {Domain::chassis_lb, velocity(1.0f)},
                                      {Domain::chassis_rb, velocity(1.0f)}}),
              Tick{14}) == AcceptResult::accepted);
    rig.step(14);
    CHECK(rig.exec(Domain::chassis_lf).execute);

    // 执行器可检测重新上电: 该域旧目标作废并给出明确原因, 其余域不受影响.
    rig.inputs.faults.device_restarted[index(Domain::chassis_lb)] = true;
    rig.step(15);
    CHECK(!rig.app.target(Domain::chassis_lb).has_value());
    CHECK(rig.exec(Domain::chassis_lb).reason == Reason::restarted);
    CHECK(rig.exec(Domain::chassis_lb).stop == StopAction::zero_output);
    CHECK(rig.app.target(Domain::chassis_lf).has_value());
    rig.inputs.faults.device_restarted[index(Domain::chassis_lb)] = false;
    rig.step(16);
    CHECK(rig.exec(Domain::chassis_lb).reason == Reason::no_target);
    CHECK(!rig.snap().permission.manual_stop);
}
} // 测试辅助

// 板级路径 (APP07): 来源选择必须锁存, 维护交接按 I05 冻结表.
// 板级在遥控掉线时按"没有新鲜遥控帧"填写整组链路输入 (拨杆 = unknown).
void source_latch_and_handover_contracts() {
    // 1) host 持权 + 遥控整组消失: 来源选择锁存, host 保持.
    {
        Rig rig;
        go_local(rig, 10);
        rig.inputs.link.switch_left = Dr16Switch::down;
        rig.inputs.link.host_link_alive = true;
        rig.inputs.link.host_request = true;
        rig.step(20);
        CHECK(rig.snap().permission.source == Source::host);
        CHECK(rig.app.accept_targets(packet({{Domain::main_yaw, position(-0.2f)}}), Tick{20}) ==
            AcceptResult::accepted);
        rig.inputs.link.remote_online = false;
        rig.inputs.link.switch_left = Dr16Switch::unknown;
        rig.inputs.link.switch_right = Dr16Switch::unknown;
        rig.step(21);
        CHECK(rig.snap().permission.source == Source::host);
        CHECK(rig.exec(Domain::main_yaw).execute);
        CHECK(rig.app.authority().selection() == Dr16Switch::down);
    }
    // 2) local 持权 + 遥控整组消失: 不转 host (即使 host 会话与请求都在).
    {
        Rig rig;
        go_local(rig, 10);
        rig.inputs.link.host_link_alive = true;
        rig.inputs.link.host_request = true;
        rig.inputs.link.remote_online = false;
        rig.inputs.link.switch_left = Dr16Switch::unknown;
        rig.step(11);
        CHECK(rig.snap().permission.source == Source::none);
        CHECK(rig.app.authority().selection() == Dr16Switch::up);
    }
    // 3) host 释放 (会话结束) 且拨杆未动 → 维护可直接接管, 不要求拨到待机位.
    {
        Rig rig;
        go_local(rig, 10);
        rig.inputs.link.switch_left = Dr16Switch::down;
        rig.inputs.link.host_link_alive = true;
        rig.inputs.link.host_request = true;
        rig.step(20);
        CHECK(rig.snap().permission.source == Source::host);
        rig.inputs.link.host_link_alive = false;
        rig.inputs.link.host_request = false;
        rig.inputs.link.maintenance_link_alive = true;
        rig.inputs.link.maintenance_request = true;
        rig.step(21);
        CHECK(rig.snap().permission.source == Source::maintenance);
        // 稳定拨杆位置不重复抢权: 遥控恢复而拨杆仍停在 Jetson 位 → 维护保持.
        rig.inputs.link.remote_online = true;
        rig.step(22);
        CHECK(rig.snap().permission.source == Source::maintenance);
        CHECK(rig.app.accept_targets(packet({{Domain::chassis_lf, velocity(1.0f)}}), Tick{22}) ==
            AcceptResult::accepted);
        // 新鲜明确切换 (Jetson → 本地) 抢回控制, 并撤销维护权与旧目标.
        rig.inputs.link.switch_left = Dr16Switch::up;
        rig.step(23);
        CHECK(rig.snap().permission.source == Source::remote);
        CHECK(!rig.app.target(Domain::chassis_lf).has_value());
        CHECK(!rig.app.authority().maintenance_holds());
    }
    // 4) 维护期间普通 Jetson 请求 (含会话新建产生的请求边沿) 不能抢正在执行的维护来源.
    {
        Rig rig;
        go_local(rig, 10);
        rig.inputs.link.switch_left = Dr16Switch::down;
        rig.inputs.link.host_link_alive = true;
        rig.inputs.link.host_request = true;
        rig.step(20);
        CHECK(rig.snap().permission.source == Source::host);
        rig.inputs.link.host_link_alive = false;
        rig.inputs.link.host_request = false;
        rig.inputs.link.maintenance_link_alive = true;
        rig.inputs.link.maintenance_request = true;
        rig.step(21);
        CHECK(rig.snap().permission.source == Source::maintenance);
        rig.inputs.link.host_link_alive = true;
        rig.inputs.link.host_request = true;
        rig.step(22);
        CHECK(rig.snap().permission.source == Source::maintenance);
        // 维护期间保持的 Jetson 请求电平不产生假边沿: 维护释放后仍需重新请求.
        rig.inputs.link.maintenance_request = false;
        rig.step(23);
        CHECK(rig.snap().permission.source == Source::none);
        rig.step(24);
        CHECK(rig.snap().permission.source == Source::none);
        rig.inputs.link.host_request = false;
        rig.step(25);
        rig.inputs.link.host_request = true;
        rig.step(26);
        CHECK(rig.snap().permission.source == Source::host);
    }
    // 5) 人工停止锁存不被遥控断流解除, 只由人工把右拨杆拨回运行档解除.
    {
        Rig rig;
        go_local(rig, 10);
        rig.inputs.link.switch_right = Dr16Switch::down;
        rig.step(11);
        CHECK(rig.snap().permission.manual_stop);
        rig.inputs.link.remote_online = false;
        rig.inputs.link.switch_left = Dr16Switch::unknown;
        rig.inputs.link.switch_right = Dr16Switch::unknown;
        rig.step(12);
        CHECK(rig.snap().permission.manual_stop);
        rig.inputs.link.remote_online = true;
        rig.inputs.link.switch_left = Dr16Switch::up;
        rig.inputs.link.switch_right = Dr16Switch::mid;
        rig.step(13);
        CHECK(!rig.snap().permission.manual_stop);
    }
}

int main() {
    first_boot_contracts();
    remote_and_manual_stop_contracts();
    remote_loss_contracts();
    host_contracts();
    maintenance_contracts();
    maintenance_host_takeover_contract();
    maintenance_not_rearmed_by_selection_contract();
    maintenance_session_epoch_contract();
    maintenance_session_does_not_disturb_remote_contract();
    maintenance_new_session_rebuilds_qualification_contract();
    source_latch_and_handover_contracts();
    atomic_target_contracts();
    epoch_and_switch_contracts();
    fault_dependency_contracts();
    recovery_contracts();
    std::puts("sentinel app authority contracts passed");
    return 0;
}
