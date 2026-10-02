// M4 输入契约: DR16 薄输入服务的一致快照、边沿语义与掉线恢复不产生虚假按键边沿.
#include "ec/drivers/dr16.hpp"
#include "ec/input/dr16_input.hpp"
#include <cstdio>
#include <cstdlib>

namespace {
void check(bool condition, const char* expression, int line) {
    if (!condition) {
        std::fprintf(stderr, "line %d: %s\n", line, expression);
        std::_Exit(3); // 非零退出且不触发 CRT 交互弹窗, ctest 要求非交互
    }
}
#define CHECK(...) check(static_cast<bool>((__VA_ARGS__)), #__VA_ARGS__, __LINE__)

using ec::Tick;
using ec::drivers::Dr16Sample;
using ec::drivers::Dr16Switch;
using ec::input::Dr16Input;

Dr16Sample base_sample() {
    Dr16Sample sample{};
    sample.ch0 = 660;
    sample.ch1 = -660;
    sample.ch2 = 0;
    sample.ch3 = 330;
    sample.switch_left = Dr16Switch::up;
    sample.switch_right = Dr16Switch::down;
    sample.mouse_x = 3;
    sample.mouse_y = -4;
    sample.mouse_z = 0;
    sample.mouse_left = true;
    sample.mouse_right = false;
    sample.keys = 0x0005;
    return sample;
}

void snapshot_contracts() {
    Dr16Input input{{20}};
    CHECK(!input.online({0}));
    CHECK(!input.snapshot({0}));
    CHECK(input.accept(base_sample(), {100}));
    CHECK(input.online({119}) && !input.online({120}));
    const auto snapshot = input.snapshot({100});
    CHECK(snapshot);
    // 轴归一化: 660 → 1, -660 → -1, 330 → 0.5.
    CHECK(snapshot->axis[0] == 1.0f);
    CHECK(snapshot->axis[1] == -1.0f);
    CHECK(snapshot->axis[2] == 0.0f);
    CHECK(snapshot->axis[3] == 0.5f);
    CHECK(snapshot->switch_left == Dr16Switch::up);
    CHECK(snapshot->switch_right == Dr16Switch::down);
    CHECK(snapshot->keys == 0x0005);
    CHECK(snapshot->mouse_left && !snapshot->mouse_right);
    CHECK(snapshot->sampled_at.ms == 100);
    // 快照字段全部来自同一帧.
    CHECK(snapshot->raw.mouse_x == 3 && snapshot->raw.ch3 == 330);
    // 过期后不再给出快照, 而不是给出旧测量.
    CHECK(!input.snapshot({120}));
    CHECK(input.accepted() == 1 && input.rejected() == 0 && input.gaps() == 0);
}

void edge_contracts() {
    Dr16Input input{{20}};
    // 首帧不产生边沿: 启动时按住键也不应报"新按下".
    CHECK(input.accept(base_sample(), {100}));
    CHECK(input.edges().keys_pressed == 0 && input.edges().keys_released == 0);
    CHECK(!input.edges().mouse_left_pressed);

    auto next = base_sample();
    next.keys = 0x0015;              // 新增 bit4
    next.mouse_left = false;
    next.mouse_right = true;
    CHECK(input.accept(next, {110}));
    CHECK(input.edges().keys_pressed == 0x0010);
    CHECK(input.edges().keys_released == 0);
    CHECK(input.edges().mouse_left_released && input.edges().mouse_right_pressed);
    CHECK(!input.edges().mouse_left_pressed && !input.edges().mouse_right_released);

    next.keys = 0x0000;              // 松开全部
    next.mouse_right = false;
    CHECK(input.accept(next, {120}));
    CHECK(input.edges().keys_released == 0x0015);
    CHECK(input.edges().keys_pressed == 0);
    CHECK(input.edges().mouse_right_released);

    // 同一帧重复提交不产生边沿(前后相同).
    CHECK(input.accept(next, {130}));
    CHECK(input.edges().keys_pressed == 0 && input.edges().keys_released == 0);

    // 掉线后恢复: 中间帧的键位变化不能被记成边沿.
    auto held = base_sample();
    held.keys = 0xffff;
    held.mouse_left = true;
    CHECK(input.accept(held, {200})); // 从 130 之后超过 TTL, 视为不连续
    CHECK(input.gaps() == 1);
    CHECK(input.edges().keys_pressed == 0 && input.edges().keys_released == 0);
    CHECK(!input.edges().mouse_left_pressed);
    // 恢复后的下一帧才恢复正常边沿比较.
    auto released = held;
    released.keys = 0xfffe;
    released.mouse_left = false;
    CHECK(input.accept(released, {210}));
    CHECK(input.edges().keys_pressed == 0);
    CHECK(input.edges().keys_released == 0x0001);
    CHECK(input.edges().mouse_left_released);

    // 人工 reset 后再来一帧同样不产生边沿, 并且样本被丢弃.
    input.reset();
    CHECK(!input.online({210}));
    CHECK(!input.snapshot({210}));
    CHECK(input.accept(held, {220}));
    CHECK(input.edges().keys_pressed == 0 && input.edges().keys_released == 0);
    CHECK(input.online({220}));
}

void time_contracts() {
    Dr16Input input{{20}};
    CHECK(input.accept(base_sample(), {100}));
    // 时间倒退的帧被拒绝, 也不刷新时效.
    auto next = base_sample();
    next.keys = 0xffff;
    CHECK(!input.accept(next, {99}));
    CHECK(input.rejected() == 1);
    CHECK(input.online({100}) && input.online({119}) && !input.online({120}));
    CHECK(input.snapshot({119}));
    CHECK(input.snapshot({100})->keys == 0x0005);
    CHECK(input.edges().keys_pressed == 0);
    // 时基回绕: 只要新鲜度成立, 边沿比较仍然连续.
    Dr16Input wrap{{500}};
    CHECK(wrap.accept(base_sample(), {0xfffffff0u}));
    auto after = base_sample();
    after.keys = 0x0000;
    CHECK(wrap.accept(after, {4}));
    CHECK(wrap.gaps() == 0);
    CHECK(wrap.edges().keys_released == 0x0005);
    CHECK(wrap.online({4}));
}
} // 测试辅助

int main() {
    snapshot_contracts();
    edge_contracts();
    time_contracts();
    std::puts("dr16 input contracts passed");
    return 0;
}
