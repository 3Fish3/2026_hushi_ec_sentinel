// M4 运动学契约: 参数化四全向轮的正解/逆解、线性叠加、正逆一致与退化布局检查.
// 合成几何只证明公式本身, 不填写任何实车几何参数.
#include "ec/motion/omni_kinematics.hpp"
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <numbers>

namespace {
void check(bool condition, const char* expression, int line) {
    if (!condition) {
        std::fprintf(stderr, "line %d: %s\n", line, expression);
        std::_Exit(3); // 非零退出且不触发 CRT 交互弹窗, ctest 要求非交互
    }
}
#define CHECK(...) check(static_cast<bool>((__VA_ARGS__)), #__VA_ARGS__, __LINE__)

using ec::motion::OmniDrive;
using ec::motion::OmniWheel;
using ec::motion::PlanarTwist;

constexpr float deg = static_cast<float>(std::numbers::pi / 180.0);
constexpr float half_pi = static_cast<float>(std::numbers::pi / 2.0);

// 合成布局: 轮心在半长 a / 半宽 b 的矩形四角, 滚动方向取绕旋转中心的切向 d = (-y, x)/|r|,
// 这只是测试用几何, 不对应任何实车, 也不包含固定角度常量.
OmniDrive tangential_layout(float a, float b, float radius) {
    OmniDrive drive{};
    const float xs[4] = {a, -a, -a, a};
    const float ys[4] = {b, b, -b, -b};
    for (std::size_t i = 0; i < 4; ++i) {
        const float angle = std::atan2(xs[i], -ys[i]);
        drive.wheels[i] = OmniWheel{xs[i], ys[i], angle, radius, 1.0f};
    }
    return drive;
}

bool near(float a, float b, float tolerance = 1e-4f) { return std::fabs(a - b) <= tolerance; }

void validation_contracts() {
    const auto drive = tangential_layout(0.4f, 0.3f, 0.06f);
    CHECK(ec::motion::omni_valid(drive));
    CHECK(ec::motion::omni_well_posed(drive));
    // 半径必须为正, 增益不能为 0.
    auto broken = drive;
    broken.wheels[1].radius_m = 0.0f;
    CHECK(!ec::motion::omni_valid(broken));
    CHECK(!ec::motion::omni_well_posed(broken));
    broken = drive;
    broken.wheels[2].gain = 0.0f;
    CHECK(!ec::motion::omni_valid(broken));
    broken = drive;
    broken.wheels[0].roll_dir_rad = std::numeric_limits<float>::quiet_NaN();
    CHECK(!ec::motion::omni_valid(broken));
    // 非有限 twist 不产生轮速.
    CHECK(!ec::motion::omni_wheel_speeds(drive,
        PlanarTwist{std::numeric_limits<float>::infinity(), 0.0f, 0.0f}));
    CHECK(!ec::motion::omni_twist(drive,
        {std::numeric_limits<float>::quiet_NaN(), 0.0f, 0.0f, 0.0f}));
}

void single_axis_contracts() {
    const float radius = 0.05f;
    const float a = 0.4f;
    const float b = 0.3f;
    const auto drive = tangential_layout(a, b, radius);

    // 纯 vx: 每个轮的接触点速度是 (vx, 0), 轮角速度 = (vx,0)·d / r.
    const auto vx = ec::motion::omni_wheel_speeds(drive, PlanarTwist{1.0f, 0.0f, 0.0f});
    CHECK(vx);
    for (std::size_t i = 0; i < 4; ++i) {
        const float expected = std::cos(drive.wheels[i].roll_dir_rad) / radius;
        CHECK(near((*vx)[i], expected));
    }
    // 纯 vy.
    const auto vy = ec::motion::omni_wheel_speeds(drive, PlanarTwist{0.0f, 1.0f, 0.0f});
    CHECK(vy);
    for (std::size_t i = 0; i < 4; ++i) {
        CHECK(near((*vy)[i], std::sin(drive.wheels[i].roll_dir_rad) / radius));
    }
    // 纯自转: 接触点速度 = omega x r, 与轮心位置和滚动方向都有关.
    const float omega = 2.0f;
    const auto wz = ec::motion::omni_wheel_speeds(drive, PlanarTwist{0.0f, 0.0f, omega});
    CHECK(wz);
    for (std::size_t i = 0; i < 4; ++i) {
        const auto& wheel = drive.wheels[i];
        const float tangential =
            (-omega * wheel.y_m) * std::cos(wheel.roll_dir_rad) + (omega * wheel.x_m) * std::sin(wheel.roll_dir_rad);
        CHECK(near((*wz)[i], tangential / radius));
    }
    // 切向排布下, 纯自转对每个轮都是同号同量级: |w| = |omega| * hypot(a, b) / r.
    const float expected_spin = omega * std::hypot(a, b) / radius;
    CHECK(near((*wz)[0], expected_spin));
    CHECK(near((*wz)[1], expected_spin));

    // 线性叠加: 三个单位基向量的轮速之和等于合成 twist 的轮速.
    const PlanarTwist twist{0.7f, -0.2f, 1.3f};
    const auto composed = ec::motion::omni_wheel_speeds(drive, twist);
    const auto wz_unit = ec::motion::omni_wheel_speeds(drive, PlanarTwist{0.0f, 0.0f, 1.0f});
    CHECK(composed && wz_unit);
    for (std::size_t i = 0; i < 4; ++i) {
        CHECK(near((*composed)[i],
            (*vx)[i] * twist.vx_mps + (*vy)[i] * twist.vy_mps + (*wz_unit)[i] * twist.wz_rad_s));
    }
    // 零 twist 输出零轮速.
    const auto zero = ec::motion::omni_wheel_speeds(drive, PlanarTwist{});
    CHECK(zero);
    for (auto speed : *zero) { CHECK(speed == 0.0f); }
}

void inverse_contracts() {
    const auto drive = tangential_layout(0.4f, 0.3f, 0.05f);
    // 正逆一致: 对一组可达轮速, 逆解应还原 twist.
    const PlanarTwist twists[5] = {{0.5f, 0.0f, 0.0f}, {0.0f, -0.8f, 0.0f}, {0.0f, 0.0f, 1.5f},
        {0.3f, 0.6f, -0.7f}, {-1.2f, 0.4f, 2.0f}};
    for (const auto& twist : twists) {
        const auto speeds = ec::motion::omni_wheel_speeds(drive, twist);
        CHECK(speeds);
        const auto back = ec::motion::omni_twist(drive, *speeds);
        CHECK(back);
        CHECK(near(back->vx_mps, twist.vx_mps, 1e-4f));
        CHECK(near(back->vy_mps, twist.vy_mps, 1e-4f));
        CHECK(near(back->wz_rad_s, twist.wz_rad_s, 1e-4f));
    }
    // 峰值轮速: 切向布局下纯自转四轮等速, 峰值就是该等速值.
    const float spin_peak = ec::motion::omni_peak_wheel_speed(drive, PlanarTwist{0.0f, 0.0f, 1.0f});
    CHECK(near(spin_peak, std::hypot(0.4f, 0.3f) / 0.05f, 1e-3f));
    CHECK(ec::motion::omni_peak_wheel_speed(drive, PlanarTwist{}) == 0.0f);
}

void degeneracy_contracts() {
    // 四个轮滚动方向相同: 只能观测一个方向, 退化为不可解.
    OmniDrive same{};
    for (std::size_t i = 0; i < 4; ++i) {
        same.wheels[i] = OmniWheel{static_cast<float>(i) * 0.1f, 0.0f, 0.0f, 0.05f, 1.0f};
    }
    CHECK(ec::motion::omni_valid(same));
    CHECK(!ec::motion::omni_well_posed(same));
    CHECK(!ec::motion::omni_twist(same, {1.0f, 1.0f, 1.0f, 1.0f}));
    // 四轮都过旋转中心且方向共线: 无法观测自转.
    OmniDrive collinear{};
    for (std::size_t i = 0; i < 4; ++i) {
        collinear.wheels[i] = OmniWheel{0.0f, 0.0f, 0.0f, 0.05f, 1.0f};
    }
    CHECK(!ec::motion::omni_well_posed(collinear));
    // 三轮非退化布局在四轮公式下仍可解: 第四个轮只是重复约束(方向相同, 位置相同).
    OmniDrive three{};
    three.wheels[0] = OmniWheel{0.0f, 0.3f, 0.0f, 0.05f, 1.0f};
    three.wheels[1] = OmniWheel{0.3f, 0.0f, half_pi, 0.05f, 1.0f};
    three.wheels[2] = OmniWheel{-0.3f, -0.3f, 135.0f * deg, 0.05f, 1.0f};
    three.wheels[3] = three.wheels[2];
    CHECK(ec::motion::omni_well_posed(three));
    const PlanarTwist twist{0.2f, -0.4f, 0.5f};
    const auto speeds = ec::motion::omni_wheel_speeds(three, twist);
    CHECK(speeds);
    const auto back = ec::motion::omni_twist(three, *speeds);
    CHECK(back);
    CHECK(near(back->vx_mps, twist.vx_mps, 1e-4f));
    CHECK(near(back->vy_mps, twist.vy_mps, 1e-4f));
    CHECK(near(back->wz_rad_s, twist.wz_rad_s, 1e-4f));
}

void gain_contracts() {
    // 传动方向取反: 该轮输出反号, 逆解仍应还原 twist.
    const auto base = tangential_layout(0.4f, 0.3f, 0.05f);
    const PlanarTwist twist{0.5f, 0.2f, -0.9f};
    const auto base_speeds = ec::motion::omni_wheel_speeds(base, twist);
    CHECK(base_speeds);
    auto flipped = base;
    flipped.wheels[1].gain = -1.0f;
    flipped.wheels[3].gain = -1.0f;
    const auto flipped_speeds = ec::motion::omni_wheel_speeds(flipped, twist);
    CHECK(flipped_speeds);
    CHECK(near((*flipped_speeds)[1], -(*base_speeds)[1], 1e-5f));
    CHECK(near((*flipped_speeds)[3], -(*base_speeds)[3], 1e-5f));
    CHECK(near((*flipped_speeds)[0], (*base_speeds)[0], 1e-5f));
    const auto back = ec::motion::omni_twist(flipped, *flipped_speeds);
    CHECK(back);
    CHECK(near(back->vx_mps, twist.vx_mps) && near(back->vy_mps, twist.vy_mps) &&
        near(back->wz_rad_s, twist.wz_rad_s));
    // 半径是线性尺度: 半径翻倍则轮速减半.
    const auto wider = tangential_layout(0.4f, 0.3f, 0.1f);
    const auto wider_speeds = ec::motion::omni_wheel_speeds(wider, twist);
    CHECK(wider_speeds);
    for (std::size_t i = 0; i < 4; ++i) {
        CHECK(near((*wider_speeds)[i], (*base_speeds)[i] * 0.5f, 1e-5f));
    }
}
} // 测试辅助

int main() {
    validation_contracts();
    single_axis_contracts();
    inverse_contracts();
    degeneracy_contracts();
    gain_contracts();
    std::puts("omni kinematics contracts passed");
    return 0;
}
