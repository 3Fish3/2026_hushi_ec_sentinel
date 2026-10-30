// Offline only: bounded DJI current square excitation and persistent stop latch contracts.
#include "sentinel_dji_identification.hpp"
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <limits>

namespace {
using namespace ec::sentinel::app;
void check(bool value, int line) {
    if (!value) { std::fprintf(stderr, "dji identification line %d\n", line); std::_Exit(3); }
}
#define CHECK(...) check(static_cast<bool>((__VA_ARGS__)), __LINE__)
DjiIdentificationPolicy policy() {
    return {-1.4f, 1.4f, 1.5f, 2.5f, 1.0f, 0.05f, 10000, 2000000, 10000, 3000};
}
DjiIdentificationRecipe recipe() { return {1, 0.6f, 10000, 10000, 2, 10000}; }
DjiIdentificationEvidence evidence() {
    DjiIdentificationEvidence e{};
    e.maintenance = e.executing = e.enabled = e.feedback_valid = e.isolated = true;
    e.grant = {1, ec::CommandSource::maintenance};
    e.config_epoch = 1; e.reference_epoch = 1;
    e.target_rad = 0.1f; e.position_rad = 0.1f; e.velocity_rad_s = 0.0f;
    e.current_limit_a = 2.5f; e.loop_current_a = 0.1f;
    return e;
}
void waveform() {
    DjiIdentification test;
    const auto e = evidence();
    CHECK(test.start(policy(), recipe(), e, {1000}));
    CHECK(test.tick(e, {1000}) == 0.0f);
    for (std::uint32_t t = 3000; t < 71000; t += 2000) {
        const auto value = test.tick(e, {t});
        const auto elapsed = t-1000;
        const float expected = elapsed < 10000 || elapsed >= 50000 ? 0.0f :
            ((elapsed-10000)/10000 % 2 == 0 ? 0.6f : -0.6f);
        CHECK(std::fabs(value-expected) < 1e-6f);
    }
    CHECK(test.state() == DjiIdentificationState::complete);
    CHECK(!test.active() && !test.latched());
    CHECK(!test.start(policy(), recipe(), e, {75000})); // same run cannot restart
}
void amplitude_and_current_envelope() {
    DjiIdentification test;
    auto r = recipe();
    r.amplitude_a = 2.0f; CHECK(!test.start(policy(), r, evidence(), {0})); // over max_amplitude_a
    r = recipe(); r.amplitude_a = 0.0f; CHECK(!test.start(policy(), r, evidence(), {0}));
    r = recipe(); r.cycles = 0; CHECK(!test.start(policy(), r, evidence(), {0}));
    r = recipe(); r.amplitude_a = std::numeric_limits<float>::quiet_NaN();
    CHECK(!test.start(policy(), r, evidence(), {0}));
    // 电流上限核对发生在 tick: 启动只看位置/速度/反馈 envelope, 叠加电流超限在推进时锁存.
}
void fault_latch_current() {
    DjiIdentification test;
    auto e = evidence();
    CHECK(test.start(policy(), recipe(), e, {1000}));
    CHECK(test.tick(e, {1000}) == 0.0f);       // baseline, requested 0
    auto hot = e; hot.loop_current_a = 2.2f;
    // 推进到方波段 (elapsed>=10000): 环路 2.2A + 激励 0.6A = 2.8A > 2.5A => current 故障.
    float last = 0.0f;
    for (std::uint32_t t = 3000; t <= 13000; t += 2000) { last = test.tick(hot, {t}); }
    CHECK(last == 0.0f);
    CHECK(test.state() == DjiIdentificationState::fault);
    CHECK(test.fault() == DjiIdentificationFault::current);
    CHECK(!test.reset(true, false)); // 需要 disabled confirmed
    CHECK(test.reset(true, true));
    CHECK(!test.latched());
}
void fault_latch_fields() {
    const int n = 9;
    for (int change = 0; change < n; ++change) {
        DjiIdentification test;
        auto e = evidence();
        CHECK(test.start(policy(), recipe(), e, {1000}));
        static_cast<void>(test.tick(e, {1000}));
        switch (change) {
        case 0: e.enabled = false; break;
        case 1: e.maintenance = false; break;
        case 2: e.feedback_age_us = 11000; break;
        case 3: e.position_rad = 1.5f; break;   // 越 max_rad
        case 4: e.velocity_rad_s = 1.1f; break; // 越 max_velocity
        case 5: e.isolated = false; break;
        case 6: e.grant = {2, ec::CommandSource::maintenance}; break;
        case 7: e.target_rad = 0.1f + 0.06f; break; // 越 tracking
        case 8: e.current_limit_a = 0.0f; break;
        default: break;
        }
        const auto v = test.tick(e, {3000});
        CHECK(v == 0.0f);
        CHECK(test.latched());
    }
}
void timing_step_guard() {
    DjiIdentification test;
    auto e = evidence();
    CHECK(test.start(policy(), recipe(), e, {1000}));
    static_cast<void>(test.tick(e, {1000}));
    static_cast<void>(test.tick(e, {3000}));
    const auto v = test.tick(e, {3000 + 4000}); // gap > max_step_us 3000
    CHECK(v == 0.0f);
    CHECK(test.fault() == DjiIdentificationFault::timing);
}
} // namespace
int main() {
    waveform();
    amplitude_and_current_envelope();
    fault_latch_current();
    fault_latch_fields();
    timing_step_guard();
    std::printf("dji identification contracts ok\n");
    return 0;
}
