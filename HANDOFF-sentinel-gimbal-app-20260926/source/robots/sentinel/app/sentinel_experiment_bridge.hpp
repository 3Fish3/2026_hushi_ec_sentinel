#pragma once
#include "sentinel_app.hpp"
#include "sentinel_execution.hpp"
#include "sentinel_experiment.hpp"
#include <cmath>
namespace ec::sentinel::app {
// 唯一的只读组合点: 请求字节不能创造这里的任何事实.
inline excitation::Guards deployment_experiment_guards(const SentinelApp& app,
    const SentinelExecutor& executor,const excitation::Controller& controller,Domain domain,Tick now) noexcept {
    auto g=executor.experiment_guards(domain,now);
    const auto& d=controller.deployment();
    const auto& snapshot=app.snapshot();
    g.manual_stop=snapshot.permission.manual_stop;
    g.remote_online=snapshot.input.remote_online;
    g.maintenance_authorized=app.authority().maintenance_holds();
    g.operator_approved=g.maintenance_authorized && g.remote_online && app.authority().maintenance_commissioning_selected();
    g.config_epoch=app.config_epoch();g.reference_epoch=app.reference_epoch();
    if(controller.hardware_capable()) {
        g.axis_identity=controller.root().axis_identity;
        g.config_hash=d.config_hash;g.safety_reference_id=d.safety_reference_id;
        if (domain == Domain::fire_yaw) {
            const excitation::RuntimeQuery query{executor.experiment_time(now).us,
                controller.root().boot_id, controller.root().axis_identity, g.param_epoch,
                g.config_epoch, g.reference_epoch, d.guardian_version, excitation::Axis::fire_yaw};
            const auto runtime = executor.experiment_runtime(query);
            // Only fresh, identity-bound evidence from a pinned local source can admit.
            // The query contains expectations, not approval or lease creation commands.
            g.guardian_approved = runtime.ready();
            g.guardian_clear = runtime.ready();
            g.lease_active = runtime.ready() && runtime.evidence.lease_active;
            g.guardian_state = runtime.evidence.guardian_state;
            if (runtime.ready()) { g.guardian_version = runtime.evidence.guardian_version; }
        }
    }
    const auto& geometry=app.joint_geometry(domain);
    g.limits_valid=controller.hardware_capable() && geometry.min_rad && geometry.max_rad &&
        std::isfinite(*geometry.min_rad) && std::isfinite(*geometry.max_rad) &&
        *geometry.min_rad<=d.min_rad && *geometry.max_rad>=d.max_rad;
    g.stop_strategy_verified=controller.hardware_capable() && domain==Domain::fire_yaw && d.stop_policy==excitation::StopPolicy::zero_output;
    // Recorder readiness is independent of command observation; never infer it here.
    for(std::size_t i=0;i<domain_count;++i) {
        if(i==domain_index(domain)) continue;
        if(app.target(static_cast<Domain>(i)) || snapshot.executions[i].execute) g.other_axes_idle=false;
    }
    // Missing provisioning is not simulation and cannot manufacture device evidence.
    // Offline simulations supply their own Guards directly to Controller/Wire.
    return g;
}
} // namespace ec::sentinel::app
