#pragma once
#include "sentinel_experiment_runtime.hpp"
// Independent controlled-build runtime source; static deployment data is NOT live evidence.
// No default provider. Implementations must read actual local guardian + lease state,
// independently bind boot/axis/epochs, and return unavailable on uncertainty.
// Callback/context lifetime: entire board lifetime. Bounded, nonblocking, no UART/PC work.
#if defined(EC_TRUSTED_EXPERIMENT_RUNTIME_HEADER)
#include EC_TRUSTED_EXPERIMENT_RUNTIME_HEADER
#else
namespace ec::sentinel::deployment {
inline app::excitation::RuntimeBinding experiment_runtime_binding() noexcept { return {}; }
} // namespace ec::sentinel::deployment
#endif
