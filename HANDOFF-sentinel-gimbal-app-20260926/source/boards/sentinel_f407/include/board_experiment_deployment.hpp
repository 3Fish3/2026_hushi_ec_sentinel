#pragma once
#include "sentinel_experiment.hpp"
#include "sentinel_oscillation_guard.hpp"
// Local, controlled-build provisioning boundary. No UART command can mutate it.
// Replacing this provider requires independently reviewed physical measurements,
// Guardian approval and a nonzero, reboot-distinct board boot identity source.
// Deliberately contains no example mechanical limits and no approved record.
#if defined(EC_TRUSTED_EXPERIMENT_PROVIDER_HEADER)
#include EC_TRUSTED_EXPERIMENT_PROVIDER_HEADER
#else
namespace ec::sentinel::deployment {
inline app::excitation::TrustedDeploymentSafetyConfig experiment_config() noexcept { return {}; }
inline app::excitation::TrustedDeploymentRoot experiment_root() noexcept { return {}; }
// FireYaw Guardian thresholds: default unconfigured never admits; a controlled provider
// header must supply versioned, hash-identified, on-site-approved limits.
inline app::oscillation::GuardianLimits experiment_guardian_limits() noexcept { return {}; }
} // namespace ec::sentinel::deployment
#endif
