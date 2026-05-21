#ifndef LIDAR_LOCALIZATION_REGISTRATION_BACKEND_POLICY_HPP_
#define LIDAR_LOCALIZATION_REGISTRATION_BACKEND_POLICY_HPP_

#include <string>

namespace lidar_localization
{

enum class RegistrationBackend
{
  kInvalid = 0,
  kPclGicp,
  kPclNdt,
  kNdtOmp,
  kGicpOmp,
  kSmallGicp,
  kSmallVgicp,
};

inline RegistrationBackend parseRegistrationBackend(const std::string & registration_method)
{
  if (registration_method == "GICP") {
    return RegistrationBackend::kPclGicp;
  }
  if (registration_method == "NDT") {
    return RegistrationBackend::kPclNdt;
  }
  if (registration_method == "NDT_OMP") {
    return RegistrationBackend::kNdtOmp;
  }
  if (registration_method == "GICP_OMP") {
    return RegistrationBackend::kGicpOmp;
  }
  if (registration_method == "SMALL_GICP") {
    return RegistrationBackend::kSmallGicp;
  }
  if (registration_method == "SMALL_VGICP") {
    return RegistrationBackend::kSmallVgicp;
  }
  return RegistrationBackend::kInvalid;
}

inline bool isValidRegistrationBackend(RegistrationBackend backend)
{
  return backend != RegistrationBackend::kInvalid;
}

inline bool usesFilteredTarget(RegistrationBackend backend)
{
  return backend == RegistrationBackend::kPclGicp ||
         backend == RegistrationBackend::kGicpOmp ||
         backend == RegistrationBackend::kSmallGicp ||
         backend == RegistrationBackend::kSmallVgicp;
}

inline bool usesFilteredTarget(const std::string & registration_method)
{
  return usesFilteredTarget(parseRegistrationBackend(registration_method));
}

inline bool supportsNdtInitializer(RegistrationBackend backend)
{
  return usesFilteredTarget(backend);
}

inline bool supportsNdtInitializer(const std::string & registration_method)
{
  return supportsNdtInitializer(parseRegistrationBackend(registration_method));
}

inline bool isSmallGicpBackend(RegistrationBackend backend)
{
  return backend == RegistrationBackend::kSmallGicp ||
         backend == RegistrationBackend::kSmallVgicp;
}

inline const char * smallGicpRegistrationType(RegistrationBackend backend)
{
  return backend == RegistrationBackend::kSmallVgicp ? "VGICP" : "GICP";
}

inline int resolveRegistrationThreadCount(int requested_threads, int fallback_threads)
{
  if (requested_threads > 0) {
    return requested_threads;
  }
  return fallback_threads > 0 ? fallback_threads : 1;
}

}  // namespace lidar_localization

#endif  // LIDAR_LOCALIZATION_REGISTRATION_BACKEND_POLICY_HPP_
