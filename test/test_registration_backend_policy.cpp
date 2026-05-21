#include "lidar_localization/registration_backend_policy.hpp"

#include <cassert>
#include <string>

namespace ll = lidar_localization;

void test_parse_registration_backend()
{
  assert(ll::parseRegistrationBackend("GICP") == ll::RegistrationBackend::kPclGicp);
  assert(ll::parseRegistrationBackend("NDT") == ll::RegistrationBackend::kPclNdt);
  assert(ll::parseRegistrationBackend("NDT_OMP") == ll::RegistrationBackend::kNdtOmp);
  assert(ll::parseRegistrationBackend("GICP_OMP") == ll::RegistrationBackend::kGicpOmp);
  assert(ll::parseRegistrationBackend("SMALL_GICP") == ll::RegistrationBackend::kSmallGicp);
  assert(ll::parseRegistrationBackend("SMALL_VGICP") == ll::RegistrationBackend::kSmallVgicp);
  assert(ll::parseRegistrationBackend("UNKNOWN") == ll::RegistrationBackend::kInvalid);
}

void test_filtered_target_and_initializer_classification()
{
  assert(ll::usesFilteredTarget(ll::RegistrationBackend::kPclGicp));
  assert(ll::usesFilteredTarget(ll::RegistrationBackend::kGicpOmp));
  assert(ll::usesFilteredTarget(ll::RegistrationBackend::kSmallGicp));
  assert(ll::usesFilteredTarget(ll::RegistrationBackend::kSmallVgicp));
  assert(!ll::usesFilteredTarget(ll::RegistrationBackend::kPclNdt));
  assert(!ll::usesFilteredTarget(ll::RegistrationBackend::kNdtOmp));
  assert(!ll::usesFilteredTarget(ll::RegistrationBackend::kInvalid));

  assert(ll::supportsNdtInitializer("GICP"));
  assert(ll::supportsNdtInitializer("SMALL_VGICP"));
  assert(!ll::supportsNdtInitializer("NDT"));
  assert(!ll::supportsNdtInitializer("UNKNOWN"));
}

void test_small_gicp_classification()
{
  assert(ll::isSmallGicpBackend(ll::RegistrationBackend::kSmallGicp));
  assert(ll::isSmallGicpBackend(ll::RegistrationBackend::kSmallVgicp));
  assert(!ll::isSmallGicpBackend(ll::RegistrationBackend::kGicpOmp));
  assert(
    std::string(ll::smallGicpRegistrationType(ll::RegistrationBackend::kSmallGicp)) == "GICP");
  assert(
    std::string(ll::smallGicpRegistrationType(ll::RegistrationBackend::kSmallVgicp)) == "VGICP");
}

void test_thread_count_resolution()
{
  assert(ll::resolveRegistrationThreadCount(4, 8) == 4);
  assert(ll::resolveRegistrationThreadCount(0, 8) == 8);
  assert(ll::resolveRegistrationThreadCount(-1, 8) == 8);
  assert(ll::resolveRegistrationThreadCount(0, 0) == 1);
  assert(ll::resolveRegistrationThreadCount(0, -2) == 1);
}

int main()
{
  test_parse_registration_backend();
  test_filtered_target_and_initializer_classification();
  test_small_gicp_classification();
  test_thread_count_resolution();
  return 0;
}
