#include "glim_prior_map_localizer/ply_loader.hpp"

#include <cassert>
#include <cstdio>
#include <fstream>
#include <string>

#include <unistd.h>

void test_loads_ascii_xyz_with_extra_scalar()
{
  const std::string path =
    "/tmp/glim_prior_map_loader_" + std::to_string(getpid()) + ".ply";
  {
    std::ofstream stream(path);
    stream << "ply\n"
           << "format ascii 1.0\n"
           << "element vertex 2\n"
           << "property uchar intensity\n"
           << "property float z\n"
           << "property float x\n"
           << "property double y\n"
           << "end_header\n"
           << "7 3.0 1.0 2.0\n"
           << "8 6.0 4.0 5.0\n";
  }

  const auto points = glim_prior_map_localizer::loadPlyXyz(path);
  assert(points.size() == 2);
  assert(points[0].isApprox(Eigen::Vector3f(1.0F, 2.0F, 3.0F)));
  assert(points[1].isApprox(Eigen::Vector3f(4.0F, 5.0F, 6.0F)));
  std::remove(path.c_str());
}

int main()
{
  test_loads_ascii_xyz_with_extra_scalar();
  return 0;
}
