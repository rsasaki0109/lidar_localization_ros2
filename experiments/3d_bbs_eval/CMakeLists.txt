# Evaluation-only build of the 3D-BBS binding; not part of the package build.
#   cmake -S experiments/3d_bbs_eval -B <build> -DBBS3D_ROOT=<3d_bbs checkout> \
#         -DBBS3D_LIB=<path to libcpu_bbs3d.so> -DCMAKE_BUILD_TYPE=Release
cmake_minimum_required(VERSION 3.16)
project(bbs3d_eval CXX)
set(CMAKE_CXX_STANDARD 17)
find_package(Eigen3 REQUIRED)
find_package(OpenMP REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter Development.Module)
find_package(pybind11 REQUIRED)
# libcpu_bbs3d uses boost::filesystem (voxelmaps_io.cpp) without linking it.
find_package(Boost REQUIRED COMPONENTS filesystem)
set(BBS3D_ROOT "" CACHE PATH "3d_bbs source checkout")
set(BBS3D_LIB "" CACHE FILEPATH "libcpu_bbs3d.so built from BBS3D_ROOT with -DBUILD_CUDA=OFF")
pybind11_add_module(bbs3d_py bbs3d_pybind.cpp)
target_include_directories(bbs3d_py PRIVATE ${BBS3D_ROOT}/bbs3d/include ${EIGEN3_INCLUDE_DIR})
target_link_libraries(bbs3d_py PRIVATE ${BBS3D_LIB} OpenMP::OpenMP_CXX Boost::filesystem)
set_target_properties(bbs3d_py PROPERTIES BUILD_RPATH_USE_ORIGIN OFF)
