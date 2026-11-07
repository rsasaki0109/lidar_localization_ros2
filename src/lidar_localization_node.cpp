#include <lidar_localization/lidar_localization_component.hpp>

#include <rclcpp/executors/multi_threaded_executor.hpp>

#include <cstdlib>
#if defined(__GLIBC__)
#include <malloc.h>
#endif

int main(int argc, char * argv[])
{
#if defined(__GLIBC__)
  // glibc gives each thread that allocates its own arena and keeps what is freed in it.
  // With about 40 threads (DDS, the executor, OpenMP) the node held 1.5-2.2 GB on the
  // looped Boreas fixture (tools/benchmark/soak_replay.py), with 0.3-0.7 GB in use; with
  // two arenas it holds 0.55-0.8 GB, at the same alignment times. Set before any thread
  // starts; MALLOC_ARENA_MAX in the environment still wins.
  if (std::getenv("MALLOC_ARENA_MAX") == nullptr) {
    mallopt(M_ARENA_MAX, 2);
  }
#endif
  rclcpp::init(argc, argv);
  {
    // The default callback group remains mutually-exclusive. IMU preintegration
    // and opt-in timer publication use dedicated guarded groups so neither is
    // starved by long cloud registration callbacks during bag replay.
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 3);
    rclcpp::NodeOptions options;
    std::shared_ptr<PCLLocalization> pcl_l = std::make_shared<PCLLocalization>(options);

    executor.add_node(pcl_l->get_node_base_interface());
    executor.spin();
    executor.remove_node(pcl_l->get_node_base_interface());
    pcl_l.reset();
  }
  rclcpp::shutdown();

  return 0;
}
