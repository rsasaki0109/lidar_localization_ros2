#ifndef LIDAR_LOCALIZATION_HEAP_RELEASE_HPP_
#define LIDAR_LOCALIZATION_HEAP_RELEASE_HPP_

#if defined(__GLIBC__)
#include <malloc.h>
#endif

namespace lidar_localization
{

// glibc keeps freed memory in its arenas and returns only the top of each heap to the
// OS. A registration target (crop, search tree, NDT voxel grid) is freed in the middle
// of the heap when the next one replaces it, so on the looped Boreas fixture the lowest
// resident memory of each lap rose by 84 MB an hour. Trimming after each target build
// returns those pages: the lowest memory stays flat. Returns whether memory was released;
// false where the allocator is not glibc.
inline bool releaseFreedHeapMemory()
{
#if defined(__GLIBC__)
  return malloc_trim(0) != 0;
#else
  return false;
#endif
}

}  // namespace lidar_localization

#endif  // LIDAR_LOCALIZATION_HEAP_RELEASE_HPP_
