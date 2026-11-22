# 3D-BBS (vendored subset)

The hierarchical voxel map and the discrete transformation of
[3D-BBS](https://github.com/KOKIAOKI/3d_bbs) (Koki Aoki, Kenji Koide et al., "3D-BBS:
Global Localization for 3D Point Cloud Scan Matching Using Branch-and-Bound
Algorithm", ICRA 2024), used by the optional `bbs3d` engine of the G2 global search
(`docs/global_localization.md`).

- Licence: **MIT**, Copyright (c) 2023 KokiAoki, in [LICENSE](LICENSE). Keep it with
  these files.
- Upstream revision: `41529a34a2fb9618b5ff560fb3c2363f1615666d` (2025-10-30).
- Files, copied unmodified:

  | here | upstream |
  |---|---|
  | `include/cpu_bbs3d/voxelmaps.hpp` | `bbs3d/include/cpu_bbs3d/voxelmaps.hpp` |
  | `include/discrete_transformation/discrete_transformation.hpp` | `bbs3d/include/discrete_transformation/discrete_transformation.hpp` |
  | `src/voxelmaps.cpp` | `bbs3d/src/cpu_bbs3d/voxelmaps.cpp` |

  Not copied: the library's `BBS3D` class (`bbs3d.cpp`, which returns a single
  answer), the voxel map file I/O (`voxelmaps_io.cpp`, the only user of
  Boost.Filesystem), the GPU code, the PCD loaders and the test programs.
- Dependencies of these files: Eigen and the Boost hash headers, both already
  needed by PCL. OpenMP is not needed here.

The top-k search built on them is in
`include/lidar_localization/bbs3d_topk_search.hpp`; its angular split, initial set and
voxel scoring follow upstream `bbs3d.cpp` and are under the same MIT licence (see the
header comment there).
