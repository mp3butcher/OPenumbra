# OPenumbra PVS octree

OPenumbra contains an adaptive C++17 octree for triangle occupancy and PVS/occlusion-culling preprocessing.

## Masked software rasterizer

The project integrates Intel's archived [MaskedOcclusionCulling](https://github.com/GameTechDev/MaskedOcclusionCulling) through CMake `FetchContent`, pinned to a known revision. It renders occupied octree leaf AABBs into the hierarchical depth buffer and uses `TestRect()` to classify empty leaves from a supplied point of view. The result also includes a debug per-pixel depth image.

`ViewPoint::worldToClip` is a row-major matrix multiplying `[x y z 1]`; clip-space `w` must be positive in front of the camera. MOC requires a width divisible by 8 and a height divisible by 4.

```cpp
ViewPoint pov{};
// Fill pov.worldToClip with the camera's row-major world-to-clip matrix.
OctreeRasterizer rasterizer(1920, 1080); // 1920 and 1080 satisfy MOC alignment rules
RasterizedOctree frame = rasterizer.rasterize(tree, pov);
// frame.visibleEmptyLeaves contains empty cells that may be visible from pov.
```

The classification is conservative: empty cells are queried using projected AABB rectangles, so a visible result may include false positives. The upstream rasterizer is archived under Apache 2.0; see its repository for the license and platform notes.

```bash
cmake -S . -B build
cmake --build build
ctest --test-dir build --output-on-failure
```
