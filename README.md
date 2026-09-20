# OPenumbra PVS octree

This branch contains an adaptive C++17 octree for triangle occupancy and PVS/occlusion-culling preprocessing.

Each child has an explicit local Morton code:

```text
bit 0: X: 0 = left,  1 = right
bit 1: Y: 0 = bottom, 1 = top
bit 2: Z: 0 = near,   1 = far
```

Neighbor lookup does not scan leaves or compare bounds. It climbs the parent chain until the requested face can be crossed, switches the corresponding Morton bit, and descends through the saved path. `getNeighbor()` returns one same-depth/coarser neighbor; `getNeighbors()` additionally expands a finer adaptive neighbor into terminal cells touching the face.

```bash
cmake -S . -B build
cmake --build build
ctest --test-dir build --output-on-failure
```
