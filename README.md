# OPenumbra Octree

This repository contains a compact C++17 octree intended for triangle occupancy and PVS-oriented occlusion tests.

The key design point is that every child stores its local index relative to its parent. That index is interpreted as a 3-bit Morton code:

- bit 0 = X (left/right)
- bit 1 = Y (bottom/top)
- bit 2 = Z (near/far)

This matters because the neighbor lookup can climb through parent nodes and switch the corresponding child bit to find the adjacent occupied cell without a full scan of all leaves.

## Build

```bash
cmake -S . -B build
cmake --build build
./build/openumbra_octree_demo
```

## Included concepts

- Vec3 / Triangle / AABB
- `OctreeNode` with `localIndex`, `morton`, and `parent`
- recursive subdivision of triangles that overlap a cell
- `getNeighbor()` using the parent chain and local Morton codes
- simple occupied-cell query / PVS-style occupancy inspection
