#include <cassert>
#include <iostream>
#include "openu/Octree.hpp"
#include "openu/OctreeRasterizer.hpp"

using namespace openu;

int main() {
    OcclusionOctree tree({{-8, -8, -8}, {8, 8, 8}}, 4, 1);
    tree.insert(Triangle{{-1, -1, 0}, {1, -1, 0}, {0, 1, 0}});
    tree.insert(Triangle{{4, 4, 4}, {5, 4, 4}, {4, 5, 4}});

    auto leaves = tree.leaves();
    assert(!leaves.empty());
    assert(tree.occupiedLeafCount() > 0);

    ViewPoint view{};
    // Identity clip transform is sufficient for the smoke test; real callers
    // should supply their camera's world-to-clip matrix.
    view.view.m[0] = view.view.m[5] = view.view.m[10] = view.view.m[15] = 1.0f;
    view.projection.m[0] = view.projection.m[5] = view.projection.m[10] = view.projection.m[15] = 1.0f;
  
    OctreeRasterizer rasterizer(256, 256);
    const RasterizedOctree frame = rasterizer.rasterize(tree, view);
    assert(frame.depth.size() == 256u * 256u);

    std::cout << "leaves=" << leaves.size() << " occupied=" << tree.occupiedLeafCount()
              << " visible_empty=" << frame.visibleEmptyLeaves.size() << '\n';
    return 0;
}
