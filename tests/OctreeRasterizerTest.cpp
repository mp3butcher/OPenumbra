#include <cassert>
#include <iostream>
#include <stdexcept>
#include <unordered_set>
#include <vector>
#include "openu/Octree.hpp"
#include "openu/OctreeRasterizer.hpp"

using namespace openu;

namespace {
ViewPoint testView() {
    ViewPoint v{};
    v.projection.m[0] = 1.0f; v.projection.m[5] = 1.0f;
    v.projection.m[10] = 1.0f; v.projection.m[14] = 1.0f;
    return v;
}

void testCompiledTraversal() {
    OcclusionOctree tree({{-1, -1, 1}, {1, 1, 9}}, 2, 1);
    tree.insert(Triangle{{-.8f, -.8f, 1.2f}, {.8f, -.8f, 1.2f}, {0, .8f, 1.2f}});
    tree.insert(Triangle{{-.7f, -.7f, 1.3f}, {.7f, -.7f, 1.3f}, {0, .7f, 1.3f}});

    CompiledOctree compiled = tree.compile();
    assert(compiled.nodeCount() == tree.leaves().size());
    assert(compiled.locate({0, 0, 1.1f}) != InvalidNode);

    OctreeRasterizer rasterizer(256, 256);
    const ViewPoint view = testView();
    const auto first = rasterizer.visibleCells(compiled, view);
    const auto second = rasterizer.visibleCells(compiled, view);
    assert(first.size() == second.size());
    assert(!first.empty());

    const RasterizedOctree result = rasterizer.rasterize(compiled, view);
    assert(result.depth.size() == 256u * 256u);
    std::unordered_set<const OctreeNode*> unique(result.visibleEmptyLeaves.begin(), result.visibleEmptyLeaves.end());
    unique.insert(result.occludedEmptyLeaves.begin(), result.occludedEmptyLeaves.end());
    assert(unique.size() == result.visibleEmptyLeaves.size() + result.occludedEmptyLeaves.size());
}

void testResolutionValidation() {
    bool rejected = false;
    try { OctreeRasterizer invalid(255, 256); } catch (const std::invalid_argument&) { rejected = true; }
    assert(rejected);
}
}

int main() {
    testResolutionValidation();
    testCompiledTraversal();
    std::cout << "Compiled OctreeRasterizer tests passed\n";
    return 0;
}
