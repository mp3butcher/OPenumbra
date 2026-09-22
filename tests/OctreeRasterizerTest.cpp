#include <cassert>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <unordered_set>
#include <vector>

#include "openu/Octree.hpp"
#include "openu/OctreeRasterizer.hpp"

using namespace openu;

namespace {

ViewPoint makeViewFromProjView(const float legacy[16],const float viewmatrix[16]) {
    ViewPoint view{};
    view.view = Mat4::identity();
    view.projection.m[0] = legacy[0]; view.projection.m[1] = legacy[1]; view.projection.m[2] = legacy[2]; view.projection.m[3] = legacy[3];
    view.projection.m[4] = legacy[4]; view.projection.m[5] = legacy[5]; view.projection.m[6] = legacy[6]; view.projection.m[7] = legacy[7];
    view.projection.m[8] = legacy[8]; view.projection.m[9] = legacy[9]; view.projection.m[10] = legacy[10]; view.projection.m[11] = legacy[11];
    view.projection.m[12] = legacy[12]; view.projection.m[13] = legacy[13]; view.projection.m[14] = legacy[14]; view.projection.m[15] = legacy[15];


    view.view.m[0] = viewmatrix[0]; view.view.m[1] = viewmatrix[1]; view.view.m[2] = viewmatrix[2]; view.view.m[3] = viewmatrix[3];
    view.view.m[4] = viewmatrix[4]; view.view.m[5] = viewmatrix[5]; view.view.m[6] = viewmatrix[6]; view.view.m[7] = viewmatrix[7];
    view.view.m[8] = viewmatrix[8]; view.view.m[9] = viewmatrix[9]; view.view.m[10] = viewmatrix[10]; view.view.m[11] = viewmatrix[11];
    view.view.m[12] = viewmatrix[12]; view.view.m[13] = viewmatrix[13]; view.view.m[14] = viewmatrix[14]; view.view.m[15] = viewmatrix[15];
    return view;
}

void testResolutionValidation() {
    bool rejected = false;
    try {
        OctreeRasterizer invalid(255, 256);
    } catch (const std::invalid_argument&) {
        rejected = true;
    }
    assert(rejected);

    rejected = false;
    try {
        OctreeRasterizer invalid(256, 255);
    } catch (const std::invalid_argument&) {
        rejected = true;
    }
    assert(rejected);
}

void testRasterizesAndClassifiesLeaves() {
    OcclusionOctree tree({{-1.0f, -1.0f, 1.0f}, {1.0f, 1.0f, 9.0f}}, 2, 1);
    tree.insert(Triangle{{-0.8f, -0.8f, 1.2f}, {0.8f, -0.8f, 1.2f}, {0.0f, 0.8f, 1.2f}});
    tree.insert(Triangle{{-0.7f, -0.7f, 1.3f}, {0.7f, -0.7f, 1.3f}, {0.0f, 0.7f, 1.3f}});

    const auto leaves = tree.leaves();
    assert(!leaves.empty());
    assert(tree.occupiedLeafCount() > 0);

    const float projection[16] = {
        2.98564, 0, 0, 0,
        0, -3.73205, 0, 0,
        0, 0, 9.52472e-05, 0.0415094,
        0, 0, -1, 0
    };
    const float viewmat[16] = {
        1, 0, 0, 0.080155,
        0, 0, 1, 0.0656662,
        0, -1, 0, -145.07,
        0, 0, 0, 1
    };
    ViewPoint view = makeViewFromProjView(projection,viewmat);

    OctreeRasterizer rasterizer(256, 256);
    const std::vector<const OctreeNode*> visible = rasterizer.visibleCells(tree, view);
    assert(!visible.empty());

    const RasterizedOctree result = rasterizer.rasterize(tree, view);
    assert(result.width == 256);
    assert(result.height == 256);
    assert(result.depth.size() == 256u * 256u);
    assert(!result.visibleEmptyLeaves.empty() || !result.occludedEmptyLeaves.empty());

    std::unordered_set<const OctreeNode*> allLeaves(leaves.begin(), leaves.end());
    std::unordered_set<const OctreeNode*> classified;
    for (const OctreeNode* leaf : result.visibleEmptyLeaves) {
        assert(allLeaves.count(leaf) == 1);
        assert(!leaf->occupied);
        assert(classified.insert(leaf).second);
    }
    for (const OctreeNode* leaf : result.occludedEmptyLeaves) {
        assert(allLeaves.count(leaf) == 1);
        assert(!leaf->occupied);
        assert(classified.insert(leaf).second);
    }
}

} // namespace

int main() {
    testResolutionValidation();
    testRasterizesAndClassifiesLeaves();
    std::cout << "OctreeRasterizer tests passed\n";
    return 0;
}
