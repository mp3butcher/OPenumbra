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

ViewPoint perspectiveLikeView() {
    ViewPoint view{};
    // A simple row-major projection for this test: x and y remain in clip
    // space, while positive world z becomes clip-space w/depth.
    view.worldToClip[0] = 1.0f;
    view.worldToClip[5] = 1.0f;
    view.worldToClip[15] = 0.0f;
    view.worldToClip[14] = 1.0f;
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
    // The two triangles occupy the near part of the tree. The remaining
    // terminal cells are empty and are classified from the supplied POV.
    OcclusionOctree tree({{-1.0f, -1.0f, 1.0f}, {1.0f, 1.0f, 9.0f}}, 2, 1);
    tree.insert(Triangle{{-0.8f, -0.8f, 1.2f}, {0.8f, -0.8f, 1.2f}, {0.0f, 0.8f, 1.2f}});
    tree.insert(Triangle{{-0.7f, -0.7f, 1.3f}, {0.7f, -0.7f, 1.3f}, {0.0f, 0.7f, 1.3f}});

    const auto leaves = tree.leaves();
    assert(!leaves.empty());
    assert(tree.occupiedLeafCount() > 0);

    OctreeRasterizer rasterizer(256, 256);
    const RasterizedOctree result = rasterizer.rasterize(tree, perspectiveLikeView());

    assert(result.width == 256);
    assert(result.height == 256);
    assert(result.depth.size() == 256u * 256u);
    assert(!result.visibleEmptyLeaves.empty());

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

    // Every reported leaf is empty, and no leaf is reported twice. Some empty
    // cells may legitimately be view-culled by TestRect().
    assert(result.visibleEmptyLeaves.size() + result.occludedEmptyLeaves.size() <=
           leaves.size() - tree.occupiedLeafCount());

    // Running another POV must produce a valid independent result and refresh
    // the rasterizer's depth buffer rather than accumulating the prior frame.
    ViewPoint shifted = perspectiveLikeView();
    shifted.worldToClip[3] = 0.5f;
    const RasterizedOctree shiftedResult = rasterizer.rasterize(tree, shifted);
    assert(shiftedResult.depth.size() == result.depth.size());
}

} // namespace

int main() {
    testResolutionValidation();
    testRasterizesAndClassifiesLeaves();
    std::cout << "OctreeRasterizer tests passed\n";
    return 0;
}
