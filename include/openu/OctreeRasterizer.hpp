#pragma once

#include <vector>
#include "openu/Octree.hpp"

class MaskedOcclusionCulling;

namespace openu {

struct ViewPoint {
    Vec3 position{};
    Mat4 view = Mat4::identity();
    Mat4 projection = Mat4::identity();

    // Backward-compatible helper for existing code/tests that still populate a
    // single worldToClip matrix. If the view/projection matrices are identity,
    // this returns the legacy matrix value.
    Mat4 worldToClip() const { return projection * view; }
};

struct RasterizedOctree {
    std::vector<const OctreeNode*> visibleEmptyLeaves;
    std::vector<const OctreeNode*> occludedEmptyLeaves;
    std::vector<float> depth;
    unsigned width = 0;
    unsigned height = 0;
};

class OctreeRasterizer {
public:
    OctreeRasterizer(unsigned width, unsigned height);
    ~OctreeRasterizer();
    OctreeRasterizer(const OctreeRasterizer&) = delete;
    OctreeRasterizer& operator=(const OctreeRasterizer&) = delete;

    // Conservative culling from the camera's cell frontier.
    std::vector<const OctreeNode*> visibleCells(const OcclusionOctree& tree, const ViewPoint& view) const;

    RasterizedOctree rasterize(OcclusionOctree& tree, const ViewPoint& view);

private:
    MaskedOcclusionCulling* moc_;
    unsigned width_;
    unsigned height_;
};

} // namespace openu
