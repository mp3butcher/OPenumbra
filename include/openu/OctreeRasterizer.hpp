#pragma once

#include <cstdint>
#include <vector>
#include "openu/Octree.hpp"

class MaskedOcclusionCulling;

namespace openu {

struct ViewPoint {
    // Row-major matrix multiplying [x y z 1]. The resulting clip-space w must be positive.
    float worldToClip[16]{};
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

    // Occupied leaf AABBs are rendered as conservative occluders. Empty leaves
    // are then tested against the resulting hierarchical depth buffer.
    RasterizedOctree rasterize(const OcclusionOctree& tree, const ViewPoint& view);

private:
    MaskedOcclusionCulling* moc_;
    unsigned width_;
    unsigned height_;
};

} // namespace openu
