#pragma once

#include <vector>
#include "openu/Octree.hpp"

class MaskedOcclusionCulling;

namespace openu {
struct Plan {
    Vec3 normal;
    float d;

    float distance(const Vec3& p) const {
        return p.x * normal.x + p.y * normal.y + p.z * normal.z + d;
    }
};

struct Frustum {
    std::array<Plan, 6> planes;
};
struct ClipVertex { float x, y, z, w; };
struct ViewPoint {
    Mat4 view = Mat4::identity();
    Mat4 projection = Mat4::identity();

    Mat4 worldToClip() const { return projection * view; }

    Vec3 cameraPosition() const {
        const float tx = view.m[3];
        const float ty = view.m[7];
        const float tz = view.m[11];
    
        return {
            -(view.m[0] * tx + view.m[4] * ty + view.m[8]  * tz),
            -(view.m[1] * tx + view.m[5] * ty + view.m[9]  * tz),
            -(view.m[2] * tx + view.m[6] * ty + view.m[10] * tz)
        };
    }
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

    // Compatibility path. Prefer the CompiledOctree overload for repeated POVs.
    std::vector<const OctreeNode*> visibleCells(const OcclusionOctree& tree, const ViewPoint& view) const;
    RasterizedOctree rasterize(OcclusionOctree& tree, const ViewPoint& view);

    // Traversal over precomputed integer neighbor ranges. The CompiledOctree
    // must outlive the returned pointers and must not be rebuilt after use.
    std::vector<const CompiledNode*> visibleCells(const CompiledOctree& tree, const ViewPoint& view) const;
    RasterizedOctree rasterize(const CompiledOctree& tree, const ViewPoint& view);

    //test a box against computed depth buffer
    bool testBox(std::array<ClipVertex, 8> &boxCorner);
    RasterizedOctree renderFrontToBack(const CompiledOctree& tree,  const ViewPoint& view) ;
    void renderNodeRecursive(const CompiledOctree& tree, NodeId id, const Vec3& cameraPos, const Frustum& frustum, const Mat4& clip) ;

    inline MaskedOcclusionCulling& maskedOcclusionCulling() { return *moc_; }
private:
    MaskedOcclusionCulling* moc_;
    RasterizedOctree rasterizeNodes(const std::vector<const OctreeNode*>& nodes, const ViewPoint& view);

    unsigned width_;
    unsigned height_;
    mutable std::vector<std::uint32_t> visitStamp_;
    mutable std::uint32_t traversalStamp_ = 0;
    mutable std::vector<NodeId> frontier_;
};

} // namespace openu
