#include "openu/OctreeRasterizer.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>
#include "MaskedOcclusionCulling.h"

namespace openu {
namespace {
struct ClipVertex { float x, y, z, w; };

ClipVertex transform(const Vec3& p, const ViewPoint& v) {
    const float* m = v.worldToClip;
    return {m[0]*p.x + m[1]*p.y + m[2]*p.z + m[3],
            m[4]*p.x + m[5]*p.y + m[6]*p.z + m[7],
            m[8]*p.x + m[9]*p.y + m[10]*p.z + m[11],
            m[12]*p.x + m[13]*p.y + m[14]*p.z + m[15]};
}

std::array<Vec3, 8> corners(const AABB& b) {
    return {{{b.min.x,b.min.y,b.min.z}, {b.max.x,b.min.y,b.min.z}, {b.min.x,b.max.y,b.min.z}, {b.max.x,b.max.y,b.min.z},
             {b.min.x,b.min.y,b.max.z}, {b.max.x,b.min.y,b.max.z}, {b.min.x,b.max.y,b.max.z}, {b.max.x,b.max.y,b.max.z}}};
}

// Twelve triangles, winding is deliberately ignored (two-sided boxes).
const unsigned boxIndices[] = {0,1,2, 1,3,2, 4,6,5, 5,6,7, 0,4,1, 1,4,5,
                               2,3,6, 3,7,6, 0,2,4, 2,6,4, 1,5,3, 3,5,7};

bool projectBounds(const AABB& box, const ViewPoint& view, float& xmin, float& ymin, float& xmax, float& ymax, float& nearestW) {
    const auto points = corners(box);
    xmin = ymin = std::numeric_limits<float>::infinity();
    xmax = ymax = -std::numeric_limits<float>::infinity();
    nearestW = std::numeric_limits<float>::infinity();
    bool anyFront = false, anyBehind = false;
    for (const Vec3& p : points) {
        const ClipVertex q = transform(p, view);
        if (q.w > 0.0f) {
            anyFront = true;
            const float x = q.x / q.w, y = q.y / q.w;
            xmin = std::min(xmin, x); xmax = std::max(xmax, x);
            ymin = std::min(ymin, y); ymax = std::max(ymax, y);
            nearestW = std::min(nearestW, q.w);
        } else anyBehind = true;
    }
    if (!anyFront) return false;
    // A cell straddling the eye plane is conservatively visible; TestRect cannot
    // represent it safely, so leave it in the visible set.
    if (anyBehind) { xmin = -1.0f; ymin = -1.0f; xmax = 1.0f; ymax = 1.0f; nearestW = 0.0001f; }
    return true;
}
} // namespace

OctreeRasterizer::OctreeRasterizer(unsigned width, unsigned height) : moc_(nullptr), width_(width), height_(height) {
    if (width == 0 || height == 0 || width % 8 != 0 || height % 4 != 0)
        throw std::invalid_argument("MaskedOcclusionCulling resolution must be width % 8 == 0 and height % 4 == 0");
    moc_ = MaskedOcclusionCulling::Create(MaskedOcclusionCulling::SSE2);
    moc_->SetResolution(width_, height_);
}

OctreeRasterizer::~OctreeRasterizer() { if (moc_) MaskedOcclusionCulling::Destroy(moc_); }

RasterizedOctree OctreeRasterizer::rasterize(OcclusionOctree& tree, const ViewPoint& view) {
    RasterizedOctree result;
    result.width = width_; result.height = height_;
    moc_->ClearBuffer();

    std::vector<OctreeNode*> leaves;
    tree.leaves(leaves);
    std::vector<ClipVertex> vertices;
    vertices.reserve(8);
    for (const OctreeNode* leaf : leaves) {
        if (!leaf->occupied) continue;
        vertices.clear();
        for (const Vec3& p : corners(leaf->bounds)) vertices.push_back(transform(p, view));
        moc_->RenderTriangles(reinterpret_cast<const float*>(vertices.data()), boxIndices, 12, nullptr,
                              MaskedOcclusionCulling::BACKFACE_NONE, MaskedOcclusionCulling::CLIP_PLANE_ALL);
    }

    for (const OctreeNode* leaf : leaves) {
        if (leaf->occupied) continue;
        float xmin, ymin, xmax, ymax, nearestW;
        if (!projectBounds(leaf->bounds, view, xmin, ymin, xmax, ymax, nearestW)) continue;
        const auto state = moc_->TestRect(xmin, ymin, xmax, ymax, nearestW);
        if (state == MaskedOcclusionCulling::VISIBLE) result.visibleEmptyLeaves.push_back(leaf);
        else if (state == MaskedOcclusionCulling::OCCLUDED) result.occludedEmptyLeaves.push_back(leaf);
    }
    result.depth.resize(static_cast<std::size_t>(width_) * height_);
    moc_->ComputePixelDepthBuffer(result.depth.data(), false);
    return result;
}

} // namespace openu
