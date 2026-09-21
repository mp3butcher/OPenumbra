#include "openu/OctreeRasterizer.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <unordered_set>
#include <vector>

#include "MaskedOcclusionCulling.h"

namespace openu {
namespace {

struct ClipVertex {
    float x, y, z, w;
};

std::array<Vec3, 8> corners(const AABB& b) {
    return {{{b.min.x, b.min.y, b.min.z}, {b.max.x, b.min.y, b.min.z}, {b.min.x, b.max.y, b.min.z}, {b.max.x, b.max.y, b.min.z},
             {b.min.x, b.min.y, b.max.z}, {b.max.x, b.min.y, b.max.z}, {b.min.x, b.max.y, b.max.z}, {b.max.x, b.max.y, b.max.z}}};
}

ClipVertex transform(const Vec3& p, const Mat4& worldToClip) {
    const float x = worldToClip.m[0] * p.x + worldToClip.m[1] * p.y + worldToClip.m[2] * p.z + worldToClip.m[3];
    const float y = worldToClip.m[4] * p.x + worldToClip.m[5] * p.y + worldToClip.m[6] * p.z + worldToClip.m[7];
    const float z = worldToClip.m[8] * p.x + worldToClip.m[9] * p.y + worldToClip.m[10] * p.z + worldToClip.m[11];
    const float w = worldToClip.m[12] * p.x + worldToClip.m[13] * p.y + worldToClip.m[14] * p.z + worldToClip.m[15];
    return {x, y, z, w};
}

bool aabbIntersectsFrustum(const AABB& box, const ViewPoint& view) {
    const Mat4 clip = view.worldToClip();
    for (const Vec3& v : corners(box)) {
        const ClipVertex q = transform(v, clip);
        if (q.w > 0.0f && std::abs(q.x) <= q.w && std::abs(q.y) <= q.w) return true;
    }
    return false;
}

const unsigned boxIndices[] = {0, 1, 2, 1, 3, 2, 4, 6, 5, 5, 6, 7,
                               0, 4, 1, 1, 4, 5, 2, 3, 6, 3, 7, 6,
                               0, 2, 4, 2, 6, 4, 1, 5, 3, 3, 5, 7};

bool projectBounds(const AABB& box, const ViewPoint& view, float& xmin, float& ymin, float& xmax, float& ymax, float& nearestW) {
    const Mat4 clip = view.worldToClip();
    xmin = ymin = std::numeric_limits<float>::infinity();
    xmax = ymax = -std::numeric_limits<float>::infinity();
    nearestW = std::numeric_limits<float>::infinity();
    bool anyFront = false;
    bool anyBehind = false;

    for (const Vec3& p : corners(box)) {
        const ClipVertex q = transform(p, clip);
        if (q.w > 0.0f) {
            anyFront = true;
            const float x = q.x / q.w;
            const float y = q.y / q.w;
            xmin = std::min(xmin, x); xmax = std::max(xmax, x);
            ymin = std::min(ymin, y); ymax = std::max(ymax, y);
            nearestW = std::min(nearestW, q.w);
        } else {
            anyBehind = true;
        }
    }
    if (!anyFront) return false;
    if (anyBehind) {
        xmin = -1.0f; ymin = -1.0f; xmax = 1.0f; ymax = 1.0f; nearestW = 0.0001f;
    }
    return true;
}

} // namespace

OctreeRasterizer::OctreeRasterizer(unsigned width, unsigned height)
    : moc_(nullptr), width_(width), height_(height) {
    if (width == 0 || height == 0 || width % 8 != 0 || height % 4 != 0) {
        throw std::invalid_argument("MaskedOcclusionCulling resolution must be width % 8 == 0 and height % 4 == 0");
    }
    moc_ = MaskedOcclusionCulling::Create(MaskedOcclusionCulling::SSE2);
    moc_->SetResolution(width_, height_);
}

OctreeRasterizer::~OctreeRasterizer() {
    if (moc_) MaskedOcclusionCulling::Destroy(moc_);
}

std::vector<const OctreeNode*> OctreeRasterizer::visibleCells(const OcclusionOctree& tree, const ViewPoint& view) const {
    std::vector<const OctreeNode*> visible;
    std::unordered_set<const OctreeNode*> visited;
    std::vector<const OctreeNode*> frontier;

    const OctreeNode* start = tree.locate(view.cameraPosition());
    if (!start) return visible;

    frontier.push_back(start);

    while (!frontier.empty()) {
        const OctreeNode* node = frontier.back();
        frontier.pop_back();

        if (!node || visited.count(node) != 0) continue;
        visited.insert(node);

        if (!node->occupied) visible.push_back(node);

        const Axis axes[3] = {Axis::X, Axis::Y, Axis::Z};
        for (Axis axis : axes) {
            const Direction directions[2] = {Direction::Negative, Direction::Positive};
            for (Direction dir : directions) {
                std::vector<OctreeNode*> neighbors;
                node->getNeighbors(axis, dir, neighbors);
                for (OctreeNode* neighbor : neighbors) {
                    if (!neighbor || visited.count(neighbor) != 0) continue;
                    if (!aabbIntersectsFrustum(neighbor->bounds, view)) continue;
                    frontier.push_back(neighbor);
                }
            }
        }
    }

    return visible;
}

RasterizedOctree OctreeRasterizer::rasterize(OcclusionOctree& tree, const ViewPoint& view) {
    RasterizedOctree result;
    result.width = width_;
    result.height = height_;
    moc_->ClearBuffer();

    const std::vector<const OctreeNode*> nodes = visibleCells(tree, view);
    std::vector<ClipVertex> vertices;
    vertices.reserve(8);

    for (const OctreeNode* node : nodes) {
        if (!node || !node->occupied) continue;
        vertices.clear();
        for (const Vec3& p : corners(node->bounds)) {
            const ClipVertex q = transform(p, view.worldToClip());
            vertices.push_back(q);
        }
        moc_->RenderTriangles(reinterpret_cast<const float*>(vertices.data()), boxIndices, 12, nullptr,
                              MaskedOcclusionCulling::BACKFACE_NONE, MaskedOcclusionCulling::CLIP_PLANE_ALL);
    }

    for (const OctreeNode* node : nodes) {
        if (!node || node->occupied) continue;
        float xmin, ymin, xmax, ymax, nearestW;
        if (!projectBounds(node->bounds, view, xmin, ymin, xmax, ymax, nearestW)) continue;
        const auto state = moc_->TestRect(xmin, ymin, xmax, ymax, nearestW);
        if (state == MaskedOcclusionCulling::VISIBLE) result.visibleEmptyLeaves.push_back(node);
        else if (state == MaskedOcclusionCulling::OCCLUDED) result.occludedEmptyLeaves.push_back(node);
    }

    result.depth.resize(static_cast<std::size_t>(width_) * height_);
    moc_->ComputePixelDepthBuffer(result.depth.data(), false);
    return result;
}

} // namespace openu
