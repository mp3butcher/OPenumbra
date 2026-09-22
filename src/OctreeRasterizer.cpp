#include "openu/OctreeRasterizer.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <unordered_set>
#include <vector>

#include "MaskedOcclusionCulling.h"
#include <iostream>
namespace openu {
namespace {
struct ClipVertex { float x, y, z, w; };

std::array<Vec3, 8> corners(const AABB& b) {
    return {{{b.min.x,b.min.y,b.min.z}, {b.max.x,b.min.y,b.min.z}, {b.min.x,b.max.y,b.min.z}, {b.max.x,b.max.y,b.min.z},
             {b.min.x,b.min.y,b.max.z}, {b.max.x,b.min.y,b.max.z}, {b.min.x,b.max.y,b.max.z}, {b.max.x,b.max.y,b.max.z}}};
}

ClipVertex transform(const Vec3& p, const Mat4& m) {
    return {m.m[0]*p.x + m.m[1]*p.y + m.m[2]*p.z + m.m[3],
            m.m[4]*p.x + m.m[5]*p.y + m.m[6]*p.z + m.m[7],
            m.m[8]*p.x + m.m[9]*p.y + m.m[10]*p.z + m.m[11],
            m.m[12]*p.x + m.m[13]*p.y + m.m[14]*p.z + m.m[15]};
}

bool aabbIntersectsFrustum(const AABB& box, const Mat4& clip) {
    for (const Vec3& p : corners(box)) {
        const ClipVertex q = transform(p, clip);
        if (q.w > 0.0f && std::abs(q.x) <= q.w && std::abs(q.y) <= q.w) return true;
    }
    return false;
}

bool projectBounds(const AABB& box, const Mat4& clip, float& xmin, float& ymin,
                  float& xmax, float& ymax, float& nearestW) {
    xmin = ymin = std::numeric_limits<float>::infinity();
    xmax = ymax = -std::numeric_limits<float>::infinity();
    nearestW = std::numeric_limits<float>::infinity();
    bool front = false, behind = false;
    for (const Vec3& p : corners(box)) {
        const ClipVertex q = transform(p, clip);
        if (q.w > 0.0f) {
            front = true;
            xmin = std::min(xmin, q.x / q.w); xmax = std::max(xmax, q.x / q.w);
            ymin = std::min(ymin, q.y / q.w); ymax = std::max(ymax, q.y / q.w);
            nearestW = std::min(nearestW, q.w);
        } else behind = true;
    }
    if (!front) return false;
    if (behind) { xmin = -1.0f; ymin = -1.0f; xmax = 1.0f; ymax = 1.0f; nearestW = 0.0001f; }
    return true;
}

const unsigned boxIndices[] = {0,1,2, 1,3,2, 4,6,5, 5,6,7, 0,4,1, 1,4,5,
                               2,3,6, 3,7,6, 0,2,4, 2,6,4, 1,5,3, 3,5,7};

} // namespace

OctreeRasterizer::OctreeRasterizer(unsigned width, unsigned height)
    : moc_(nullptr), width_(width), height_(height) {
    if (width == 0 || height == 0 || width % 8 != 0 || height % 4 != 0)
        throw std::invalid_argument("MaskedOcclusionCulling resolution must be width % 8 == 0 and height % 4 == 0");
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
    const Mat4 clip = view.worldToClip();
    while (!frontier.empty()) {
        const OctreeNode* node = frontier.back(); frontier.pop_back();
        if (!node || !visited.insert(node).second) continue;
        if (!node->occupied) visible.push_back(node);
        for (Axis axis : {Axis::X, Axis::Y, Axis::Z}) {
            for (Direction dir : {Direction::Negative, Direction::Positive}) {
                std::vector<const OctreeNode*> neighbors;
                node->getNeighbors(axis, dir, neighbors);
                for (const OctreeNode* neighbor : neighbors) {
                    if (neighbor && !visited.count(neighbor) && aabbIntersectsFrustum(neighbor->bounds, clip)) frontier.push_back(neighbor);
                }
            }
        }
    }
    return visible;
}

std::vector<const OctreeNode*> OctreeRasterizer::visibleCells(const CompiledOctree& tree, const ViewPoint& view) const {
    std::vector<const OctreeNode*> visible;
    if (tree.nodeCount() == 0) return visible;

    if (visitStamp_.size() != tree.nodeCount()) visitStamp_.assign(tree.nodeCount(), 0);
    ++traversalStamp_;
    if (traversalStamp_ == 0) {
        std::fill(visitStamp_.begin(), visitStamp_.end(), 0);
        traversalStamp_ = 1;
    }

    NodeId start = InvalidNode;
    const OctreeNode* sourceStart = nullptr;
    // CompiledOctree currently stores terminal cells, so locate once by source
    // pointer; all subsequent operations use integer IDs and flat ranges.
    // This linear lookup is outside the hot neighbor traversal loop.
    for (NodeId id = 0; id < tree.nodeCount(); ++id) {
        const CompiledNode& n = tree.node(id);
        if (n.source && n.bounds.contains(view.cameraPosition())) { start = id; sourceStart = n.source; break; }
    }
    (void)sourceStart;
    if (start == InvalidNode)
        return visible;

    frontier_.clear();
    frontier_.push_back(start);
    const Mat4 clip = view.worldToClip();
    while (!frontier_.empty()) {
        const NodeId id = frontier_.back(); frontier_.pop_back();
        if (id == InvalidNode || id >= tree.nodeCount() || visitStamp_[id] == traversalStamp_) continue;
        visitStamp_[id] = traversalStamp_;
        const CompiledNode& node = tree.node(id);
        if (!node.occupied) visible.push_back(node.source);
        for (std::size_t face = 0; face < 6; ++face) {
            const std::uint32_t begin = node.neighborBegin[face];
            const std::uint32_t end = begin + node.neighborCount[face];
            const auto& neighbors = tree.neighborStorage();
            for (std::uint32_t i = begin; i < end; ++i) {
                //std::cout<<i<<" chek neighbor node" <<std::endl;
                const NodeId neighbor = neighbors[i];
                if (neighbor == InvalidNode || visitStamp_[neighbor] == traversalStamp_) continue;
                if (aabbIntersectsFrustum(tree.node(neighbor).bounds, clip)) frontier_.push_back(neighbor);
            }
        }
    }
    return visible;
}

RasterizedOctree OctreeRasterizer::rasterizeNodes(const std::vector<const OctreeNode*>& nodes, const ViewPoint& view) {
    RasterizedOctree result;
    result.width = width_; result.height = height_;
    moc_->ClearBuffer();
    const Mat4 clip = view.worldToClip();
    std::array<ClipVertex, 8> vertices{};

    for (const OctreeNode* node : nodes) {

        std::cerr<<" check drawing node"<<std::endl;
        if (!node || !node->occupied) continue;
        const auto points = corners(node->bounds);
        for (std::size_t i = 0; i < points.size(); ++i) vertices[i] = transform(points[i], clip);
        std::cerr<<" drawing node"<<std::endl;
        moc_->RenderTriangles(reinterpret_cast<const float*>(vertices.data()), boxIndices, 12, nullptr,
                              MaskedOcclusionCulling::BACKFACE_NONE, MaskedOcclusionCulling::CLIP_PLANE_ALL);
    }
    for (const OctreeNode* node : nodes) {
        if (!node || node->occupied) continue;
        float xmin, ymin, xmax, ymax, nearestW;
        if (!projectBounds(node->bounds, clip, xmin, ymin, xmax, ymax, nearestW)) continue;
        const auto state = moc_->TestRect(xmin, ymin, xmax, ymax, nearestW);
        if (state == MaskedOcclusionCulling::VISIBLE) result.visibleEmptyLeaves.push_back(node);
        else if (state == MaskedOcclusionCulling::OCCLUDED) result.occludedEmptyLeaves.push_back(node);
    }
    result.depth.resize(static_cast<std::size_t>(width_) * height_);
    moc_->ComputePixelDepthBuffer(result.depth.data(), false);
    return result;
}

RasterizedOctree OctreeRasterizer::rasterize(OcclusionOctree& tree, const ViewPoint& view) {
    return rasterizeNodes(visibleCells(tree, view), view);
}

RasterizedOctree OctreeRasterizer::rasterize(const CompiledOctree& tree, const ViewPoint& view) {
    return rasterizeNodes(visibleCells(tree, view), view);
}

} // namespace openu
