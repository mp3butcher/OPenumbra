#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <vector>

namespace openu {

struct Vec3 {
    float x = 0, y = 0, z = 0;
    Vec3() = default;
    Vec3(float x_, float y_, float z_) : x(x_), y(y_), z(z_) {}
    Vec3 operator+(const Vec3& v) const { return {x + v.x, y + v.y, z + v.z}; }
    Vec3 operator-(const Vec3& v) const { return {x - v.x, y - v.y, z - v.z}; }
    Vec3 operator*(float s) const { return {x * s, y * s, z * s}; }
    Vec3 operator/(float s) const { return {x / s, y / s, z / s}; }
};

struct Mat4 {
    float m[16]{};

    static Mat4 identity() {
        Mat4 out{};
        out.m[0] = 1.0f; out.m[5] = 1.0f; out.m[10] = 1.0f; out.m[15] = 1.0f;
        return out;
    }

    Mat4 operator*(const Mat4& rhs) const {
        Mat4 out{};
        for (int row = 0; row < 4; ++row) {
            for (int col = 0; col < 4; ++col) {
                float sum = 0.0f;
                for (int k = 0; k < 4; ++k) {
                    sum += m[row * 4 + k] * rhs.m[k * 4 + col];
                }
                out.m[row * 4 + col] = sum;
            }
        }
        return out;
    }

    Vec3 transformPoint(const Vec3& p) const {
        const float x = m[0] * p.x + m[1] * p.y + m[2] * p.z + m[3];
        const float y = m[4] * p.x + m[5] * p.y + m[6] * p.z + m[7];
        const float z = m[8] * p.x + m[9] * p.y + m[10] * p.z + m[11];
        const float w = m[12] * p.x + m[13] * p.y + m[14] * p.z + m[15];
        if (std::abs(w) > 1e-6f) return {x / w, y / w, z / w};
        return {x, y, z};
    }
};

struct Triangle {
    Vec3 a, b, c;
    Triangle() = default;
    Triangle(Vec3 a_, Vec3 b_, Vec3 c_) : a(a_), b(b_), c(c_) {}
};

struct AABB {
    Vec3 min, max;
    Vec3 center() const { return (min + max) * 0.5f; }

    bool intersects(const AABB& b) const {
        return min.x <= b.max.x && max.x >= b.min.x &&
               min.y <= b.max.y && max.y >= b.min.y &&
               min.z <= b.max.z && max.z >= b.min.z;
    }

    bool contains(const Vec3& p) const {
        return p.x >= min.x && p.x <= max.x && p.y >= min.y && p.y <= max.y &&
               p.z >= min.z && p.z <= max.z;
    }

    bool intersectsTriangle(const Triangle& t) const {
        const Vec3 center = this->center();
        const Vec3 half = {(max.x - min.x) * 0.5f, (max.y - min.y) * 0.5f, (max.z - min.z) * 0.5f};
        const Vec3 v[3] = {t.a - center, t.b - center, t.c - center};
        const Vec3 e[3] = {v[1] - v[0], v[2] - v[1], v[0] - v[2]};

        const auto separated = [&](const Vec3& axis) {
            const float n2 = axis.x * axis.x + axis.y * axis.y + axis.z * axis.z;
            if (n2 < 1e-12f) return false;
            const float p0 = v[0].x * axis.x + v[0].y * axis.y + v[0].z * axis.z;
            const float p1 = v[1].x * axis.x + v[1].y * axis.y + v[1].z * axis.z;
            const float p2 = v[2].x * axis.x + v[2].y * axis.y + v[2].z * axis.z;
            const float radius = half.x * std::abs(axis.x) + half.y * std::abs(axis.y) + half.z * std::abs(axis.z);
            return std::min({p0, p1, p2}) > radius || std::max({p0, p1, p2}) < -radius;
        };

        if (separated({1, 0, 0}) || separated({0, 1, 0}) || separated({0, 0, 1})) return false;
        const Vec3 normal = {e[0].y * e[1].z - e[0].z * e[1].y,
                             e[0].z * e[1].x - e[0].x * e[1].z,
                             e[0].x * e[1].y - e[0].y * e[1].x};
        if (separated(normal)) return false;

        const Vec3 axes[3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
        for (const Vec3& edge : e) {
            for (const Vec3& boxAxis : axes) {
                const Vec3 axis = {edge.y * boxAxis.z - edge.z * boxAxis.y,
                                   edge.z * boxAxis.x - edge.x * boxAxis.z,
                                   edge.x * boxAxis.y - edge.y * boxAxis.x};
                if (separated(axis)) return false;
            }
        }
        return true;
    }
};

enum class Axis : std::uint8_t { X, Y, Z };
enum class Direction : std::uint8_t { Negative, Positive };

struct OctreeNode {
    using ChildCode = std::uint8_t;
    static constexpr ChildCode X = 1u;
    static constexpr ChildCode Y = 2u;
    static constexpr ChildCode Z = 4u;

    AABB bounds{};
    OctreeNode* parent = nullptr;
    std::array<std::unique_ptr<OctreeNode>, 8> children{};
    ChildCode localMorton = 0; // bit 0 X, bit 1 Y, bit 2 Z
    std::uint32_t depth = 0;
    bool occupied = false;
    std::vector<Triangle> triangles;

    bool terminal() const { return children[0] == nullptr; }
    static ChildCode mask(Axis axis) { return axis == Axis::X ? X : axis == Axis::Y ? Y : Z; }

    OctreeNode* getNeighbor(Axis axis, Direction direction) const {
        const ChildCode bit = mask(axis);
        const bool positive = direction == Direction::Positive;
        const OctreeNode* pivot = this;

        while (pivot->parent != nullptr) {
            const bool onPositiveSide = (pivot->localMorton & bit) != 0;
            if (onPositiveSide != positive) break;
            pivot = pivot->parent;
        }
        if (pivot->parent == nullptr) return nullptr;

        OctreeNode* result = pivot->parent->children[pivot->localMorton ^ bit].get();
        if (!result) return nullptr;

        std::vector<ChildCode> suffix;
        for (const OctreeNode* n = this; n != pivot; n = n->parent) suffix.push_back(n->localMorton);
        std::reverse(suffix.begin(), suffix.end());

        for (ChildCode code : suffix) {
            if (result->terminal()) break;
            const ChildCode next = static_cast<ChildCode>((code & ~bit) | (positive ? 0 : bit));
            if (!result->children[next]) break;
            result = result->children[next].get();
        }
        return result;
    }

    void getNeighbors(Axis axis, Direction direction, std::vector<OctreeNode*>& out) const {
        const ChildCode bit = mask(axis);
        const bool positive = direction == Direction::Positive;
        const OctreeNode* pivot = this;
        while (pivot->parent != nullptr) {
            if (((pivot->localMorton & bit) != 0) != positive) break;
            pivot = pivot->parent;
        }
        if (pivot->parent == nullptr) return;

        OctreeNode* candidate = pivot->parent->children[pivot->localMorton ^ bit].get();
        if (!candidate) return;
        std::vector<ChildCode> suffix;
        for (const OctreeNode* n = this; n != pivot; n = n->parent) suffix.push_back(n->localMorton);
        std::reverse(suffix.begin(), suffix.end());
        for (ChildCode code : suffix) {
            if (candidate->terminal()) break;
            const ChildCode next = static_cast<ChildCode>((code & ~bit) | (positive ? 0 : bit));
            if (!candidate->children[next]) break;
            candidate = candidate->children[next].get();
        }
        collectFaceLeaves(candidate, axis, positive, out);
    }

private:
    static void collectFaceLeaves(OctreeNode* node, Axis axis, bool positiveFromSource,
                                 std::vector<OctreeNode*>& out) {
        if (node->terminal()) { out.push_back(node); return; }
        const ChildCode bit = mask(axis);
        const ChildCode face = positiveFromSource ? 0 : bit;
        for (std::uint8_t i = 0; i < 8; ++i) {
            if ((i & bit) == face) collectFaceLeaves(node->children[i].get(), axis, positiveFromSource, out);
        }
    }
};

class OcclusionOctree {
public:
    OcclusionOctree(AABB bounds, std::size_t maxDepth = 8, std::size_t maxTrianglesPerCell = 1)
        : maxDepth_(maxDepth), maxTrianglesPerCell_(std::max<std::size_t>(1, maxTrianglesPerCell)) {
        root_ = std::make_unique<OctreeNode>();
        root_->bounds = bounds;
    }

    OctreeNode* root() { return root_.get(); }
    const OctreeNode* root() const { return root_.get(); }

    const OctreeNode* locate(const Vec3& p) const {
        const OctreeNode* node = root_.get();
        while (node && !node->terminal()) {
            const Vec3 center = node->bounds.center();
            std::uint8_t code = 0;
            if (p.x >= center.x) code |= OctreeNode::X;
            if (p.y >= center.y) code |= OctreeNode::Y;
            if (p.z >= center.z) code |= OctreeNode::Z;
            node = node->children[code].get();
            if (node == nullptr) break;
        }
        return node;
    }

    void insert(const Triangle& triangle) { insertTriangle(root_.get(), triangle); }
    void insert(const std::vector<Triangle>& triangles) { for (const auto& t : triangles) insert(t); }

    void leaves(std::vector<OctreeNode*>& out) { collectLeaves(root_.get(), out); }
    std::vector<OctreeNode*> leaves() { std::vector<OctreeNode*> out; leaves(out); return out; }

    std::size_t occupiedLeafCount() const { return countOccupied(root_.get()); }

private:
    static AABB childBounds(const AABB& b, std::uint8_t code) {
        const Vec3 c = b.center();
        return {{(code & OctreeNode::X) ? c.x : b.min.x, (code & OctreeNode::Y) ? c.y : b.min.y, (code & OctreeNode::Z) ? c.z : b.min.z},
                {(code & OctreeNode::X) ? b.max.x : c.x, (code & OctreeNode::Y) ? b.max.y : c.y, (code & OctreeNode::Z) ? b.max.z : c.z}};
    }

    void insertTriangle(OctreeNode* node, const Triangle& triangle) {
        if (!node->bounds.intersectsTriangle(triangle)) return;
        if (node->terminal()) {
            node->triangles.push_back(triangle);
            node->occupied = true;
            if (node->depth < maxDepth_ && node->triangles.size() > maxTrianglesPerCell_) subdivide(node);
            return;
        }
        for (auto& child : node->children) insertTriangle(child.get(), triangle);
        refreshOccupied(node);
    }

    void subdivide(OctreeNode* node) {
        const auto old = std::move(node->triangles);
        for (std::uint8_t i = 0; i < 8; ++i) {
            auto child = std::make_unique<OctreeNode>();
            child->parent = node; child->localMorton = i; child->depth = node->depth + 1;
            child->bounds = childBounds(node->bounds, i);
            node->children[i] = std::move(child);
        }
        for (const auto& triangle : old) for (auto& child : node->children) insertTriangle(child.get(), triangle);
        refreshOccupied(node);
    }

    static void refreshOccupied(OctreeNode* node) {
        node->occupied = false;
        for (const auto& child : node->children) if (child->occupied) { node->occupied = true; break; }
    }

    static void collectLeaves(OctreeNode* node, std::vector<OctreeNode*>& out) {
        if (node->terminal()) { out.push_back(node); return; }
        for (auto& child : node->children) collectLeaves(child.get(), out);
    }

    static std::size_t countOccupied(const OctreeNode* node) {
        if (node->terminal()) return node->occupied ? 1u : 0u;
        std::size_t count = 0; for (const auto& child : node->children) count += countOccupied(child.get()); return count;
    }

    std::unique_ptr<OctreeNode> root_;
    std::size_t maxDepth_;
    std::size_t maxTrianglesPerCell_;
};

} // namespace openu
