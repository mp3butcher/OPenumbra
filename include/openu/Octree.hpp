#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <stdexcept>
#include <utility>
#include <vector>

namespace openu {

struct Vec3 {
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;

    Vec3() = default;
    Vec3(float x_, float y_, float z_) : x(x_), y(y_), z(z_) {}

    Vec3 operator+(const Vec3& other) const { return {x + other.x, y + other.y, z + other.z}; }
    Vec3 operator-(const Vec3& other) const { return {x - other.x, y - other.y, z - other.z}; }
    Vec3 operator*(float scalar) const { return {x * scalar, y * scalar, z * scalar}; }
    Vec3 operator/(float scalar) const { return {x / scalar, y / scalar, z / scalar}; }
    Vec3& operator+=(const Vec3& other) {
        x += other.x; y += other.y; z += other.z; return *this;
    }
    Vec3& operator-=(const Vec3& other) {
        x -= other.x; y -= other.y; z -= other.z; return *this;
    }

    float dot(const Vec3& other) const { return x * other.x + y * other.y + z * other.z; }
    float lengthSq() const { return x * x + y * y + z * z; }
};

inline Vec3 operator*(float scalar, const Vec3& v) { return v * scalar; }

struct Triangle {
    Vec3 a;
    Vec3 b;
    Vec3 c;

    Triangle() = default;
    Triangle(const Vec3& a_, const Vec3& b_, const Vec3& c_) : a(a_), b(b_), c(c_) {}

    Vec3 normal() const {
        const Vec3 u = b - a;
        const Vec3 v = c - a;
        const Vec3 n = {
            u.y * v.z - u.z * v.y,
            u.z * v.x - u.x * v.z,
            u.x * v.y - u.y * v.x,
        };
        const float len = std::sqrt(n.x * n.x + n.y * n.y + n.z * n.z);
        if (len <= 1e-8f) {
            return {0.0f, 0.0f, 0.0f};
        }
        return n / len;
    }

    struct Bounds {
        Vec3 min;
        Vec3 max;
    };

    Bounds bounds() const {
        return {{
            std::min(std::min(a.x, b.x), c.x),
            std::min(std::min(a.y, b.y), c.y),
            std::min(std::min(a.z, b.z), c.z),
        }, {
            std::max(std::max(a.x, b.x), c.x),
            std::max(std::max(a.y, b.y), c.y),
            std::max(std::max(a.z, b.z), c.z),
        }};
    }
};

struct AABB {
    Vec3 min;
    Vec3 max;

    static AABB fromCenterHalfExtent(const Vec3& center, const Vec3& halfExtent) {
        return {center - halfExtent, center + halfExtent};
    }

    Vec3 center() const { return (min + max) * 0.5f; }
    Vec3 halfExtent() const { return (max - min) * 0.5f; }

    bool contains(const Vec3& p) const {
        return p.x >= min.x && p.x <= max.x &&
               p.y >= min.y && p.y <= max.y &&
               p.z >= min.z && p.z <= max.z;
    }

    bool intersects(const AABB& other) const {
        return (min.x <= other.max.x && max.x >= other.min.x) &&
               (min.y <= other.max.y && max.y >= other.min.y) &&
               (min.z <= other.max.z && max.z >= other.min.z);
    }

    bool intersectsTriangle(const Triangle& triangle) const {
        // Fast reject by triangle bounds.
        const auto tri = triangle.bounds();
        if (tri.max.x < min.x || tri.min.x > max.x) return false;
        if (tri.max.y < min.y || tri.min.y > max.y) return false;
        if (tri.max.z < min.z || tri.min.z > max.z) return false;

        // Vertex-in-box test.
        if (contains(triangle.a) || contains(triangle.b) || contains(triangle.c)) {
            return true;
        }

        // Box-corner test against triangle edges.
        const std::array<Vec3, 8> corners = {
            Vec3{min.x, min.y, min.z}, Vec3{max.x, min.y, min.z}, Vec3{min.x, max.y, min.z}, Vec3{max.x, max.y, min.z},
            Vec3{min.x, min.y, max.z}, Vec3{max.x, min.y, max.z}, Vec3{min.x, max.y, max.z}, Vec3{max.x, max.y, max.z}
        };

        const std::array<Vec3, 3> triVerts = {triangle.a, triangle.b, triangle.c};
        const std::array<std::pair<Vec3, Vec3>, 3> edges = {
            std::make_pair(triangle.a, triangle.b),
            std::make_pair(triangle.b, triangle.c),
            std::make_pair(triangle.c, triangle.a)
        };

        for (const auto& edge : edges) {
            for (const Vec3& corner : corners) {
                // This is a coarse test; for a real engine one would use robust SAT/Separating Axis.
                const auto v0 = edge.first - corner;
                const auto v1 = edge.second - corner;
                if ((v0.x == 0.0f && v1.x == 0.0f) || (v0.y == 0.0f && v1.y == 0.0f) || (v0.z == 0.0f && v1.z == 0.0f)) {
                    return true;
                }
            }
        }

        // Box facet test: if triangle plane intersects the box.
        const Vec3 n = triangle.normal();
        if (n.x == 0.0f && n.y == 0.0f && n.z == 0.0f) {
            return false;
        }

        // Signed distances to box corners.
        float dmin = std::numeric_limits<float>::max();
        float dmax = -std::numeric_limits<float>::max();
        for (const Vec3& p : triVerts) {
            const float d = n.x * p.x + n.y * p.y + n.z * p.z;
            dmin = std::min(dmin, d);
            dmax = std::max(dmax, d);
        }

        const float boxDMin = n.x * min.x + n.y * min.y + n.z * min.z;
        const float boxDMax = n.x * max.x + n.y * max.y + n.z * max.z;

        return !(dmax < boxDMin || dmin > boxDMax);
    }
};

enum class Axis : std::uint8_t {
    X = 0,
    Y = 1,
    Z = 2,
};

enum class Direction : std::uint8_t {
    Negative = 0,
    Positive = 1,
};

struct OctreeNode {
    AABB bounds;
    std::array<std::unique_ptr<OctreeNode>, 8> children{};
    OctreeNode* parent = nullptr;

    // Local child code relative to the parent: bit 0 = X, bit 1 = Y, bit 2 = Z.
    std::uint8_t localIndex = 0;
    std::uint64_t morton = 0;
    std::uint32_t depth = 0;

    bool isLeaf = true;
    bool occupied = false;
    std::vector<Triangle> triangles;

    static std::uint8_t axisMask(Axis axis) {
        switch (axis) {
            case Axis::X: return 1u;
            case Axis::Y: return 2u;
            case Axis::Z: return 4u;
        }
        return 0u;
    }

    // This is the implicit Morton-style neighbor walk the user requested.
    // We climb the parents until we find a node whose cousin across the face exists, then
    // we descend using the same suffix path but with the axis bit forced toward the neighbor side.
    OctreeNode* getNeighbor(Axis axis, Direction dir) const {
        const std::uint8_t mask = axisMask(axis);
        const bool positive = (dir == Direction::Positive);

        NodePath path;
        const OctreeNode* cursor = this;
        while (cursor) {
            if (cursor->parent != nullptr) {
                path.push_back(cursor->localIndex);
            }
            cursor = cursor->parent;
        }
        std::reverse(path.begin(), path.end());

        // Climb until we are not still on the same side as the face direction.
        const OctreeNode* ancestor = this;
        while (ancestor->parent != nullptr) {
            const std::uint8_t local = ancestor->localIndex;
            const bool onPositiveSide = ((local & mask) != 0u);
            if ((positive && onPositiveSide) || (!positive && !onPositiveSide)) {
                ancestor = ancestor->parent;
                continue;
            }
            break;
        }

        if (ancestor->parent == nullptr && ((positive && ((ancestor->localIndex & mask) == 0u)) || (!positive && ((ancestor->localIndex & mask) != 0u)))) {
            // Root itself is the face boundary; no neighbor in that direction.
            return nullptr;
        }

        // Find the sibling across the face at the ancestor level.
        OctreeNode* candidate = nullptr;
        if (ancestor->parent != nullptr) {
            candidate = ancestor->parent->children[ancestor->localIndex ^ mask].get();
        }
        if (!candidate) {
            return nullptr;
        }

        // Descend to the same relative depth using the suffix path.
        // The suffix keeps all bits equal except the axis bit, which must be forced toward the neighbor side.
        std::vector<std::uint8_t> suffix;
        const OctreeNode* cur = this;
        while (cur && cur != ancestor) {
            suffix.push_back(cur->localIndex);
            cur = cur->parent;
        }
        std::reverse(suffix.begin(), suffix.end());

        OctreeNode* result = candidate;
        for (const std::uint8_t index : suffix) {
            const std::uint8_t nextIndex = (index & static_cast<std::uint8_t>(~mask)) | (positive ? mask : 0u);
            if (!result->children[nextIndex]) {
                return result;
            }
            result = result->children[nextIndex].get();
        }

        return result;
    }

    std::vector<OctreeNode*> faceNeighbors(Axis axis) const {
        std::vector<OctreeNode*> result;
        if (children[0] == nullptr && isLeaf) {
            return result;
        }
        const auto neg = getNeighbor(axis, Direction::Negative);
        const auto pos = getNeighbor(axis, Direction::Positive);
        if (neg) result.push_back(neg);
        if (pos) result.push_back(pos);
        return result;
    }

private:
    using NodePath = std::vector<std::uint8_t>;
};

class OcclusionOctree {
public:
    OcclusionOctree(AABB worldBounds, std::size_t maxDepth = 8, std::size_t maxTrianglesPerCell = 1)
        : root_(std::make_unique<OctreeNode>()),
          maxDepth_(maxDepth),
          maxTrianglesPerCell_(maxTrianglesPerCell),
          worldBounds_(worldBounds) {
        root_->bounds = worldBounds_;
        root_->depth = 0;
    }

    void insert(const Triangle& triangle) {
        insertTriangle(root_.get(), triangle);
    }

    void insert(const std::vector<Triangle>& triangles) {
        for (const auto& t : triangles) {
            insert(t);
        }
    }

    std::vector<OctreeNode*> leaves() {
        std::vector<OctreeNode*> collected;
        collectLeaves(root_.get(), collected);
        return collected;
    }

    std::size_t occupiedLeafCount() const {
        return countOccupiedLeaves(root_.get());
    }

    std::vector<const Triangle*> query(const AABB& region) const {
        std::vector<const Triangle*> out;
        collectTriangles(root_.get(), region, out);
        return out;
    }

private:
    static AABB childBounds(const AABB& parent, std::uint8_t childIndex) {
        const Vec3 center = parent.center();
        const Vec3 half = parent.halfExtent() * 0.5f;
        const float hx = half.x;
        const float hy = half.y;
        const float hz = half.z;

        const float minX = (childIndex & 1u) ? center.x : parent.min.x;
        const float maxX = (childIndex & 1u) ? parent.max.x : center.x;
        const float minY = (childIndex & 2u) ? center.y : parent.min.y;
        const float maxY = (childIndex & 2u) ? parent.max.y : center.y;
        const float minZ = (childIndex & 4u) ? center.z : parent.min.z;
        const float maxZ = (childIndex & 4u) ? parent.max.z : center.z;

        return {{minX, minY, minZ}, {maxX, maxY, maxZ}};
    }

    void insertTriangle(OctreeNode* node, const Triangle& triangle) {
        if (!node) {
            return;
        }

        if (!node->bounds.intersectsTriangle(triangle)) {
            return;
        }

        if (node->isLeaf) {
            node->triangles.push_back(triangle);
            node->occupied = !node->triangles.empty();

            if (node->triangles.size() > maxTrianglesPerCell_ && node->depth < maxDepth_) {
                subdivide(node);
            }

            return;
        }

        for (auto& child : node->children) {
            if (child) {
                insertTriangle(child.get(), triangle);
            }
        }
    }

    void subdivide(OctreeNode* node) {
        if (!node || !node->isLeaf) {
            return;
        }

        std::vector<Triangle> batch = std::move(node->triangles);
        node->triangles.clear();
        node->isLeaf = false;

        for (std::uint8_t i = 0; i < 8; ++i) {
            auto child = std::make_unique<OctreeNode>();
            child->bounds = childBounds(node->bounds, i);
            child->depth = node->depth + 1;
            child->localIndex = i;
            child->morton = (node->morton << 3) | i;
            child->parent = node;
            node->children[i] = std::move(child);
        }

        for (const auto& tri : batch) {
            for (auto& child : node->children) {
                if (child && child->bounds.intersectsTriangle(tri)) {
                    insertTriangle(child.get(), tri);
                }
            }
        }

        node->occupied = false;
        for (const auto& child : node->children) {
            if (child && child->occupied) {
                node->occupied = true;
                break;
            }
        }
    }

    void collectLeaves(OctreeNode* node, std::vector<OctreeNode*>& out) const {
        if (!node) return;
        if (node->isLeaf) {
            out.push_back(node);
            return;
        }
        for (const auto& child : node->children) {
            if (child) collectLeaves(child.get(), out);
        }
    }

    void collectTriangles(OctreeNode* node, const AABB& region, std::vector<const Triangle*>& out) const {
        if (!node) return;
        if (!node->bounds.intersects(region)) return;

        if (node->isLeaf) {
            for (const auto& tri : node->triangles) {
                if (region.intersectsTriangle(tri)) {
                    out.push_back(&tri);
                }
            }
            return;
        }

        for (const auto& child : node->children) {
            if (child) collectTriangles(child.get(), region, out);
        }
    }

    std::size_t countOccupiedLeaves(OctreeNode* node) const {
        if (!node) return 0;
        if (node->isLeaf) {
            return node->occupied ? 1u : 0u;
        }
        std::size_t total = 0;
        for (const auto& child : node->children) {
            if (child) total += countOccupiedLeaves(child.get());
        }
        return total;
    }

    std::unique_ptr<OctreeNode> root_;
    std::size_t maxDepth_ = 8;
    std::size_t maxTrianglesPerCell_ = 1;
    AABB worldBounds_{};
};

}  // namespace openu
