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
        return min.x <= b.max.x && max.x >= b.min.x && min.y <= b.max.y && max.y >= b.min.y && min.z <= b.max.z && max.z >= b.min.z;
    }

    bool contains(const Vec3& p) const {
        return p.x >= min.x && p.x <= max.x && p.y >= min.y && p.y <= max.y && p.z >= min.z && p.z <= max.z;
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
        const Vec3 normal = {e[0].y * e[1].z - e[0].z * e[1].y, e[0].z * e[1].x - e[0].x * e[1].z, e[0].x * e[1].y - e[0].y * e[1].x};
        if (separated(normal)) return false;
        const Vec3 axes[3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
        for (const Vec3& edge : e) for (const Vec3& boxAxis : axes) {
            const Vec3 axis = {edge.y * boxAxis.z - edge.z * boxAxis.y, edge.z * boxAxis.x - edge.x * boxAxis.z, edge.x * boxAxis.y - edge.y * boxAxis.x};
            if (separated(axis)) return false;
        }
        return true;
    }
};

enum class Axis : std::uint8_t { X, Y, Z };
enum class Direction : std::uint8_t { Negative, Positive };

struct OctreeNode {
    using ChildCode = std::uint8_t;
    static constexpr ChildCode X = 1u, Y = 2u, Z = 4u;
    AABB bounds{};
    OctreeNode* parent = nullptr;
    std::array<std::unique_ptr<OctreeNode>, 8> children{};
    ChildCode localMorton = 0;
    std::uint32_t depth = 0;
    bool occupied = false;
    std::vector<Triangle> triangles;
    bool terminal() const { return children[0] == nullptr; }
    static ChildCode mask(Axis axis) { return axis == Axis::X ? X : axis == Axis::Y ? Y : Z; }
    OctreeNode* getNeighbor(Axis axis, Direction direction) const;
    void getNeighbors(Axis axis, Direction direction, std::vector<OctreeNode*>& out) const;
};

class OcclusionOctree {
public:
    OcclusionOctree(AABB bounds, std::size_t maxDepth = 8, std::size_t maxTrianglesPerCell = 1);
    OctreeNode* root() { return root_.get(); }
    const OctreeNode* root() const { return root_.get(); }
    void insert(const Triangle& triangle);
    void insert(const std::vector<Triangle>& triangles);
    void leaves(std::vector<OctreeNode*>& out);
    void leaves(std::vector<const OctreeNode*>& out) const;
    std::vector<OctreeNode*> leaves();
    std::size_t occupiedLeafCount() const;
private:
    static AABB childBounds(const AABB& b, std::uint8_t code);
    void insertTriangle(OctreeNode* node, const Triangle& triangle);
    void subdivide(OctreeNode* node);
    static void refreshOccupied(OctreeNode* node);
    static void collectLeaves(OctreeNode* node, std::vector<OctreeNode*>& out);
    static void collectLeaves(const OctreeNode* node, std::vector<const OctreeNode*>& out);
    static std::size_t countOccupied(const OctreeNode* node);
    std::unique_ptr<OctreeNode> root_;
    std::size_t maxDepth_;
    std::size_t maxTrianglesPerCell_;
};

} // namespace openu
