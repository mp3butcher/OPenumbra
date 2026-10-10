#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <vector>
#include <fstream>

#include <unordered_map>
#include <unordered_set>
#include <memory.h>

namespace openu {

    constexpr float EPSILON = 1e-6f;
struct Vec3 {
    float x, y, z;
    Vec3(): x(0), y(0), z(0) {};
    Vec3(float x_, float y_, float z_) : x(x_), y(y_), z(z_) {}
    Vec3 operator+(const Vec3& v) const { return {x + v.x, y + v.y, z + v.z}; }
    Vec3 operator-(const Vec3& v) const { return {x - v.x, y - v.y, z - v.z}; }
    Vec3 operator*(float s) const { return {x * s, y * s, z * s}; }
    Vec3 operator/(float s) const { return {x / s, y / s, z / s}; }
    bool operator==(const Vec3& v) const {
        return x==v.x&&y==v.y&&z==v.z;
    } bool operator!=(const Vec3& v) const {
        return x!=v.x||y!=v.y||z!=v.z;
    }
    Vec3 cross(const Vec3& o) const {
        return { y * o.z - z * o.y, z * o.x - x * o.z, x * o.y - y * o.x };
    }
    float dot(const Vec3& o) const { return x * o.x + y * o.y + z * o.z; }

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
    AABB(): min({__FLT_MAX__,__FLT_MAX__,__FLT_MAX__}),max({-__FLT_MAX__,-__FLT_MAX__,-__FLT_MAX__}) {};
    AABB(Vec3 mi, Vec3 ma): min(mi), max(ma) {};
    Vec3 center() const { return (min + max) * 0.5f; }

    // bool operator<(const AABB& rhs) const
    // {if (min!=rhs.min) return min<rhs.min; else return max<rhs.max;}
    void expandBy(const Triangle&t){
        min.x = std::min(t.a.x, min.x); min.x = std::min(t.b.x, min.x); min.x = std::min(t.c.x, min.x);
        min.y = std::min(t.a.y, min.y); min.y = std::min(t.b.y, min.y); min.y = std::min(t.c.y, min.y);
        min.z = std::min(t.a.z, min.z); min.z = std::min(t.b.z, min.z); min.z = std::min(t.c.z, min.z);
        max.x = std::max(t.a.x, max.x); max.x = std::max(t.b.x, max.x); max.x = std::max(t.c.x, max.x);
        max.y = std::max(t.a.y, max.y); max.y = std::max(t.b.y, max.y); max.y = std::max(t.c.y, max.y);
        max.z = std::max(t.a.z, max.z); max.z = std::max(t.b.z, max.z); max.z = std::max(t.c.z, max.z);
    }

    // interscetion of 2 BB
    void inter(const AABB&t){
        min.x = std::max(t.min.x, min.x);
        min.y = std::max(t.min.y, min.y);
        min.z = std::max(t.min.z, min.z);
        max.x = std::min(t.max.x, max.x);
        max.y = std::min(t.max.y, max.y);
        max.z = std::min(t.max.z, max.z);
    }

    //
    void expandBy(const AABB&t){
        min.x = std::min(t.min.x, min.x);
        min.y = std::min(t.min.y, min.y);
        min.z = std::min(t.min.z, min.z);
        max.x = std::max(t.max.x, max.x);
        max.y = std::max(t.max.y, max.y);
        max.z = std::max(t.max.z, max.z);
    }
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

enum class Face : std::uint8_t {
    NegX,
    PosX,
    NegY,
    PosY,
    NegZ,
    PosZ
};

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

    AABB trianglesbounds{};
    //std::vector<Triangle> triangles;

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

    void getNeighbors(Axis axis, Direction direction, std::vector<const OctreeNode*>& out) const {
        const ChildCode bit = mask(axis);
        const bool positive = direction == Direction::Positive;
        const OctreeNode* pivot = this;

        // 1. Remontée pour trouver l'ancêtre commun (Pivot)
        while (pivot->parent != nullptr) {
            const bool onPositiveSide = (pivot->localMorton & bit) != 0;
            if (onPositiveSide != positive) break;
            pivot = pivot->parent;
        }
        if (pivot->parent == nullptr) return; // Pas de voisin (bord de l'octree)

        // 2. Passage à la branche voisine au niveau du pivot
        OctreeNode* neighborBranch = pivot->parent->children[pivot->localMorton ^ bit].get();
        if (!neighborBranch) return;

        // 3. Descente vers le niveau équivalent à "this"
        std::vector<ChildCode> suffix;
        for (const OctreeNode* n = this; n != pivot; n = n->parent) suffix.push_back(n->localMorton);
        std::reverse(suffix.begin(), suffix.end());

        OctreeNode* result = neighborBranch;
        for (ChildCode code : suffix) {
            if (result->terminal()) break;
            const ChildCode next = static_cast<ChildCode>((code & ~bit) | (positive ? 0 : bit));
            if (!result->children[next]) break;
            result = result->children[next].get();
        }

        // 4. Collecte de tous les sous-voisins (si le voisin est plus subdivisé)
        // La face adjacente correspond aux enfants qui ont (localMorton & bit) == (positive ? 0 : bit)
        const ChildCode targetTargetBitValue = positive ? 0 : bit;
        collectLeavesFacing(result, bit, targetTargetBitValue, out);
    }

private:
    static void collectLeavesFacing(const OctreeNode* node, ChildCode bit, ChildCode targetValue, std::vector<const OctreeNode*>& out) {
        if (!node) return;

        if (node->terminal()) {
            out.push_back(node);
            return;
        }

        // On ne descend que dans les enfants qui touchent la face du nœud d'origine
        for (size_t i = 0; i < 8; ++i) {
            if (node->children[i] && ((static_cast<ChildCode>(i) & bit) == targetValue)) {
                collectLeavesFacing(node->children[i].get(), bit, targetValue, out);
            }
        }
    }
};

class CompiledOctree;

class OcclusionOctree {
public:
    OcclusionOctree(AABB bounds, std::size_t maxDepth = 8, std::size_t maxTrianglesPerCell = 1)
        : maxDepth_(maxDepth), maxTrianglesPerCell_(std::max<std::size_t>(0, maxTrianglesPerCell)) {
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

    void insert(const Triangle& trianglebounds) { insertTriangle(root_.get(), trianglebounds ); }
    //void insert(const std::vector<Triangle>& triangles) { for (const auto& t : triangles) insert(t); }

    void leaves(std::vector<OctreeNode*>& out) { collectLeaves(root_.get(), out); }
    std::vector<OctreeNode*> leaves() { std::vector<OctreeNode*> out; leaves(out); return out; }

    std::size_t occupiedLeafCount() const { return countOccupied(root_.get()); }

    CompiledOctree* compile() const;

private:
    static AABB childBounds(const AABB& b, std::uint8_t code) {
        const Vec3 c = b.center();
        return AABB(Vec3((code & OctreeNode::X) ? c.x : b.min.x, (code & OctreeNode::Y) ? c.y : b.min.y, (code & OctreeNode::Z) ? c.z : b.min.z),
        Vec3((code & OctreeNode::X) ? b.max.x : c.x, (code & OctreeNode::Y) ? b.max.y : c.y, (code & OctreeNode::Z) ? b.max.z : c.z));
        // return {{(code & OctreeNode::X) ? c.x : b.min.x, (code & OctreeNode::Y) ? c.y : b.min.y, (code & OctreeNode::Z) ? c.z : b.min.z},
        //         {(code & OctreeNode::X) ? b.max.x : c.x, (code & OctreeNode::Y) ? b.max.y : c.y, (code & OctreeNode::Z) ? b.max.z : c.z}};
    }

// Clips a segment against a single infinite plane defined by a component axis, a direction, and a coordinate value
bool clipAgainstPlane(Vec3& p0, Vec3& p1, int axis, bool isMaxPlane, float planeValue) {
    // Get the component values for the current axis (0=x, 1=y, 2=z)
    auto getComp = [](const Vec3& v, int a) { return (a == 0) ? v.x : ((a == 1) ? v.y : v.z); };

    float v0 = getComp(p0, axis);
    float v1 = getComp(p1, axis);

    // Determine if points are "inside" the valid half-space
    bool inside0 = isMaxPlane ? (v0 <= planeValue) : (v0 >= planeValue);
    bool inside1 = isMaxPlane ? (v1 <= planeValue) : (v1 >= planeValue);

    // Case 1: Both points outside -> Segment completely discarded
    if (!inside0 && !inside1) return false;

    // Case 2: Both points inside -> Segment completely kept unmodified
    if (inside0 && inside1) return true;

    // Case 3: One inside, one outside -> Compute intersection point
    float t = (planeValue - v0) / (v1 - v0);
    Vec3 intersection = p0 + (p1 - p0) * t;

    if (!inside0) {
        p0 = intersection; // p0 was outside, clip it to the plane boundary
    } else {
        p1 = intersection; // p1 was outside, clip it to the plane boundary
    }

    return true;
}

// Complete implementation using Sutherland-Hodgman to clip a segment against all 6 AABB planes
void clipSegmentAABB(Vec3 p0, Vec3 p1, const AABB& box, AABB& outBox, bool& found) {
    // Clip against the 6 planes sequentially: -X, +X, -Y, +Y, -Z, +Z
    if (!clipAgainstPlane(p0, p1, 0, false, box.min.x)) return;
    if (!clipAgainstPlane(p0, p1, 0, true,  box.max.x)) return;
    if (!clipAgainstPlane(p0, p1, 1, false, box.min.y)) return;
    if (!clipAgainstPlane(p0, p1, 1, true,  box.max.y)) return;
    if (!clipAgainstPlane(p0, p1, 2, false, box.min.z)) return;
    if (!clipAgainstPlane(p0, p1, 2, true,  box.max.z)) return;

    // If the segment survived all planes, the clipped endpoints are inside the AABB
    expandAABB(outBox, p0);
    expandAABB(outBox, p1);
    found = true;
}


struct IntersectionResult {
    bool intersects;
    AABB aabb;
};

// Helper to expand an AABB to include a point
void expandAABB(AABB& box, const Vec3& p) {
    box.min.x = std::min(box.min.x, p.x);
    box.min.y = std::min(box.min.y, p.y);
    box.min.z = std::min(box.min.z, p.z);
    box.max.x = std::max(box.max.x, p.x);
    box.max.y = std::max(box.max.y, p.y);
    box.max.z = std::max(box.max.z, p.z);
}

bool intersectSegmentTriangle(const Vec3& p0, const Vec3& p1, const Triangle& tri, Vec3& outIntersection) {
    Vec3 edge1 = tri.b - tri.a;
    Vec3 edge2 = tri.c - tri.a;
    Vec3 rayVector = p1 - p0;

    Vec3 h = rayVector.cross(edge2);
    float a = edge1.dot(h);

    if (std::abs(a) < EPSILON) return false; // Ray is parallel to the triangle

    float f = 1.0f / a;
    Vec3 s = p0 - tri.a;
    float u = f * s.dot(h);

    if (u < 0.0f || u > 1.0f) return false;

    Vec3 q = s.cross(edge1);
    float v = f * rayVector.dot(q);

    if (v < 0.0f || u + v > 1.0f) return false;

    float t = f * edge2.dot(q);

    // Check if the intersection point lies strictly within the segment length [0, 1]
    if (t >= 0.0f && t <= 1.0f) {
        outIntersection = p0 + rayVector * t;
        return true;
    }

    return false;
}
IntersectionResult computeAABBTriangleIntersection(const AABB& box, const Triangle& tri) {
    AABB resultBox = { {INFINITY, INFINITY, INFINITY}, {-INFINITY, -INFINITY, -INFINITY} };
    bool hasIntersection = false;

    // 1. Check if triangle vertices are inside the AABB

    if (box.contains(tri.a)) {
        expandAABB(resultBox, tri.a);
        hasIntersection = true;
    }
    if (box.contains(tri.b)) {
        expandAABB(resultBox, tri.b);
        hasIntersection = true;
    }
    if (box.contains(tri.c)) {
        expandAABB(resultBox, tri.c);
        hasIntersection = true;
    }


    // 2. Check if AABB vertices are inside the triangle (using barycentric coordinates or similar tests)
    // Add box corners if they lie inside the triangle's 2D projections / 3D volume.

    // 3. Test edges of triangle against box faces and vice versa
        Vec3 edgeStart = tri.a;
        Vec3 edgeEnd = tri.b;
        clipSegmentAABB(edgeStart, edgeEnd, box, resultBox, hasIntersection);

        edgeStart = tri.b;
        edgeEnd = tri.c;
        clipSegmentAABB(edgeStart, edgeEnd, box, resultBox, hasIntersection);

        edgeStart = tri.c;
        edgeEnd = tri.a;
        clipSegmentAABB(edgeStart, edgeEnd, box, resultBox, hasIntersection);
  // 3. Process AABB structural edges piercing through the Triangle face
    // Generate the 12 structural edges of the AABB
    Vec3 corners[8] = {
        {box.min.x, box.min.y, box.min.z}, {box.max.x, box.min.y, box.min.z},
        {box.max.x, box.max.y, box.min.z}, {box.min.x, box.max.y, box.min.z},
        {box.min.x, box.min.y, box.max.z}, {box.max.x, box.min.y, box.max.z},
        {box.max.x, box.max.y, box.max.z}, {box.min.x, box.max.y, box.max.z}
    };

    // Indices defining the 12 edges connecting the corners
    std::pair<int, int> boxEdges[12] = {
        {0,1}, {1,2}, {2,3}, {3,0}, // Bottom face
        {4,5}, {5,6}, {6,7}, {7,4}, // Top face
        {0,4}, {1,5}, {2,6}, {3,7}  // Pillars
    };

    for (const auto& edge : boxEdges) {
        Vec3 intersectPt;
        if (intersectSegmentTriangle(corners[edge.first], corners[edge.second], tri, intersectPt)) {
            expandAABB(resultBox, intersectPt);
            hasIntersection = true;
        }
    }
    if (!hasIntersection) {
        return {false, {}};
    }

    return {true, resultBox};
}
    void insertTriangle(OctreeNode* node, const Triangle& triangle) {
        if (!node->bounds.intersectsTriangle(triangle)) return;
        if (node->terminal()) {
            auto res = computeAABBTriangleIntersection(node->bounds, triangle);
            node->trianglesbounds.expandBy(res.aabb);
            if(!res.intersects||!node->trianglesbounds.intersects(node->bounds))
            node->occupied = true;
            node->occupied = true;
            if (node->depth < maxDepth_ ) subdivide(node ,triangle);
            return;
        }

        for (auto& child : node->children) insertTriangle(child.get(), triangle);
        refreshOccupied(node);
    }

    // void insertTriangleBounds(OctreeNode* node, const AABB& trianglebounds) {
    //     if (!node->bounds.intersects(trianglebounds)) return;
    //     if (node->terminal()) {
    //         node->trianglesbounds=trianglebounds;
    //         node->trianglesbounds.inter(node->bounds);
    //         node->occupied = true;
    //         if (node->depth < maxDepth_ ) subdivide(node ,node->trianglesbounds);
    //         return;
    //     }

    //     for (auto& child : node->children) insertTriangleBounds(child.get(), trianglebounds);
    //     refreshOccupied(node);
    // }

    void subdivide(OctreeNode* node,const Triangle& triangle) {
       // const auto b = std::move(node->trianglesbounds);
        for (std::uint8_t i = 0; i < 8; ++i) {
            auto child = std::make_unique<OctreeNode>();
            child->parent = node; child->localMorton = i; child->depth = node->depth + 1;
            child->bounds = childBounds(node->bounds, i);
            node->children[i] = std::move(child);
        }

        for (auto& child : node->children) insertTriangle(child.get(), triangle);
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

using NodeId = std::uint32_t;
static constexpr NodeId InvalidNode = std::numeric_limits<NodeId>::max();

struct  alignas(8) CompiledNode {
    AABB bounds;
    AABB trianglesbounds;
    std::array<NodeId, 8> children;
    std::array<std::uint32_t, 6> neighborBegin;
    std::array<std::uint16_t, 6> neighborCount;
    bool occupied;
     // Padding explicite pour boucher les 3 octets vides après le bool
     std::uint8_t padding1;
     std::uint8_t padding2;
     std::uint8_t padding3;

     // IMPORTANT : On dit au compilateur de garder le comportement trivial en mémoire
    // CompiledNode() = default;
};
static_assert(std::is_trivially_copyable<CompiledNode>::value,
    "Erreur : CompiledNode n'est plus trivialement copiable !");
static_assert(sizeof(CompiledNode) == 120,
    "Erreur : La taille de la structure a changé de manière inattendue !");

class CompiledOctree {
public:

friend class CompiledOctreeSerializer;
    CompiledOctree() = default;
    CompiledOctree(const CompiledOctree&) = delete;
    CompiledOctree& operator=(const CompiledOctree&) = delete;

    static CompiledOctree* build(const OcclusionOctree& tree);
    static CompiledOctree* buildLevelDown(const CompiledOctree& fineLevel) ;

    std::size_t nodeCount() const { return nodes_.size(); }
    const CompiledNode& node(NodeId id) const { return nodes_.at(id); }
    const std::vector<NodeId>& neighborStorage() const { return neighborStorage_; }

    NodeId locate(const Vec3& point) const {
        if (nodes_.empty()) return InvalidNode;
        NodeId current = 0;
        while (current < nodes_.size()) {
            const CompiledNode& node = nodes_[current];
            if (node.children[0] == InvalidNode) return current;
            const Vec3 center = node.bounds.center();
            std::uint8_t code = 0;
            if (point.x >= center.x) code |= 1u;
            if (point.y >= center.y) code |= 2u;
            if (point.z >= center.z) code |= 4u;
            current = node.children[code];
            if (current == InvalidNode) return InvalidNode;
        }
        return InvalidNode;
    }

private:
    std::vector<CompiledNode> nodes_;
    std::vector<NodeId> neighborStorage_;
    static constexpr std::array<std::pair<Axis, Direction>, 6> faces = {
        std::make_pair(Axis::X, Direction::Negative),
        std::make_pair(Axis::X, Direction::Positive),
        std::make_pair(Axis::Y, Direction::Negative),
        std::make_pair(Axis::Y, Direction::Positive),
        std::make_pair(Axis::Z, Direction::Negative),
        std::make_pair(Axis::Z, Direction::Positive)
    };
    static bool childTouchesParentFace(const AABB& child, const AABB& parent, std::size_t faceIndex) {
        const float eps = 1e-5f;
        switch (faceIndex) {
            case 0: return std::abs(child.min.x - parent.min.x) < eps; // Face X-
            case 1: return std::abs(child.max.x - parent.max.x) < eps; // Face X+
            case 2: return std::abs(child.min.y - parent.min.y) < eps; // Face Y-
            case 3: return std::abs(child.max.y - parent.max.y) < eps; // Face Y+
            case 4: return std::abs(child.min.z - parent.min.z) < eps; // Face Z-
            case 5: return std::abs(child.max.z - parent.max.z) < eps; // Face Z+
            default: return false;
        }
    }
};



// Structure temporaire pour regrouper les données pendant la fusion
struct ParentCandidate {
    AABB bounds;
    bool occupied = false;
    std::vector<NodeId> childrenIds; // Les indices (0 à 7) des enfants dans le niveau n
};
// 1. Définition du Hash pour vos AABB
struct BoundsHash {
    std::size_t operator()(const AABB& bounds) const {
        // On utilise le centre de la boîte, qui est unique pour chaque parent
        Vec3 c = bounds.center();

        // Extraction des bits des floats pour un hachage binaire pur
        std::uint32_t ix, iy, iz;
        memcpy(&ix, &c.x, sizeof(float));
        memcpy(&iy, &c.y, sizeof(float));
        memcpy(&iz, &c.z, sizeof(float));

        // Combinaison de hachage de type "std::hash_combine" (façon Boost)
        std::size_t seed = 0;
        auto hash_combine = [&seed](std::uint32_t val) {
            seed ^= std::hash<std::uint32_t>{}(val) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
        };

        hash_combine(ix);
        hash_combine(iy);
        hash_combine(iz);

        return seed;
    }
};

// 2. Définition du comparateur d'égalité (indispensable pour std::unordered_map)
struct BoundsEqual {
    bool operator()(const AABB& lhs, const AABB& rhs) const {
        // Une petite tolérance (epsilon) est recommandée pour éviter les écarts de float
        const float eps = 1e-5f;
        return std::abs(lhs.min.x - rhs.min.x) < eps &&
               std::abs(lhs.min.y - rhs.min.y) < eps &&
               std::abs(lhs.min.z - rhs.min.z) < eps &&
               std::abs(lhs.max.x - rhs.max.x) < eps &&
               std::abs(lhs.max.y - rhs.max.y) < eps &&
               std::abs(lhs.max.z - rhs.max.z) < eps;
    }
};
// Fonction pour générer le niveau inférieur (plus grossier)
inline CompiledOctree* CompiledOctree::buildLevelDown(const CompiledOctree& fineLevel) {
    CompiledOctree* coarseLevel = new CompiledOctree();

    // Hash pour regrouper les nœuds enfants par leur boîte englobante parente
    // Clé : Centre de la boîte parente (ou une clé unique basée sur les coordonnées)
    // Pour l'exemple, on utilise un système de clé basé sur la boîte parente simulée
// Passez une référence vers la boîte englobante de la racine de l'arbre global (ou ses coordonnées min)
Vec3 rootMin={__FLT_MAX__,__FLT_MAX__,__FLT_MAX__} ;
for(auto n: fineLevel.nodes_){
    const AABB& b=n.bounds;
    rootMin.x = std::min(b.min.x,rootMin.x);
    rootMin.y = std::min(b.min.y,rootMin.y);
    rootMin.z = std::min(b.min.z,rootMin.z);
}
auto getParentBounds = [rootMin](const AABB& childBounds) {
    AABB parent;

    // 1. Taille exacte de l'enfant et du parent
    Vec3 childSize = childBounds.max - childBounds.min;
    Vec3 parentSize = childSize * 2.0f;

    // 2. Travailler en coordonnées locales par rapport au "min" de la racine
    Vec3 localCenter = childBounds.center() - rootMin;

    // 3. Aligner sur la grille locale du parent, puis réappliquer le décalage de la racine
    parent.min.x = std::floor(localCenter.x / parentSize.x) * parentSize.x + rootMin.x;
    parent.min.y = std::floor(localCenter.y / parentSize.y) * parentSize.y + rootMin.y;
    parent.min.z = std::floor(localCenter.z / parentSize.z) * parentSize.z + rootMin.z;

    // 4. Calcul du max
    parent.max = parent.min + parentSize;

    return parent;
};

    // 1. Regrouper les nœuds fins par parent géométrique
    // Associe les coordonnées de la bounding box parente à son candidat
    // (Note : Dans une structure de production, utilisez une clé entière/hash unique pour vos Bounds)
    std::vector<ParentCandidate> parents;
    std::unordered_map<AABB, std::size_t,BoundsHash, BoundsEqual> boundsToParentIdx;

    // Tableau pour savoir à quel parent appartient chaque nœud fin
    std::vector<std::size_t> childToParentMap(fineLevel.nodes_.size());

    for (NodeId i = 0; i < fineLevel.nodes_.size(); ++i) {
        AABB pBounds = getParentBounds(fineLevel.nodes_[i].bounds);

        std::unordered_map<AABB, std::size_t,BoundsHash, BoundsEqual>::iterator  it = boundsToParentIdx.find(pBounds);
        if (it == boundsToParentIdx.end()) {
            std::size_t newIdx = parents.size();
            boundsToParentIdx[pBounds] = newIdx;

            ParentCandidate p;
            p.bounds = pBounds;
            p.occupied = fineLevel.nodes_[i].occupied;
            p.childrenIds.push_back(i);
            parents.push_back(p);

            childToParentMap[i] = newIdx;
        } else {
            std::size_t pIdx = it->second;
            parents[pIdx].occupied |= fineLevel.nodes_[i].occupied; // Logique "Conservative"
            parents[pIdx].childrenIds.push_back(i);

            childToParentMap[i] = pIdx;
        }
    }

    // 2. Allouer les nœuds du niveau grossier
    coarseLevel->nodes_.resize(parents.size());
    for (std::size_t i = 0; i < parents.size(); ++i) {
        coarseLevel->nodes_[i].bounds = parents[i].bounds;
        coarseLevel->nodes_[i].occupied = parents[i].occupied;
        for (auto& child : coarseLevel->nodes_[i].children) child = InvalidNode;
    }

    // 3. Fusionner les voisins géométriquement sans appeler l'arbre d'origine
    std::vector<std::vector<NodeId>> coarseFaceNeighbors(parents.size() * 6);

    for (std::size_t i = 0; i < parents.size(); ++i) {
        const auto& pCandidate = parents[i];

        for (std::size_t f = 0; f < 6; ++f) {
            // Utiliser un set pour éviter les doublons de voisins (plusieurs enfants peuvent partager le même voisin)
            std::unordered_set<NodeId> uniqueCoarseNeighbors;

            for (NodeId childId : pCandidate.childrenIds) {
                // Étape CRITIQUE : Vérifier si cet enfant touche la face 'f' du parent.
                // (Si l'enfant est à l'intérieur de l'octree et ne touche pas la face externe 'f',
                // ses voisins sur cette face sont ses frères, donc internes au parent -> On les ignore).
                if (!childTouchesParentFace(fineLevel.nodes_[childId].bounds, pCandidate.bounds, f)) {
                    continue;
                }

                // Parcourir les voisins de l'enfant au niveau fin
                std::uint32_t start = fineLevel.nodes_[childId].neighborBegin[f];
                std::uint32_t count = fineLevel.nodes_[childId].neighborCount[f];

                for (std::uint32_t n = 0; n < count; ++n) {
                    NodeId fineNeighborId = fineLevel.neighborStorage_[start + n];

                    // Trouver le parent de ce voisin fin
                    std::size_t coarseNeighborIdx = childToParentMap[fineNeighborId];

                    // Si le parent du voisin n'est pas le nœud actuel lui-même, c'est un voisin valide !
                    if (coarseNeighborIdx != i) {
                        uniqueCoarseNeighbors.insert(static_cast<NodeId>(coarseNeighborIdx));
                    }
                }
            }

            // Ajouter les voisins uniques trouvés pour cette face
            auto& fNeighbors = coarseFaceNeighbors[i * 6 + f];
            for (NodeId cNeigh : uniqueCoarseNeighbors) {
                fNeighbors.push_back(cNeigh);
            }
        }
    }

    // 4. Copie finale dans le stockage linéaire du niveau grossier
    std::size_t totalNeighbors = 0;
    for (auto& v : coarseFaceNeighbors) totalNeighbors += v.size();
    coarseLevel->neighborStorage_.reserve(totalNeighbors);

    for (std::size_t i = 0; i < parents.size(); ++i) {
        for (std::size_t f = 0; f < 6; ++f) {
            coarseLevel->nodes_[i].neighborBegin[f] = static_cast<std::uint32_t>(coarseLevel->neighborStorage_.size());
            auto& neighbors = coarseFaceNeighbors[i * 6 + f];
            coarseLevel->nodes_[i].neighborCount[f] = static_cast<std::uint32_t>(neighbors.size());

            for (NodeId neighborId : neighbors) {
                coarseLevel->neighborStorage_.push_back(neighborId);
            }
        }
    }

    return coarseLevel;
}

inline CompiledOctree* CompiledOctree::build(const OcclusionOctree& tree) {
    CompiledOctree *outp = new CompiledOctree();
    CompiledOctree &out = *outp;
    std::vector<const OctreeNode*> leaves;
    tree.root()->getNeighbors(Axis::X, Direction::Positive, leaves);
    // Intentionally keep the compile path simple and deterministic: gather the
    // terminal cells in-tree and assign contiguous IDs to them.
    std::vector<const OctreeNode*> allLeaves;
    std::vector<const OctreeNode*> pending;
    pending.push_back(tree.root());
    while (!pending.empty()) {
        const OctreeNode* node = pending.back();
        pending.pop_back();
        if (!node) continue;
        if (node->terminal()) {
            allLeaves.push_back(node);
            continue;
        }
        for (auto& child : node->children) pending.push_back(child.get());
    }

    out.nodes_.resize(allLeaves.size());
    for (std::size_t i = 0; i < allLeaves.size(); ++i) {
        out.nodes_[i].bounds = allLeaves[i]->bounds;
        out.nodes_[i].trianglesbounds = allLeaves[i]->trianglesbounds;
        out.nodes_[i].occupied = allLeaves[i]->occupied;
        for (auto& child : out.nodes_[i].children) child = InvalidNode;
    }
    std::unordered_map<const OctreeNode*, NodeId> nodeToIndex;
    nodeToIndex.reserve(allLeaves.size());
    for (std::size_t i = 0; i < allLeaves.size(); ++i) {
        nodeToIndex[allLeaves[i]] = static_cast<NodeId>(i);
    }
    // Precompute a simple, conservative face-neighbor adjacency by reusing the
    // adaptive neighbor logic while compiling. This is O(N * faces), but the cost
    // is paid once at compile time, not every frame.
    std::vector<std::vector<NodeId>> faceNeighbors(allLeaves.size() * 6);
    for (std::size_t i = 0; i < allLeaves.size(); ++i) {
        const OctreeNode* leaf = allLeaves[i];
        for (std::size_t f = 0; f < faces.size(); ++f) {
            std::vector<const OctreeNode*> neighbors;
            leaf->getNeighbors(faces[f].first, faces[f].second, neighbors);
            for (const OctreeNode* neighbor : neighbors) {
                if (!neighbor) continue;
                auto it = nodeToIndex.find(neighbor);//std::find(allLeaves.begin(), allLeaves.end(), neighbor);
                if (it == nodeToIndex.end()) continue;
                faceNeighbors[i * 6 + f].push_back(it->second);//static_cast<NodeId>(std::distance(allLeaves.begin(), it)));
            }
        }
    }

    std::size_t totalNeighbors = 0;
    for (auto& v : faceNeighbors) totalNeighbors += v.size();
    out.neighborStorage_.reserve(totalNeighbors);
    for (std::size_t i = 0; i < allLeaves.size(); ++i) {
        for (std::size_t f = 0; f < 6; ++f) {
            out.nodes_[i].neighborBegin[f] = static_cast<std::uint32_t>(out.neighborStorage_.size());
            out.nodes_[i].neighborCount[f] = static_cast<std::uint16_t>(faceNeighbors[i * 6 + f].size());
            out.neighborStorage_.insert(out.neighborStorage_.end(),
                                       faceNeighbors[i * 6 + f].begin(),
                                       faceNeighbors[i * 6 + f].end());
        }
    }

    return outp;
}

inline CompiledOctree * OcclusionOctree::compile() const {
    return CompiledOctree::build(*this);
}

class CompiledOctreeSerializer {
    private:
        static constexpr std::uint32_t MagicHeader = 0x3854434F; // "OCT8" in ASCII
        static constexpr std::uint32_t FormatVersion = 1;

    public:
        // Writes the octree to a binary file
        static bool saveToFile(const CompiledOctree& octree, const std::string& filepath) {
            std::ofstream out(filepath, std::ios::binary);
            if (!out.is_open()) return false;

            // 1. Write Header
            out.write(reinterpret_cast<const char*>(&MagicHeader), sizeof(MagicHeader));
            out.write(reinterpret_cast<const char*>(&FormatVersion), sizeof(FormatVersion));

            // 2. Write Metadata (Sizes)
            std::uint64_t nodeCount = octree.nodeCount();
            std::uint64_t storageSize = octree.neighborStorage().size();
            out.write(reinterpret_cast<const char*>(&nodeCount), sizeof(nodeCount));
            out.write(reinterpret_cast<const char*>(&storageSize), sizeof(storageSize));

            // 3. Write Nodes (skipping runtime pointers)
            for (std::size_t i = 0; i < nodeCount; ++i) {
                const auto& node = octree.node(static_cast<NodeId>(i));
//                out.write(reinterpret_cast<const char*>(&node), sizeof(CompiledNode));
                out.write(reinterpret_cast<const char*>(&node.bounds), sizeof(node.bounds));
                out.write(reinterpret_cast<const char*>(node.children.data()), sizeof(node.children));
                out.write(reinterpret_cast<const char*>(node.neighborBegin.data()), sizeof(node.neighborBegin));
                out.write(reinterpret_cast<const char*>(node.neighborCount.data()), sizeof(node.neighborCount));
                out.write(reinterpret_cast<const char*>(&node.occupied), sizeof(node.occupied));
                out.write(reinterpret_cast<const char*>(&node.trianglesbounds), sizeof(node.trianglesbounds));
            }

            // 4. Write Neighbor Storage
            if (storageSize > 0) {
                out.write(reinterpret_cast<const char*>(octree.neighborStorage().data()),
                          storageSize * sizeof(NodeId));
            }
    out.close();
            return out.good();
        }

        // Loads the octree from a binary file into an existing instance
        // Note: Requires modifying CompiledOctree or adding a friendship to allow data population.
        static bool loadFromFile(CompiledOctree& octree, const std::string& filepath) {
            std::ifstream in(filepath, std::ios::binary);
            if (!in.is_open()) return false;

            // 1. Read and verify Header
            std::uint32_t magic = 0;
            std::uint32_t version = 0;
            in.read(reinterpret_cast<char*>(&magic), sizeof(magic));
            in.read(reinterpret_cast<char*>(&version), sizeof(version));

            if (magic != MagicHeader || version != FormatVersion) {
                return false; // Invalid file format or version mismatch
            }

            // 2. Read Metadata
            std::uint64_t nodeCount = 0;
            std::uint64_t storageSize = 0;
            in.read(reinterpret_cast<char*>(&nodeCount), sizeof(nodeCount));
            in.read(reinterpret_cast<char*>(&storageSize), sizeof(storageSize));

            // 3. Access internal vectors (Assumes Serializer is a 'friend' class of CompiledOctree)
            octree.nodes_.resize(nodeCount);
            octree.neighborStorage_.resize(storageSize);

            // 4. Read and reconstruct Nodes
            for (std::size_t i = 0; i < nodeCount; ++i) {
                auto& node = octree.nodes_[i];
               //  in.read(reinterpret_cast<char*>(&node), sizeof(CompiledNode));
                in.read(reinterpret_cast<char*>(&node.bounds), sizeof(node.bounds));
                in.read(reinterpret_cast<char*>(node.children.data()), sizeof(node.children));
                in.read(reinterpret_cast<char*>(node.neighborBegin.data()), sizeof(node.neighborBegin));
                in.read(reinterpret_cast<char*>(node.neighborCount.data()), sizeof(node.neighborCount));
                in.read(reinterpret_cast<char*>(&node.occupied), sizeof(node.occupied));
                in.read(reinterpret_cast<char*>(&node.trianglesbounds), sizeof(node.trianglesbounds));
            }

            // 5. Read Neighbor Storage
            if (storageSize > 0) {
                in.read(reinterpret_cast<char*>(octree.neighborStorage_.data()),
                        storageSize * sizeof(NodeId));
            }
    in.close();
            return in.good();
        }
    };

} // namespace openu
