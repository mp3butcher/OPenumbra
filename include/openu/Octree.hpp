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

    void insert(const Triangle& triangle) { insertTriangle(root_.get(), triangle); }
    void insert(const std::vector<Triangle>& triangles) { for (const auto& t : triangles) insert(t); }

    void leaves(std::vector<OctreeNode*>& out) { collectLeaves(root_.get(), out); }
    std::vector<OctreeNode*> leaves() { std::vector<OctreeNode*> out; leaves(out); return out; }

    std::size_t occupiedLeafCount() const { return countOccupied(root_.get()); }

    CompiledOctree* compile() const;

private:
    static AABB childBounds(const AABB& b, std::uint8_t code) {
        const Vec3 c = b.center();
        return {{(code & OctreeNode::X) ? c.x : b.min.x, (code & OctreeNode::Y) ? c.y : b.min.y, (code & OctreeNode::Z) ? c.z : b.min.z},
                {(code & OctreeNode::X) ? b.max.x : c.x, (code & OctreeNode::Y) ? b.max.y : c.y, (code & OctreeNode::Z) ? b.max.z : c.z}};
    }

    void insertTriangle(OctreeNode* node, const Triangle& triangle) {
        if (!node->bounds.intersectsTriangle(triangle)) return;
        if (node->terminal()) {
            node->occupied = true;
            if (node->depth < maxDepth_ ) subdivide(node ,triangle);
            return;
        }
        for (auto& child : node->children) insertTriangle(child.get(), triangle);
        refreshOccupied(node);
    }

    void subdivide(OctreeNode* node,const Triangle& triangle) {
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

struct CompiledNode {
    AABB bounds{};
    std::array<NodeId, 8> children{};
    std::array<std::uint32_t, 6> neighborBegin{};
    std::array<std::uint16_t, 6> neighborCount{};
    bool occupied = false;
};

class CompiledOctree {
public:

friend class CompiledOctreeSerializer;
    CompiledOctree() = default;
    CompiledOctree(const CompiledOctree&) = delete;
    CompiledOctree& operator=(const CompiledOctree&) = delete;

    static CompiledOctree* build(const OcclusionOctree& tree);

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
};

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
        out.nodes_[i].occupied = allLeaves[i]->occupied;
        for (auto& child : out.nodes_[i].children) child = InvalidNode;
    }

    // Precompute a simple, conservative face-neighbor adjacency by reusing the
    // adaptive neighbor logic while compiling. This is O(N * faces), but the cost
    // is paid once at compile time, not every frame.
    std::vector<std::vector<NodeId>> faceNeighbors(allLeaves.size() * 6);
    for (std::size_t i = 0; i < allLeaves.size(); ++i) {
        const OctreeNode* leaf = allLeaves[i];
        const std::array<std::pair<Axis, Direction>, 6> faces = {
            std::make_pair(Axis::X, Direction::Negative),
            std::make_pair(Axis::X, Direction::Positive),
            std::make_pair(Axis::Y, Direction::Negative),
            std::make_pair(Axis::Y, Direction::Positive),
            std::make_pair(Axis::Z, Direction::Negative),
            std::make_pair(Axis::Z, Direction::Positive)
        };
        for (std::size_t f = 0; f < faces.size(); ++f) {
            std::vector<const OctreeNode*> neighbors;
            leaf->getNeighbors(faces[f].first, faces[f].second, neighbors);
            for (const OctreeNode* neighbor : neighbors) {
                if (!neighbor) continue;
                auto it = std::find(allLeaves.begin(), allLeaves.end(), neighbor);
                if (it == allLeaves.end()) continue;
                faceNeighbors[i * 6 + f].push_back(static_cast<NodeId>(std::distance(allLeaves.begin(), it)));
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
    
        // // This internal struct mirrors CompiledNode but excludes the runtime source pointer
        // // to ensure perfectly predictable binary layout and padding on disk.
        // struct DiskNode {
        //     AABB bounds;
        //     std::array<NodeId, 8> children;
        //     std::array<std::uint32_t, 6> neighborBegin;
        //     std::array<std::uint16_t, 6> neighborCount;
        //     bool occupied;
        // };
    
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
                out.write(reinterpret_cast<const char*>(&node), sizeof(CompiledNode));
            }
    
            // 4. Write Neighbor Storage
            if (storageSize > 0) {
                out.write(reinterpret_cast<const char*>(octree.neighborStorage().data()), 
                          storageSize * sizeof(NodeId));
            }
    
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
                in.read(reinterpret_cast<char*>(&node), sizeof(CompiledNode));
            }
    
            // 5. Read Neighbor Storage
            if (storageSize > 0) {
                in.read(reinterpret_cast<char*>(octree.neighborStorage_.data()), 
                        storageSize * sizeof(NodeId));
            }
    
            return in.good();
        }
    };
    
} // namespace openu
