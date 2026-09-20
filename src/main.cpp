#include <iostream>
#include <vector>

#include "openu/Octree.hpp"

int main() {
    using namespace openu;

    const AABB world = {{-8.0f, -8.0f, -8.0f}, {8.0f, 8.0f, 8.0f}};
    OcclusionOctree octree(world, 6, 1);

    std::vector<Triangle> triangles = {
        Triangle{{-2.0f, -1.0f, -2.0f}, {2.0f, -1.0f, -2.0f}, {0.0f, 2.0f, -2.0f}},
        Triangle{{3.0f, 0.0f, 1.0f}, {5.0f, 0.0f, 2.0f}, {4.0f, 2.5f, 2.0f}},
        Triangle{{-6.0f, -2.0f, 3.0f}, {-3.0f, -2.0f, 1.0f}, {-4.5f, 2.0f, 4.0f}},
    };

    octree.insert(triangles);

    const auto leaves = octree.leaves();
    std::cout << "Leaf count: " << leaves.size() << '\n';
    std::cout << "Occupied leaf count: " << octree.occupiedLeafCount() << '\n';

    const AABB query = {{-2.0f, -2.0f, -2.0f}, {2.0f, 2.0f, 2.0f}};
    const auto hits = octree.query(query);
    std::cout << "Triangles overlapping query volume: " << hits.size() << '\n';

    // Example neighbor lookup on a non-root leaf.
    // This picks the positive-X neighbor of the first leaf if any.
    if (!leaves.empty()) {
        const auto* firstLeaf = leaves.front();
        auto* neighbor = firstLeaf->getNeighbor(Axis::X, Direction::Positive);
        std::cout << "Positive-X neighbor exists: " << (neighbor != nullptr ? "yes" : "no") << '\n';
    }

    return 0;
}
