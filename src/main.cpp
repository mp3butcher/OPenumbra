#include <cassert>
#include <iostream>
#include <vector>
#include "openu/Octree.hpp"

using namespace openu;

int main() {
    OcclusionOctree tree({{-8, -8, -8}, {8, 8, 8}}, 4, 1);
    tree.insert(Triangle{{-1, -1, 0}, {1, -1, 0}, {0, 1, 0}});
    tree.insert(Triangle{{4, 4, 4}, {5, 4, 4}, {4, 5, 4}});

    auto leaves = tree.leaves();
    assert(!leaves.empty());
    assert(tree.occupiedLeafCount() > 0);

    // The lookup is parent/Morton based; it does not perform a global leaf scan.
    const OctreeNode* leaf = leaves.front();
    const OctreeNode* positiveX = leaf->getNeighbor(Axis::X, Direction::Positive);
    if (positiveX) {
        std::vector<OctreeNode*> all;
        leaf->getNeighbors(Axis::X, Direction::Positive, all);
        assert(!all.empty());
    }

    std::cout << "leaves=" << leaves.size() << " occupied=" << tree.occupiedLeafCount() << '\n';
    return 0;
}
