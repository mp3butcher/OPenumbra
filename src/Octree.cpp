
#include "openu/Octree.hpp"
#include <assert.h>
using namespace openu;

// Writes the octree to a binary file
bool CompiledOctreeSerializer::saveToFile(const CompiledOctree& octree, const std::string& filepath) {
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
        // out.write(reinterpret_cast<const char*>(&node.bounds), sizeof(node.bounds));
        // out.write(reinterpret_cast<const char*>(node.children.data()), sizeof(node.children));
        // out.write(reinterpret_cast<const char*>(node.neighborBegin.data()), sizeof(node.neighborBegin));
        // out.write(reinterpret_cast<const char*>(node.neighborCount.data()), sizeof(node.neighborCount));
        // out.write(reinterpret_cast<const char*>(&node.occupied), sizeof(node.occupied));
        // out.write(reinterpret_cast<const char*>(&node.trianglesbounds), sizeof(node.trianglesbounds));
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
bool CompiledOctreeSerializer::loadFromFile(CompiledOctree& octree, const std::string& filepath) {
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
        // in.read(reinterpret_cast<char*>(&node.bounds), sizeof(node.bounds));
        // in.read(reinterpret_cast<char*>(node.children.data()), sizeof(node.children));
        // in.read(reinterpret_cast<char*>(node.neighborBegin.data()), sizeof(node.neighborBegin));
        // in.read(reinterpret_cast<char*>(node.neighborCount.data()), sizeof(node.neighborCount));
        // in.read(reinterpret_cast<char*>(&node.occupied), sizeof(node.occupied));
        //  in.read(reinterpret_cast<char*>(&node.trianglesbounds), sizeof(node.trianglesbounds));
    }

    // 5. Read Neighbor Storage
    if (storageSize > 0) {
        in.read(reinterpret_cast<char*>(octree.neighborStorage_.data()),
                storageSize * sizeof(NodeId));
    }
    in.close();
    return in.good();
}


unsigned int OcclusionOctree::floodFillThenErode(OctreeNode* start)
{
    std::queue<OctreeNode*> queue,queueempties;
    std::unordered_set<OctreeNode*> visited;
    std::unordered_set<OctreeNode*> frontierOccupied;
    std::vector<OctreeNode*> region;
    std::vector<OctreeNode*> missed;

    queueempties.push(start);
    visited.insert(start);
    region.push_back(start);

    auto leav=this->leaves();
    std::vector<OctreeNode*> oqps;
    for(auto &l:leav)if(l->occupied)oqps.push_back(l);

    // Flood fill only through unoccupied leaves
    while (!queueempties.empty()) {
        OctreeNode* current = queueempties.front();
        queueempties.pop();

        for (std::size_t face = 0; face < 6; ++face) {
            Axis axis = (face < 2) ? Axis::X : (face < 4) ? Axis::Y : Axis::Z;
            Direction dir = (face % 2 == 0) ? Direction::Negative : Direction::Positive;

            std::vector<const OctreeNode*> neighbors;
            current->getNeighbors(axis, dir, neighbors);

            for (const OctreeNode* n : neighbors) {
                if (!n) continue;

                OctreeNode* ptr = const_cast<OctreeNode*>(n);

                if (visited.count(ptr)) {
                    continue;
                }

                visited.insert(ptr);

                if (ptr->occupied) {
                    // This occupied leaf is the frontier boundary of the region
                    frontierOccupied.insert(ptr);
                    continue;
                }

                // continue flood filling through unoccupied leaves
                queueempties.push(ptr);
                region.push_back(ptr);
            }
        }
    }

    //  return region; // exterior leaves

    // floodfill interior cells
    // seed are neighbors of frontier cells that aren't in region (exteriors cells)
    while(1)
    {
        for (const OctreeNode* oqp : frontierOccupied) {

            for (std::size_t face = 0; face < 6; ++face) {
                Axis axis = (face < 2) ? Axis::X : (face < 4) ? Axis::Y : Axis::Z;
                Direction dir = (face % 2 == 0) ? Direction::Negative : Direction::Positive;

                std::vector<const OctreeNode*> neighbors;
                oqp->getNeighbors(axis, dir, neighbors);

                for (const OctreeNode* oqpn : neighbors) {
                    if (!oqpn) continue;
                    OctreeNode* oqpnptr = const_cast<OctreeNode*>(oqpn);

                    if (visited.count(oqpnptr)) {
                        continue;
                    }

                    queue.push(oqpnptr);
                    visited.insert(oqpnptr);
                    oqpnptr->occupied = true; //fill interior

                    // Flood fill only through unoccupied leaves
                    while (!queue.empty()) {
                        OctreeNode* current = queue.front();
                        queue.pop();

                        for (std::size_t face = 0; face < 6; ++face) {
                            Axis axis = (face < 2) ? Axis::X : (face < 4) ? Axis::Y : Axis::Z;
                            Direction dir = (face % 2 == 0) ? Direction::Negative : Direction::Positive;

                            std::vector<const OctreeNode*> neighbors;
                            current->getNeighbors(axis, dir, neighbors);

                            for (const OctreeNode* n : neighbors) {
                                if (!n) continue;

                                OctreeNode* ptr = const_cast<OctreeNode*>(n);

                                if (visited.count(ptr)) {
                                    continue;
                                }
                                if(ptr->occupied){
                                    missed.push_back(ptr);
                                    //   frontierOccupied.insert(ptr);
                                    continue;
                                }
                                visited.insert(ptr);

                                ptr->occupied = true; //fill interior
                                // continue flood filling through interior occupied leaves
                                queue.push(ptr);

                            }
                        }
                    }

                }

            }
        }
        for(auto ptr:missed)
        {
            //not reached during emptyfill so relaunch it with ptr neighbors as seed;
            frontierOccupied.insert(ptr);
            std::vector<const OctreeNode*> neighborb;
            ptr->getNeighbors( Axis::X, Direction::Negative, neighborb);
            ptr->getNeighbors( Axis::X, Direction::Positive, neighborb);
            ptr->getNeighbors( Axis::Y, Direction::Negative, neighborb);
            ptr->getNeighbors( Axis::Y, Direction::Positive, neighborb);
            ptr->getNeighbors( Axis::Z, Direction::Negative, neighborb);
            ptr->getNeighbors( Axis::Z, Direction::Positive, neighborb);
            for( auto ne :neighborb){

                if(!ne->occupied)
                {
                    auto fok=const_cast<OctreeNode*>(ne);
                    if (visited.count(fok))   continue;
                    queueempties.push(fok);
                    visited.insert(fok);
                    region.push_back(fok);
                }
            }


            // Flood fill only through unoccupied leaves
            while (!queueempties.empty()) {
                OctreeNode* current2 = queueempties.front();
                queueempties.pop();

                for (std::size_t face2 = 0; face2 < 6; ++face2) {
                    Axis axis2 = (face2 < 2) ? Axis::X : (face2 < 4) ? Axis::Y : Axis::Z;
                    Direction dir2 = (face2 % 2 == 0) ? Direction::Negative : Direction::Positive;

                    std::vector<const OctreeNode*> neighborsc;
                    current2->getNeighbors(axis2, dir2, neighborsc);

                    for (const OctreeNode* nc : neighborsc) {
                        if (!nc) continue;

                        OctreeNode* ptr2 = const_cast<OctreeNode*>(nc);

                        if (visited.count(ptr2)) {
                            continue;
                        }
                        if (ptr2->occupied) {
                            // This occupied leaf is the frontier boundary of the region
                            frontierOccupied.insert(ptr2);
                            continue;
                        }
                        visited.insert(ptr2);

                        // continue flood filling through unoccupied leaves
                        queueempties.push(ptr2);
                        region.push_back(ptr2);
                    }
                }
            }
        }
        if(missed.empty())break;
        missed.clear();
    }

    //debug

    if(oqps.size()!=frontierOccupied.size())
        oqps.size();

    // finally erode frontierOccupied
    for ( OctreeNode* oqp : oqps)
        oqp->occupied = false;



    return region.size();
}
/*
unsigned int OcclusionOctree::floodFillThenErode(OctreeNode* start)
{
    if (!start) return 0;

    auto leav = this->leaves();

    // 1. Gather the initial shell strictly to erase it later
    std::vector<OctreeNode*> originalOccupied;
    for (auto& l : leav) {
        if (l->occupied) originalOccupied.push_back(l);
    }

    std::unordered_set<OctreeNode*> exteriorCells;
    std::queue<OctreeNode*> queueEmpties;

    // 2. MULTI-SEED INITIALIZATION: Find all un-occluded boundary leaves
    // We treat the absolute outer skin of the Octree as the definitive source of "outside world"
    for (auto& leaf : leav) {
        if (leaf->occupied) continue;

        // Check if this empty leaf touches the absolute boundary of the Octree volume
        bool touchesWorldBorder = false;
        for (std::size_t face = 0; face < 6; ++face) {
            Axis axis = (face < 2) ? Axis::X : (face < 4) ? Axis::Y : Axis::Z;
            Direction dir = (face % 2 == 0) ? Direction::Negative : Direction::Positive;

            std::vector<const OctreeNode*> neighbors;
            leaf->getNeighbors(axis, dir, neighbors);

            if (neighbors.empty()) { // No neighbor means it's the edge of the world
                touchesWorldBorder = true;
                break;
            }
        }

        // If it's on the border, or it's our explicit camera start node, it's guaranteed Exterior
        if (touchesWorldBorder || leaf == start) {
            exteriorCells.insert(leaf);
            queueEmpties.push(leaf);
        }
    }

    // 3. Flood fill the exterior through empty space only
    while (!queueEmpties.empty()) {
        OctreeNode* current = queueEmpties.front();
        queueEmpties.pop();

        for (std::size_t face = 0; face < 6; ++face) {
            Axis axis = (face < 2) ? Axis::X : (face < 4) ? Axis::Y : Axis::Z;
            Direction dir = (face % 2 == 0) ? Direction::Negative : Direction::Positive;

            std::vector<const OctreeNode*> neighbors;
            current->getNeighbors(axis, dir, neighbors);

            for (const OctreeNode* n : neighbors) {
                if (!n) continue;
                OctreeNode* ptr = const_cast<OctreeNode*>(n);

                if (ptr->occupied || exteriorCells.count(ptr)) continue;

                exteriorCells.insert(ptr);
                queueEmpties.push(ptr);
            }
        }
    }

    // 4. SOLIDIFICATION: Every air node not reached by the exterior flood fill
    // is a true inner cavity of an object. We fill it.
    for (auto& leaf : leav) {
        if (!leaf->occupied && exteriorCells.count(leaf) == 0) {
            leaf->occupied = true;
        }
    }

    // 5. THE CRITICAL STEP FOR YOUR PIPELINE: Complete Erase of Original Shell
    // We completely empty out the initial conservative shell, leaving ONLY
    // the newly solidified interior core cores.
    for (OctreeNode* shellNode : originalOccupied) {
        shellNode->occupied = false;
    }

    return static_cast<unsigned int>(exteriorCells.size());
}
*/
