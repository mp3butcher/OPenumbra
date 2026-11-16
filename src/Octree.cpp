
#include "openu/Octree.hpp"
#include <functional>
#include <assert.h>
using namespace openu;
#if 1
 bool OctreeNode::buildselfBoundGeometry(const uint8_t targetlevel, const uint8_t maxdepth) {
    // Reset geometry buffers
  //   boundpointsvert.clear();
  //   boundpointsind.clear();

    bool havenextlod = false;
    auto starti = boundpointsind.size();
    auto startv = boundpointsvert.size();
    // Precomputed CCW cube indices by faces (12 triangles, 36 indices)
    // Groups are: (X-,X+,Y-,Y+,Z-,Z+) as deduced from vertex layout below
    // Map the face index order used elsewhere (X-,X+,Y-,Y+,Z-,Z+) to the group index in boxIndices
    static const unsigned boxIndices[] = {
        0,4,2, 2,4,6,    // X-
        1,3,5, 3,7,5,    // X+
        0,1,4, 1,5,4,    // Y-
        2,6,3, 3,6,7,    // Y+
        0,2,1, 1,2,3,    // Z-
        4,5,6, 5,7,6     // Z+
    };


    // Vertex deduplication key (bit-exact)
    struct Key { std::uint32_t x, y, z; bool operator==(Key const& o) const noexcept { return x==o.x && y==o.y && z==o.z; } };
    struct KeyHash { std::size_t operator()(Key const& k) const noexcept {
            // cheap mix
            return (static_cast<std::size_t>(k.x) * 1664525u) ^ (static_cast<std::size_t>(k.y) * 22695477u) ^ (static_cast<std::size_t>(k.z) >> 1);
        }
    };
    std::unordered_map<Key, std::uint32_t, KeyHash> vertMap;
    vertMap.reserve(4096);

    //rebuild vertmap from existing
    for(auto &ind:boundpointsind){
        Key k;
        auto v = boundpointsvert[ind];
        memcpy(&k.x, &v.x, sizeof(float));
        memcpy(&k.y, &v.y, sizeof(float));
        memcpy(&k.z, &v.z, sizeof(float));
        vertMap[k]=ind;
    }

    auto addVertex = [&](const Vec3& v) -> std::uint32_t {
        Key k;
        static_assert(sizeof(float) == sizeof(std::uint32_t), "requires 32-bit float");
        memcpy(&k.x, &v.x, sizeof(float));
        memcpy(&k.y, &v.y, sizeof(float));
        memcpy(&k.z, &v.z, sizeof(float));
        auto it = vertMap.find(k);
        if (it != vertMap.end()) return it->second;
        std::uint32_t idx = static_cast<std::uint32_t>(boundpointsvert.size());
        boundpointsvert.push_back(v);
        vertMap.emplace(k, idx);
        return idx;
    };
    // Helper: collect terminal leaves intersecting 'query' (prune by AABB intersection)
    std::function<void(const OctreeNode*, const AABB&, std::vector<const OctreeNode*>&)> collectIntersecting;
    collectIntersecting = [&](const OctreeNode* node, const AABB& query, std::vector<const OctreeNode*>& out) {
        if (!node) return;
        if (!node->bounds.intersects(query)) return; // prune
        if (node->terminal()) {
            out.push_back(node);
            return;
        }
        for (const auto& c : node->children) collectIntersecting(c.get(), query, out);
    };
    // post-order traversal: children first so child geometry exists before parent aggregation

    auto process = [&](auto&& self, OctreeNode* node) -> void {
        // auto process = [&](OctreeNode* node) {
        if (!node) return;

        if (!node->terminal()) {
            // recurse into children first
            for (const auto& c : node->children) self(self,c.get());
            // Parent aggregation is implicit because we use a single global (deduped) vertex/index buffer.
            return;
        }

        // terminal leaf: only emit geometry for occupied leaves
        if (!node->occupied) return;
        havenextlod=havenextlod||node->depth==maxdepth;

        const AABB& b = node->bounds;

        // build cube 8 corners consistent with boxIndices mapping:
        // 0:(min,min,min), 1:(max,min,min), 2:(min,max,min), 3:(max,max,min),
        // 4:(min,min,max), 5:(max,min,max), 6:(min,max,max), 7:(max,max,max)
        Vec3 vCorner[8];
        vCorner[0] = {b.min.x, b.min.y, b.min.z};
        vCorner[1] = {b.max.x, b.min.y, b.min.z};
        vCorner[2] = {b.min.x, b.max.y, b.min.z};
        vCorner[3] = {b.max.x, b.max.y, b.min.z};
        vCorner[4] = {b.min.x, b.min.y, b.max.z};
        vCorner[5] = {b.max.x, b.min.y, b.max.z};
        vCorner[6] = {b.min.x, b.max.y, b.max.z};
        vCorner[7] = {b.max.x, b.max.y, b.max.z};


        // faces in the rest of the code use order: X-,X+,Y-,Y+,Z-,Z+
        const std::array<std::pair<Axis, Direction>, 6> faces = {
            std::make_pair(Axis::X, Direction::Negative),
            std::make_pair(Axis::X, Direction::Positive),
            std::make_pair(Axis::Y, Direction::Negative),
            std::make_pair(Axis::Y, Direction::Positive),
            std::make_pair(Axis::Z, Direction::Negative),
            std::make_pair(Axis::Z, Direction::Positive)
        };
        bool facedone=false;
        // for each face, collect neighbors and decide whether it's a boundary face  faces.size();
        for (std::size_t f = 0; f < 6; ++f) {
            // face slab thickness: tiny but proportional to node size to avoid FP misses w/ coarser neighbors
            // const float sizeX = b.max.x - b.min.x;
            // const float sizeY = b.max.y - b.min.y;
            // const float sizeZ = b.max.z - b.min.z;
            // const float eps = std::max({sizeX, sizeY, sizeZ}) * 1e-6f + 1e-1f;

            // AABB faceAABB;
            // switch (f) {
            // case 0: // NegX
            //     faceAABB.min = { b.min.x - eps, b.min.y+eps, b.min.z+eps };
            //     faceAABB.max = { b.max.x- eps , b.max.y-eps, b.max.z-eps };
            //     break;
            // case 1: // PosX
            //     faceAABB.min = { b.min.x +eps, b.min.y+eps, b.min.z+eps };
            //     faceAABB.max = { b.max.x + eps, b.max.y-eps, b.max.z-eps };
            //     break;
            // case 2: // NegY
            //     faceAABB.min = { b.min.x+eps, b.min.y - eps, b.min.z+eps };
            //     faceAABB.max = { b.max.x-eps, b.max.y-eps , b.max.z-eps };
            //     break;
            // case 3: // PosY
            //     faceAABB.min = { b.min.x+eps, b.min.y+eps , b.min.z+eps };
            //     faceAABB.max = { b.max.x-eps, b.max.y + eps, b.max.z -eps};
            //     break;
            // case 4: // NegZ
            //     faceAABB.min = { b.min.x+eps, b.min.y+eps, b.min.z - eps };
            //     faceAABB.max = { b.max.x-eps, b.max.y-eps, b.max.z-eps  };
            //     break;
            // case 5: // PosZ
            //     faceAABB.min = { b.min.x+eps, b.min.y+eps, b.min.z  +eps};
            //     faceAABB.max = { b.max.x-eps, b.max.y-eps, b.max.z + eps };
            //     break;
            // default:
            //     continue;
            // }

            std::vector<const OctreeNode*> neighbors;
            // auto root=this;
            // while(root->parent)
            //     root = root->parent;
            // collectIntersecting(root, faceAABB, neighbors);
            //collectIntersecting(node->getNeighbor(faces[f].first, faces[f].second), faceAABB, neighbors);
            node->getNeighbors(faces[f].first, faces[f].second, neighbors);

            bool anyNeighborOccupied = false;
            for (const OctreeNode* nb : neighbors) {
                if (nb && nb->occupied&&nb!=node) {
                    //force contour decision  to little ones
                    if (nb->depth > node->depth) {
                        continue;
                    }{ anyNeighborOccupied = true; break; }
                }
            }
            if (anyNeighborOccupied) continue; // internal face, skip
            facedone=true;
            // This face is on the boundary: emit its two triangles using precomputed CCW indices.
            const unsigned* triIdx = boxIndices + (f * 6); // 6 indices (2 triangles) per group

            for (int t = 0; t < 6; ++t) {
                unsigned cornerIdx = triIdx[t];
                Vec3 const& corner = vCorner[cornerIdx];
                std::uint32_t vid = addVertex(corner);
                // rely on implicit conversion when pushing into boundpointsind (value_type may be uint16 or uint32)
                boundpointsind.push_back(static_cast<decltype(boundpointsind)::value_type>(vid));
            }
        }
        if(!facedone)
            facedone=false;
    };

        process(process,this);

        return havenextlod;
}
#else
 bool OctreeNode ::buildselfBoundGeometry(const uint8_t targetlevel, const uint8_t maxdepth) {
     // Reset geometry buffers
     //   boundpointsvert.clear();
     //   boundpointsind.clear();
     //   uint8_t curlod=0, lodcount=maxdepth-targetlevel-depth;
     //   uint8_t truncatemaxdepth=maxdepth-1;
     //   uint8_t tlevel = targetlevel+this->depth;
     // //  boundlodcount.push_back(0);//start af 1st lod

     bool havenextlod = false;
     auto starti = boundpointsind.size();
     auto startv = boundpointsvert.size();


     // Precomputed CCW cube indices by faces (groups: Z-, Z+, Y-, Y+, X-, X+)
     static const unsigned boxIndices[] = {
         0,1,2, 1,3,2,    // Z-
         4,6,5, 5,6,7,    // Z+
         0,4,1, 1,4,5,    // Y-
         2,3,6, 3,7,6,    // Y+
         0,2,4, 2,6,4,    // X-
         1,5,3, 3,5,7     // X+
     };
     // static const unsigned boxIndices[] = {
     //     0,2,1, 1,2,3,    // Z-
     //     4,5,6, 5,7,6,    // Z+
     //     0,1,4, 1,5,4,    // Y-
     //     2,6,3, 3,6,7,    // Y+
     //     0,4,2, 2,4,6,    // X-
     //     1,3,5, 3,7,5     // X+
     // };
     const int faceToBoxGroup[6] = {4, 5, 2, 3, 0, 1}; // Map (X-,X+,Y-,Y+,Z-,Z+) -> groups above

     // small helper to produce face corners in CCW outward order for face f (0..5)
     auto faceCorners = [](const AABB& b, std::size_t f, Vec3 out[4]) {
         switch (f) {
         case 0: // X- (x = min.x)  (v0..v3 CCW looking from outside: -X)
             out[0] = {b.min.x, b.min.y, b.min.z};
             out[1] = {b.min.x, b.max.y, b.min.z};
             out[2] = {b.min.x, b.max.y, b.max.z};
             out[3] = {b.min.x, b.min.y, b.max.z};
             break;
         case 1: // X+ (x = max.x)
             out[0] = {b.max.x, b.min.y, b.min.z};
             out[1] = {b.max.x, b.min.y, b.max.z};
             out[2] = {b.max.x, b.max.y, b.max.z};
             out[3] = {b.max.x, b.max.y, b.min.z};
             break;
         case 2: // Y- (y = min.y)
             out[0] = {b.min.x, b.min.y, b.min.z};
             out[1] = {b.min.x, b.min.y, b.max.z};
             out[2] = {b.max.x, b.min.y, b.max.z};
             out[3] = {b.max.x, b.min.y, b.min.z};
             break;
         case 3: // Y+ (y = max.y)
             out[0] = {b.min.x, b.max.y, b.min.z};
             out[1] = {b.max.x, b.max.y, b.min.z};
             out[2] = {b.max.x, b.max.y, b.max.z};
             out[3] = {b.min.x, b.max.y, b.max.z};
             break;
         case 4: // Z- (z = min.z)
             out[0] = {b.min.x, b.min.y, b.min.z};
             out[1] = {b.max.x, b.min.y, b.min.z};
             out[2] = {b.max.x, b.max.y, b.min.z};
             out[3] = {b.min.x, b.max.y, b.min.z};
             break;
         case 5: // Z+ (z = max.z)
             out[0] = {b.min.x, b.min.y, b.max.z};
             out[1] = {b.min.x, b.max.y, b.max.z};
             out[2] = {b.max.x, b.max.y, b.max.z};
             out[3] = {b.max.x, b.min.y, b.max.z};
             break;
         default:
             for (int i=0;i<4;++i) out[i] = {0,0,0};
         }
     };

     // Dedup vertices (bit-exact): map Key -> global vertex index (size_t)
     struct Key { std::uint32_t x,y,z; bool operator==(Key const& o) const noexcept { return x==o.x && y==o.y && z==o.z; } };
     struct KeyHash { std::size_t operator()(Key const& k) const noexcept {
             return (static_cast<std::size_t>(k.x) * 1664525u) ^ (static_cast<std::size_t>(k.y) * 22695477u) ^ (static_cast<std::size_t>(k.z) >> 1);
         } };
     std::unordered_map<Key, std::size_t, KeyHash> vertMap;
     vertMap.reserve(4096);

     //rebuild vertmap from existing
     for(auto &ind:boundpointsind){
         Key k;
         auto v = boundpointsvert[ind];
         memcpy(&k.x, &v.x, sizeof(float));
         memcpy(&k.y, &v.y, sizeof(float));
         memcpy(&k.z, &v.z, sizeof(float));
         vertMap[k]=ind;
     }

     auto addVertexGlobal = [&](const Vec3& v)->std::size_t {
         Key k;
         static_assert(sizeof(float)==sizeof(std::uint32_t), "float must be 32-bit");
         memcpy(&k.x, &v.x, sizeof(float));
         memcpy(&k.y, &v.y, sizeof(float));
         memcpy(&k.z, &v.z, sizeof(float));
         auto it = vertMap.find(k);
         if (it!=vertMap.end()) return it->second;
         std::size_t idx = boundpointsvert.size();
         boundpointsvert.push_back(v);
         vertMap.emplace(k, idx);
         return idx;
     };

     // Quad structure produced by each node
     struct Quad {
         std::size_t face; // 0..5: face id (X-,X+,Y-,Y+,Z-,Z+)
         Vec3 v[4];        // CCW outward
     };

     // collectIntersecting: collect terminal leaves intersecting an AABB (pruned by bounds.intersects)
     std::function<void(const OctreeNode*, const AABB&, std::vector<const OctreeNode*>&)> collectIntersecting;
     collectIntersecting = [&](const OctreeNode* node, const AABB& query, std::vector<const OctreeNode*>& out) {
         if (!node) return;
         if (!node->bounds.intersects(query)) return;
         if (node->terminal()) {
             out.push_back(node);
             return;
         }
         for (const auto& c : node->children) collectIntersecting(c.get(), query, out);
     };

     // process: returns vector<Quad> for subtree rooted at node (children-first)
     std::function<std::vector<Quad>(OctreeNode*)> process;
     process = [&](OctreeNode* node) -> std::vector<Quad> {
         std::vector<Quad> result;
         if (!node) return result;

         // Internal node: process children first and try to merge their face quads
         if (!node->terminal()) {
             // collect per-child quads
             std::array<std::vector<Quad>, 8> childQuads;
             std::array<OctreeNode*, 8> childPtr{};
             for (std::size_t i = 0; i < 8; ++i) {
                 childPtr[i] = node->children[i] ? node->children[i].get() : nullptr;
                 if (childPtr[i]) childQuads[i] = process(childPtr[i]);
             }

             // For each parent face try to merge 4 child face quads into one parent face quad
             // child index sets touching parent faces:
             static const int faceChildren[6][4] = {
                 {0,2,4,6}, // X-  (x=0)
                 {1,3,5,7}, // X+  (x=1)
                 {0,1,4,5}, // Y-  (y=0)
                 {2,3,6,7}, // Y+  (y=1)
                 {0,1,2,3}, // Z-  (z=0)
                 {4,5,6,7}  // Z+  (z=1)
             };

             for (std::size_t f = 0; f < 6; ++f) {
                 // We can only merge if each child has exactly one quad that matches the child's full face.
                 bool canMerge = true;
                 // expected child face corners for each child
                 Vec3 expectedCorners[4][4];

                 for (int ci = 0; ci < 4; ++ci) {
                     int childIndex = faceChildren[f][ci];
                     OctreeNode* cnode = childPtr[childIndex];
                     if (!cnode) { canMerge = false; break; } // missing child -> cannot merge
                     // child's expected face corners (in child's local bounds) for face f
                     faceCorners(cnode->bounds, f, expectedCorners[ci]);

                     // find a matching quad in childQuads[childIndex]
                     bool found = false;
                     for (auto it = childQuads[childIndex].begin(); it != childQuads[childIndex].end(); ++it) {
                         if (it->face != f) continue;
                         // exact match of 4 vertices (bit-exact)
                         bool match = true;
                         for (int k = 0; k < 4; ++k) {
                             const Vec3& a = it->v[k];
                             const Vec3& b = expectedCorners[ci][k];
                             if (!(a.x==b.x && a.y==b.y && a.z==b.z)) { match = false; break; }
                         }
                         if (match) { found = true; break; }
                     }
                     if (!found) { canMerge = false; break; }
                 }

                 if (canMerge) {
                     // Remove the matched child quads and add one parent face quad
                     for (int ci = 0; ci < 4; ++ci) {
                         int childIndex = faceChildren[f][ci];
                         // erase first matching quad (we know one exists)
                         auto &vec = childQuads[childIndex];
                         for (auto it = vec.begin(); it != vec.end(); ++it) {
                             if (it->face != f) continue;
                             bool match = true;
                             for (int k = 0; k < 4; ++k) {
                                 const Vec3& a = it->v[k];
                                 const Vec3& b = expectedCorners[ci][k];
                                 if (!(a.x==b.x && a.y==b.y && a.z==b.z)) { match = false; break; }
                             }
                             if (match) { vec.erase(it); break; }
                         }
                     }
                     // add parent-level quad (covering whole parent face)
                     Quad q;
                     q.face = f;
                     faceCorners(node->bounds, f, q.v);
                     result.push_back(std::move(q));
                 }
             }

             // append all remaining child quads into result
             for (std::size_t ci = 0; ci < 8; ++ci) {
                 for (auto &q : childQuads[ci]) result.push_back(std::move(q));
             }

             return result;
         }

         // Terminal node: produce quads for faces that are external (neighbors unoccupied)
         if (!node->occupied) return result;

         havenextlod=havenextlod||node->depth==maxdepth;
         const AABB& b = node->bounds;
         // faces order used elsewhere: X-,X+,Y-,Y+,Z-,Z+
         for (std::size_t f = 0; f < 6; ++f) {
             // face slab thickness proportional to node size
             const float sx = b.max.x - b.min.x;
             const float sy = b.max.y - b.min.y;
             const float sz = b.max.z - b.min.z;
             const float eps = std::max({sx, sy, sz}) * 1e-6f + 1e-12f;

             AABB faceAABB;
             switch (f) {
             case 0: faceAABB.min={b.min.x-eps,b.min.y,b.min.z}; faceAABB.max={b.min.x+eps,b.max.y,b.max.z}; break;
             case 1: faceAABB.min={b.max.x-eps,b.min.y,b.min.z}; faceAABB.max={b.max.x+eps,b.max.y,b.max.z}; break;
             case 2: faceAABB.min={b.min.x,b.min.y-eps,b.min.z}; faceAABB.max={b.max.x,b.min.y+eps,b.max.z}; break;
             case 3: faceAABB.min={b.min.x,b.max.y-eps,b.min.z}; faceAABB.max={b.max.x,b.max.y+eps,b.max.z}; break;
             case 4: faceAABB.min={b.min.x,b.min.y,b.min.z-eps}; faceAABB.max={b.max.x,b.max.y,b.min.z+eps}; break;
             case 5: faceAABB.min={b.min.x,b.min.y,b.max.z-eps}; faceAABB.max={b.max.x,b.max.y,b.max.z+eps}; break;
             default: faceAABB = AABB{};
             }

             std::vector<const OctreeNode*> neighbors;
             collectIntersecting(node, faceAABB, neighbors);

             bool anyNeighborOccupied = false;
             for (const OctreeNode* nb : neighbors) {
                 if (nb == node) continue;
                 if (nb && nb->occupied) { anyNeighborOccupied = true; break; }
             }
             if (anyNeighborOccupied) continue;

             Quad q;
             q.face = f;
             faceCorners(b, f, q.v);
             result.push_back(std::move(q));
         }

         return result;
     };

     // Get all quads for the whole tree
     std::vector<Quad> quads;
      quads = process(this);

     // Flush quads into global vertex/index buffers with dedup
     using IndexT = typename std::remove_reference<decltype(boundpointsind)>::type::value_type;
     for (const Quad& q : quads) {
         // add 4 verts
         std::size_t ids[4];
         for (int k = 0; k < 4; ++k) ids[k] = addVertexGlobal(q.v[k]);

         // emit two triangles (0,1,2) and (0,2,3)
         auto pushIdx = [&](std::size_t id) {
             boundpointsind.push_back(static_cast<IndexT>(id));
         };
         pushIdx(ids[0]); pushIdx(ids[1]); pushIdx(ids[2]);
         pushIdx(ids[0]); pushIdx(ids[2]); pushIdx(ids[3]);
     }

     return havenextlod;
 }
#endif
void OcclusionOctree::buildBoundGeometryFromLevel(uint8_t fromlevel)
 {
    std::unordered_map<OctreeNode*, bool> havenextlod;
     std::vector<OctreeNode*> havenextlodtoremove;auto olddepth=maxDepth_;
    auto rbuildBoundGeometry = [&](auto&& self, OctreeNode* node, uint8_t level) -> void {
        if (!node) return;

        if (fromlevel == node->depth || node->terminal()) {
            havenextlod[node] =true;// node->buildselfBoundGeometry(level, maxDepth_);
        }
        else {
            for (auto& c : node->children) {
                self(self, c.get(), level );
            }
        }
    };
    //init  havenextlod
rbuildBoundGeometry(rbuildBoundGeometry, root_.get(), fromlevel);

     auto buildBoundGeometrylod = [&]() -> void {

         //build lof 0
        // havenextlodcopy=havenextlod;
        //  havenextlod.clear();
        // rbuildBoundGeometry(rbuildBoundGeometry, root_.get(), fromlevel);
        //  for(auto geomnode:havenextlodcopy)
        for(auto geomnode:havenextlod)
        {
            OctreeNode& n = *geomnode.first;
            auto starti=n.boundpointsind.size();
            n.boundlodcount.push_back(starti);
            geomnode.second = n.buildselfBoundGeometry(fromlevel,maxDepth_);//extract geom
            n.boundlodcount.push_back(n.boundpointsind.size());
            if (!geomnode.second||starti== n.boundlodcount.back()){
                //no more lod
                auto beststart =n.boundlodcount[n.boundlodcount.size()-2];
                auto bestend = n.boundlodcount.back();
                while(n.boundlodcount.size()<2*(olddepth-fromlevel) ){
                        n.boundlodcount.push_back(beststart); n.boundlodcount.push_back(bestend);
                }
                assert(n.boundlodcount.size()==2*(olddepth-fromlevel));
                havenextlodtoremove.push_back(&n);
            }

        }
        for(auto geomnode:havenextlodtoremove){
            havenextlod.erase(havenextlod.find(geomnode));
        }
        havenextlodtoremove.clear();
    };

     //todo ensure geometries is empty

     if(maxDepth_>fromlevel)   buildBoundGeometrylod();//build lod 0
     bool haveprune=false;
     while(maxDepth_-1>fromlevel)//tfor(uint8_t curlod = fromlevel+1; curlod < maxDepth_; ++curlod)
     {
         //prune last level
         auto newdepth=maxDepth_-1; haveprune=false;
         auto prune = [&](auto&& self, OctreeNode* node) -> void {

             if(!node->terminal()){
             if(node->depth<newdepth){
                 for (const auto& c : node->children) self(self,c.get());
                 return;
             }

             //to prune

                 //debug
                 for (const auto& c : node->children)  if(!c->terminal())
                         newdepth = node->depth;//fok

                 bool alloqp=true;
                 for (const auto& c : node->children)if(!c->occupied){ alloqp=false;break;}

                 node->occupied = alloqp;
                 for (  auto& c : node->children) c = nullptr; //smart pointer deref
                 haveprune=true;
             }
         };

         prune(prune,root_.get());
         maxDepth_=newdepth;

         if(!haveprune)//same geometry as last level
         {
             haveprune=haveprune;
         }
         buildBoundGeometrylod();//build lod 0

     }
 }

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
 //       out.write(reinterpret_cast<const char*>(&node), sizeof(CompiledNode));
        out.write(reinterpret_cast<const char*>(&node.bounds), sizeof(node.bounds));
        out.write(reinterpret_cast<const char*>(node.children.data()), sizeof(node.children));
        out.write(reinterpret_cast<const char*>(node.neighborBegin.data()), sizeof(node.neighborBegin));
        out.write(reinterpret_cast<const char*>(node.neighborCount.data()), sizeof(node.neighborCount));
        out.write(reinterpret_cast<const char*>(&node.occupied), sizeof(node.occupied));

        std::uint64_t iCount = node.boundpointsind.size();
        out.write(reinterpret_cast<const char*>(&iCount), sizeof(iCount));
        for (std::size_t ix = 0; ix < iCount; ++ix)
            out.write(reinterpret_cast<const char*>(&node.boundpointsind[ix]), sizeof(uint));

        iCount = node.boundpointsvert.size();
        out.write(reinterpret_cast<const char*>(&iCount), sizeof(iCount));
        for (std::size_t ix = 0; ix < iCount; ++ix){
            out.write(reinterpret_cast<const char*>(&node.boundpointsvert[ix].x), sizeof(float));
            out.write(reinterpret_cast<const char*>(&node.boundpointsvert[ix].y), sizeof(float));
            out.write(reinterpret_cast<const char*>(&node.boundpointsvert[ix].z), sizeof(float));
        }
        iCount = node.boundlodcount.size();
        out.write(reinterpret_cast<const char*>(&iCount), sizeof(iCount));
        for (std::size_t ix = 0; ix < iCount; ++ix){
            out.write(reinterpret_cast<const char*>(&node.boundlodcount[ix]), sizeof(unsigned int));
        }
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
    std::uint64_t iCount = 0;
    std::uint64_t storageSize = 0;
    in.read(reinterpret_cast<char*>(&nodeCount), sizeof(nodeCount));
    in.read(reinterpret_cast<char*>(&storageSize), sizeof(storageSize));

    // 3. Access internal vectors (Assumes Serializer is a 'friend' class of CompiledOctree)
    octree.nodes_.resize(nodeCount);
    octree.neighborStorage_.resize(storageSize);

    // 4. Read and reconstruct Nodes
    for (std::size_t i = 0; i < nodeCount; ++i) {
        auto& node = octree.nodes_[i];
     //     in.read(reinterpret_cast<char*>(&node), sizeof(CompiledNode));
        in.read(reinterpret_cast<char*>(&node.bounds), sizeof(node.bounds));
        in.read(reinterpret_cast<char*>(node.children.data()), sizeof(node.children));
        in.read(reinterpret_cast<char*>(node.neighborBegin.data()), sizeof(node.neighborBegin));
        in.read(reinterpret_cast<char*>(node.neighborCount.data()), sizeof(node.neighborCount));
        in.read(reinterpret_cast<char*>(&node.occupied), sizeof(node.occupied));

        in.read(reinterpret_cast<char*>(&iCount), sizeof(iCount));
         node.boundpointsind.resize(iCount);
        for (std::size_t ix = 0; ix < iCount; ++ix)
            in.read(reinterpret_cast<char*>(&node.boundpointsind[ix]), sizeof(uint));

        in.read(reinterpret_cast<char*>(&iCount), sizeof(iCount));
        node.boundpointsvert.resize(iCount);
        for (std::size_t ix = 0; ix < iCount; ++ix){
            in.read(reinterpret_cast<char*>(&node.boundpointsvert[ix].x), sizeof(float));
            in.read(reinterpret_cast<char*>(&node.boundpointsvert[ix].y), sizeof(float));
            in.read(reinterpret_cast<char*>(&node.boundpointsvert[ix].z), sizeof(float));
        }
        in.read(reinterpret_cast<char*>(&iCount), sizeof(iCount));
        node.boundlodcount.resize(iCount);
        for (std::size_t ix = 0; ix < iCount; ++ix){
            in.read(reinterpret_cast<char*>(&node.boundlodcount[ix]), sizeof(unsigned int));
        }


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
