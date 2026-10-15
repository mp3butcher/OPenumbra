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
#include <memory.h>
namespace openu {
namespace {

std::array<Vec3, 8> corners(const AABB& b) {
    return {{{b.min.x,b.min.y,b.min.z}, {b.max.x,b.min.y,b.min.z}, {b.min.x,b.max.y,b.min.z}, {b.max.x,b.max.y,b.min.z},
             {b.min.x,b.min.y,b.max.z}, {b.max.x,b.min.y,b.max.z}, {b.min.x,b.max.y,b.max.z}, {b.max.x,b.max.y,b.max.z}}};
}

ClipVertex transform(const Vec3& p, const Mat4& m) {
    return {
        m.m[0]*p.x + m.m[1]*p.y + m.m[2]*p.z + m.m[3],
        m.m[4]*p.x + m.m[5]*p.y + m.m[6]*p.z + m.m[7],
        m.m[8]*p.x + m.m[9]*p.y + m.m[10]*p.z + m.m[11],
        m.m[12]*p.x + m.m[13]*p.y + m.m[14]*p.z + m.m[15]
    };
}



Frustum extractFrustum(const Mat4& clip) {
    Frustum frustum;

    // Extraction basée sur les lignes de la matrice (indices : [ligne][colonne])
    // Gauche
    frustum.planes[0].normal = Vec3(clip.m[3 *4+ 0] + clip.m[0 *4+ 0], clip.m[3 *4+ 1] + clip.m[0 *4+ 1], clip.m[3 *4+ 2] + clip.m[0 *4+ 2]);
    frustum.planes[0].d      = clip.m[3 *4+ 3] + clip.m[0 *4+ 3];
    // Droite
    frustum.planes[1].normal = Vec3(clip.m[3 *4+ 0] - clip.m[0 *4+ 0], clip.m[3 *4+ 1] - clip.m[0 *4+ 1], clip.m[3 *4+ 2] - clip.m[0 *4+ 2]);
    frustum.planes[1].d      = clip.m[3 *4+ 3] - clip.m[0 *4+ 3];
    // Bas
    frustum.planes[2].normal = Vec3(clip.m[3 *4+ 0] + clip.m[1 *4+ 0], clip.m[3 *4+ 1] + clip.m[1 *4+ 1], clip.m[3 *4+ 2] + clip.m[1 *4+ 2]);
    frustum.planes[2].d      = clip.m[3 *4+ 3] + clip.m[1 *4+ 3];
    // Haut
    frustum.planes[3].normal = Vec3(clip.m[3 *4+ 0] - clip.m[1 *4+ 0], clip.m[3 *4+ 1] - clip.m[1 *4+ 1], clip.m[3 *4+ 2] - clip.m[1 *4+ 2]);
    frustum.planes[3].d      = clip.m[3 *4+ 3] - clip.m[1 *4+ 3];
    // Near (Proche : -1 <= z en espace homogène OpenGL)
    frustum.planes[4].normal = Vec3(clip.m[3 *4+ 0] + clip.m[2 *4+ 0], clip.m[3 *4+ 1] + clip.m[2 *4+ 1], clip.m[3 *4+ 2] + clip.m[2 *4+ 2]);
    frustum.planes[4].d      = clip.m[3 *4+ 3] + clip.m[2 *4+ 3];
    // Far (Lointain : z <= 1 en espace homogène OpenGL)
    frustum.planes[5].normal = Vec3(clip.m[3 *4+ 0] - clip.m[2 *4+ 0], clip.m[3 *4+ 1] - clip.m[2 *4+ 1], clip.m[3 *4+ 2] - clip.m[2 *4+ 2]);
    frustum.planes[5].d      = clip.m[3 *4+ 3] - clip.m[2 *4+ 3];

    // Normalisation indispensable pour le calcul exact des boîtes englobantes
    for (auto& plane : frustum.planes) {
        float invLength = 1.0f / std::sqrt(plane.normal.x * plane.normal.x +
                                           plane.normal.y * plane.normal.y +
                                           plane.normal.z * plane.normal.z);
        plane.normal.x *= invLength;
        plane.normal.y *= invLength;
        plane.normal.z *= invLength;
        plane.d        *= invLength;
    }

    return frustum;
}

bool aabbIntersectsFrustum(const AABB& box, const Frustum& frustum) {
    // Calcul du centre et des demi-extensions de la boîte
    Vec3 center  = (box.max + box.min) * 0.5f;
    Vec3 extents = (box.max - box.min) * 0.5f;

    for (const auto& plane : frustum.planes) {
        // Projection du rayon de l'AABB sur la normale du plan (toujours positif)
        float projectionRadius = extents.x * std::abs(plane.normal.x) +
                                 extents.y * std::abs(plane.normal.y) +
                                 extents.z * std::abs(plane.normal.z);

        // Si le centre de la boîte est plus loin derrière le plan que son propre rayon,
        // la boîte est intégralement en dehors du frustum.
        if (plane.distance(center) < -projectionRadius) {
            return false;
        }
    }

    // La boîte est visible (partiellement ou totalement)
    return true;
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
    moc_ = MaskedOcclusionCulling::Create();
    MaskedOcclusionCulling::Implementation implementation = moc_->GetImplementation();
    switch (implementation) {
    case MaskedOcclusionCulling::SSE2: printf("Using SSE2 version\n"); break;
    case MaskedOcclusionCulling::SSE41: printf("Using SSE41 version\n"); break;
    case MaskedOcclusionCulling::AVX2: printf("Using AVX2 version\n"); break;
    case MaskedOcclusionCulling::AVX512: printf("Using AVX-512 version\n"); break;
    }
    moc_->SetResolution(width_, height_);
}

OctreeRasterizer::~OctreeRasterizer() {
    if (moc_) MaskedOcclusionCulling::Destroy(moc_);
}

std::vector<const OctreeNode*> OctreeRasterizer::visibleCells(const OcclusionOctree& tree, const ViewPoint& view) const {
    std::vector<const OctreeNode*> visible;
    std::unordered_set<const OctreeNode*> visited;
    std::vector<const OctreeNode*> frontier;
    auto pos = view.cameraPosition();
    const OctreeNode* start = tree.locate(pos);
    if (!start) return visible;
    frontier.push_back(start);
    const Mat4 clip = view.worldToClip();
    while (!frontier.empty())
    {
        const OctreeNode *node = frontier.back();
        frontier.pop_back();
        if (!node || !visited.insert(node).second)
            continue;
        if (node->occupied)
            visible.push_back(node);
        else{
            for (Axis axis : {Axis::X, Axis::Y, Axis::Z})
            {
                for (Direction dir : {Direction::Negative, Direction::Positive})
                {
                    std::vector<const OctreeNode *> neighbors;
                    node->getNeighbors(axis, dir, neighbors);
                    for (const OctreeNode *neighbor : neighbors)
                    {
                        if (neighbor && !visited.count(neighbor) && aabbIntersectsFrustum(neighbor->bounds, clip))
                            frontier.push_back(neighbor);
                    }
                }
            }
        }
    }
    return visible;
}

std::vector<const CompiledNode*> OctreeRasterizer::visibleCells(const CompiledOctree& tree, const ViewPoint& view) const {
    std::vector<const CompiledNode*> visible;
    if (tree.nodeCount() == 0) return visible;

    if (visitStamp_.size() != tree.nodeCount()) visitStamp_.assign(tree.nodeCount(), 0);
    ++traversalStamp_;
    if (traversalStamp_ == 0) {
        std::fill(visitStamp_.begin(), visitStamp_.end(), 0);
        traversalStamp_ = 1;
    }
    auto pos = view.cameraPosition();
    NodeId start = InvalidNode;
    const OctreeNode* sourceStart = nullptr;
    // CompiledOctree currently stores terminal cells, so locate once by source
    // pointer; all subsequent operations use integer IDs and flat ranges.
    // This linear lookup is outside the hot neighbor traversal loop.
    // for (NodeId id = 0; id < tree.nodeCount(); ++id) {
    //     const CompiledNode& n = tree.node(id);
    //     // if (n.source && n.bounds.contains(pos)) { start = id; sourceStart = n.source; break; }
    //     if ( n.bounds.contains(pos)) { start = id; break; }
    // }
    start = tree.locate(pos);
    // (void)sourceStart;
    if (start == InvalidNode)
        return visible;

    frontier_.clear();
    frontier_.push_back(start);
    const Mat4 clip = view.worldToClip();
    while (!frontier_.empty())
    {
        const NodeId id = frontier_.back();
        frontier_.pop_back();
        if (id == InvalidNode || id >= tree.nodeCount() || visitStamp_[id] == traversalStamp_)
            continue;
        visitStamp_[id] = traversalStamp_;
        const CompiledNode &node = tree.node(id);
        if (node.occupied)
            visible.push_back(&node);
        else{
            for (std::size_t face = 0; face < 6; ++face)
            {
                const std::uint32_t begin = node.neighborBegin[face];
                const std::uint32_t end = begin + node.neighborCount[face];
                const auto &neighbors = tree.neighborStorage();
                for (std::uint32_t i = begin; i < end; ++i)
                {
                    // std::cout<<i<<" chek neighbor node" <<std::endl;
                    const NodeId neighbor = neighbors[i];
                    if (neighbor == InvalidNode || visitStamp_[neighbor] == traversalStamp_)
                        continue;
                    if (aabbIntersectsFrustum(tree.node(neighbor).bounds, clip))
                        frontier_.push_back(neighbor);
                }
            }
        }
    }
    return visible;
}
const Vec3 faceNormales[6] = {
    {0.0f, 0.0f, -1.0f}, // Avant
    {0.0f, 0.0f, 1.0f},  // Arrière
    {0.0f, -1.0f, 0.0f}, // Bas
    {0.0f, 1.0f, 0.0f},  // Haut
    {-1.0f, 0.0f, 0.0f}, // Gauche
    {1.0f, 0.0f, 0.0f}   // Droite
};

const unsigned cubeFacesIndices[6][6] = {
    {0,1,2, 1,3,2}, // Face Avant  (Z min)
    {4,6,5, 5,6,7}, // Face Arrière (Z max)
    {0,4,1, 1,4,5}, // Face Bas     (Y min)
    { 2,3,6, 3,7,6}, // Face Haut    (Y max)
    {0,2,4, 2,6,4}, // Face Gauche  (X min)
    {1,5,3, 3,5,7}  // Face Droite  (X max)
};
RasterizedOctree OctreeRasterizer::rasterizeNodes(const std::vector<const OctreeNode*>& nodes, const ViewPoint& view) {
    RasterizedOctree result;
    result.width = width_; result.height = height_;
    moc_->SetNearClipPlane(0.1f);
    moc_->ClearBuffer();
    const Mat4 clip = view.worldToClip();
    std::array<ClipVertex, 8> vertices{};

    for (const OctreeNode* node : nodes) {

        if (!node || !node->occupied) continue;
        // Reprenez vos vraies bornes d'origine (SANS le shrink de 0.9)
        const auto points = corners(node->bounds);
        for (std::size_t i = 0; i < points.size(); ++i) {
            vertices[i] = transform(points[i], clip);
        }

        std::vector<uint> indices;
        // Calcul du vecteur allant du centre du nœud vers la caméra (en World Space)
        Vec3 nodeCenter = (node->bounds.min + node->bounds.max) * 0.5f;
        Vec3 toCamera = view.cameraPosition() - nodeCenter;


        // On cherche quelles faces regardent la caméra (produit scalaire > 0)
        for (int f = 5; f >= 0; --f) {
            float dot = faceNormales[f].x * toCamera.x +
                        faceNormales[f].y * toCamera.y +
                        faceNormales[f].z * toCamera.z;

            if (dot > 0.0f)
            { // La face est orientée vers la caméra !
                // moc_->RenderTriangles(
                //     reinterpret_cast<const float*>(vertices.data()),
                //     cubeFacesIndices[f],
                //     2, // On ne dessine que les 2 triangles de cette face visible
                //     nullptr,
                //     MaskedOcclusionCulling::BACKFACE_CW, // Plus besoin de culling, on l'a fait au-dessus
                //     MaskedOcclusionCulling::CLIP_PLANE_ALL
                // );
                indices.push_back(cubeFacesIndices[f][0]);
                indices.push_back(cubeFacesIndices[f][1]);
                indices.push_back(cubeFacesIndices[f][2]);
                indices.push_back(cubeFacesIndices[f][3]);
                indices.push_back(cubeFacesIndices[f][4]);
                indices.push_back(cubeFacesIndices[f][5]);
            }
        }
        moc_->RenderTriangles(
            reinterpret_cast<const float*>(vertices.data()),
            indices.data(),
            indices.size()/3, // On ne dessine que les 2 triangles de cette face visible
            nullptr,
            MaskedOcclusionCulling::BACKFACE_CCW, // Plus besoin de culling, on l'a fait au-dessus
            MaskedOcclusionCulling::CLIP_PLANE_ALL
            );
    }
    if(false)for (const OctreeNode* node : nodes) {
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
using namespace std;
bool OctreeRasterizer::testBox(std::array<ClipVertex, 8> &boxCorner)
{
    float xmin =  numeric_limits<float>::infinity();
    float ymin =  numeric_limits<float>::infinity();
    float xmax = -numeric_limits<float>::infinity();
    float ymax = -numeric_limits<float>::infinity();
    float zMax = -numeric_limits<float>::infinity(); // we want the largest z = 1/w (closest)
    bool anyFront = false;
    bool anyBehind = false;
    for (const ClipVertex &q : boxCorner) {
        // q is clip-space (proj * view * pos)
        if (q.w > 0.0f) {
            anyFront = true;
            float ndcX = q.x / q.w;
            float ndcY = q.y / q.w;
            xmin = min(xmin, ndcX);
            xmax = max(xmax, ndcX);
            ymin = min(ymin, ndcY);
            ymax = max(ymax, ndcY);
            zMax = max(zMax, 1.0f / q.w); // largest 1/w = closest
        } else {
            anyBehind = true;
        }
    }

    if (!anyFront) {
        // All behind — conservative: treat as not visible or handle as you need
        return false;
    }
    if (anyBehind && anyFront) {
        // straddles near plane — conservative option:
        return true; // don't occlude
        // Or do proper clipping against near plane to compute correct ndc/w
    }

    // pass wmin = 1.0f / zMax (clamp zMax)
    zMax = max(zMax, 1e-6f);
    float wmin = 1.0f / zMax;
    return moc_->TestRect(xmin, ymin, xmax, ymax, wmin) == MaskedOcclusionCulling::VISIBLE;
}

RasterizedOctree OctreeRasterizer::rasterize(OcclusionOctree& tree, const ViewPoint& view) {
    return rasterizeNodes(visibleCells(tree, view), view);
}
RasterizedOctree OctreeRasterizer::renderFrontToBack(const CompiledOctree& tree, const ViewPoint& view)  {
    RasterizedOctree result;
    if (tree.nodeCount()==0) return result;
    const Mat4 clip = view.worldToClip();
    const Frustum frustum = extractFrustum(clip);
    moc_->ClearBuffer();
    moc_->SetNearClipPlane(0.0001);
    const Vec3 cameraPos = view.cameraPosition();
    // Lance le parcours récursif à partir de la racine (toujours à l'index 0)
    renderNodeRecursive(tree, 0, cameraPos, frustum, clip);

    result.width = width_; result.height = height_;


    result.depth.resize(static_cast<std::size_t>(width_) * height_);
    moc_->ComputePixelDepthBuffer(result.depth.data(), false);
    return result;
}
std::array<ClipVertex, 8> vertices{};
std::array<uint, 36> indices{};
RasterizedOctree OctreeRasterizer::rasterize(const CompiledOctree& tree, const ViewPoint& view) {
    // return rasterizeNodes(visibleCells(tree, view), view);
    // std::vector<const CompiledNode*> visible;
    uint nbvisibles{0};
    // if (tree.nodeCount() == 0) return visible;

    moc_->ClearBuffer();
    moc_->SetNearClipPlane(0.0001);
    if (visitStamp_.size() != tree.nodeCount()) visitStamp_.assign(tree.nodeCount(), 0);
    ++traversalStamp_;
    if (traversalStamp_ == 0) {
        std::fill(visitStamp_.begin(), visitStamp_.end(), 0);
        traversalStamp_ = 1;
    }
    auto pos = view.cameraPosition();
    NodeId start = InvalidNode;
    const OctreeNode* sourceStart = nullptr;
    start=tree.locate(pos);

    frontier_.clear();
    visitStamp_[start] = traversalStamp_;
    //frontier_.push_back(start);
    const CompiledNode &node = tree.node(start);
    const Mat4 clip = view.worldToClip();
    auto frustum=extractFrustum(clip);
    for (std::size_t face = 0; face < 6; ++face)
    {
        const std::uint32_t begin = node.neighborBegin[face];
        const std::uint32_t end = begin + node.neighborCount[face];
        const auto &neighbors = tree.neighborStorage();
        for (std::uint32_t i = begin; i < end; ++i)
        {
            const NodeId neighbor = neighbors[i];
            if (neighbor == InvalidNode || visitStamp_[neighbor] == traversalStamp_)
                continue;
            if (aabbIntersectsFrustum(tree.node(neighbor).bounds, frustum))
                frontier_.push_back(neighbor);

        }
    }
    while (!frontier_.empty())
    {
        const NodeId id = frontier_.back();
        frontier_.pop_back();
        if (id == InvalidNode || id >= tree.nodeCount() || visitStamp_[id] == traversalStamp_)
            continue;
        visitStamp_[id] = traversalStamp_;
        const CompiledNode &node = tree.node(id);
        if (node.occupied)
        {

            nbvisibles++;
            // Reprenez vos vraies bornes d'origine (SANS le shrink de 0.9)
            const auto points = corners(node.bounds);
            for (std::size_t i = 0; i < points.size(); ++i)
            {
                vertices[i] = transform(points[i], clip);
            }

           // std::array<uint, 36> indices{};
            uint curidx = 0;
            // Calcul du vecteur allant du centre du nœud vers la caméra (en World Space)
            Vec3 nodeCenter = (node.bounds.min + node.bounds.max) * 0.5f;
            Vec3 toCamera = pos - nodeCenter;

            // On cherche quelles faces regardent la caméra (produit scalaire > 0)
            for (int f = 0; f < 6; ++f)
            {
                float dot = faceNormales[f].x * toCamera.x +
                            faceNormales[f].y * toCamera.y +
                            faceNormales[f].z * toCamera.z;

                if (dot > 0.0f)
                {
                    memcpy(&indices[curidx], cubeFacesIndices[f], sizeof(uint) * 6 );
                    curidx += 6;
                }
            }
            moc_->RenderTriangles(
                reinterpret_cast<const float *>(vertices.data()),
                indices.data(),
                curidx / 3,
                nullptr,
                MaskedOcclusionCulling::BACKFACE_NONE,
                MaskedOcclusionCulling::CLIP_PLANE_NONE);
        }
        else{
            float xMin = node.bounds.min.x, xMax = node.bounds.max.x;//+theta*extents.x;
            float yMin = node.bounds.min.y, yMax = node.bounds.max.y;//+theta*extents.x;
            float zMin = node.bounds.min.z, zMax = node.bounds.max.z;//+theta*extents.x;

            // 3. Projection directe des 8 sommets (déroulée pour le cache CPU)
            ClipVertex q0 = transform(openu::Vec3(xMin, yMin, zMin) , clip);
            ClipVertex q1 = transform(Vec3(xMax, yMin, zMin) , clip);
            ClipVertex q2 = transform(Vec3(xMin, yMax, zMin) , clip);
            ClipVertex q3 = transform(Vec3(xMax, yMax, zMin) , clip);
            ClipVertex q4 = transform(Vec3(xMin, yMin, zMax) , clip);
            ClipVertex q5 = transform(Vec3(xMax, yMin, zMax) , clip);
            ClipVertex q6 = transform(Vec3(xMin, yMax, zMax) , clip);
            ClipVertex q7 = transform(Vec3(xMax, yMax, zMax) , clip);

                // Stratégie de repli conservative à plein écran (identique à votre code)
                // visible= rasterizer->moc_->TestRect(-1.0f, -1.0f, 1.0f, 1.0f, 0.0001f) == MaskedOcclusionCulling::VISIBLE;
                // 5. Division perspective immédiate (plus besoin de tester si q.w > 0)
                float iw0 = 1.0f / q0.w; float iw1 = 1.0f / q1.w; float iw2 = 1.0f / q2.w; float iw3 = 1.0f / q3.w;
                float iw4 = 1.0f / q4.w; float iw5 = 1.0f / q5.w; float iw6 = 1.0f / q6.w; float iw7 = 1.0f / q7.w;

                // 6. Recherche ultra-rapide des extrêmes NDC
                float xmin = std::min({q0.x * iw0, q1.x * iw1, q2.x * iw2, q3.x * iw3, q4.x * iw4, q5.x * iw5, q6.x * iw6, q7.x * iw7});
                float xmax = std::max({q0.x * iw0, q1.x * iw1, q2.x * iw2, q3.x * iw3, q4.x * iw4, q5.x * iw5, q6.x * iw6, q7.x * iw7});
                float ymin = std::min({q0.y * iw0, q1.y * iw1, q2.y * iw2, q3.y * iw3, q4.y * iw4, q5.y * iw5, q6.y * iw6, q7.y * iw7});
                float ymax = std::max({q0.y * iw0, q1.y * iw1, q2.y * iw2, q3.y * iw3, q4.y * iw4, q5.y * iw5, q6.y * iw6, q7.y * iw7});
                float faresttW =std::max({q0.w, q1.w, q2.w, q3.w, q4.w, q5.w, q6.w, q7.w});
                float nearestW =std::min({q0.w, q1.w, q2.w, q3.w, q4.w, q5.w, q6.w, q7.w});

                if( nearestW<0.0001f|| moc_->TestRect(xmin, ymin, xmax, ymax, std::max(0.0001f, nearestW)) == MaskedOcclusionCulling::VISIBLE)

            // float xmin, ymin, xmax, ymax, nearestW;
            // xmin = ymin = std::numeric_limits<float>::infinity();
            // xmax = ymax = -std::numeric_limits<float>::infinity();
            // nearestW = std::numeric_limits<float>::infinity();
            // bool front = false, behind = false;
            // for (const Vec3& p : corners(node.bounds)) {
            //     const ClipVertex q = transform(p, clip);
            //     if (q.w > 0.0f) {
            //         front = true;
            //         xmin = std::min(xmin, q.x / q.w); xmax = std::max(xmax, q.x / q.w);
            //         ymin = std::min(ymin, q.y / q.w); ymax = std::max(ymax, q.y / q.w);
            //         nearestW = std::min(nearestW, q.w);
            //     } else behind = true;
            // }
            // if (!front)  continue;
            // if (behind) { xmin = -1.0f; ymin = -1.0f; xmax = 1.0f; ymax = 1.0f; nearestW = 0.0001f; }
            // if( moc_->TestRect(xmin, ymin, xmax, ymax, nearestW) == MaskedOcclusionCulling::VISIBLE)
                for (std::size_t face = 0; face < 6; ++face)
                {
                    const std::uint32_t begin = node.neighborBegin[face];
                    const std::uint32_t end = begin + node.neighborCount[face];
                    const auto &neighbors = tree.neighborStorage();
                    for (std::uint32_t i = begin; i < end; ++i)
                    {
                        // std::cout<<i<<" chek neighbor node" <<std::endl;
                        const NodeId neighbor = neighbors[i];
                        if (neighbor == InvalidNode || visitStamp_[neighbor] == traversalStamp_)
                            continue;
                        if (aabbIntersectsFrustum(tree.node(neighbor).bounds, frustum))
                            //   {
                            //     float xmin, ymin, xmax, ymax, nearestW;
                            //     xmin = ymin = std::numeric_limits<float>::infinity();
                            //     xmax = ymax = -std::numeric_limits<float>::infinity();
                            //     nearestW = std::numeric_limits<float>::infinity();
                            //     bool front = false, behind = false;
                            //     for (const ClipVertex& q : boxCorner) {
                            //         if (q.w > 0.0f) {
                            //             front = true;
                            //             xmin = std::min(xmin, q.x / q.w); xmax = std::max(xmax, q.x / q.w);
                            //             ymin = std::min(ymin, q.y / q.w); ymax = std::max(ymax, q.y / q.w);
                            //             nearestW = std::min(nearestW, q.w);
                            //         } else behind = true;
                            //     }
                            //     if (!front) return false;
                            //     if (behind) { xmin = -1.0f; ymin = -1.0f; xmax = 1.0f; ymax = 1.0f; nearestW = 0.0001f; }
                            //     //return true;

                            //     return moc_->TestRect(xmin, ymin, xmax, ymax, nearestW) == MaskedOcclusionCulling::VISIBLE;
                            //   }
                            frontier_.push_back(neighbor);

                    }
                }
        }
    }
    RasterizedOctree result;
    result.width = width_; result.height = height_;
    //
    std::cout<<nbvisibles<<std::endl;


    result.depth.resize(static_cast<std::size_t>(width_) * height_);
    moc_->ComputePixelDepthBuffer(result.depth.data(), false);
    return result;
}

void OctreeRasterizer::renderNodeRecursive(const CompiledOctree& tree, NodeId id, const Vec3& cameraPos, const Frustum& frustum, const Mat4& clip) {
    if (id == InvalidNode) return;

    const CompiledNode& node =tree.node(id);

    // 1. Culling Frustum : Si la boîte englobante est hors du frustum, on coupe la branche
    if (!aabbIntersectsFrustum(node.bounds, frustum)) {
        return;
    }

    // 2. Si c'est une feuille terminale, on traite le rendu logiciel
    if (node.children[0] == InvalidNode) {
        if (node.occupied) {
            // Reprenez vos vraies bornes d'origine (SANS le shrink de 0.9)
            const auto points = corners(node.bounds);
            for (std::size_t i = 0; i < points.size(); ++i)
            {
                vertices[i] = transform(points[i], clip);
            }

            uint curidx = 0;
            // Calcul du vecteur allant du centre du nœud vers la caméra (en World Space)
            Vec3 nodeCenter = (node.bounds.min + node.bounds.max) * 0.5f;
            Vec3 toCamera = cameraPos - nodeCenter;

            // On cherche quelles faces regardent la caméra (produit scalaire > 0)
            for (int f = 0; f < 6; ++f)
            {
                float dot = faceNormales[f].x * toCamera.x +
                            faceNormales[f].y * toCamera.y +
                            faceNormales[f].z * toCamera.z;

                if (dot > 0.0f)
                {
                    memcpy(&indices[curidx], cubeFacesIndices[f], sizeof(uint) * 6 );
                    curidx += 6;
                }
            }
            moc_->RenderTriangles(
                reinterpret_cast<const float *>(vertices.data()),
                indices.data(),
                curidx / 3,
                nullptr,
                MaskedOcclusionCulling::BACKFACE_NONE,
                MaskedOcclusionCulling::CLIP_PLANE_NONE);
        }
        return;
    }
    float xMin = node.bounds.min.x, xMax = node.bounds.max.x;
    float yMin = node.bounds.min.y, yMax = node.bounds.max.y;
    float zMin = node.bounds.min.z, zMax = node.bounds.max.z;
    ClipVertex q0 = transform(openu::Vec3(xMin, yMin, zMin) , clip);
    ClipVertex q1 = transform(Vec3(xMax, yMin, zMin) , clip);
    ClipVertex q2 = transform(Vec3(xMin, yMax, zMin) , clip);
    ClipVertex q3 = transform(Vec3(xMax, yMax, zMin) , clip);
    ClipVertex q4 = transform(Vec3(xMin, yMin, zMax) , clip);
    ClipVertex q5 = transform(Vec3(xMax, yMin, zMax) , clip);
    ClipVertex q6 = transform(Vec3(xMin, yMax, zMax) , clip);
    ClipVertex q7 = transform(Vec3(xMax, yMax, zMax) , clip);
    float iw0 = 1.0f / q0.w; float iw1 = 1.0f / q1.w; float iw2 = 1.0f / q2.w; float iw3 = 1.0f / q3.w;
    float iw4 = 1.0f / q4.w; float iw5 = 1.0f / q5.w; float iw6 = 1.0f / q6.w; float iw7 = 1.0f / q7.w;
    float xmin = std::min({q0.x * iw0, q1.x * iw1, q2.x * iw2, q3.x * iw3, q4.x * iw4, q5.x * iw5, q6.x * iw6, q7.x * iw7});
    float xmax = std::max({q0.x * iw0, q1.x * iw1, q2.x * iw2, q3.x * iw3, q4.x * iw4, q5.x * iw5, q6.x * iw6, q7.x * iw7});
    float ymin = std::min({q0.y * iw0, q1.y * iw1, q2.y * iw2, q3.y * iw3, q4.y * iw4, q5.y * iw5, q6.y * iw6, q7.y * iw7});
    float ymax = std::max({q0.y * iw0, q1.y * iw1, q2.y * iw2, q3.y * iw3, q4.y * iw4, q5.y * iw5, q6.y * iw6, q7.y * iw7});
    float faresttW =std::max({q0.w, q1.w, q2.w, q3.w, q4.w, q5.w, q6.w, q7.w});
    float nearestW =std::min({q0.w, q1.w, q2.w, q3.w, q4.w, q5.w, q6.w, q7.w});
    //nearestW=std::max(0.0001f, nearestW);
    //if(faresttW>0.0001f && moc_->TestRect(xmin, ymin, xmax, ymax, nearestW) == MaskedOcclusionCulling::VISIBLE)

    if( nearestW<0.0001f || moc_->TestRect(xmin, ymin, xmax, ymax, std::max(0.0001f, nearestW)) == MaskedOcclusionCulling::VISIBLE)
    {
        // 3. C'est un nœud interne : Calcul de l'ordre de visite des 8 enfants (Front-to-Back)
        Vec3 center = node.bounds.center();

        // Détermination de l'index de l'enfant dans lequel se trouve la caméra par rapport au centre
        std::uint8_t closestChildIndex = 0;
        if (cameraPos.x >= center.x) closestChildIndex |= 1u;
        if (cameraPos.y >= center.y) closestChildIndex |= 2u;
        if (cameraPos.z >= center.z) closestChildIndex |= 4u;

        // L'opérateur XOR (^) applique l'ordre optimal du plus proche au plus lointain
        for (std::uint8_t i = 0; i < 8; ++i) {
            std::uint8_t childOrder = i ^ closestChildIndex;
            NodeId childId = node.children[childOrder];

            if (childId != InvalidNode) {
                renderNodeRecursive(tree, childId, cameraPos, frustum, clip);
            }
        }
    }
}
} // namespace openu
