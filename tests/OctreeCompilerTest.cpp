#include <cassert>
#include <iostream>
#include <stdexcept>
#include <unordered_set>
#include <vector>
#include "openu/Octree.hpp"
#include "openu/OctreeRasterizer.hpp"
#include <float.h>
#include <fstream>

#include "MaskedOcclusionCulling.h"
using namespace openu;
static void TonemapDepth(const float *depth, unsigned char *image, int w, int h)
{
    // Trouver les valeurs min/max de W
    float minW = FLT_MAX, maxW = -FLT_MAX; // Initialisation corrigée pour maxW
    for (int i = 0; i < w*h; ++i)
    {
        if (depth[i] > 0.0f)
        {
            minW = std::min(minW, depth[i]);
            maxW = std::max(maxW, depth[i]);
        }
    }

    // Calcul de la plage (range)
    float range = maxW - minW;

    // Tonemap des valeurs de profondeur
    for (int i = 0; i < w*h; ++i)
    {
        int intensity = 0;
        
        if (depth[i] > 0.0f)
        {
            // Sécurité : Si le quad est parfaitement plat (range proche de 0),
            // on force une intensité moyenne fixe au lieu de diviser par zéro.
            if (range < 1e-6f)
            {
                intensity = 128; // Gris moyen uniforme pour le quad
            }
            else
            {
                intensity = static_cast<int>(223.0f * (depth[i] - minW) / range + 32.0f);
            }
        }
            
        image[i * 3 + 0] = static_cast<unsigned char>(intensity);
        image[i * 3 + 1] = static_cast<unsigned char>(intensity);
        image[i * 3 + 2] = static_cast<unsigned char>(intensity);
    }
}
// Désactive le "padding" de la structure pour coller parfaitement au format binaire du fichier
#pragma pack(push, 1)
struct BMPHeader {
    // Entête du fichier (14 octets)
    uint16_t fileType{0x4D42};  // "BM" en ASCII
    uint32_t fileSize{0};       // Taille totale du fichier en octets
    uint16_t reserved1{0};
    uint16_t reserved2{0};
    uint32_t offsetData{54};    // Position du premier pixel (14 + 40 octets d'entêtes)

    // Entête de l'image (40 octets - DIB Header)
    uint32_t size{40};          // Taille de cette entête
    int32_t  width{0};          // Largeur de l'image
    int32_t  height{0};         // Hauteur de l'image
    uint16_t planes{1};         // Toujours 1
    uint16_t bitCount{24};      // 24 bits par pixel (RGB)
    uint32_t compression{0};    // 0 = Pas de compression
    uint32_t sizeImage{0};      // Taille des données de pixels (0 si pas de compression)
    int32_t  xPixelsPerMeter{0};
    int32_t  yPixelsPerMeter{0};
    uint32_t colorsUsed{0};
    uint32_t colorsImportant{0};
};
#pragma pack(pop)

bool saveBitmap(const std::string& filename, const char* pixelData, int32_t width, int32_t height) {
    // 1. Calcul du padding (Chaque ligne d'un BMP doit être un multiple de 4 octets)
    int rowSize = (width * 3 + 3) & ~3;
    int paddingSize = rowSize - (width * 3);
    uint32_t dataSize = rowSize * height;
    // 2. Configuration de l'entête
    BMPHeader header;
    header.width = width;
    header.height = height; // Note : Une hauteur positive écrit l'image de bas en haut
    header.fileSize = sizeof(BMPHeader) + dataSize;
    header.sizeImage = dataSize;

    // 3. Écriture du fichier binaire
    std::ofstream file(filename, std::ios::binary);
    if (!file) return false;

    // Écriture de l'entête
    file.write(reinterpret_cast<const char*>(&header), sizeof(header));

    // Écriture des pixels ligne par ligne pour gérer le padding
    char paddingBytes[3] = {0, 0, 0};
    for (int y = 0; y < height; ++y) {
        // Pointeur vers le début de la ligne actuelle dans votre tableau de char
        const char* rowPtr = pixelData + (y * width * 3);
        
        // Écriture des pixels (Attention : Le BMP stocke en format BGR, pas RGB !)
        file.write(rowPtr, width * 3);
        
        // Écriture des octets de bourrage (padding) si nécessaire
        if (paddingSize > 0) {
            file.write(paddingBytes, paddingSize);
        }
    }

    return true;
}

openu::ClipVertex transform(const Vec3& p, const Mat4& m) {
    return { 
            m.m[0]*p.x + m.m[1]*p.y + m.m[2]*p.z + m.m[3],
            m.m[4]*p.x + m.m[5]*p.y + m.m[6]*p.z + m.m[7],
            m.m[8]*p.x + m.m[9]*p.y + m.m[10]*p.z + m.m[11],
            m.m[12]*p.x + m.m[13]*p.y + m.m[14]*p.z + m.m[15]
        };
}

namespace {

ViewPoint makeViewFromProjView(const float legacy[16],const float viewmatrix[16]) {
    ViewPoint view{};
    view.view = Mat4::identity();
    view.projection.m[0] = legacy[0]; view.projection.m[1] = legacy[1]; view.projection.m[2] = legacy[2]; view.projection.m[3] = legacy[3];
    view.projection.m[4] = legacy[4]; view.projection.m[5] = legacy[5]; view.projection.m[6] = legacy[6]; view.projection.m[7] = legacy[7];
    view.projection.m[8] = legacy[8]; view.projection.m[9] = legacy[9]; view.projection.m[10] = legacy[10]; view.projection.m[11] = legacy[11];
    view.projection.m[12] = legacy[12]; view.projection.m[13] = legacy[13]; view.projection.m[14] = legacy[14]; view.projection.m[15] = legacy[15];


    view.view.m[0] = viewmatrix[0]; view.view.m[1] = viewmatrix[1]; view.view.m[2] = viewmatrix[2]; view.view.m[3] = viewmatrix[3];
    view.view.m[4] = viewmatrix[4]; view.view.m[5] = viewmatrix[5]; view.view.m[6] = viewmatrix[6]; view.view.m[7] = viewmatrix[7];
    view.view.m[8] = viewmatrix[8]; view.view.m[9] = viewmatrix[9]; view.view.m[10] = viewmatrix[10]; view.view.m[11] = viewmatrix[11];
    view.view.m[12] = viewmatrix[12]; view.view.m[13] = viewmatrix[13]; view.view.m[14] = viewmatrix[14]; view.view.m[15] = viewmatrix[15];
    return view;
}
void testCompiledSerialization() {

    std::cerr<<" create octree"<<std::endl;
    OcclusionOctree tree({{-10, -10, -10}, {10, 10, 10}},8, 0);
      tree.insert(Triangle{{-1.8f, -8.8f, -5.2f}, {.8f, -8.8f, -5.2f}, {0, 8.8f, -10.2f}});
   tree.insert(Triangle{{5.7f, 1.7f, -1.3f}, {.7f, 7.7f, -1.3f}, {0, -8.7f, -1.3f}});
  //  tree.insert(Triangle{{15.7f, -10.7f, -1.3f}, {.7f, -70.7f, 1.3f}, {0, -8.7f, 10.3f}});

  std::vector<Triangle> tris;
   tree.insert(Triangle{{ 0.0,  0.5, -5.0},{-0.5, -0.5, -5.0},{ 0.5, -0.5, -5.0}});
  tree.insert(Triangle{{ 0.0,  0.5, -2.0},{-0.5, -0.5, -2.0} ,{ 0.5, -0.5, -2.0}});

   tris.push_back(Triangle{{ 0.0,  0.5, -5.0},{-0.5, -0.5, -5.0},{ 0.5, -0.5, -5.0}});
   tris.push_back(Triangle{{ 0.0,  0.5, -2.0},{-0.5, -0.5, -2.0} ,{ 0.5, -0.5, -2.0}});
    std::cerr<<tree.leaves().size()<<std::endl;

    std::cerr<<" compiling octree"<<std::endl;
    CompiledOctree &compiled = *tree.compile();

    std::cerr<<" compiled octree"<<std::endl;
    assert(compiled.nodeCount() == tree.leaves().size());
    assert(compiled.locate({-0.080155, 0.07, 0.0656662}) != InvalidNode);

CompiledOctreeSerializer ser;
ser.saveToFile(compiled,"octrecompiled.db");

CompiledOctree compiledread;
ser.loadFromFile(compiledread,"octrecompiled.db");
assert(compiled.nodeCount() == compiledread.nodeCount());

}

void testResolutionValidation() {
    bool rejected = false;
    try { OctreeRasterizer invalid(255, 256); } catch (const std::invalid_argument&) { rejected = true; }
    assert(rejected);
}
}

int main() {
    testResolutionValidation();
    testCompiledSerialization();
    std::cout << "Compiled Octree Serialization tests passed\n";
    return 0;
}
