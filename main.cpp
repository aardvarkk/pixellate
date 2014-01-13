#include <assimp/IOSystem.hpp>
#include <assimp/Exporter.hpp>
#include <assimp/Importer.hpp>
#include <assimp/Scene.h>
#include <cstdlib>
#include <iostream>
#include <vector>

#include "geomalgorithms.h"

static const float kMaxDimCM    = 50.f;  // Maximum dimension of the output in centimetres
static const float kBlockSizeCM = 1.27f; // Block size in centimetres (0.5in = 0.0127m)
static const int   kStopNum     = 0;     // Only make this many cubes (faster for debug)

void calc_scene_stats(
  aiScene* scene,
  Point& pmin,
  Point& pmax,
  Point& dims,
  Point& centre
  )
{
  // Find out the min/max coords in all dimensions
  pmin = Point(+FLT_MAX);
  pmax = Point(-FLT_MAX);
  for (unsigned int i = 0; i < scene->mNumMeshes; ++i) {
    auto m = scene->mMeshes[i];
    for (unsigned int j = 0; j < m->mNumVertices; ++j) {
      auto v = m->mVertices[j];
      pmin.x = std::min(pmin.x, v.x);
      pmin.y = std::min(pmin.y, v.y);
      pmin.z = std::min(pmin.z, v.z);
      pmax.x = std::max(pmax.x, v.x);
      pmax.y = std::max(pmax.y, v.y);
      pmax.z = std::max(pmax.z, v.z);
    }
  }

  dims   = pmax - pmin;
  centre = pmin + pmax;
  centre.x /= 2;
  centre.y /= 2;
  centre.z /= 2;
}

// Collada uses RHS, 
void add_cubes(aiScene* scene, std::vector<Point> locs, float cube_size)
{
  // Get the half cube size
  auto hc = cube_size / 2.f;

  // Get a handy mesh reference
  auto m          = *scene->mMeshes;

  // Vertices -- 8 per cube
  m->mNumVertices = 8 * locs.size();
  m->mVertices    = new aiVector3D[m->mNumVertices];

  // Faces -- use quads, so 6 per cube
  m->mNumFaces    = 6 * locs.size();
  m->mFaces       = new aiFace[m->mNumFaces];

  // Iterate through the locations and add the cubes
  int fidx = 0;
  int vidx = 0;
  for (auto p : locs) {

    // Vertices
    m->mVertices[vidx+0] = aiVector3D(p.x - hc, p.y - hc, p.z - hc);
    m->mVertices[vidx+1] = aiVector3D(p.x - hc, p.y - hc, p.z + hc);
    m->mVertices[vidx+2] = aiVector3D(p.x - hc, p.y + hc, p.z - hc);
    m->mVertices[vidx+3] = aiVector3D(p.x - hc, p.y + hc, p.z + hc);
    m->mVertices[vidx+4] = aiVector3D(p.x + hc, p.y - hc, p.z - hc);
    m->mVertices[vidx+5] = aiVector3D(p.x + hc, p.y - hc, p.z + hc);
    m->mVertices[vidx+6] = aiVector3D(p.x + hc, p.y + hc, p.z - hc);
    m->mVertices[vidx+7] = aiVector3D(p.x + hc, p.y + hc, p.z + hc);

    // Faces
    m->mFaces[fidx+0].mNumIndices = 4;
    m->mFaces[fidx+0].mIndices    = new unsigned int[m->mFaces[fidx+0].mNumIndices];
    m->mFaces[fidx+0].mIndices[0] = vidx+0;
    m->mFaces[fidx+0].mIndices[1] = vidx+1;
    m->mFaces[fidx+0].mIndices[2] = vidx+3;
    m->mFaces[fidx+0].mIndices[3] = vidx+2;

    m->mFaces[fidx+1].mNumIndices = 4;
    m->mFaces[fidx+1].mIndices    = new unsigned int[m->mFaces[fidx+1].mNumIndices];
    m->mFaces[fidx+1].mIndices[0] = vidx+0;
    m->mFaces[fidx+1].mIndices[1] = vidx+2;
    m->mFaces[fidx+1].mIndices[2] = vidx+6;
    m->mFaces[fidx+1].mIndices[3] = vidx+4;

    m->mFaces[fidx+2].mNumIndices = 4;
    m->mFaces[fidx+2].mIndices    = new unsigned int[m->mFaces[fidx+2].mNumIndices];
    m->mFaces[fidx+2].mIndices[0] = vidx+0;
    m->mFaces[fidx+2].mIndices[1] = vidx+4;
    m->mFaces[fidx+2].mIndices[2] = vidx+5;
    m->mFaces[fidx+2].mIndices[3] = vidx+1;

    m->mFaces[fidx+3].mNumIndices = 4;
    m->mFaces[fidx+3].mIndices    = new unsigned int[m->mFaces[fidx+3].mNumIndices];
    m->mFaces[fidx+3].mIndices[0] = vidx+1;
    m->mFaces[fidx+3].mIndices[1] = vidx+5;
    m->mFaces[fidx+3].mIndices[2] = vidx+7;
    m->mFaces[fidx+3].mIndices[3] = vidx+3;

    m->mFaces[fidx+4].mNumIndices = 4;
    m->mFaces[fidx+4].mIndices    = new unsigned int[m->mFaces[fidx+4].mNumIndices];
    m->mFaces[fidx+4].mIndices[0] = vidx+2;
    m->mFaces[fidx+4].mIndices[1] = vidx+3;
    m->mFaces[fidx+4].mIndices[2] = vidx+7;
    m->mFaces[fidx+4].mIndices[3] = vidx+6;

    m->mFaces[fidx+5].mNumIndices = 4;
    m->mFaces[fidx+5].mIndices    = new unsigned int[m->mFaces[fidx+5].mNumIndices];
    m->mFaces[fidx+5].mIndices[0] = vidx+4;
    m->mFaces[fidx+5].mIndices[1] = vidx+6;
    m->mFaces[fidx+5].mIndices[2] = vidx+7;
    m->mFaces[fidx+5].mIndices[3] = vidx+5;

    vidx += 8;
    fidx += 6;
  }
}

int main(int argc, char* argv[])
{
  Assimp::Importer importer;
  auto scene = importer.ReadFile("Moosehead.obj", 0);
  if (!scene) {
    std::cerr << importer.GetErrorString() << std::endl;
    return EXIT_FAILURE;
  }

  if (scene->mNumMeshes != 1) {
    std::cerr << "Only support a single mesh, and this model has " << scene->mNumMeshes << std::endl;
    return EXIT_FAILURE;
  }

  if (!(*scene->mMeshes)->HasFaces()) {
    std::cerr << "Model has no faces" << std::endl;
    return EXIT_FAILURE;
  }

  Assimp::Exporter exporter;
  for (size_t i = 0; i < exporter.GetExportFormatCount(); ++i) {
    auto f = exporter.GetExportFormatDescription(i);
    std::cout << f->id << std::endl;
    std::cout << f->fileExtension << std::endl;
    std::cout << f->description << std::endl;
    std::cout << std::endl;
  }

  aiScene* pixellated;
  aiCopyScene(scene, &pixellated);

  // Convenience
  auto px = pixellated;

  // Get initial scaling values
  Point pmin, pmax, dims, centre;
  calc_scene_stats(px, pmin, pmax, dims, centre);

  // Shift centre of overall bounding box to (0,0,0) and scale so our max dimension is 
  auto max_dim = std::max(dims.x, std::max(dims.y, dims.z));
  auto scaling = kMaxDimCM / max_dim;
  for (unsigned int i = 0; i < px->mNumMeshes; ++i) {
    auto m = px->mMeshes[i];
    for (unsigned int j = 0; j < m->mNumVertices; ++j) {
      auto v = &m->mVertices[j];
      v->x = (v->x - centre.x) * scaling;
      v->y = (v->y - centre.y) * scaling;
      v->z = (v->z - centre.z) * scaling;
    }
  }

  // Recalculate after adjustment
  calc_scene_stats(px, pmin, pmax, dims, centre);

  // How many blocks are we going to use?
  int min_x = static_cast<int>(floor(pmin.x / kBlockSizeCM));
  int max_x = static_cast<int>(ceil (pmax.x / kBlockSizeCM));
  int min_y = static_cast<int>(floor(pmin.y / kBlockSizeCM));
  int max_y = static_cast<int>(ceil (pmax.y / kBlockSizeCM));
  int min_z = static_cast<int>(floor(pmin.z / kBlockSizeCM));
  int max_z = static_cast<int>(ceil (pmax.z / kBlockSizeCM));
  
  // Turn all meshes into single list of triangles
  std::vector<Triangle> triangles;
  for (unsigned int i = 0; i < px->mNumMeshes; ++i) {
    auto m = px->mMeshes[i];
    for (unsigned int j = 0; j < m->mNumFaces; ++j) {
      auto f = m->mFaces[j];
      if (f.mNumIndices != 3) {
        std::cerr << "Ack." << std::endl;
        return EXIT_FAILURE;
      }
      Triangle t;
      t.V0 = Point(m->mVertices[f.mIndices[0]].x, m->mVertices[f.mIndices[0]].y, m->mVertices[f.mIndices[0]].z);
      t.V1 = Point(m->mVertices[f.mIndices[1]].x, m->mVertices[f.mIndices[1]].y, m->mVertices[f.mIndices[1]].z);
      t.V2 = Point(m->mVertices[f.mIndices[2]].x, m->mVertices[f.mIndices[2]].y, m->mVertices[f.mIndices[2]].z);
      triangles.push_back(t);
    }
  }

  // Create our new mesh -- it'll be only one
  px->mNumMeshes = 1;
  *px->mMeshes   = new aiMesh;
  std::vector<Point> cube_locs;

  int iter  = 0;
  int total = (max_x - min_x + 1) * (max_y - min_y + 1) * (max_z - min_z + 1);

  // Examine the 3D space by the centre of each cube
  // Check both + and - of each of these...
  for (int x = min_x; x <= max_x; ++x) {
    for (int y = min_y; y <= max_y; ++y) {
      for (int z = min_z; z <= max_z; ++z) {

        //x = y = z = 0;

        // Centre of the potential cube
        Point pt = Point(x, y, z) * kBlockSizeCM;

        // Cast rays in multiple directions (+X, -X, etc.)
        Ray rpx;
        rpx.P0 = pt;
        rpx.P1 = pt + Point(+1, 0, 0);

        Ray rnx;
        rnx.P0 = pt;
        rnx.P1 = pt + Point(-1, 0, 0);

        // Go through ALL triangles in the model!
        int ray_intersects_px = 0;
        int ray_intersects_nx = 0;

        for (auto t : triangles) {
          // Calculate a ray from this point (does direction matter?)
          // Can check along each axis to be careful...
          Point intersection_pt;

          auto intersects_px = intersect3D_RayTriangle(rpx, t, &intersection_pt);
          if (intersects_px > 0) {
            ++ray_intersects_px;
          }
          auto intersects_nx = intersect3D_RayTriangle(rnx, t, &intersection_pt);
          if (intersects_nx > 0) {
            ++ray_intersects_nx;
          }
        }

        // If we have an odd number of intersections, we should put a cube around this point
        // as it should be "inside" the model
        // Try checking rays in multiple directions for safety
        if (
          (ray_intersects_px % 2 != 0) && 
          (ray_intersects_nx % 2 != 0)
          ) {
          cube_locs.push_back(pt);
          //add_cube(px, pt, kBlockSizeCM);
        }

        // If it intersects, we want a block here...
        // Otherwise, we want no block here, so just continue...

        std::cout << "Finished " << ++iter << " of " << total << " iterations" << std::endl;
        std::cout << cube_locs.size() << " cubes" << std::endl;

        // Don't do it all if we request to stop short...
        if (kStopNum && cube_locs.size() >= kStopNum) {
          goto hop_out;
        }
      }
    }
  }

hop_out:

  // Add all of the cubes
  add_cubes(px, cube_locs, kBlockSizeCM);

  exporter.Export(px, "collada", "pixellated.dae");

  for (unsigned int i = 0; i < px->mNumMeshes; ++i) {
    delete px->mMeshes[i];
  }

  return EXIT_SUCCESS;
}