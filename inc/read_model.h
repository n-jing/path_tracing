#ifndef READ_MODEL_JJ_H
#define READ_MODEL_JJ_H


#include <vector>
#include "../thirdparty/inc/tiny_obj_loader.h"
class Scene;

int ReadModel(const char *dir, const char *mtl_base_dir, Scene &world);

int LoadScene(const tinyobj::attrib_t &attrib, const std::vector<tinyobj::shape_t> &shapes, const std::vector<tinyobj::material_t> &materials, Scene &world);

int AddOneShape(const tinyobj::attrib_t &attrib, const std::vector<tinyobj::material_t> &material, const tinyobj::mesh_t &m, size_t iter, Scene &world);

int AddOneTriangle(const tinyobj::attrib_t &attrib, const std::vector<tinyobj::material_t> &materials, int *const ver_idx, int *const norm_idx, int mtl_idx, Scene &world);

#endif // READ_MODEL_JJ_H
