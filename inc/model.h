#ifndef MODEL_JJ_H
#define MODEL_JJ_H

#include <vector>
#include <Eigen/Core>
#include "kd_tree.h"
#include "..//thirdparty/inc/tiny_obj_loader.h"
class Beam;
class Object;
class Material;

class Model
{
public:
  Model(const char *const path);
  ~Model();
  int GetNearObj(const Beam &b, size_t &idx, double &dst) const;
  Object *GetObject(size_t idx) const;

private:
  int build(const char *const path);
  int load_model(const tinyobj::attrib_t &attrib, const std::vector<tinyobj::shape_t> &shapes, const std::vector<tinyobj::material_t> &materials);
  int load_one(const tinyobj::attrib_t &attrib, const tinyobj::mesh_t &m, size_t iter);

private:
  std::vector<Object*> obj_ptr_vec_;
  std::vector<Material*> mtl_ptr_vec_;
  jing_m::KdTree *kd_tree_;
};

#endif // MODEL_JJ_H
