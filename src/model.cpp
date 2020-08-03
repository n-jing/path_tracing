#include "../inc/model.h"
#include <iostream>
#include "../thirdparty/inc/tiny_obj_loader.h"
#include "../inc/macros.inc"
#include "../inc/kd_tree.h"
#include "../inc/material.h"
#include "../inc/object.h"

using namespace Eigen;
using namespace std;

Model::Model(const char *const path)
{
  build(path);
  kd_tree_ = new jing_m::KdTree(obj_ptr_vec_);
}

Model::~Model()
{
  delete kd_tree_;
  for (size_t itr = 0; itr < obj_ptr_vec_.size(); ++itr)
  {
    delete obj_ptr_vec_[itr];
  }
  for (size_t itr = 0; itr < mtl_ptr_vec_.size(); ++itr)
  {
    delete mtl_ptr_vec_[itr];
  }
}

int Model::build(const char *const path)
{
  tinyobj::attrib_t attrib;
  vector<tinyobj::shape_t> shapes(5000);
  vector<tinyobj::material_t> materials(5000);
  string err;
  const string path_dir(string(path).substr(0, string(path).rfind("/") + 1));

  bool isLoad = LoadObj(&attrib, &shapes, &materials, &err, path, path_dir.c_str(), true);
  if (!isLoad)
  {
    cerr << err << endl;
    throw -1;
  }

  load_model(attrib, shapes, materials);

  return 0;
}

int Model::load_model(const tinyobj::attrib_t &attrib, const vector<tinyobj::shape_t> &shapes, const vector<tinyobj::material_t> &materials)
{
  mtl_ptr_vec_.clear();
  mtl_ptr_vec_.reserve(materials.size());
  for (size_t itr = 0; itr < materials.size(); ++itr)
  {
    const tinyobj::material_t &mtl = materials[itr];
    Material * mtl_ptr = new Material;
    mtl_ptr->set_name(mtl.name);
    mtl_ptr->set_illum(mtl.illum);
    mtl_ptr->set_ns(mtl.shininess);
    mtl_ptr->set_ni(mtl.ior);

    mtl_ptr->set_kd(mtl.diffuse);
    mtl_ptr->set_ka(mtl.ambient);
    mtl_ptr->set_ks(mtl.specular);
    mtl_ptr->set_tf(mtl.transmittance);
    mtl_ptr->set_type();
    mtl_ptr_vec_.push_back(mtl_ptr);
  }
  size_t sum = 0;
  for (auto shp : shapes)
  {
    sum += shp.mesh.num_face_vertices.size();
  }
  obj_ptr_vec_.clear();
  obj_ptr_vec_.reserve(sum);
  for (auto shp : shapes)
  {
    const tinyobj::mesh_t &m = shp.mesh;
    for (size_t iter = 0; iter < m.num_face_vertices.size(); ++iter)
    {
      assert(m.num_face_vertices[iter] == 3);
      load_one(attrib, m, iter);
    }
  }

  return 0;
}

int Model::load_one(const tinyobj::attrib_t &attrib, const tinyobj::mesh_t &m, size_t iter)
{
  int ver_idx[3] = {0, 0, 0};
  int norm_idx[3] = {0, 0, 0};
  tinyobj::index_t triangle[3] = {m.indices[3 * iter],
                                  m.indices[3 * iter + 1],
                                  m.indices[3 * iter + 2]};


  for (size_t it = 0; it < 3; ++it)
  {
    ver_idx[it] = triangle[it].vertex_index;
    norm_idx[it] = triangle[it].normal_index;
  }
  const int mtl_idx = m.material_ids[iter];
  assert(mtl_idx != -1);
  assert(mtl_idx < mtl_ptr_vec_.size());

  const Material &obj_mtl = *mtl_ptr_vec_[mtl_idx];
  Object *obj_ptr = nullptr;
  switch (obj_mtl.GetType())
  {
  case diffuse:
    obj_ptr = new DiffObject;
    break;
  case reflection:
    obj_ptr = new RefObject;
    break;
  case refraction:
    obj_ptr = new RefraObject;
    break;
  case emission:
    obj_ptr = new EmiObject;
    break;
  }
  obj_ptr->set_mtl(mtl_ptr_vec_[mtl_idx]);
  for (size_t iter = 0; iter < 3; ++iter)
  {
    const int v_idx = ver_idx[iter];
    double v[3] = {attrib.vertices[3 * v_idx],
                   attrib.vertices[3 * v_idx + 1],
                   attrib.vertices[3 * v_idx + 2]};
    obj_ptr->set_vertex(v, iter);
    const int n_idx = norm_idx[iter];
    double n[3] = {attrib.normals[3 * n_idx],
                   attrib.normals[3 * n_idx + 1],
                   attrib.normals[3 * n_idx + 2]};
    obj_ptr->set_norm(n, iter);
  }

  obj_ptr_vec_.push_back(obj_ptr);
  return 0;
}

int Model::GetNearObj(const Beam &b, size_t &idx, double &dst) const
{
  kd_tree_->GetNearObj(obj_ptr_vec_, b, idx, dst);

  return 0;
}

Object *Model::GetObject(size_t idx) const
{
  return obj_ptr_vec_[idx];
}
