#include "../inc/object.h"
#include <random>
#include <Eigen/Geometry>
#include <iostream>
#include <algorithm>
#include "../inc/macros.inc"
#include "../inc/beam.h"
#include "../inc/material.h"

using namespace Eigen;
using namespace std;

Object::Object()
{
  mtl_ = nullptr;
  for (size_t itr = 0; itr < 3; ++itr)
  {
    vertex_[itr] = Vector3d(0, 0, 0);
    norm_[itr] = Vector3d(0, 0, 0);
  }
}

Object::~Object()
{
  
}

Vector3d Object::GetLe() const
{
  return Vector3d(0, 0, 0);
}

Vector3d Object::GetCenter() const
{
  return (vertex_[0] + vertex_[1] + vertex_[2]) / 3.0;
}

Vector3d Object::GetBoxLength() const
{
  Vector3d minB;
  Vector3d maxB;
  GetBoundingBox(minB, maxB);
  return maxB - minB;
}

int Object::set_mtl(Material *const mtl)
{
  mtl_ = mtl;
  return 0;
}
Vector3d Object::GetVertex(size_t itr) const
{
  return vertex_[itr];
}

int Object::set_vertex(const double *const v, size_t itr)
{
  for (size_t i = 0; i < 3; ++i)
  {
    vertex_[itr][i] = v[i];
  }

  return 0;
}

int Object::set_norm(const double *const n, size_t itr)
{
  for (size_t i = 0; i < 3; ++i)
  {
    norm_[itr][i] = n[i];
  }

  return 0;
}

int Object::GetBoundingBox(Vector3d &minB, Vector3d &maxB) const
{
  for (size_t itr = 0; itr < 3; ++itr)
  {
    minB[itr] = min(vertex_[0][itr],
                min(vertex_[1][itr], vertex_[2][itr]));
    maxB[itr] = max(vertex_[0][itr],
                max(vertex_[1][itr], vertex_[2][itr]));
  }

  return 0;
}

Vector3d Object::GetNorm() const
{
  const Vector3d e1 = vertex_[1] - vertex_[0];
  const Vector3d e2 = vertex_[2] - vertex_[0];
  return e1.cross(e2).normalized();
}

double Object::GetIntersect(const Vector3d &o, const Vector3d &d) const
{
  const Vector3d e1 = vertex_[1] - vertex_[0];
  const Vector3d e2 = vertex_[2] - vertex_[0];
  const Vector3d p = o - vertex_[0];

  const double de = d.cross(e2).dot(e1);
  if (de < EPS)
    return numeric_limits<double>::max();

  const double t_no = p.cross(e1).dot(e2);
  const double beta_no = d.cross(e2).dot(p);
  const double gamma_no = p.cross(e1).dot(d);
  const double t = t_no / de;
  const double beta = beta_no / de;
  const double gamma = gamma_no / de;
  const double alpha = 1 - beta - gamma;
  if (t <= EPS)
    return numeric_limits<double>::max();
    
  const Vector3d obj_norm = GetNorm();
  assert(d.dot(obj_norm) <= 0);
  if (d.dot(obj_norm) > 0)
    return numeric_limits<double>::max();
  
  if (   (beta >= 0  && beta <= 1)
         && (gamma >= 0 && gamma <= 1)
         && (alpha >= 0 && alpha <= 1))
    return t;
  else
    return numeric_limits<double>::max();
}

