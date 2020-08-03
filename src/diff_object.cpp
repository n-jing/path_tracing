#include "../inc/object.h"
#include <random>
#include <iostream>
#include <fstream>
#include <Eigen/Geometry>
#include "../inc/macros.inc"
#include "../inc/beam.h"
#include "../inc/material.h"

using namespace Eigen;
using namespace std;

Beam DiffObject::GetRefBeam(const Beam &b) const
{
  double t = GetIntersect(b.GetOrigin(), b.GetDir());
  assert(t != numeric_limits<double>::max());
  Beam new_b;
  new_b.set_origin(b.GetOrigin() + t * b.GetDir());

  static unsigned int seed = 0;
  seed += 1;
  default_random_engine e(seed);
  uniform_real_distribution<double> u(0, 1);

  const double theta = 0.5 * acos(1 - 2 * u(e));
  // const double theta = acos(pow(u(e), 1.0 / 3)); 
  const double phi = 2 * PI * u(e);

  const double x = sin(theta) * cos(phi);
  const double y = sin(theta) * sin(phi);
  const double z = cos(theta);
  
  const Vector3d e1 = vertex_[1] - vertex_[0];
  const Vector3d coords_x = e1.normalized();
  const Vector3d coords_z = GetNorm();
  const Vector3d coords_y = coords_z.cross(coords_x);
  const Vector3d d = x * coords_x + y * coords_y + z * coords_z;
  assert(fabs(d.norm() - 1) < EPS);

  
  new_b.set_dir(d);
  // new_b.set_dir(GetNorm());

  return new_b;
}

Vector3d DiffObject::GetRefFactorDirFromIt(const Vector3d &in, const Vector3d &out) const
{
  const double cos_alpha = in.dot(GetNorm());
  if (cos_alpha <= 0)
    return Vector3d(0, 0, 0);
  assert(cos_alpha >= 0);

  return cos_alpha * mtl_->GetKd();
}

