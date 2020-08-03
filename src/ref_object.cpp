#include "../inc/object.h"
#include <random>
#include <iostream>
#include <Eigen/Geometry>
#include "../inc/macros.inc"
#include "../inc/beam.h"
#include "../inc/material.h"

using namespace Eigen;
using namespace std;

Beam RefObject::GetRefBeam(const Beam &b) const
{
  double t = GetIntersect(b.GetOrigin(), b.GetDir());
  assert(t != numeric_limits<double>::max());
  Beam new_b;
  new_b.set_origin(b.GetOrigin() + t * b.GetDir());


  double alpha = -b.GetDir().dot(GetNorm());
  assert(alpha > 0);
  Vector3d para = b.GetDir() + alpha * GetNorm();
  const Vector3d out = para + alpha * GetNorm();
  // new_b.set_dir(out);
  // assert((new_b.GetDir().norm() - 1) < EPS);
  // assert(new_b.GetDir().dot(GetNorm()) > 0);
  // double cos_angle = (new_b.GetDir() - b.GetDir()).normalized().dot(GetNorm());
  // assert(fabs(cos_angle - 1) < EPS);
  // return new_b;

  static unsigned int seed = 0;
  seed += 10;
  default_random_engine e(seed);
  uniform_real_distribution<double> u(0, 1);

  assert(mtl_);
  const Material &mtl = *mtl_;
  const double Ns = mtl.GetNs();
  const Vector3d Norm = GetNorm();
  
  size_t count = 0;
  while (true)
  {
    ++count;
    if (count > 100)
    {
      new_b.set_dir(out);
      return new_b;
    }
    double cos_theta = pow(u(e), 1.0 / (Ns + 1.0));
    double sin_theta = sqrt(1 - cos_theta * cos_theta);

    double phi = 2 * PI * u(e);
    double cos_phi = cos(phi);
    double sin_phi = sin(phi);

    double x = sin_theta * sin_phi;
    double y = sin_theta * cos_phi;
    double z = cos_theta;

    const Vector3d coords_z = out;
    size_t max_idx[3] = {0, 1, 2};
    size_t midx = *max_element(max_idx, max_idx + 3,
                               [&coords_z] (size_t l, size_t r) -> bool
                               {
                                 return coords_z[l] < coords_z[r];
                               });
    Vector3d coords_x;
    coords_x[midx] = coords_z[(midx + 1) % 3];
    coords_x[(midx + 1) % 3] = - coords_z[midx];
    coords_x[(midx + 2) % 3] = 0;
    coords_x.normalize();
    // coords_x = (GetVertex(1) - GetVertex(0)).normalized();
    const Vector3d coords_y = coords_z.cross(coords_x);
    const Vector3d d = x * coords_x + y * coords_y + z * coords_z;
    assert(fabs(d.norm() - 1) < EPS);
    if (d.dot(Norm) < EPS)
      continue;
    
    // Vector3d factor = GetRefFactorDirFromIt(d, -b.GetDir());
    // if (u(e) > max(factor[0], max(factor[1], factor[2])))
    //   continue;
    // else
    {
      new_b.set_dir(d);
      return new_b;
    }
  }

  return new_b;
}

Vector3d RefObject::GetRefFactorDirFromIt(const Vector3d &in, const Vector3d &out) const
{
  const Vector3d H = (in + out).normalized();
  double alpha = H.dot(GetNorm());
  if (alpha <= 0)
    return Vector3d(0, 0, 0);

  assert(alpha >= 0);
  alpha = pow(alpha, mtl_->GetNs());
  // alpha *= (mtl_->GetNs() + 2.0) / (2 * PI);

  double beta = in.dot(GetNorm());
  if (beta < 0)
    return Vector3d(0, 0, 0);

  assert(beta >= 0);

  return beta * mtl_->GetKd() + alpha * mtl_->GetKs();
}

