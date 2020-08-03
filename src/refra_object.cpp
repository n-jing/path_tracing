#include "../inc/object.h"
#include <random>
#include <Eigen/Geometry>
#include "../inc/macros.inc"
#include "../inc/beam.h"
#include "../inc/material.h"

using namespace Eigen;
using namespace std;

double RefraObject::GetIntersect(const Vector3d &o, const Vector3d &d) const
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
    
  if (   (beta >= 0  && beta <= 1)
         && (gamma >= 0 && gamma <= 1)
         && (alpha >= 0 && alpha <= 1))
    return t;
  else
    return numeric_limits<double>::max();
}


Beam RefraObject::GetRefBeam(const Beam &b) const
{
  double t = GetIntersect(b.GetOrigin(), b.GetDir());
  assert(t != numeric_limits<double>::max());
  Beam new_b;
  new_b.set_origin(b.GetOrigin() + t * b.GetDir());

  if (b.GetDir().dot(GetNorm()) <= 0)
    new_b.set_dir(OutRef(b));
  else
    new_b.set_dir(InnerRef(b));
  return new_b;
}

Vector3d RefraObject::OutRef(const Beam &b) const
{
  static unsigned int seed = 0;
  seed += 10;
  default_random_engine e(seed);
  uniform_real_distribution<double> u(0, 1);

  if (u(e) < 0.5)
  {
    double cos_theta = mtl_->GetNs() == 0 ? u(e) : pow(u(e), 1.0 / (mtl_->GetNs() + 1));
    double sin_theta = sqrt(1 - cos_theta * cos_theta);
    double phi = 2 * PI * u(e);
    double cos_phi = cos(phi);
    double sin_phi = sin(phi);
    
    double x = sin_theta * sin_phi;
    double y = sin_theta * cos_phi;
    double z = cos_theta;

    const Vector3d e1 = vertex_[1] - vertex_[0];
    const Vector3d coords_x = e1.normalized();
    const Vector3d coords_z = GetNorm();
    const Vector3d coords_y = coords_z.cross(coords_x);
    const Vector3d d = x * coords_x + y * coords_y + z * coords_z;
    assert(fabs(d.norm() - 1) < EPS);

    return d;
  }
  else
  {
    const Vector3d n = GetNorm();
    const Vector3d &dir = b.GetDir();
    double cos_theta1 = -dir.dot(n);
    assert(cos_theta1 <= 1 && cos_theta1 >= 0);
    double sin_theta1 = sqrt(1 - cos_theta1 * cos_theta1);
    double sin_theta2 = sin_theta1 / mtl_->GetNi();
    double cos_theta2 = sqrt(1 - sin_theta2 * sin_theta2);
      
    const Vector3d para = (dir + cos_theta1 * n).normalized();
    const Vector3d d = -cos_theta2 * n + sin_theta2 * para;
    assert(fabs(d.norm() - 1) < EPS);

    return d;
  }
}

Vector3d RefraObject::InnerRef(const Beam &b) const
{
  static unsigned int seed = 0;
  seed += 10;
  default_random_engine e(seed);
  uniform_real_distribution<double> u(0, 1);

  const Vector3d n = GetNorm();
  const Vector3d dir = b.GetDir();
  double cos_theta1 = n.dot(dir);
  double sin_theta1 = sqrt(1 - cos_theta1 * cos_theta1);
  if (sin_theta1 * mtl_->GetNi() >= 1)
  {
    const Vector3d d = dir - cos_theta1 * n; - cos_theta1 * n;
    return d;
  }
  else
  {
    if (u(e) > 0.5)
    {
      double sin_theta2 = sin_theta1 * mtl_->GetNi();
      double cos_theta2 = sqrt(1 - sin_theta2 * sin_theta2);
      const Vector3d para = (dir - cos_theta1 * n).normalized();
      const Vector3d d = cos_theta2 * n + sin_theta2 * para;

      return d;
    }
    else
    {
      const Vector3d d = dir - cos_theta1 * n; - cos_theta1 * n;

      return d;
    }
  }
}


Vector3d RefraObject::GetRefFactorDirFromIt(const Vector3d &in, const Vector3d &out) const
{
  if (in.dot(GetNorm()) <= 0)
    return Vector3d(1, 1, 1);
  else
  {
    if (out.dot(GetNorm()) >= 0)
    {
      double R = (mtl_->GetNi() - 1) / (mtl_->GetNi() + 1);
      double R0 = pow(R, 2);
      double cos_slk = in.dot(GetNorm());
      assert(cos_slk >= 0);
      double ref_f = R0 + (1 - R0) * pow(1 - cos_slk, 5);

      double alpha = in.dot(GetNorm());
      assert(alpha >= 0);
      const Vector3d H = (in + out).normalized();
      double beta = H.dot(GetNorm());
      assert(beta >= 0);
      beta = pow(beta, this->mtl_->GetNs());
      return ref_f * alpha * mtl_->GetKd() + ref_f * beta * mtl_->GetKs();
    }
    else
    {
      return Vector3d(1, 1, 1) - mtl_->GetTf();
    }
  }
}
