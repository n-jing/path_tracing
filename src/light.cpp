#include "../inc/light.h"
#include <cmath>
#include <random>
#include <iostream>
#include <algorithm>
#include <fstream>
#include "../inc/macros.inc"
#include "../inc/beam.h"

using namespace Eigen;
using namespace std;

Light::~Light()
{
  
}

Vector3d Light::GetLe() const
{
  return le_;
}

Vector3d Light::GetCenter() const
{
  return center_;
}

SphereLight::~SphereLight()
{
  
}

QuardLight::~QuardLight()
{
  
}

double Light::GetRadius() const
{
  return numeric_limits<double>::max();
}

double SphereLight::IsIntersect(const Beam &b) const
{
  const Vector3d oc = GetCenter() - b.GetOrigin();
  const Vector3d oc_unit = oc.normalized();
  const double cos_alpha = oc_unit.dot(b.GetDir());
  if (cos_alpha < 0)
    return numeric_limits<double>::max();
  assert(cos_alpha <= 1);
  const double sin_alpha = sqrt(1 - cos_alpha * cos_alpha);
  double dst = oc.norm() * sin_alpha;
  if (dst >= radius_ + EPS)
    return numeric_limits<double>::max();

  double half_chord = sqrt(radius_ * radius_ - dst * dst);

  if (oc.norm() * cos_alpha - half_chord < 0 + EPS)
    return numeric_limits<double>::max();
  
  return oc.norm() * cos_alpha - half_chord;
}

double QuardLight::IsIntersect(const Beam &b) const
{
  const double o_dot_n = b.GetOrigin().dot(norm_);
  const double d_dot_n = b.GetDir().dot(norm_);
  const double ct_dot_n = GetCenter().dot(norm_);
  const double no = ct_dot_n - o_dot_n;
  const double de = d_dot_n;
  const double t = no / de;
  if (t < 0)
    return numeric_limits<double>::max();

  const Vector3d pt = b.GetOrigin() + t * b.GetDir();
  const Vector3d dst = pt - GetCenter();

  if (fabs(dst.dot(cross_a_)) > width_ / 2.0
      || fabs(dst.dot(cross_b_)) > height_ / 2.0)
    return numeric_limits<double>::max();
  else
    return t;
}


// bool SphereLight::IsShade(const Eigen::Vector3d &o, const Eigen::Vector3d &d, double dst) const
// {
//   const Vector3d oc = GetCenter() - o;
//   const Vector3d oc_unit = oc.normalized();
//   const double cos_alpha = oc_unit.dot(d);
//   assert(cos_alpha >= 0);
//   assert(cos_alpha <= 1);
//   const double sin_alpha = sqrt(1 - cos_alpha);
//   double dr = oc.norm() * sin_alpha;
//   assert(dr <= radius_);
//   const double chord = sqrt(radius_ * radius_ - dr * dr);
//   const double t = oc.norm() * cos_alpha - chord;

//   if (dst < t + EPS)
//     return false;
//   else
//     return true;
// }

bool QuardLight::IsShade(const Eigen::Vector3d &o, const Eigen::Vector3d &d, double dst) const
{
  // (o + t * d - center_).dot(norm_) = 0;
  const double o_dot_n = o.dot(norm_);
  const double d_dot_n = d.dot(norm_);
  const double ct_dot_n = GetCenter().dot(norm_);
  const double no = ct_dot_n - o_dot_n;
  const double de = d_dot_n;
  const double t = no / de;
  assert(t >= 0);

  if (dst < t - EPS)
    return false;
  else
    return true;
}

Vector3d SphereLight::GetRandomPointLight() const
{
  static unsigned int seed = 0;
  ++seed;
  default_random_engine e(seed);
  uniform_real_distribution<double> u(0, 1);
  const double phi = acos(2 * u(e) - 1);
  const double theta = 2 * PI * u(e);

  const double x = sin(phi) * cos(theta);
  const double y = sin(phi) * sin(theta);
  const double z = cos(phi);

  assert(fabs(x * x + y * y + z * z - 1) < EPS);
  const Vector3d p(radius_ * x, radius_ * y, radius_ * z);

  return GetCenter() + p;
}

Vector3d QuardLight::GetRandomPointLight() const
{
  static unsigned int seed = 0;
  ++seed;
  default_random_engine e(seed);
  uniform_real_distribution<double> u(-0.5, 0.5);

  const double x = u(e) * width_;
  const double y = u(e) * height_;
  const Vector3d p = x * cross_a_ + y * cross_b_;

  return GetCenter() + p;
}

Vector3d SphereLight::GetNorm(const Vector3d &p) const
{
  return (p - GetCenter()).normalized();
}

Vector3d QuardLight::GetNorm(const Vector3d &p) const
{
  return norm_;
}


Vector3d SphereLight::LightLe(const Vector3d &ltpt, const Vector3d &o, const Vector3d &o_norm) const
{
  const double length = (ltpt - o).norm();
  const Vector3d dir = (o - ltpt).normalized();
  const double alpha = dir.dot(GetNorm(ltpt));
  const double beta = -dir.dot(o_norm);
  if (alpha <= 0 || beta <= 0)
    return Vector3d(0, 0, 0);
  // assert(alpha >= 0);
  // assert(beta >= 0);
  const double factor = alpha * beta / (length * length) / PI;

  return factor * GetLe();
  // return le_;
}


Vector3d QuardLight::LightLe(const Vector3d &ltpt, const Vector3d &o, const Vector3d &o_norm) const
{
  const double length = (ltpt - o).norm();
  const Vector3d dir = (o - ltpt).normalized();
  double alpha = fabs(dir.dot(GetNorm(ltpt)));
  double beta = -dir.dot(o_norm);
  if (alpha < 0)
    alpha = 0;
  if (beta < 0)
    beta = 0;
  const double factor = alpha * beta  / (length * length);

  return factor * GetLe();
}

double SphereLight::GetRadius() const
{
  return radius_;
}
