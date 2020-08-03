#include "../inc/beam.h"

using namespace Eigen;

Beam::Beam()
{
  o_ = Vector3d(0, 0, 0);
  d_ = Vector3d(1, 0, 0);
}

Vector3d Beam::GetOrigin() const
{
  return o_;
}

Vector3d Beam::GetDir() const
{
  return d_;
}

int Beam::set_origin(const Vector3d &o)
{
  o_ = o;

  return 0;
}

int Beam::set_dir(const Vector3d &d)
{
  d_ = d;

  return 0;
}
