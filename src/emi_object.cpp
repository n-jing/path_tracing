#include "../inc/object.h"
#include <random>
#include <Eigen/Geometry>
#include "../inc/macros.inc"
#include "../inc/beam.h"
#include "../inc/material.h"

using namespace Eigen;
using namespace std;

Beam EmiObject::GetRefBeam(const Beam &b) const
{
  return b;
}

Vector3d EmiObject::GetRefFactorDirFromIt(const Eigen::Vector3d &in, const Eigen::Vector3d &out) const
{
  return Vector3d(0, 0, 0);
}

Vector3d EmiObject::GetLe() const
{
  Vector3d kd = mtl_->GetKd();
  const Vector3d ka = mtl_->GetKa();
  for (size_t itr = 0; itr < 3; ++itr)
    kd *= ka[itr];

  return kd;
}
