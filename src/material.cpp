#include "../inc/material.h"


using namespace Eigen;
using namespace std;


Material::Material()
{
  
}

Material::~Material()
{
  
}

MtlType Material::GetType() const
{
  return type_;
}

Vector3d Material::GetKd() const
{
  return Kd_;
}

Vector3d Material::GetKa() const
{
  return Ka_;
}

Vector3d Material::GetTf() const
{
  return Tf_;
}

Vector3d Material::GetKs() const
{
  return Ks_;
}

double Material::GetNs() const
{
  return Ns_;
}

double Material::GetNi() const
{
  return Ni_;
}

int Material::set_name(const string &name)
{
  name_ = name;

  return 0;
}

int Material::set_kd(const double kd[3])
{
  for (size_t itr = 0; itr < 3; ++itr)
  {
    Kd_[itr] = kd[itr];
  }

  return 0;
}

int Material::set_ka(const double ka[3])
{
  for (size_t itr = 0; itr < 3; ++itr)
  {
    Ka_[itr] = ka[itr];
  }

  return 0;
}

int Material::set_tf(const double tf[3])
{
  for (size_t itr = 0; itr < 3; ++itr)
  {
    Tf_[itr] = tf[itr];
  }

  return 0;
}

int Material::set_ks(const double ks[3])
{
  for (size_t itr = 0; itr < 3; ++itr)
  {
    Ks_[itr] = ks[itr];
  }

  return 0;
}

double Material::set_ns(double ns)
{
  Ns_ = ns;
  return Ns_;
}

double Material::set_ni(double ni)
{
  Ni_ = ni;
  return Ni_;
}

int Material::set_illum(int il)
{
  illum_ = il;
  return illum_;
}

int Material::set_type()
{
  if (Ka_.norm())
    type_ = emission;
  else if (Tf_ != Vector3d(1.0, 1.0, 1.0))
    type_ = refraction;
  else if (Ks_.norm())
    type_ = reflection;
  else
    type_ = diffuse;
  
  return 0;
}
