#ifndef MATERIAL_JJ_H
#define MATERIAL_JJ_H

#include <string>
#include <Eigen/Core>

enum MtlType
{
  diffuse,
  reflection,
  refraction,
  emission
};

class Material
{
public:
  Material();
  ~Material();
  MtlType GetType() const;
  Eigen::Vector3d GetKd() const;
  Eigen::Vector3d GetKa() const;
  Eigen::Vector3d GetTf() const;
  Eigen::Vector3d GetKs() const;
  double GetNs() const; 
  double GetNi() const;

  int set_kd(const double kd[3]);
  int set_ka(const double ka[3]);
  int set_tf(const double tf[3]);
  int set_ks(const double ks[3]);
  int set_name(const std::string &name);
  double set_ns(double ns); 
  double set_ni(double ni);
  int set_illum(int il);
  int set_type();

private:
  std::string name_;
  int illum_;
  Eigen::Vector3d Kd_; //the diffuse reflectivity using RGB values
  Eigen::Vector3d Ka_; //the ambient reflectivity using RGB values
  Eigen::Vector3d Tf_; //the transmission filter using RGB values
  Eigen::Vector3d Ks_; //the specular reflectivity using RGB values
  double Ns_; //the specular exponent for the current material
  double Ni_; //optical density for the surface

  MtlType type_;
};

#endif // MATERIAL_JJ_H
