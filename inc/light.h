#ifndef LIGHT_JJ_H
#define LIGHT_JJ_H

#include <Eigen/Core>

class Beam;

class Light
{
public:
Light(const Eigen::Vector3d &l, const Eigen::Vector3d &c) : le_(l), center_(c) {}
  virtual ~Light();

public:
  Eigen::Vector3d GetLe() const;
  virtual double IsIntersect(const Beam &b) const = 0;
  virtual Eigen::Vector3d GetRandomPointLight() const = 0;
  virtual Eigen::Vector3d GetNorm(const Eigen::Vector3d &p) const = 0;
  virtual Eigen::Vector3d LightLe(const Eigen::Vector3d &ltpt, const Eigen::Vector3d &o, const Eigen::Vector3d &o_norm) const = 0;
  Eigen::Vector3d GetCenter() const;
  virtual double GetRadius() const;
private:
  Eigen::Vector3d le_;
  Eigen::Vector3d center_;
};

class SphereLight : public Light
{
public:
SphereLight(const Eigen::Vector3d &l, const Eigen::Vector3d &c, double r) : Light(l, c)
  {
    radius_ = r;
  }
  ~SphereLight();

public:
  double GetRadius() const;
  double IsIntersect(const Beam &b) const;
  Eigen::Vector3d GetRandomPointLight() const;
  Eigen::Vector3d LightLe(const Eigen::Vector3d &ltpt, const Eigen::Vector3d &o, const Eigen::Vector3d &o_norm) const;
  Eigen::Vector3d GetNorm(const Eigen::Vector3d &p) const;

private:
  bool IsShade(const Eigen::Vector3d &o, const Eigen::Vector3d &d, double dst) const;
  double radius_;
};


class QuardLight : public Light
{
public:
QuardLight(const Eigen::Vector3d &l, const Eigen::Vector3d &c, const Eigen::Vector3d &n, const Eigen::Vector3d &a, const Eigen::Vector3d &b, double w, double h) : Light(l, c) 
  {
    norm_ = n;
    width_ = w;
    height_ = h;
    cross_a_ = a;
    cross_b_ = b;
  }
  ~QuardLight();

public:
  double IsIntersect(const Beam &b) const;
  Eigen::Vector3d GetRandomPointLight() const;
  Eigen::Vector3d LightLe(const Eigen::Vector3d &ltpt, const Eigen::Vector3d &o, const Eigen::Vector3d &o_norm) const;
  Eigen::Vector3d GetNorm(const Eigen::Vector3d &p) const;

private:
  bool IsShade(const Eigen::Vector3d &o, const Eigen::Vector3d &d, double dst) const;
  Eigen::Vector3d norm_;
  Eigen::Vector3d cross_a_;
  Eigen::Vector3d cross_b_;
  double width_;
  double height_;
};

#endif // LIGHT_JJ_H
