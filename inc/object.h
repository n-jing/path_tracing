#ifndef OBJECT_JJ_H
#define OBJECT_JJ_H

#include <Eigen/Core>
class Beam;
class Material;

class Object
{
public:
  Object();
  virtual ~Object();

public:
  virtual Beam GetRefBeam(const Beam &b) const = 0;
  virtual Eigen::Vector3d GetRefFactorDirFromIt(const Eigen::Vector3d &in, const Eigen::Vector3d &out) const = 0;
  virtual Eigen::Vector3d GetLe() const;
  virtual double GetIntersect(const Eigen::Vector3d &o, const Eigen::Vector3d &d) const;

public:
  Eigen::Vector3d GetNorm() const;
  Eigen::Vector3d GetCenter() const;
  Eigen::Vector3d GetBoxLength() const;
  int GetBoundingBox(Eigen::Vector3d &minB, Eigen::Vector3d &maxB) const;
  int set_mtl(Material *const mtl);
  int set_vertex(const double *const v, size_t itr);
  int set_norm(const double *const n, size_t itr);
  Eigen::Vector3d GetVertex(size_t itr) const;
  
protected:
  Material *mtl_;
  Eigen::Vector3d vertex_[3];
  Eigen::Vector3d norm_[3];
};


class DiffObject: public Object
{
public:
DiffObject() : Object(){}
  ~DiffObject() {}

public:
  Beam GetRefBeam(const Beam &b) const;
  Eigen::Vector3d GetRefFactorDirFromIt(const Eigen::Vector3d &in, const Eigen::Vector3d &out) const;
};


class RefObject: public Object
{
public:
RefObject() : Object(){}
  ~RefObject(){}

public:
  Beam GetRefBeam(const Beam &b) const;
  Eigen::Vector3d GetRefFactorDirFromIt(const Eigen::Vector3d &in, const Eigen::Vector3d &out) const;
};


class EmiObject: public Object
{
public:
EmiObject() : Object(){}
  ~EmiObject(){}

public:
  Beam GetRefBeam(const Beam &b) const;
  Eigen::Vector3d GetRefFactorDirFromIt(const Eigen::Vector3d &in, const Eigen::Vector3d &out) const;
  Eigen::Vector3d GetLe() const;
};

class RefraObject: public Object
{
public:
RefraObject() : Object(){}
  ~RefraObject(){}

public:
  Beam GetRefBeam(const Beam &b) const;
  Eigen::Vector3d GetRefFactorDirFromIt(const Eigen::Vector3d &in, const Eigen::Vector3d &out) const;
  double GetIntersect(const Eigen::Vector3d &o, const Eigen::Vector3d &d) const;

private:
  Eigen::Vector3d OutRef(const Beam &b) const;
  Eigen::Vector3d InnerRef(const Beam &b) const;

};

#endif // OBJECT_JJ_H
