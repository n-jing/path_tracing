#ifndef BEAM_JJ_H
#define BEAM_JJ_H

#include <Eigen/Core>

class Beam
{
public:
Beam(const Eigen::Vector3d &o, const Eigen::Vector3d &d) : o_(o), d_(d) {}
  Beam();
  ~Beam(){}
  Eigen::Vector3d GetOrigin() const;
  Eigen::Vector3d GetDir() const;
  int set_origin(const Eigen::Vector3d &o);
  int set_dir(const Eigen::Vector3d &d);
private:
  Eigen::Vector3d o_;
  Eigen::Vector3d d_;
};


#endif // BEAM_JJ_H
