#ifndef INTERSECT_JJ_H
#define INTERSECT_JJ_H

#include "../inc/material.h"
#include "../inc/object.h"
#include <Eigen/Core>


class Intersect
{
public:
  Intersect();
  ~Intersect();

  Object obj_;
  double dst_;
  MtlType type_;
};



#endif // INTERSECT_JJ_H
