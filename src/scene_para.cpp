#include "../inc/scene.h"
#include "../inc/camera.h"
#include "../inc/model.h"
#include <iostream>
#include <Eigen/Core>
#include <Eigen/Geometry>

using namespace Eigen;
using namespace std;

const Scene& cup(const char *const dir)
{
  const Vector3d le(40, 40, 40);
  const Vector3d c(-2.758771896,1.5246,0);
  const Vector3d n(1,0,0);
  const Vector3d ca(0, 1, 0);
  const Vector3d cb(0, 0, 1);
  const double w = 1;
  const double h = 1;
  const Light *const lt = new QuardLight(le, c, n, ca, cb, w, h);
  vector<const Light*> *lt_vec = new vector<const Light*>(1, lt);

  const Vector3d origin(0.0, 0.64, 0.52);
  const Vector3d lookat(0.0, 0.40, 0.3);
  const Vector3d up(0, 1.0, -2.4/2.2);
  const double fovy = 60;
  const size_t width = 512;
  const size_t height = 512;
  Camera *camera = new Camera(origin, lookat, up, fovy, width, height);
  Model *model = new Model(dir);
  const Scene &scene = Scene::Instance(model, camera, lt_vec);

  return scene;
}

const Scene& room(const char *const dir)
{
  Vector3d le(50, 50, 40);
  Vector3d ct(0.0,1.589,-1.274);
  double r = 0.2;
  const Light *const lt = new SphereLight(le, ct, r);
  vector<const Light*> *lt_vec = new vector<const Light*>(1, lt);

  Vector3d origin(0.0, 0.0, 4);
  Vector3d lookat(0.0, 0.0, 0.0);
  Vector3d up(0.0, 1.0, 0.0);
  double fovy = 50;
  size_t width = 512;
  size_t height = 512;
  Camera *camera = new Camera(origin, lookat, up, fovy, width, height);
  Model *model = new Model(dir);
  const Scene &scene = Scene::Instance(model, camera, lt_vec);

  return scene;
}

const Scene& VeachMIS(const char *const dir)
{
  const Vector3d le[5] = {{800,800,800},
                          {901.803,901.803,901.803},
                          {100,100,100},
                          {11.1111,11.1111,11.1111},
                          {1.23457,1.23457,1.23457}};
  const Vector3d pos[5] = {{-10,10,4},
                           {3.75,0,0},
                           {1.25,0,0},
                           {-1.25,0,0},
                           {-3.75,0,0}};
  const double r[5] = {0.5, 0.033, 0.1, 0.3, 0.9};
  vector<const Light*> *lt_vec = new vector<const Light*>(5);
  for (size_t itr = 0; itr < 5; ++itr)
  {
    (*lt_vec)[itr] = new SphereLight(le[itr], pos[itr], r[itr]);
  }

  Vector3d origin(0.0, 2.0, 15.0);
  Vector3d lookat(0.0, 1.69521, 14.0476);
  Vector3d up(0.0,0.952421,-0.304787);
  double fovy = 28;
  size_t width = 1152;
  size_t height = 864;
  Camera *camera = new Camera(origin, lookat, up, fovy, width, height);
  Model *model = new Model(dir);
  const Scene &scene = Scene::Instance(model, camera, lt_vec);

  return scene;
}

