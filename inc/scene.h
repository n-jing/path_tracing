#ifndef SCENE_JJ_H
#define SCENE_JJ_H

#include <vector>
#include "../inc/light.h"

class Model;
class Camera;

class Scene
{
public:
  ~Scene(){}
  static const Scene &Instance(const Model *const m, Camera *const camera, const std::vector<const Light*> *const l)
  {
    static Scene scene = Scene(m, camera, l);
    return scene;
  }
  int render(size_t samples = 1, size_t depth = 1) const;
  Eigen::Vector3d path_tracing(const Eigen::Vector3d &o, const Eigen::Vector3d &d, size_t depth) const;
  Camera *GetCamera() const;
  std::vector<double> GetRadius() const;
private:
Scene(const Model *const m, Camera *const camera, const std::vector<const Light*> *const l):model_(m), camera_(camera), lt_(l){}
  Scene(const Scene&);                 // Prevent copy-construction
  Scene& operator=(const Scene&);      // Prevent assignment
  const Model *const model_;
  Camera *const camera_;
  const std::vector<const Light*> *const lt_;
};

#endif // SCENE_JJ_H
