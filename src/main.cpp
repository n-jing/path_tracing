#include "../inc/scene.h"
#include "../inc/camera.h"
#include "../inc/model.h"
#include "../inc/scene_para.h"
#include <iostream>
#include <Eigen/Core>
#include <Eigen/Geometry>

using namespace Eigen;
using namespace std;

typedef const Scene& (*Fun)(const char *const dir);

int main(int argc, char *argv[])
{
  char dir0[] = "../scenes/Scene01/cup.obj";
  char dir1[] = "../scenes/Scene02/room.obj";
  char dir2[] = "../scenes/Scene03/VeachMIS.obj";

  char *path[3] = {dir0, dir1, dir2};
  Fun scene_f[3] = {cup, room, VeachMIS};
  constexpr size_t ID = 1;
  cerr << path[ID] << endl;
  
  string str_iamge(path[ID]);
  str_iamge = str_iamge.substr(str_iamge.rfind("/") + 1);
  str_iamge = str_iamge.substr(0, str_iamge.rfind(".")) + ".ppm";

  const Scene &scene = scene_f[ID](path[ID]);
  constexpr size_t samples = 20;
  constexpr size_t depth = 5;
  scene.render(samples, depth);
  scene.GetCamera()->SaveImage(str_iamge.c_str());

  return 0;
}
