#include "../inc/camera.h"
#include <cmath>
#include <algorithm>
#include <random>
#include <iostream>
#include <fstream>
#include <omp.h>
#include <Eigen/Geometry>
#include "../inc/macros.inc"

using namespace Eigen;
using namespace std;

Camera::~Camera()
{
  delete[] image;
}


Vector3d Camera::GetViewBeam(size_t pxl_i, size_t pxl_j) const
{
  static unsigned int seed = 0;
  seed += 1;
  default_random_engine e(seed);
  uniform_real_distribution<double> u(0, 1);
  Vector3d w_length = ( pxl_i - width_ / 2.0 + u(e)) * len_per_pxl_ * cross_;
  Vector3d h_length = (height_ / 2.0 - pxl_j + u(e)) * len_per_pxl_ * up_;
  Vector3d pst = lookat_ + w_length + h_length;
  Vector3d drct = pst - origin_;
  drct.normalize();

  // static int idx = 100;
  // ++idx;
  // string str("pixel" + to_string(idx) + ".obj");
  // ofstream f_out(str.c_str());
  // f_out << "v " << pst[0] << " "<< pst[1] << " " << pst[2] << endl;
  // f_out << "v " << lookat_[0] << " " << lookat_[1] << " " << lookat_[2] << endl;
  // f_out << "v " << origin_[0] << " " << origin_[1] << " " << origin_[2] << endl;
  // f_out << "f 1 2 3" << endl;
  // f_out.close();
  // getchar();
  
  return drct;
}

int Camera::add_pixel(size_t i, size_t j, const Vector3d &color)
{
  image[i + j * width_] += color;

  return 0;
}

int Camera::set_unit()
{
//#pragma omp parallel for
  for (size_t j = 0; j < height_; ++j)
    for (size_t i = 0; i < width_; ++i)
    {
      for (size_t itr = 0; itr < 3; ++itr)
      {
        assert(image[i + j * width_][itr] >= 0);
        if (image[i + j * width_][itr] >= 1 - EPS)
          image[i + j * width_][itr] = 1;
        image[i + j* width_][itr] = pow(image[i + j * width_][itr], 1 / 2.2);
        image[i + j * width_][itr] *= 255;
      }
    }

  return 0;
}

int Camera::SaveImage(const char *dir)
{
  set_unit();
  ofstream f_out(dir, ios::binary); // b - binary mode
  if (!f_out)
  {
    cerr << "error in image file!" << endl;
    return -1;
  }
  f_out << "P6\n"<< width_ << " " << height_ << endl << "255" << endl;

  for (size_t j = 0; j < height_; ++j)
    for (size_t i = 0; i < width_; ++i)
    {
      f_out << static_cast<unsigned char>(image[i + j * width_][0])
            << static_cast<unsigned char>(image[i + j * width_][1])
            << static_cast<unsigned char>(image[i + j * width_][2]);
    }

  f_out.close();
  return 0;
}

size_t Camera::GetWidth() const
{
  return width_;
}

size_t Camera::GetHeight() const
{
  return height_;
}

Vector3d Camera::GetOrigin() const
{
  return origin_;
}

Vector3d Camera::GetLookAt() const
{
  return lookat_;
}
