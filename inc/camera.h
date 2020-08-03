#ifndef CAMERA_H
#define CAMERA_H

#include <Eigen/Core>
#include "macros.inc"

class Camera
{
public:
Camera(const Eigen::Vector3d &o, const Eigen::Vector3d &lt, const Eigen::Vector3d &up, double f, size_t w, size_t h) : origin_(o), lookat_(lt), up_(up), fovy_(f), width_(w), height_(h)
  {
    image = new Eigen::Vector3d[w * h];

    double length = 2 * (lookat_ - origin_).norm() * tan(fovy_ / 2.0 * PI / 180);
    len_per_pxl_ = length / height_;

    up_.normalize();
    // cross_ = up_.cross((lookat_ - origin_));
    cross_ = -(lookat_ - origin_).cross(up_);
    cross_.normalize();
  }
  ~Camera();

public:
  Eigen::Vector3d GetViewBeam(size_t pixel_i, size_t pixel_j) const;
  int add_pixel(size_t i, size_t j, const Eigen::Vector3d &color);
  int SaveImage(const char *dir = "image.ppm");
  size_t GetWidth() const;
  size_t GetHeight() const;
  Eigen::Vector3d GetOrigin() const;
  Eigen::Vector3d GetLookAt() const;
private:
  int set_unit();
private:
  Eigen::Vector3d origin_;
  Eigen::Vector3d lookat_;
  Eigen::Vector3d up_;
  Eigen::Vector3d cross_;
  double fovy_;
  size_t width_;
  size_t height_;
  double len_per_pxl_;
  Eigen::Vector3d *image;
};

#endif // CAMERA_H
