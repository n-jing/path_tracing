#include "../inc/scene.h"
#include <numeric>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <omp.h>
#include "../inc/model.h"
#include "../inc/camera.h"
#include "../inc/beam.h"
#include "../inc/object.h"
#include <Eigen/Geometry>

#define LIGHT_SAMPLE 5

using namespace std;
using namespace Eigen;

int Scene::render(size_t samples, size_t depth) const
{
  const size_t kW = camera_->GetWidth();
  const size_t kH = camera_->GetHeight();
  for (size_t jtr = 0; jtr < kH; ++jtr)
  {
    cerr << "\rfinishing:" << fixed << setprecision(2) << jtr * 1.0 / kH * 100 << "%" << jtr;
#pragma omp parallel for
    for (size_t itr = 0; itr < kW; ++itr)
    {
      for (size_t i = 0; i < samples; ++i)
      {
        Vector3d dir = camera_->GetViewBeam(itr, jtr);
        Vector3d color = path_tracing(camera_->GetOrigin(), dir, depth);
        camera_->add_pixel(itr, jtr, color / (1.0 * samples));
      }
    } 
  }

  return 0;
}

Vector3d Scene::path_tracing(const Vector3d &o, const Vector3d &d, size_t depth) const
{
  if (depth == 0)
    return Vector3d(0, 0, 0);

  Beam b(o, d);
  size_t obj_idx = 0;
  double dst = numeric_limits<double>::max();
  model_->GetNearObj(b, obj_idx, dst);
  if (dst == numeric_limits<double>::max())
    return Vector3d(0, 0, 0);

  const Object *const obj_ptr = model_->GetObject(obj_idx);
  assert(obj_ptr->GetIntersect(o, d) != numeric_limits<double>::max());

  Vector3d color(0, 0, 0);
  if (obj_ptr->GetLe() != Vector3d(0, 0, 0))
    return obj_ptr->GetLe();
    // color = obj_ptr->GetLe();
  
  const Vector3d it_point = o + dst * d;

  vector<double> r = GetRadius();
  vector<double> sample_factor(lt_->size());

  double r_min = *min_element(r.begin(), r.end());
  if (find(r.begin(), r.end(), numeric_limits<double>::max()) == r.begin())
  {
    sample_factor = vector<double>(lt_->size(), 1.0);
  }
  else
  {
    vector<size_t> r_idx(lt_->size());
    iota(r_idx.begin(), r_idx.end(), 0);
    size_t r_max_id = *max_element(r_idx.begin(), r_idx.end(),
                                   [&r] (size_t lidx, size_t ridx) -> bool
                                   {
                                     return r[lidx] < r[ridx];
                                   });
    for (size_t itr = 0; itr < lt_->size(); ++itr)
    {
      if (r[itr] == r_min)
        sample_factor[itr] = r[itr] / r[r_max_id] * 1.0;
      else
        sample_factor[itr] = r[itr] / r[r_max_id] * 1.0 * 5;
    }
  }

  for (size_t itr = 0; itr < lt_->size(); ++itr)
  {
    const Light &light = *((*lt_)[itr]);
    for (size_t lt_itr = 0; lt_itr < LIGHT_SAMPLE; ++lt_itr)
    {
      Vector3d ltpt = light.GetRandomPointLight();
      Vector3d ref_dir = (ltpt - it_point).normalized();
      double ref_dst = numeric_limits<double>::max();
      size_t ref_idx = 0;
      model_->GetNearObj(Beam(it_point, ref_dir), ref_idx, ref_dst);
      if (ref_dst <= light.IsIntersect(Beam(it_point, ref_dir)) - EPS)
        continue;
      Vector3d le = light.LightLe(ltpt, it_point, obj_ptr->GetNorm());
      Vector3d ref_factor = obj_ptr->GetRefFactorDirFromIt(ref_dir, -d);
      for (size_t c_itr = 0; c_itr < 3; ++c_itr)
        color[c_itr] += le[c_itr] * ref_factor[c_itr] * sample_factor[itr] / LIGHT_SAMPLE;
    }
  }

  Beam ref_b = obj_ptr->GetRefBeam(b);
  Vector3d indi_factor = obj_ptr->GetRefFactorDirFromIt(ref_b.GetDir(), -b.GetDir());
  Vector3d indi_color = path_tracing(ref_b.GetOrigin(), ref_b.GetDir(), depth - 1);

  for (size_t itr = 0; itr < 3; ++itr)
  {
    indi_color[itr] *= indi_factor[itr];
    if (indi_color[itr] > 0.8 - EPS)
      indi_color[itr] = 0.8;
  } 

  return color + indi_color;
}

Camera *Scene::GetCamera() const
{
  return camera_;
}

vector<double> Scene::GetRadius() const
{
  vector<double> r(lt_->size());
  for (size_t itr = 0; itr < lt_->size(); ++itr)
  {
    r[itr] = (*lt_)[itr]->GetRadius();
  }

  return r;
}
