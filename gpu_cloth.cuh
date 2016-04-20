#ifndef GPU_CLOTH_H_
#define GPU_CLOTH_H_

#include <thrust/device_vector.h>
#include <thrust/reduce.h>
#include <thrust/sort.h>
#include <thrust/iterator/zip_iterator.h>

#include "cloth.h"

struct CudaCloth : public Cloth {
  const int BLOCK_SIZE = 128;

  thrust::device_vector<glm::vec3> d_X;
  thrust::device_vector<glm::vec3> d_X_prev;
  thrust::device_vector<glm::vec3> d_X_prev_prev;
  thrust::device_vector<glm::vec3> d_F;
  thrust::device_vector<glm::vec3> d_temp;
  thrust::device_vector<float> d_M;

  thrust::device_vector<Spring> d_springs_struct_x;
  thrust::device_vector<Spring> d_springs_struct_y;
  thrust::device_vector<Spring> d_springs_shear;
  thrust::device_vector<Spring> d_springs_bend_x;
  thrust::device_vector<Spring> d_springs_bend_y;

  CudaCloth(uint32_t w, uint32_t h, float sw, float sh) : Cloth(w, h, sw, sh),
    d_X(X.begin(), X.end()),
    d_X_prev(X_prev.begin(), X_prev.end()),
    d_X_prev_prev(X_prev_prev.begin(), X_prev_prev.end()),
    d_F(F.begin(), F.end()),
    d_M(M.begin(), M.end()),
    d_temp(X.size()) {}

  virtual ~CudaCloth() {}

  virtual void init();
  virtual void add_springs(
      float kd_struct, float ks_struct,
      float kd_bend, float ks_bend,
      float kd_shear, float ks_shear);

  virtual void timestep(float dt);

  void satisfy_constraints();
  void satisfy_spring_constraints(const thrust::device_vector<Spring>& springs);

  void compute_forces(float dt);
  void compute_next_positions(float dt);
  void compute_spring_forces(const thrust::device_vector<Spring>& springs, float dt);
  void compute_wind_forces();
  void compute_collisions() {
    compute_self_collisions();
    compute_object_collisions();
  }
  void compute_self_collisions();
  void compute_object_collisions();

};

#endif
