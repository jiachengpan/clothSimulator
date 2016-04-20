#include "gpu_cloth.cuh"
#include "simulator.h"

__global__ 
void kernel_gravity_force(
    const glm::vec3 *X, const glm::vec3 *X_prev, const float *M, size_t size,
    const glm::vec3 gravity, float damping, float dt,
    glm::vec3 *F)
{
  const int pos = threadIdx.x + blockDim.x * blockIdx.x;

  if (pos < size) {
    F[pos] = gravity * M[pos] + damping * ((X[pos] - X_prev[pos]) / dt);
  }
}

__global__
void kernel_next_position(
    glm::vec3 *X, glm::vec3 *X_prev, glm::vec3 *X_prev_prev, 
    const glm::vec3 *F, const float *M, const float dt, size_t size)
{
  const int pos = threadIdx.x + blockDim.x * blockIdx.x;

  // TODO: stride and loop to optimise this, increasing thread computation load?
  if (pos < size) {
    glm::vec3 curr = X[pos];
    glm::vec3 prev = X_prev[pos];
    float m = M[pos];
    X[pos] = (m > 0.0000001f) ? (curr + curr - prev + dt * dt * F[pos] / m) : curr;
    X_prev_prev[pos] = prev;
    X_prev[pos] = curr;
  }
}

__global__
void kernel_spring_force(
    const CudaCloth::Spring *springs, const glm::vec3 *X, const glm::vec3 *X_prev,
    glm::vec3 *F,
    float dt, size_t size)
{
  const int pos = threadIdx.x + blockDim.x * blockIdx.x;
  
  if (pos < (size>>1)) {
    CudaCloth::Spring s = springs[pos];
    glm::vec3 p1 = X[s.p1];
    glm::vec3 p2 = X[s.p2];

    glm::vec3 spring_force = -s.ks * (glm::distance(p1, p2) - s.rest_length) * glm::normalize(p1 - p2);

    F[s.p1] += spring_force;
    F[s.p2] -= spring_force;

    s = springs[pos + (size>>1)];
    p1 = X[s.p1];
    p2 = X[s.p2];

    spring_force = -s.ks * (glm::distance(p1, p2) - s.rest_length) * glm::normalize(p1 - p2);

    F[s.p1] += spring_force;
    F[s.p2] -= spring_force;
  }
}


__global__
void kernel_satisfy_constraints(
    const CudaCloth::Spring *springs, const float *M, glm::vec3 *X,
    size_t size)
{
  const int pos = threadIdx.x + blockDim.x * blockIdx.x;

  if (pos < (size>>1)) {
    CudaCloth::Spring s = springs[pos];
    float m1 = M[s.p1];
    float m2 = M[s.p2];
    glm::vec3 p12 = X[s.p2] - X[s.p1];
    glm::vec3 correct_v = p12 * (1 - s.rest_length / glm::length(p12)) * 0.1f;
    if (m1 > 0.0000001f) X[s.p1] += correct_v;
    if (m2 > 0.0000001f) X[s.p2] -= correct_v;

    __syncthreads();
    s = springs[pos + (size>>1)];
    m1 = M[s.p1];
    m2 = M[s.p2];
    p12 = X[s.p2] - X[s.p1];
    correct_v = p12 * (1 - s.rest_length / glm::length(p12)) * 0.1f;
    if (m1 > 0.0000001f) X[s.p1] += correct_v;
    if (m2 > 0.0000001f) X[s.p2] -= correct_v;
  }
}

void CudaCloth::init()
{
  add_springs(KD_STRUCT, KS_STRUCT, KD_BEND, KS_BEND, KD_SHEAR, KS_SHEAR);
}

void CudaCloth::add_springs(
    float kd_struct, float ks_struct,
    float kd_bend, float ks_bend,
    float kd_shear, float ks_shear)
{
  // struct x
  for (uint32_t start = 0; start <= 1; ++start) {
    for (uint32_t y = 0; y < height; ++y) {
      for (uint32_t x = start; x < width-1; x += 2) {
        uint32_t p1 = x + width * y;
        uint32_t p2 = p1 + 1;
        springs_struct_x.push_back(Spring(p1, p2, glm::distance(X[p1], X[p2]), kd_struct, ks_struct));
      }
    }
  }

  // struct y
  for (uint32_t start = 0; start <= 1; ++start) {
    for (uint32_t x = 0; x < width; ++x) {
      for (uint32_t y = start; y < height-1; y += 2) {
        uint32_t p1 = x + width * y;
        uint32_t p2 = p1 + width;
        springs_struct_y.push_back(Spring(p1, p2, glm::distance(X[p1], X[p2]), kd_struct, ks_struct));
      }
    }
  }

  // shear
  for (uint32_t start = 0; start <= 1; ++start) {
    for (uint32_t y = start; y < height-1; y += 2) {
      for (uint32_t x = start; x < width-1; x += 2) {
        uint32_t p1 = x + width * y;
        uint32_t p2 = p1 + width + 1;
        springs_shear.push_back(Spring(p1, p2, glm::distance(X[p1], X[p2]), kd_shear, ks_shear));

        p1 += 1;
        p2 -= 1;
        springs_shear.push_back(Spring(p1, p2, glm::distance(X[p1], X[p2]), kd_shear, ks_shear));
      }
    }
  }

  // bend x
  for (uint32_t start = 0; start <= 2; start += 2) {
    for (uint32_t y = 0; y < height; ++y) {
      for (uint32_t x = start; x < width-2; x += 4) {
        uint32_t p1 = x + width * y;
        uint32_t p2 = p1 + 2;
        springs_bend_x.push_back(Spring(p1, p2, glm::distance(X[p1], X[p2]), kd_bend, ks_bend));

        p1 += 1;
        p2 += 1;
        springs_bend_x.push_back(Spring(p1, p2, glm::distance(X[p1], X[p2]), kd_bend, ks_bend));
      }
    }
  }

  // bend y
  for (uint32_t start = 0; start <= 2; start += 2) {
    for (uint32_t x = 0; x < width; ++x) {
      for (uint32_t y = start; y < height-2; y += 4) {
        uint32_t p1 = x + width * y;
        uint32_t p2 = p1 + width + width;
        springs_bend_y.push_back(Spring(p1, p2, glm::distance(X[p1], X[p2]), kd_bend, ks_bend));

        p1 += width;
        p2 += width;
        springs_bend_y.push_back(Spring(p1, p2, glm::distance(X[p1], X[p2]), kd_bend, ks_bend));
      }
    }
  }

  d_springs_shear = springs_shear;
  d_springs_struct_x = springs_struct_x;
  d_springs_struct_y = springs_struct_y;
  d_springs_bend_x = springs_bend_x;
  d_springs_bend_y = springs_bend_y;
}

void CudaCloth::timestep(float dt)
{
  satisfy_constraints();
  compute_forces(dt);
  compute_next_positions(dt);

  thrust::copy(d_X.begin(), d_X.end(), X.begin());
}

void CudaCloth::satisfy_constraints()
{
  cudaEvent_t start, stop;
  cudaEventCreate(&start);
  cudaEventCreate(&stop);

  cudaEventRecord(start);
  for (uint32_t i = 0; i < 10; ++i) {
    satisfy_spring_constraints(d_springs_struct_x);
    satisfy_spring_constraints(d_springs_struct_y);
    satisfy_spring_constraints(d_springs_shear);
    satisfy_spring_constraints(d_springs_bend_x);
    satisfy_spring_constraints(d_springs_bend_y);
  }
  cudaEventRecord(stop);
  cudaEventSynchronize(stop);

  float milliseconds = 0;
  cudaEventElapsedTime(&milliseconds, start, stop);
  cout << "satisfy_constraints: " << milliseconds << "ms" << endl;
}

void CudaCloth::satisfy_spring_constraints(const thrust::device_vector<Spring>& springs)
{
  kernel_satisfy_constraints<<<(springs.size()/2 + BLOCK_SIZE - 1) / BLOCK_SIZE, BLOCK_SIZE>>>
    (thrust::raw_pointer_cast(springs.data()),
     thrust::raw_pointer_cast(d_M.data()),
     thrust::raw_pointer_cast(d_X.data()),
     springs.size());
}

void CudaCloth::compute_forces(float dt)
{

  kernel_gravity_force<<<(d_X.size() + BLOCK_SIZE - 1) / BLOCK_SIZE, BLOCK_SIZE>>>
    (thrust::raw_pointer_cast(d_X.data()),
     thrust::raw_pointer_cast(d_X_prev.data()),
     thrust::raw_pointer_cast(d_M.data()), d_X.size(),
     GRAVITY, DAMPING, dt,
     thrust::raw_pointer_cast(d_F.data()));

  compute_spring_forces(d_springs_struct_x, dt);
  compute_spring_forces(d_springs_struct_y, dt);
  compute_spring_forces(d_springs_bend_x, dt);
  compute_spring_forces(d_springs_bend_y, dt);
  compute_spring_forces(d_springs_shear, dt);
}

void CudaCloth::compute_next_positions(float dt)
{
  kernel_next_position<<<(X.size() + BLOCK_SIZE - 1) / BLOCK_SIZE, BLOCK_SIZE>>>
    (thrust::raw_pointer_cast(d_X.data()),
     thrust::raw_pointer_cast(d_X_prev.data()),
     thrust::raw_pointer_cast(d_X_prev_prev.data()),
     thrust::raw_pointer_cast(d_F.data()),
     thrust::raw_pointer_cast(d_M.data()),
     dt, X.size());
}

void CudaCloth::compute_spring_forces(const thrust::device_vector<Spring>& springs, float dt)
{
  kernel_spring_force<<<(springs.size()/2 + BLOCK_SIZE - 1) / BLOCK_SIZE, BLOCK_SIZE>>>
    (thrust::raw_pointer_cast(springs.data()),
     thrust::raw_pointer_cast(d_X.data()),
     thrust::raw_pointer_cast(d_X_prev.data()),
     thrust::raw_pointer_cast(d_F.data()),
     dt, springs.size());
}

void CudaCloth::compute_wind_forces()
{
}

void CudaCloth::compute_self_collisions()
{
}

void CudaCloth::compute_object_collisions()
{
}


Simulator::Simulator(bool gpu)
{
  if (gpu) {
    cloth = new CudaCloth(50, 50, 0.5, 0.5);
  } else {
    cloth = new Cloth(10, 10, 0.5, 0.5);
  }
  cloth->init();
}

void Simulator::compute_timestep(float dt)
{
  cudaEvent_t start, stop;
  cudaEventCreate(&start);
  cudaEventCreate(&stop);

  cudaEventRecord(start);
  cloth->timestep(dt);
  
  cudaEventRecord(stop);
  cudaEventSynchronize(stop);

  float milliseconds = 0;
  cudaEventElapsedTime(&milliseconds, start, stop);
  cout << "overall: " << milliseconds << "ms" << endl;
}

void Simulator::draw()
{
  cloth->draw();
}

