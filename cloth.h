#include <cstdint>
#include <vector>
#include <algorithm>
#include <iostream>

#include <GL/glew.h>
#include <glm/glm.hpp>
#include <map>

using namespace std;

const float KD_STRUCT = .1f;
const float KS_STRUCT = 35.f;
const float KD_BEND   = .1f;
const float KS_BEND   = 55.f;
const float KD_SHEAR  = .1f;
const float KS_SHEAR  = 35.f;
const float DAMPING   = -.0125f;
const glm::vec3 GRAVITY = glm::vec3(0.f, -9.8, 0.f);
//const glm::vec3 GRAVITY = glm::vec3(0.f, 0.f, 0.f);

struct Cloth {
  vector<glm::vec3> X;
  vector<glm::vec3> X_prev;
  vector<glm::vec3> X_prev_prev;
  vector<glm::vec3> F;
  vector<float> M;

  struct Spring {
    uint32_t  p1, p2;
    float     rest_length;
    float     kd, ks;
    Spring() {}
    Spring(uint32_t _p1, uint32_t _p2, float _rest_length, float _kd, float _ks) :
      p1(_p1), p2(_p2), rest_length(_rest_length), kd(_kd), ks(_ks) {}
  };

  vector<Spring> springs_struct_x;
  vector<Spring> springs_struct_y;
  vector<Spring> springs_shear;
  vector<Spring> springs_bend_x;
  vector<Spring> springs_bend_y;

  glm::vec3 wind_force;

  multimap<tuple<int, int, int>, uint32_t> collision_grid;

  uint32_t width;
  uint32_t height;
  float size_w;
  float size_h;

  Cloth(uint32_t w, uint32_t h, float sw, float sh);
  
  void add_springs(
      float kd_struct, float ks_struct,
      float kd_bend, float ks_bend,
      float kd_shear, float ks_shear);
  void add_wind(glm::vec3 f);

  void timestep(float dt);

  void satisfy_constraints();
  void satisfy_spring_constraints(const vector<Spring>& springs);

  void compute_forces(float dt);
  void compute_next_positions(float dt);
  void compute_spring_forces(const vector<Spring>& springs, float dt);
  void compute_wind_forces();
  void compute_collisions() {
    compute_self_collisions();
    compute_object_collisions();
  }
  void compute_self_collisions();
  void compute_object_collisions();

  void draw() const;

  inline glm::vec3 get_velocity(uint32_t x, float dt) { return (X[x] - X_prev[x]) / dt; }
};
