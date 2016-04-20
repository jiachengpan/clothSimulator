#include "cloth.h"
#include <cstdio>
#include <iostream>

using namespace std;

const glm::vec3 scrubs(175./255., 250./255., 255./255.);
const glm::vec3 skyblue(11./255., 203./255., 234./255.);

Cloth::Cloth(uint32_t w, uint32_t h, float sw, float sh) : 
  X(w*h), X_prev(w*h), X_prev_prev(w*h), F(w*h), M(w*h), width(w), height(h), size_w(sw), size_h(sh)
{
  float m = (2.5f / w / h);

  for (uint32_t y = 0; y < height; ++y) {
    for (uint32_t x = 0; x < width; ++x) {
      uint32_t pos = x + width * y;
      //X[pos]      = glm::vec3(float(x) / width * size_w - size_w / 2, float(y) / height * size_h, 0.5f);
      //X_prev[pos] = glm::vec3(float(x) / width * size_w - size_w / 2, float(y) / height * size_h, 0.5f);
      X[pos]      = glm::vec3(float(x) / width * size_w - size_w / 2, .8f, float(y) / height * size_h);
      X_prev[pos] = glm::vec3(float(x) / width * size_w - size_w / 2, .8f, float(y) / height * size_h);
      //M[pos] = m * (.7f + .3 * float(y) / height);
      M[pos] = m;
      //if (x == 0) M[pos] = 0.f;
    }
  }
  M[0] = 0.;
  M[width-1] = 0.;
  M[height * width - width] = 0.;
  //M[height * width - 1] = 0.;

  //X[0] = glm::vec3(0.1f, .0f, .0f);
}

void Cloth::init()
{
  add_springs(KD_STRUCT, KS_STRUCT, KD_BEND, KS_BEND, KD_SHEAR, KS_SHEAR);
}

void Cloth::add_wind(glm::vec3 f)
{
  wind_force = f;
}

void Cloth::add_springs(
    float kd_struct, float ks_struct,
    float kd_bend, float ks_bend,
    float kd_shear, float ks_shear) 
{
  for (uint32_t y = 0; y < height; ++y) {
    for (uint32_t x = 0; x < width; ++x) {
      uint32_t p1 = x + width * y;
      if (x < width-1) {
        uint32_t p2 = p1 + 1;
        springs_struct_x.push_back(Spring(
              p1, p2, glm::distance(X[p1], X[p2]), 
              kd_struct, ks_struct));
      }
      if (x < width-2) {
        uint32_t p2 = p1 + 2;
        springs_bend_x.push_back(Spring(
              p1, p2, glm::distance(X[p1], X[p2]), 
              kd_bend, ks_bend));
      }
      if (y < height-1) {
        uint32_t p2 = p1 + width;
        springs_struct_y.push_back(Spring(
              p1, p2, glm::distance(X[p1], X[p2]), 
              kd_struct, ks_struct));
      }
      if (y < height-2) {
        uint32_t p2 = p1 + width + width;
        springs_bend_y.push_back(Spring(
              p1, p2, glm::distance(X[p1], X[p2]), 
              kd_bend, ks_bend));
      }
    }
  }

  for (uint32_t y = 0; y < height-1; ++y) {
    for (uint32_t x = 0; x < width-1; ++x) {
      uint32_t p1 = x + width * y;
      uint32_t p2 = p1 + 1 + width;
      springs_shear.push_back(Spring(
            p1, p2, glm::distance(X[p1], X[p2]), 
            kd_shear, ks_shear));

      p1 += 1;
      p2 -= 1;
      springs_shear.push_back(Spring(
            p1, p2, glm::distance(X[p1], X[p2]), 
            kd_shear, ks_shear));
    }
  }
}



void Cloth::timestep(float dt)
{
  satisfy_constraints();
  compute_forces(dt);
  compute_next_positions(dt);
  //compute_collisions();
} 

void Cloth::compute_forces(float dt)
{
  for (size_t i = 0; i < X.size(); ++i) {
    glm::vec3 velocity = get_velocity(i, dt);
    F[i] = glm::vec3(0., 0., 0.);
    // gravity and damping force
    F[i] += GRAVITY * M[i] + DAMPING * velocity;
    if (0) {
      cout << i << endl;
      cout << "dt: " << dt << endl;
      cout << "dist: " << glm::distance(X[i], X_prev[i]) << endl;
      cout << "curr: " << X[i].x << " " << X[i].y << " " << X[i].z << endl;
      cout << "prev: " << X_prev[i].x << " " << X_prev[i].y << " " << X_prev[i].z << endl;
      cout << "velocity: " << velocity.x << " " << velocity.y << " " << velocity.z << endl;
      cout << "force: " << F[i].x << " " << F[i].y << " " << F[i].z << endl;
    }
  }
  compute_spring_forces(springs_struct_x, dt);
  compute_spring_forces(springs_struct_y, dt);
  compute_spring_forces(springs_bend_x, dt);
  compute_spring_forces(springs_bend_y, dt);
  compute_spring_forces(springs_shear, dt);

  compute_wind_forces();
}

void Cloth::compute_next_positions(float dt)
{
  float dt2 = dt * dt;
  for (uint32_t i = 0; i < X.size(); ++i) {
    glm::vec3 prev = X[i];
    if (M[i] != 0 || (glm::length(X[i] - X_prev[i]) > 0.01)) {
      X[i] = X[i] + (X[i] - X_prev[i]) + dt2 * F[i] / M[i];
    }
    X_prev_prev[i] = X_prev[i];
    X_prev[i] = prev;

    // hack for ground hit
    //if (X[i].y < 0.) {
    //  X[i].y = 0.;
    //}
  }
}

void Cloth::compute_spring_forces(const vector<Spring>& springs, float dt)
{
  for (uint32_t i = 0; i < springs.size(); ++i) {
    glm::vec3 v1 = get_velocity(springs[i].p1, dt);
    glm::vec3 v2 = get_velocity(springs[i].p2, dt);

    glm::vec3 delta_p = X[springs[i].p1] - X[springs[i].p2];
    glm::vec3 delta_v = v1 - v2;
    float dist = glm::length(delta_p);

    float left_term = -springs[i].ks * (dist - springs[i].rest_length);
    float right_term= -springs[i].kd * (glm::dot(delta_v, delta_p) / dist);
    glm::vec3 spring_force = (left_term + right_term) * glm::normalize(delta_p);

    if (glm::length(spring_force) > 100) {
      spring_force = 100.f * glm::normalize(spring_force);
    }
    if (glm::length(spring_force) < 0.01) {
      spring_force = glm::vec3(0.f);
    }

    F[springs[i].p1] += spring_force;
    F[springs[i].p2] -= spring_force;
  }
}

void Cloth::compute_wind_forces()
{
  for (uint32_t y = 0; y < height-1; ++y) {
    for (uint32_t x = 0; x < width-1; ++x) {
      uint32_t p = x + width * y;
      glm::vec3 p0 = X[p];
      glm::vec3 p1 = X[p+1];
      glm::vec3 p2 = X[p+width];
      glm::vec3 p3 = X[p+1+width];

      glm::vec3 n0 = glm::normalize(glm::cross(p0-p1, p2-p1));
      glm::vec3 f = glm::dot(n0, wind_force) * n0;

      F[p] += f;
      F[p+1] += f;
      F[p+width] += f;

      n0 = glm::normalize(glm::cross(p1-p3, p2-p3));
      f = glm::dot(n0, wind_force) * n0;
      F[p+1] += f;
      F[p+width] += f;
      F[p+1+width] += f;
    }
  }
}

void Cloth::compute_self_collisions()
{
  collision_grid.clear();
  for (uint32_t i = 0; i < X.size(); ++i) {
    glm::vec3 pos = X[i];
    collision_grid.insert(make_pair(
          make_tuple(pos.x * 100, pos.y * 100, pos.z * 100), i));
  }

  for (auto it = collision_grid.begin(), end = collision_grid.end(), slow = it; 
      it != end; ++it) {
    if (slow->first == it->first) continue;
    for (; slow != it; ++slow) {
      for (auto islow = next(slow); islow != it; ++islow) {
        uint32_t a = slow->second;
        uint32_t b = islow->second;
        //if ((a == b+1) || (a == b-1) || (a == b+width) || (a == b-width) ||
        //    (a == b+1+width) || (a == b+1-width) || (a == b-1+width) || (a == b-1-width)) 
        //  continue;
        X[a] = X_prev_prev[a];
        X[b] = X_prev_prev[b];
      }
    }
  } 
}

void Cloth::compute_object_collisions()
{
}

void Cloth::satisfy_constraints()
{
  for (uint32_t i = 0; i < 2; ++i) {
    satisfy_spring_constraints(springs_struct_x);
    satisfy_spring_constraints(springs_struct_y);
    satisfy_spring_constraints(springs_shear);
    satisfy_spring_constraints(springs_bend_x);
    satisfy_spring_constraints(springs_bend_y);
  }
}

void Cloth::satisfy_spring_constraints(const vector<Spring>& springs)
{
  for (uint32_t i = 0; i < springs.size(); ++i) {
    glm::vec3 p12 = X[springs[i].p2] - X[springs[i].p1];
    float dist = glm::length(p12);

    glm::vec3 correct_v = p12 * (1 - springs[i].rest_length / dist) * 0.1f;
    if (M[springs[i].p1] > 0.0001) X[springs[i].p1] += correct_v;
    if (M[springs[i].p2] > 0.0001) X[springs[i].p2] -= correct_v;
  }
}

void Cloth::draw() const 
{
  //glBegin(GL_POINTS);
  //glColor3f(0.6f, 0.2f, 0.2f);
  //for (uint32_t i = 0; i < X.size(); ++i) {
  //  glVertex3f(X[i].x, X[i].y, X[i].z);
  //}
  //glEnd();

  glBegin(GL_TRIANGLES);
  glColor3f(skyblue.x, skyblue.y, skyblue.z);
  for (uint32_t y = 0; y < height-1; ++y) {
    for (uint32_t x = 0; x < width-1; ++x) {
      uint32_t p = x + width * y;
      glm::vec3 p0 = X[p];
      glm::vec3 p1 = X[p+1];
      glm::vec3 p2 = X[p+width];
      glm::vec3 p3 = X[p+1+width];
      glm::vec3 n0 = glm::normalize(p0);
      glm::vec3 n1 = glm::normalize(p1);
      glm::vec3 n2 = glm::normalize(p2);
      glm::vec3 n3 = glm::normalize(p3);

      glNormal3f(n0.x, n0.y, n0.z);
      glVertex3f(p0.x, p0.y, p0.z);
      glNormal3f(n2.x, n2.y, n2.z);
      glVertex3f(p2.x, p2.y, p2.z);
      glNormal3f(n1.x, n1.y, n1.z);
      glVertex3f(p1.x, p1.y, p1.z);

      glNormal3f(n1.x, n1.y, n1.z);
      glVertex3f(p1.x, p1.y, p1.z);
      glNormal3f(n2.x, n2.y, n2.z);
      glVertex3f(p2.x, p2.y, p2.z);
      glNormal3f(n3.x, n3.y, n3.z);
      glVertex3f(p3.x, p3.y, p3.z);
    }
  }
  glEnd();
}
