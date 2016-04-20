#ifndef SIMULATOR_H_
#define SIMULATOR_H_

struct Cloth;

class Simulator {
public:
  Cloth *cloth;

  Simulator(bool gpu);
  void compute_timestep(float dt);
  void draw();
};

#endif
