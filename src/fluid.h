#ifndef FLUID_H
#define FLUID_H

#include <unordered_set>
#include <unordered_map>
#include <vector>

#include "CGL/CGL.h"
#include "CGL/misc.h"
#include "collision/collisionObject.h"
#include "particle.h"

using namespace CGL;
using namespace std;

struct FluidParameters {
  FluidParameters() {}
  FluidParameters(double density)
      : density(density) {}
  ~FluidParameters() {}

  // Parameters
  double density;
};

struct Fluid {
  Fluid() {}
  Fluid(int num_x, int num_y, int num_z);
  ~Fluid();

  void buildFluid();

  void simulate(double frames_per_sec, double simulation_steps, FluidParameters *fp,
                vector<Vector3D> external_accelerations,
                vector<CollisionObject *> *collision_objects);

  void reset();

  void self_collide(Particle &p, double simulation_steps);

  // Fluid properties
  int num_particles;
  int num_x;
  int num_y;
  int num_z;

  // Fluid components
  vector<Particle> particles;
};

#endif /* FLUID_H */
