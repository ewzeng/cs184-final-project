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
  FluidParameters(double pmass)
      : particle_mass(pmass) {}
  ~FluidParameters() {}

  // Parameters
  double particle_mass;
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

  // computations from the paper Position Based Fluids
  double W(Vector3D x); // smoothing kernel
  double C_i(Particle *p); // density contraint
  void compute_density_est(Particle *p, vector<Particle *> *neighbors); // compute density_est
  double grad_W(Vector3D x); // gradient of W
  double grad_p_k_C_i(Particle* p_k, Particle* p_i, vector<Particle*>* neighbors); // grad of C_i wrt p_k
  void compute_lambda_i(Particle* p_i, vector<Particle*>* neighbors); // compute lambda_i
  void compute_position_update(Particle* p_i, vector<Particle*>* neighbors); // compute delta_pos

  // Fluid properties
  double rho_0 = 1; // rest density
  double pmass; // particle mass
  int num_particles;
  int num_x;
  int num_y;
  int num_z;

  // Fluid components
  vector<Particle> particles;
};

#endif /* FLUID_H */