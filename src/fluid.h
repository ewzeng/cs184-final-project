#ifndef FLUID_H
#define FLUID_H

#include <unordered_set>
#include <unordered_map>
#include <vector>
#include <nanogui/nanogui.h>

#include "CGL/CGL.h"
#include "CGL/misc.h"
#include "collision/plane.h"
#include "particle.h"
#include "misc/nanoflann.hpp"
#include "nanoflann_utils.h"

using namespace CGL;
using namespace std;
using namespace nanoflann;

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
  Fluid(int num);
  ~Fluid();

  void buildFluid();

  void simulate(double frames_per_sec, double simulation_steps, FluidParameters *fp,
                vector<Vector3D> external_accelerations,
                vector<Plane *> *collision_objects);

  void reset();

  void self_collide(int i, double simulation_steps); // dealing with the particle partices[i]

  void compute_neighbors(); // compute neighbor_lookup

  // computations from the paper Position Based Fluids
  double W(Vector3D x); // smoothing kernel
  double C_i(Particle *p); // density contraint
  void compute_density_est(Particle *p, vector<Particle *> *neighbors); // compute density_est
  Vector3D grad_W(Vector3D x); // gradient of W
  Vector3D grad_p_k_C_i(Particle* p_k, Particle* p_i, vector<Particle*>* neighbors); // grad of C_i wrt p_k
  void compute_lambda_i(Particle* p_i, vector<Particle*>* neighbors); // compute lambda_i
  void compute_position_update(Particle* p_i, vector<Particle*>* neighbors); // compute delta_pos
  double s_corr(Particle* p_i, Particle* p_j); // artifical pressure

  // Fluid properties
  double rho_0 = 100; // rest density
  double pmass = 1; // particle mass
  double epsilon = 10000; // the epsilon at the bottom of page 2
  double h = 0.1; // the h param for the smooth kernel W 
  double particle_radius = 0.01; // for particle collision
  int solver_iterations = 1;
  int num_particles;
  int num_x;
  int num_y;
  int num_z;

  // Fluid components
  vector<Particle> particles;

  // Neighbor map
  vector <vector<Particle*>*> neighbor_lookup;
  PointCloud cloud;
  KDTreeSingleIndexAdaptor<L2_Simple_Adaptor<double, PointCloud>, PointCloud, 3> *kdtree = NULL;
};

#endif /* FLUID_H */