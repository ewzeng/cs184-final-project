#include <iostream>
#include <math.h>
#include <random>
#include <vector>

#include "fluid.h"
#include "collision/plane.h"

using namespace std;

Fluid::Fluid(int num_x, int num_y, int num_z) {
    this->num_x = num_x;
    this->num_y = num_y;
    this->num_z = num_z;
    this->num_particles = num_x * num_y * num_z;
    buildFluid();
}

Fluid::~Fluid() {
    particles.clear();
}

void Fluid::buildFluid() {
    // Test implementation
    for (int i = 0; i < num_x; i++) {
        for (int j = 0; j < num_y; j++) {
            for (int k = 0; k < num_z; k++) {
                Vector3D pos;
                pos.x = 0.5 / num_x * i;
                pos.y = 0.5 / num_y * j;
                pos.z = - 0.5 / num_z * k;
                particles.emplace_back(pos);
            }
        }
    }
}

void Fluid::simulate(double frames_per_sec, double simulation_steps, FluidParameters *fp,
                     vector<Vector3D> external_accelerations,
                     vector<CollisionObject *> *collision_objects) {
    pmass = fp->particle_mass;
    double delta_t = 1.0f / frames_per_sec / simulation_steps;


    // Apply external forces and predict position
    //----------------------------------
    for (Particle& p : particles) {
        for (Vector3D& a : external_accelerations) {
            p.velocity += delta_t * a;
        }
        p.next_position = p.position + delta_t * p.velocity;
    }

    // Find neighboring particles
    //---------------------------
    // Idea: use an outside KDtree library (nanoflann)

    // Tweak particle positions using fancy math
    // Perform collision detection
    // Supposed to be a huge while loop
    //------------------------------------------------------------------------

    // Update velocity and apply confinements
    //---------------------------------------
    for (Particle& p : particles) {
        p.velocity = (p.next_position - p.position) / delta_t;

        // DO SOMETHING RELATED TO VORTICITY & CONFINEMENT

        p.position = p.next_position;
    }
}

void Fluid::self_collide(Particle &p, double simulation_steps) {
    // TODO
}

void Fluid::reset() {
  Particle *p = &particles[0];
  for (int i = 0; i < particles.size(); i++) {
    p->position = p->start_position;
    p->next_position = p->start_position;
    p->velocity = 0;
    p++;
  }
}

// Smoothing kernel, implemented as a simple cubic B-spline
// h will be hardcoded in
double Fluid::W(Vector3D x) {
    // TODO
    return 0.0;
}

// Density constraint for particle p
double Fluid::C_i(Particle *p) {
    return p->density_est / rho_0 - 1;
}

// Compute density estimate for particle p
void Fluid::compute_density_est(Particle *p, vector<Particle*> *neighbors) {
    p->density_est = 0;
    for (int i = 0; i < neighbors->size(); i++) {
        p->density_est += W(p->position - (*neighbors)[i]->position);
    }
    p->density_est *= pmass;
}
