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
double Fluid::W(Vector3D x) {
    double z = x.norm() / h;
    
    // we can turn this into a global variables to make things a bit faster
    double multiplier = 1 / (M_PI * h * h * h);
    if (0 <= z && z <= 1) {
        return 1 - 3 / 2 * z * z + 3 / 4 * z * z * z;
    }
    else if (z <= 2) {
        1 / 4 * (2 - z) * (2 - z) * (2 - z);
    }
    else {
        return 0;
    }
}

// Density constraint for particle p
double Fluid::C_i(Particle *p) {
    return p->density_est / rho_0 - 1;
}

// Compute density estimate for particle p
void Fluid::compute_density_est(Particle *p, vector<Particle*> *neighbors) {
    p->density_est = 0;
    for (int i = 0; i < neighbors->size(); i++) {
        p->density_est += W(p->next_position - (*neighbors)[i]->next_position);
    }
    p->density_est *= pmass;
}

// The gradient of W
double Fluid::grad_W(Vector3D x) {
    // TODO (need to do some math on paper to figure out what this is)
    return 0.0;
}

// The gradient of C_i with respective to p_k
double Fluid::grad_p_k_C_i(Particle* p_k, Particle* p_i, vector<Particle*>* neighbors) {
    if (p_i != p_k) {
        return -grad_W(p_i->next_position - p_k->next_position);
    }
    
    double sum = 0;
    for (int j = 0; j < neighbors->size(); j++) {
        sum += grad_W(p_i->next_position - (*neighbors)[j]->next_position);
    }
    return sum / rho_0;
}

// Calculate lambda_i
void Fluid::compute_lambda_i(Particle* p_i, vector<Particle*>* neighbors) {
    double denom = epsilon;
    for (int k = 0; k < neighbors->size(); k++) {
        double tmp = grad_p_k_C_i((*neighbors)[k], p_i, neighbors);
        denom = tmp * tmp;
    }
    p_i->lambda = -C_i(p_i) / denom;
}

// Compute p_i->delta_pos
void Fluid::compute_position_update(Particle* p_i, vector<Particle*>* neighbors) {
    p_i->delta_pos = 0;
    for (int j = 0; j < neighbors->size(); j++) {
        p_i->delta_pos += (p_i->lambda + (*neighbors)[j]->lambda + s_corr(p_i, (*neighbors)[j]))
            * grad_W(p_i->next_position - (*neighbors)[j]->next_position);
    }
    p_i->delta_pos = p_i->delta_pos / rho_0;
}

// Compute s_corr, the artifical pressure term
double Fluid::s_corr(Particle* p_i, Particle* p_j) {
    // TODO
    return 0.0;
}
