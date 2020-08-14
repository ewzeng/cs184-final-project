#define _USE_MATH_DEFINES

#include <iostream>
#include <math.h>
#include <random>
#include <vector>

#include "misc/nanoflann.hpp"
#include "nanoflann_utils.h"
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

Fluid::Fluid(int num) {
    this->num_particles = num;
    buildFluid();
}

Fluid::~Fluid() {
    particles.clear();
}

void Fluid::buildFluid() {
    // USE THIS INSTEAD IF WANT CUBE STARTING POINT
    //for (int i = 0; i < num_x; i++) {
    //    for (int j = 0; j < num_y; j++) {
    //        for (int k = 0; k < num_z; k++) {
    //            Vector3D pos;
    //            pos.x = 0.4 / num_x * i - 0.2;
    //            pos.y = 0.3 / num_y * j;
    //            pos.z = - 0.18 / num_z * k + 0.09;
    //            particles.emplace_back(pos);
    //        }
    //    }
    //}

    for (int i = 0; i < num_particles; i++) {
        Vector3D pos;
        pos.x = (rand() * 0.8/ RAND_MAX) - 0.4;
        pos.y = (rand() * 0.5/ RAND_MAX);
        pos.z = - (rand() * 0.18 / RAND_MAX) + 0.09;
        particles.emplace_back(pos);
    }
}

void Fluid::simulate(double frames_per_sec, double simulation_steps, FluidParameters *fp,
                     vector<Vector3D> external_accelerations,
                     vector<Plane *> *collision_objects) {
    double delta_t = 1.0f / frames_per_sec / simulation_steps;


    // Apply external forces and predict position
    //----------------------------------
    for (Particle& p : particles) {
        for (Vector3D& a : external_accelerations) {
            p.velocity += delta_t * (a + p.forces / pmass);
        }
        p.next_position = p.position + delta_t * p.velocity;
    }

    // Find neighboring particles (using nanoflann)
    //---------------------------
    compute_neighbors();
    //// Placeholder code here
    //neighbor_lookup.clear(); // should I free each entry first?
    //for (int i = 0; i < particles.size(); i++) {
    //    neighbor_lookup.push_back(new vector<Particle*>());
    //    for (int j = 0; j < particles.size(); j++) {
    //        if (i != j) {
    //            neighbor_lookup[i]->push_back(&particles[j]);
    //        }
    //    }
    //}

    // Tweak particle positions using fancy math
    // Perform collision detection
    // Supposed to be a huge while loop
    //------------------------------------------------------------------------

    for (int it = 0; it < solver_iterations; it++) {
        for (int i = 0; i < particles.size(); i++) {
            compute_density_est(&particles[i], neighbor_lookup[i]);
        }

        for (int i = 0; i < particles.size(); i++) {
            compute_lambda_i(&particles[i], neighbor_lookup[i]);
        }

        for (int i = 0; i < particles.size(); i++) {
            compute_position_update(&particles[i], neighbor_lookup[i]);
        }

        // collisions
        for (int i = 0; i < particles.size(); i++) {
            self_collide(i, simulation_steps);
        }

        for (int i = 0; i < collision_objects->size(); i++) {
            for (Particle& p : particles) {
                (*collision_objects)[i]->collide(p);
            }
        }

        // update position
        for (Particle& p : particles) {
            p.next_position += p.delta_pos;
        }
    }

    // Update velocity and apply confinements
    //---------------------------------------
    for (int i = 0; i < particles.size(); i++) {
        particles[i].velocity = (particles[i].next_position - particles[i].position) / delta_t;
    }

    // Vorticity
    compute_omega();
    apply_vorticity();

    // Viscosity
    for (int i = 0; i < particles.size(); i++) {
        Vector3D vadjust = Vector3D(0);
        for (int j = 0; j < neighbor_lookup[i]->size(); j++) {
            vadjust += (particles[i].velocity - (*neighbor_lookup[i])[j]->velocity)
                * W(particles[i].next_position - (*neighbor_lookup[i])[j]->next_position)
                * viscosity_constant;
        }
        particles[i].velocity += vadjust;

        // Update position to final value
        particles[i].position = particles[i].next_position;
    }
}

void Fluid::self_collide(int i, double simulation_steps) {
    Vector3D total = Vector3D(0);
    for (Particle* p : *neighbor_lookup[i]) {
        if (p != &particles[i]) {
            Vector3D p2i = particles[i].next_position + particles[i].delta_pos 
                - p->next_position - p->delta_pos;
            double correction = 2 * particle_radius - p2i.norm();
            if (correction > 0) {
                total += p2i.unit() * correction * particle_bounce;
            }
        }
    }
    particles[i].delta_pos += total / simulation_steps;
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

// Part of the structure of this function comes from examples provided in the nanoflann library
void Fluid::compute_neighbors() {
    // Build pointcloud
    cloud.pts.clear();
    for (int i = 0; i < particles.size(); i++) {
        cloud.pts.push_back(&particles[i]);
    }

    // Build kdtree
    if (kdtree != NULL) free(kdtree);
    kdtree = new KDTreeSingleIndexAdaptor<L2_Simple_Adaptor<double, PointCloud>, 
                PointCloud, 3>(3, cloud, KDTreeSingleIndexAdaptorParams());
    kdtree->buildIndex();

    // Clear neighbor_lookup (this assumes we are creating on heap)
    for (int i = 0; i < neighbor_lookup.size(); i++) {
        free(neighbor_lookup[i]);
    }
    neighbor_lookup.clear();

    // create neighbor_lookup
    SearchParams params;
    params.sorted = false; // I think sorting takes more time
    vector<std::pair<size_t, double> > ret_matches; // Should we put this inside or outside the loop?

    for (int i = 0; i < particles.size(); i++) {
        vector<Particle*>* neighbors = new vector<Particle*>();
        neighbor_lookup.push_back(neighbors);
        double target[3];
        target[0] = particles[i].next_position.x;
        target[1] = particles[i].next_position.y;
        target[2] = particles[i].next_position.z;

        size_t nMatches = kdtree->radiusSearch(&target[0], 2 * h, ret_matches, params);
        for (size_t j = 0; j < nMatches; j++) {
            if (i != ret_matches[j].first) {    
                neighbors->push_back(&(particles[ret_matches[j].first]));
            }
        }
    }
}

// Smoothing kernel, implemented as a simple cubic B-spline
double Fluid::W(Vector3D x) {
    double z = x.norm() / h;
    double multiplier = 1 / (M_PI * h * h * h); // turn this into a global to make things a bit faster
    if (0 <= z && z <= 1) {
        return (1 - 3 / 2 * z * z + 3 / 4 * z * z * z) * multiplier;
    }
    else if (z <= 2) {
        return 1 / 4 * (2 - z) * (2 - z) * (2 - z) * multiplier;
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
Vector3D Fluid::grad_W(Vector3D x) {
    // By rotational symmetry, the steepest slope should be pointing towards the origin
    // We exploit this symmetry to do some clever tricks to compute the gradient
    // Be careful of signs! u points toward the origin, but when computing the abs value of the
    // gradient, we get some negative signs that we have to negate.

    if (x.norm() == 0) return x;

    Vector3D u = -x.unit();
    double z = x.norm() / h;
    double multiplier = 1 / (M_PI * h * h * h); // this into a global variables to make things a bit faster
    if (0 <= z && z <= 1) {
        return -(-3 * z + 9 / 4 * z * z) / h * multiplier * u;
    }
    else if (z <= 2) {
        return -3 / 4 * (2 - z) * (2 - z) * (-1) / h * multiplier * u;
    }
    else {
        return Vector3D(0);
    }
}

// The gradient of C_i with respective to p_k
Vector3D Fluid::grad_p_k_C_i(Particle* p_k, Particle* p_i, vector<Particle*>* neighbors) {
    if (p_i != p_k) {
        return -grad_W(p_i->next_position - p_k->next_position) / rho_0;
    }
    
    Vector3D sum = 0;
    for (int j = 0; j < neighbors->size(); j++) {
        sum += grad_W(p_i->next_position - (*neighbors)[j]->next_position);
    }
    return sum / rho_0;
}

// Calculate lambda_i
// Make sure we call compute_density_est before
void Fluid::compute_lambda_i(Particle* p_i, vector<Particle*>* neighbors) {
    double denom = epsilon;
    for (int k = 0; k < neighbors->size(); k++) {
        denom += grad_p_k_C_i((*neighbors)[k], p_i, neighbors).norm2();
    }
    p_i->lambda = -C_i(p_i) / denom;
}

// Compute p_i->delta_pos
// Make sure we call compute_lambda_i before
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
    double tmp = W(p_i->next_position - p_j->next_position) / W(Vector3D(0.2 * h, 0, 0));
    return -s_corr_constant * tmp * tmp * tmp * tmp;
}

void Fluid::compute_omega() {
    for (int i = 0; i < particles.size(); i++) {
        vector<Particle*>* neighbors = neighbor_lookup[i];
        particles[i].omega = Vector3D(0);
        for (Particle* pj : *neighbors) {
            particles[i].omega += cross(particles[i].velocity - pj->velocity,
                grad_W(particles[i].next_position - pj->next_position));
        }
    }
}

// Apply voriticity
// I couldn't understand the math in the paper, so I used the following link to help:
// https://interactivecomputergraphics.github.io/SPH-Tutorial/slides/06_vorticity.pdf
void Fluid::apply_vorticity() {
    for (int i = 0; i < particles.size(); i++) {
        vector<Particle*>* neighbors = neighbor_lookup[i];
        Vector3D eta = Vector3D(0);
        for (Particle* pj : *neighbors) {
            eta += pj->omega.norm() * grad_W(particles[i].next_position - pj->next_position);
        }
        if (eta.norm() == 0) particles[i].forces = Vector3D(0);
        else particles[i].forces = vorticity_constant * cross(eta.unit(), particles[i].omega);
    }
}
