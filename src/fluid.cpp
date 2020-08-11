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
    // Test implementation
    //for (int i = 0; i < num_x; i++) {
    //    for (int j = 0; j < num_y; j++) {
    //        for (int k = 0; k < num_z; k++) {
    //            Vector3D pos;
    //            pos.x = 0.5 / num_x * i;
    //            pos.y = 0.5 / num_y * j;
    //            pos.z = - 0.5 / num_z * k;
    //            particles.emplace_back(pos);
    //        }
    //    }
    //}

    for (int i = 0; i < num_particles; i++) {
        Vector3D pos;
        pos.x = (rand() * 1.0/ RAND_MAX) - 0.5;
        pos.y = (rand() * 0.5/ RAND_MAX);
        pos.z = - (rand() * 1.0 / RAND_MAX);
        particles.emplace_back(pos);
    }
}

void Fluid::simulate(double frames_per_sec, double simulation_steps, FluidParameters *fp,
                     vector<Vector3D> external_accelerations,
                     vector<Plane *> *collision_objects) {
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
    // Placeholder code here
    neighbor_lookup.clear(); // should I free each entry first?
    for (int i = 0; i < particles.size(); i++) {
        neighbor_lookup.push_back(new vector<Particle*>());
        for (int j = 0; j < particles.size(); j++) {
            if (i != j) {
                neighbor_lookup[i]->push_back(&particles[j]);
            }
        }
    }

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
    for (Particle& p : particles) {
        p.velocity = (p.next_position - p.position) / delta_t;

        // DO SOMETHING RELATED TO VORTICITY & CONFINEMENT

        p.position = p.next_position;
    }

}

void Fluid::self_collide(int i, double simulation_steps) {
    Vector3D total = Vector3D(0);
    int cnt = 0;
    for (Particle* p : *neighbor_lookup[i]) {
        if (p != &particles[i]) {
            Vector3D p2i = particles[i].position - p->position;
            double correction = 2 * particle_radius - p2i.norm();
            if (correction > 0) {
                total += p2i.unit() * correction;
                cnt++;
            }
        }
    }
    if (cnt > 0) particles[i].delta_pos += total / cnt / simulation_steps;
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

        size_t nMatches = kdtree->radiusSearch(&target[0], h, ret_matches, params);
        for (size_t i = 0; i < nMatches; i++) {
            neighbors->push_back(&(particles[ret_matches[i].first]));
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
        return -3 / 4 * (2 - z) * (2 - z) * (-z) / h * multiplier * u;
    }
    else {
        return Vector3D(0);
    }
}

// The gradient of C_i with respective to p_k
Vector3D Fluid::grad_p_k_C_i(Particle* p_k, Particle* p_i, vector<Particle*>* neighbors) {
    if (p_i != p_k) {
        return -grad_W(p_i->next_position - p_k->next_position);
    }
    
    Vector3D sum = 0;
    for (int j = 0; j < neighbors->size(); j++) {
        sum += grad_W(p_i->next_position - (*neighbors)[j]->next_position);
    }
    return sum / rho_0;
}

// Calculate lambda_i
void Fluid::compute_lambda_i(Particle* p_i, vector<Particle*>* neighbors) {
    double denom = epsilon;
    for (int k = 0; k < neighbors->size(); k++) {
        denom += grad_p_k_C_i((*neighbors)[k], p_i, neighbors).norm2();
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
