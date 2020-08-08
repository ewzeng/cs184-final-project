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

    //for (int i = 0; i < num_particles; i++) {
    //    Vector3D pos;
    //    pos.x = (rand() * 2.0/ RAND_MAX) - 1;
    //    pos.y = (rand() * 2.0/ RAND_MAX) - 1;
    //    pos.z = -1;
    //    
    //}
}

void Fluid::simulate(double frames_per_sec, double simulation_steps, FluidParameters *fp,
                     vector<Vector3D> external_accelerations,
                     vector<CollisionObject *> *collision_objects) {
    for (Particle& p : particles) {
        p.position -= Vector3D(0.0001, 0.0001, 0);
    }
}

void Fluid::self_collide(Particle &p, double simulation_steps) {
    // TODO
}

void Fluid::reset() {
  Particle *p = &particles[0];
  for (int i = 0; i < particles.size(); i++) {
    p->position = p->start_position;
    p->last_position = p->start_position;
    p++;
  }
}
