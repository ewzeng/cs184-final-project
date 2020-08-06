#include <iostream>
#include <math.h>
#include <random>
#include <vector>

#include "fluid.h"
#include "collision/plane.h"
#include "collision/sphere.h"

using namespace std;

Fluid::Fluid(int num_particles) {
    this->num_particles = num_particles;
    buildFluid();
}

Fluid::~Fluid() {
    particles.clear();
}

void Fluid::buildFluid() {
    // Test implementation
    for (int i = 0; i < num_particles; i++) {
        Vector3D pos;
        pos.x = (rand() / RAND_MAX) * 10;
        pos.y = (rand() / RAND_MAX) * 10;
        pos.z = (rand() / RAND_MAX) * 10;
        particles.emplace_back(pos);
    }
}

void Fluid::simulate(double frames_per_sec, double simulation_steps, FluidParameters *fp,
                     vector<Vector3D> external_accelerations,
                     vector<CollisionObject *> *collision_objects) {
    // TODO
}

void Fluid::self_collide(Particle &p, double simulation_steps) {
    // TODO
}

void Fluid::reset() {
  Particles *p = &particles[0];
  for (int i = 0; i < particles.size(); i++) {
    p->position = p->start_position;
    p->last_position = p->start_position;
    p++;
  }
}
