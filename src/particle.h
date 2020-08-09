#ifndef PARTICLE_H
#define PARTICLE_H

#include "CGL/CGL.h"
#include "CGL/misc.h"
#include "CGL/vector3D.h"

using namespace CGL;

struct Particle {
  Particle(Vector3D position)
      : start_position(position), position(position),
        next_position(position), velocity(0) {}

  // static values
  Vector3D start_position;

  // dynamic values
  Vector3D position;
  Vector3D next_position;
  Vector3D forces;
  Vector3D velocity;
  double density_est; // density estimate
  double lambda; // needed for the math
  Vector3D delta_pos; // for updating the particle's position 
};

#endif /* PARTICLE_H */
