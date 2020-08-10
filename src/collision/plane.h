#ifndef COLLISIONOBJECT_PLANE_H
#define COLLISIONOBJECT_PLANE_H

#include "CGL/CGL.h"
#include "../particle.h"

using namespace CGL;
using namespace std;

#define SURFACE_OFFSET 0.0001

struct Plane {
public:
  Plane(const Vector3D &point, const Vector3D &normal, double friction)
      : point(point), normal(normal.unit()), friction(friction) {}

  void collide(Particle& pm) {
      double lp = dot(pm.position - point, normal);
      double p = dot(pm.next_position - point, normal);
      if ((lp >= 0 && p < 0) || (lp <= 0 && p > 0)) {
          Vector3D tangent = pm.next_position - p * normal;
          tangent -= p / abs(p) * SURFACE_OFFSET * normal;
          pm.next_position = pm.position + (tangent - pm.position) * (1 - friction);
      }
  }

  Vector3D point;
  Vector3D normal;

  double friction;
};

#endif /* COLLISIONOBJECT_PLANE_H */
