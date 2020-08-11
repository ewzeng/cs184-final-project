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
      Vector3D next = pm.next_position + pm.delta_pos;
      double lp = dot(pm.position - point, normal);
      double p = dot(next - point, normal);
      if ((lp >= 0 && p < 0) || (lp <= 0 && p > 0)) {
          Vector3D tangent = next - p * normal;
          tangent -= p / abs(p) * SURFACE_OFFSET * normal;
          pm.delta_pos = pm.position + (tangent - pm.position) * (1 - friction) - pm.next_position;
      }
  }

  Vector3D point;
  Vector3D normal;

  double friction;
};

#endif /* COLLISIONOBJECT_PLANE_H */
