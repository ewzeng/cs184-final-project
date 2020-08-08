#include "iostream"
#include <nanogui/nanogui.h>

#include "plane.h"

using namespace std;
using namespace CGL;

#define SURFACE_OFFSET 0.0001

void Plane::collide(Particle &pm) {
    // TODO (Part 3): Handle collisions with planes.
    double lp = dot(pm.last_position - point, normal);
    double p = dot(pm.position - point, normal);
    if ((lp >= 0 && p < 0) || (lp <= 0 && p > 0)) {
         Vector3D tangent = pm.position - p * normal;
         tangent -= p / abs(p) * SURFACE_OFFSET * normal;
         pm.position = pm.last_position + (tangent - pm.last_position) * (1 - friction);
    }
}
