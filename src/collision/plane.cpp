#include "iostream"
#include <nanogui/nanogui.h>

#include "plane.h"

using namespace std;
using namespace CGL;

#define SURFACE_OFFSET 0.0001

void Plane::collide(Particle &pm) {
    // TODO (Part 3): Handle collisions with planes.
    double lp = dot(pm.position - point, normal);
    double p = dot(pm.next_position - point, normal);
    if ((lp >= 0 && p < 0) || (lp <= 0 && p > 0)) {
         Vector3D tangent = pm.next_position - p * normal;
         tangent -= p / abs(p) * SURFACE_OFFSET * normal;
         pm.next_position = pm.position + (tangent - pm.position) * (1 - friction);
    }
}