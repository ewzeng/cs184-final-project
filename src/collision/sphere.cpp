#include <nanogui/nanogui.h>

#include "../clothMesh.h"
#include "../misc/sphere_drawing.h"
#include "sphere.h"

using namespace nanogui;
using namespace CGL;

void Sphere::collide(PointMass &pm) {
    // TODO (Part 3): Handle collisions with spheres.
	Vector3D o2p = pm.position - origin;
	if (o2p.norm() >= radius) {
		return;
	}

	Vector3D tangent = origin + o2p.unit() * radius;
	pm.position = pm.last_position + (tangent - pm.last_position) * (1 - friction);

}

void Sphere::render(GLShader &shader) {
  // We decrease the radius here so flat triangles don't behave strangely
  // and intersect with the sphere when rendered
  m_sphere_mesh.draw_sphere(shader, origin, radius * 0.92);
}
