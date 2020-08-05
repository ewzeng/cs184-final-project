#include <iostream>
#include <math.h>
#include <random>
#include <vector>

#include "cloth.h"
#include "collision/plane.h"
#include "collision/sphere.h"

using namespace std;

Cloth::Cloth(double width, double height, int num_width_points,
             int num_height_points, float thickness) {
  this->width = width;
  this->height = height;
  this->num_width_points = num_width_points;
  this->num_height_points = num_height_points;
  this->thickness = thickness;

  buildGrid();
  buildClothMesh();
}

Cloth::~Cloth() {
  point_masses.clear();
  springs.clear();

  if (clothMesh) {
    delete clothMesh;
  }
}

void Cloth::buildGrid() {
    // TODO (Part 1): Build a grid of masses and springs.

    // Build masses
    double w_step = width / (num_width_points - 1);
    double h_step = height / (num_height_points - 1);
    
    for (int j = 0; j < num_height_points; j++) {
        for (int i = 0; i < num_width_points; i++) {
            Vector3D pos;
            if (orientation == HORIZONTAL) {
                pos.x = i * w_step;
                pos.y = 1;
                pos.z = j * h_step;
            }
            else {
                pos.x = i * w_step;
                pos.y = j * h_step;
                pos.z = (2.0 * rand() / RAND_MAX - 1) / 1000;
            }

            vector<int> s{ i, j };
            bool p = std::find(pinned.begin(), pinned.end(), s) != pinned.end();
            point_masses.emplace_back(pos, p);
        }
    }

    // Build springs
    int len = point_masses.size();
    for (int j = 0; j < num_height_points; j++) {
        for (int i = 0; i < num_width_points; i++) {
            int curr = j * num_width_points + i;
            
            int up1 = curr - num_width_points;
            if (j > 0)
                springs.emplace_back(&point_masses[curr], &point_masses[up1], STRUCTURAL);

            int left1 = curr - 1;
            if (i > 0)
                springs.emplace_back(&point_masses[curr], &point_masses[left1], STRUCTURAL);

            int upleft = up1 - 1;
            if (j > 0 && i > 0)
                springs.emplace_back(&point_masses[curr], &point_masses[upleft], SHEARING);

            int upright = up1 + 1;
            if (j > 0 && i < num_width_points - 1)
                springs.emplace_back(&point_masses[curr], &point_masses[upright], SHEARING);

            int left2 = curr - 2;
            if (i > 1)
                springs.emplace_back(&point_masses[curr], &point_masses[left2], BENDING);

            int up2 = up1 - num_width_points;
            if (j > 1)
                springs.emplace_back(&point_masses[curr], &point_masses[up2], BENDING);
        }
    }
}

void Cloth::simulate(double frames_per_sec, double simulation_steps, ClothParameters *cp,
                     vector<Vector3D> external_accelerations,
                     vector<CollisionObject *> *collision_objects) {
  double mass = width * height * cp->density / num_width_points / num_height_points;
  double delta_t = 1.0f / frames_per_sec / simulation_steps;

  // TODO (Part 2): Compute total force acting on each point mass.

  // calculate external forces
  for (PointMass &pm : point_masses) {
      pm.forces = Vector3D(0);
      for (Vector3D &a : external_accelerations) {
          pm.forces += a * mass;
      }
  }

  // calculate spring forces
  for (Spring &s : springs) {
      Vector3D a2b = s.pm_b->position - s.pm_a->position;
      double f = cp->ks * (a2b.norm() - s.rest_length);

      if (s.spring_type == STRUCTURAL && cp->enable_structural_constraints) {
          s.pm_a->forces += f * a2b.unit();
          s.pm_b->forces -= f * a2b.unit();
      }
      
      if (s.spring_type == SHEARING && cp->enable_shearing_constraints) {
          s.pm_a->forces += f * a2b.unit();
          s.pm_b->forces -= f * a2b.unit();
      }

      if (s.spring_type == BENDING && cp->enable_bending_constraints) {
          s.pm_a->forces += f * a2b.unit() * 0.2;
          s.pm_b->forces -= f * a2b.unit() * 0.2;
      }
  }

  // take care of pinned masses
  for (PointMass& pm : point_masses) {
      if (pm.pinned) pm.forces = Vector3D(0);
  }

  // TODO (Part 2): Use Verlet integration to compute new point mass positions
  for (PointMass& pm : point_masses) {
      Vector3D newpos = pm.position + (1 - cp->damping / 100) * (pm.position - pm.last_position) 
          + pm.forces / mass * delta_t * delta_t;
      pm.last_position = pm.position;
      pm.position = newpos;
  }

  // TODO (Part 4): Handle self-collisions.
  build_spatial_map();
  for (PointMass& pm : point_masses) {
      self_collide(pm, simulation_steps);
  }

  // TODO (Part 3): Handle collisions with other primitives.
    for (int i = 0; i < collision_objects->size(); i++) {
        for (PointMass& pm : point_masses) {
            (*collision_objects)[i]->collide(pm);
        }
    }


  // TODO (Part 2): Constrain the changes to be such that the spring does not change
  // in length more than 10% per timestep [Provot 1995].
  for (Spring &s : springs) {
      Vector3D a2b = s.pm_b->position - s.pm_a->position;
      double over = a2b.norm() - s.rest_length * 1.1;
      if (over > 0) {
          if (!s.pm_a->pinned && !s.pm_b->pinned) {
              s.pm_a->position += over / 2 * a2b.unit();
              s.pm_b->position -= over / 2 * a2b.unit();
          }
          else if (s.pm_a->pinned && !s.pm_b->pinned) {
              s.pm_b->position -= over * a2b;
          }
          else if (!s.pm_a->pinned && s.pm_b->pinned) {
              s.pm_a->position += over * a2b;
          }
      }
  }

}

void Cloth::build_spatial_map() {
  for (const auto &entry : map) {
    delete(entry.second);
  }
  map.clear();

  // TODO (Part 4): Build a spatial map out of all of the point masses.
  for (PointMass& pm : point_masses) {
      float k = hash_position(pm.position);
      if (map.find(k) == map.end()) {
          // Not found
          map[k] = new vector<PointMass*>();
      }
      map[k]->push_back(&pm);
  }
}

void Cloth::self_collide(PointMass &pm, double simulation_steps) {
    // TODO (Part 4): Handle self-collision for a given point mass.
    float k = hash_position(pm.position);
    Vector3D total = Vector3D(0);
    int cnt = 0;
    for (PointMass* p : *map[k]) {
        if (p != &pm) {
            Vector3D p2pm = pm.position - p->position;
            double correction = 2 * thickness - p2pm.norm();
            if (correction > 0) {
                total += p2pm.unit() * correction;
                cnt++;
            }
        }
    }
    if (cnt > 0) pm.position += total / cnt / simulation_steps;
}

float Cloth::hash_position(Vector3D pos) {
    // TODO (Part 4): Hash a 3D position into a unique float identifier that represents membership in some 3D box volume.
    double w = 3 * width / num_width_points;
    double h = 3 * height / num_height_points;
    double t = max(w, h);

    int x = floor(pos.x / w);
    int y = floor(pos.y / h);
    int z = floor(pos.z / t);

    // Hash function not great. Can we do SHA?
    return x * 1000 * 1000 + y * 1000 + z;
}

///////////////////////////////////////////////////////
/// YOU DO NOT NEED TO REFER TO ANY CODE BELOW THIS ///
///////////////////////////////////////////////////////

void Cloth::reset() {
  PointMass *pm = &point_masses[0];
  for (int i = 0; i < point_masses.size(); i++) {
    pm->position = pm->start_position;
    pm->last_position = pm->start_position;
    pm++;
  }
}

void Cloth::buildClothMesh() {
  if (point_masses.size() == 0) return;

  ClothMesh *clothMesh = new ClothMesh();
  vector<Triangle *> triangles;

  // Create vector of triangles
  for (int y = 0; y < num_height_points - 1; y++) {
    for (int x = 0; x < num_width_points - 1; x++) {
      PointMass *pm = &point_masses[y * num_width_points + x];
      // Get neighboring point masses:
      /*                      *
       * pm_A -------- pm_B   *
       *             /        *
       *  |         /   |     *
       *  |        /    |     *
       *  |       /     |     *
       *  |      /      |     *
       *  |     /       |     *
       *  |    /        |     *
       *      /               *
       * pm_C -------- pm_D   *
       *                      *
       */
      
      float u_min = x;
      u_min /= num_width_points - 1;
      float u_max = x + 1;
      u_max /= num_width_points - 1;
      float v_min = y;
      v_min /= num_height_points - 1;
      float v_max = y + 1;
      v_max /= num_height_points - 1;
      
      PointMass *pm_A = pm                       ;
      PointMass *pm_B = pm                    + 1;
      PointMass *pm_C = pm + num_width_points    ;
      PointMass *pm_D = pm + num_width_points + 1;
      
      Vector3D uv_A = Vector3D(u_min, v_min, 0);
      Vector3D uv_B = Vector3D(u_max, v_min, 0);
      Vector3D uv_C = Vector3D(u_min, v_max, 0);
      Vector3D uv_D = Vector3D(u_max, v_max, 0);
      
      
      // Both triangles defined by vertices in counter-clockwise orientation
      triangles.push_back(new Triangle(pm_A, pm_C, pm_B, 
                                       uv_A, uv_C, uv_B));
      triangles.push_back(new Triangle(pm_B, pm_C, pm_D, 
                                       uv_B, uv_C, uv_D));
    }
  }

  // For each triangle in row-order, create 3 edges and 3 internal halfedges
  for (int i = 0; i < triangles.size(); i++) {
    Triangle *t = triangles[i];

    // Allocate new halfedges on heap
    Halfedge *h1 = new Halfedge();
    Halfedge *h2 = new Halfedge();
    Halfedge *h3 = new Halfedge();

    // Allocate new edges on heap
    Edge *e1 = new Edge();
    Edge *e2 = new Edge();
    Edge *e3 = new Edge();

    // Assign a halfedge pointer to the triangle
    t->halfedge = h1;

    // Assign halfedge pointers to point masses
    t->pm1->halfedge = h1;
    t->pm2->halfedge = h2;
    t->pm3->halfedge = h3;

    // Update all halfedge pointers
    h1->edge = e1;
    h1->next = h2;
    h1->pm = t->pm1;
    h1->triangle = t;

    h2->edge = e2;
    h2->next = h3;
    h2->pm = t->pm2;
    h2->triangle = t;

    h3->edge = e3;
    h3->next = h1;
    h3->pm = t->pm3;
    h3->triangle = t;
  }

  // Go back through the cloth mesh and link triangles together using halfedge
  // twin pointers

  // Convenient variables for math
  int num_height_tris = (num_height_points - 1) * 2;
  int num_width_tris = (num_width_points - 1) * 2;

  bool topLeft = true;
  for (int i = 0; i < triangles.size(); i++) {
    Triangle *t = triangles[i];

    if (topLeft) {
      // Get left triangle, if it exists
      if (i % num_width_tris != 0) { // Not a left-most triangle
        Triangle *temp = triangles[i - 1];
        t->pm1->halfedge->twin = temp->pm3->halfedge;
      } else {
        t->pm1->halfedge->twin = nullptr;
      }

      // Get triangle above, if it exists
      if (i >= num_width_tris) { // Not a top-most triangle
        Triangle *temp = triangles[i - num_width_tris + 1];
        t->pm3->halfedge->twin = temp->pm2->halfedge;
      } else {
        t->pm3->halfedge->twin = nullptr;
      }

      // Get triangle to bottom right; guaranteed to exist
      Triangle *temp = triangles[i + 1];
      t->pm2->halfedge->twin = temp->pm1->halfedge;
    } else {
      // Get right triangle, if it exists
      if (i % num_width_tris != num_width_tris - 1) { // Not a right-most triangle
        Triangle *temp = triangles[i + 1];
        t->pm3->halfedge->twin = temp->pm1->halfedge;
      } else {
        t->pm3->halfedge->twin = nullptr;
      }

      // Get triangle below, if it exists
      if (i + num_width_tris - 1 < 1.0f * num_width_tris * num_height_tris / 2.0f) { // Not a bottom-most triangle
        Triangle *temp = triangles[i + num_width_tris - 1];
        t->pm2->halfedge->twin = temp->pm3->halfedge;
      } else {
        t->pm2->halfedge->twin = nullptr;
      }

      // Get triangle to top left; guaranteed to exist
      Triangle *temp = triangles[i - 1];
      t->pm1->halfedge->twin = temp->pm2->halfedge;
    }

    topLeft = !topLeft;
  }

  clothMesh->triangles = triangles;
  this->clothMesh = clothMesh;
}
