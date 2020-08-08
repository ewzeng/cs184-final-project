#ifndef COLLISIONOBJECT
#define COLLISIONOBJECT

#include <nanogui/nanogui.h>

#include "CGL/CGL.h"
#include "../particle.h"

using namespace CGL;
using namespace std;
using namespace nanogui;

class CollisionObject {
public:
  virtual void collide(Particle &pm) = 0;

private:
  double friction;
};

#endif /* COLLISIONOBJECT */
