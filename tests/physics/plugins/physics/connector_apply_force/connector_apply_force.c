#include <ode/ode.h>
#include <plugins/physics.h>

static dBodyID body = NULL;

void webots_physics_init() {
  body = dWebotsGetBodyFromDEF("ACTIVE_ROBOT");
}

void webots_physics_step() {
  static int counter = 0;
  static double force = 0;
  ++counter;

  if (counter < 3 || force > 2.5)
    return;

  force += 0.1;
  dBodyAddRelForce(body, 0, 0, -force);
}

int webots_physics_collide(dGeomID g1, dGeomID g2) {
  return 0;
}

void webots_physics_cleanup() {
}
