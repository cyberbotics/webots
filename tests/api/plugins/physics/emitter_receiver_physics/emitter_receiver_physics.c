#include <ode/ode.h>
#include <plugins/physics.h>

void webots_physics_init() {
  return;
}

void webots_physics_step() {
  int size;
  const double *values;
  char msgString[256];
  double msgDouble[3] = {0.11, 0.22, 0.33};

  values = (double *)dWebotsReceive(&size);
  size /= sizeof(double);
  if (size == 1) {
    sprintf(msgString, "The value sent is: %.2f", values[0]);
    dWebotsSend(1, msgString, strlen(msgString) + 1);

  } else if (size == 2) {
    sprintf(msgString, "The value sent is: [%.2f, %.2f]", values[0], values[1]);
    dWebotsSend(2, msgString, strlen(msgString) + 1);

  } else if (size == 3) {
    dWebotsSend(-1, "Many values are sent", strlen("Many values are sent") + 1);

  } else if (size == 7) {
    // send message from physics plugin to physics plugin
    dWebotsSend(0, msgDouble, 3 * sizeof(double));
  }
}

int webots_physics_collide(dGeomID g1, dGeomID g2) {
  return 0;
}

void webots_physics_cleanup() {
  return;
}
