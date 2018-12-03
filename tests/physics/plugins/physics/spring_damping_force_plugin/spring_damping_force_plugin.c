#include <ode/ode.h>
#include <plugins/physics.h>

static int BODIES_COUNT = 6;
static dBodyID *bodies = NULL;

void webots_physics_init() {
  int i = 0;
  bodies = malloc(BODIES_COUNT * sizeof(dBodyID));
  char name[32];
  for (i = 0; i < BODIES_COUNT; ++i) {
    sprintf(name, "END_POINT_SOLID_%d", i + 1);
    bodies[i] = dWebotsGetBodyFromDEF(name);
    if (bodies[i] == NULL)
      dWebotsConsolePrintf("Invalid body %s\n", name);
  }
}

void webots_physics_step() {
  static int step_count = 0;
  if (step_count >= 20)
    return;

  dBodyAddRelForce(bodies[0], 0.2, 0.0, 0.0);
  dBodyAddRelForce(bodies[1], 0.2, 0.0, 0.0);
  dBodyAddRelForce(bodies[2], 0.0, 5.0, 0.0);
  dBodyAddRelForce(bodies[3], 0.0, 0.0, -5.0);
  dBodyAddRelForce(bodies[4], 1.0, 0.0, 0.5);
  dBodyAddRelForce(bodies[5], 1.0, 0.0, 0.5);

  step_count++;
}

void webots_physics_cleanup() {
  free(bodies);
}
