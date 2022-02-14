#include <ode/ode.h>
#include <plugins/physics.h>

#ifdef WINDOWS
#include <direct.h>
#define GetCurrentDir _getcwd
#define SEP '\\'
#else
#include <unistd.h>
#define GetCurrentDir getcwd
#define SEP '/'
#endif

#ifndef bool
#define bool char
#endif

#ifndef true
// clang-format off
#define true ((bool)1)
// clang-format on
#endif

#ifndef false
// clang-format off
#define false ((bool)0)
// clang-format on
#endif

static dWorldID main_world = 0;
static int step_counter = 0;
static char current_path[1024];
static bool message_sent = false;

static void send_message(const char *msg, bool success) {
  char *msgString = (char *)malloc(strlen(msg) + 16);
  sprintf(msgString, "%s: %s", success ? "Success" : "Failure", msg);
  dWebotsConsolePrintf("%s\n", msgString);
  dWebotsSend(0, msgString, strlen(msgString) + 1);
  free(msgString);

  message_sent = true;
}

void webots_physics_init() {
  // the main world can be get during at this step:
  // ODE is still not clusterized
  dBodyID capsule_body = dWebotsGetBodyFromDEF("CAPSULE");
  main_world = dBodyGetWorld(capsule_body);

  if (!GetCurrentDir(current_path, sizeof(current_path)))
    current_path[0] = '\0';
}

void webots_physics_step() {
  step_counter++;

  if (message_sent)  // send only once some information
    return;

  if (strlen(current_path) <= 0)
    send_message("Cannot retrieve the current path of the physics plugin", false);

  if (!main_world)
    send_message("World badly defined", false);

  if (step_counter == 100) {
    char filename[sizeof(current_path) + 8];
    snprintf(filename, sizeof(filename), "%s%code.dif", current_path, SEP);

    FILE *f = fopen(filename, "w");

    if (f == NULL) {
      send_message("Cannot create the output file", false);
      return;
    }

    dWorldExportDIF(main_world, f, "determinism_test ");
    fclose(f);

    send_message("ODE DIF generated", true);
  }
}

int webots_physics_collide(dGeomID g1, dGeomID g2) {
  return 0;
}

void webots_physics_cleanup() {
  return;
}
