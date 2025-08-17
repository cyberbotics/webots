#include <webots/brake.h>
#include <webots/motor.h>
#include <webots/nodes.h>
#include <webots/robot.h>
#include <webots/supervisor.h>

#include "../../../lib/ts_assertion.h"
#include "../../../lib/ts_utils.h"

#include <math.h>
#include <stdio.h>

#define TIME_STEP 32

double angular_speed_of(WbNodeRef node) {
  const double *v = wb_supervisor_node_get_velocity(node);
  return sqrt(v[3] * v[3] + v[4] * v[4] + v[5] * v[5]);
}

struct HingeJoint2Data {
  WbDeviceTag driveMotor;
  WbNodeRef shaftNode;
};

void init(struct HingeJoint2Data *jd, char *driveMotorName, char *shaftDef) {
  jd->driveMotor = wb_robot_get_device(driveMotorName);
  wb_motor_set_position(jd->driveMotor, INFINITY);
  wb_motor_set_velocity(jd->driveMotor, 0.0);
  wb_brake_set_damping_constant(wb_motor_get_brake(jd->driveMotor), 28.0);
  jd->shaftNode = wb_supervisor_node_get_from_def(shaftDef);
}

void assert_same_speed_for_secs(struct HingeJoint2Data j1, struct HingeJoint2Data j2, double durationSecs) {
  const double tolerance = 0.001;

  double t = 0;
  while (wb_robot_step(TIME_STEP) != -1 && t < durationSecs) {
    t += (double)TIME_STEP / 1000.0;
    double expSpeed = angular_speed_of(j1.shaftNode);
    double actSpeed = angular_speed_of(j2.shaftNode);
    ts_assert_double_in_delta(actSpeed, expSpeed, tolerance, "Unexpected angle rate (expected = %lf, measured = %lf)", expSpeed,
                              actSpeed);
    printf("angle_rate: %lf expected: %lf\n", actSpeed, expSpeed);
  }
}

int main(int argc, char **argv) {
  ts_setup(argv[0]);

  struct HingeJoint2Data joint1, joint2;

  init(&joint1, "joint1motor", "HINGE2JOINT1_SHAFT");
  init(&joint2, "joint2motor", "HINGE2JOINT2_SHAFT");

  // Rotate the first axis of the second joint by 90 degrees
  // (takes about 1 second)
  wb_motor_set_position(wb_robot_get_device("joint2turnmotor"), 1.57);
  double t = 0.0;  // elapsed simulation time
  while (wb_robot_step(TIME_STEP) != -1 && t < 1.0) {
    t += (double)TIME_STEP / 1000.0;
  }

  // Accerate toward max velocity for 1 second.
  wb_motor_set_velocity(joint1.driveMotor, 6.28);
  wb_motor_set_velocity(joint2.driveMotor, 6.28);
  assert_same_speed_for_secs(joint1, joint2, 1.0);

  // Coast to a stop
  wb_motor_set_velocity(joint1.driveMotor, 0.0);
  wb_motor_set_available_torque(joint1.driveMotor, 0.0);
  wb_motor_set_velocity(joint2.driveMotor, 0.0);
  wb_motor_set_available_torque(joint2.driveMotor, 0.0);
  assert_same_speed_for_secs(joint1, joint2, 1.0);

  ts_send_success();
  return EXIT_SUCCESS;
}
