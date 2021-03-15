#include <webots/motor.h>
#include <webots/robot.h>
#include <webots/supervisor.h>

#include "../../../lib/ts_assertion.h"
#include "../../../lib/ts_utils.h"

#define TIME_STEP 32

int main(int argc, char **argv) {
  ts_setup(argv[0]);

  // This test suite verifies that if the position field, as set in the jointParameters, is outside of the bounds of the
  // minPosition and maxPosition of RotationalMotor, these bounds must adadpt accordingly.
  // This suite also checks that the correct pairing is done (device <-> jointParameters, device2 <-> jointParameters2)

  // SliderJoint Tests -------------------------------------------------------------------------------------------------
  WbDeviceTag slider_joint_lower_limit_motor = wb_robot_get_device("slider_joint_lower_limit_motor");

  // for motor: [minPosition, maxPosition] was [-0.1, 0.1] in world, should be [-0.2, 0.1] after loading
  ts_assert_double_equal(
    wb_motor_get_min_position(slider_joint_lower_limit_motor), -0.2,
    "'minPosition' of SliderJoint (for motor) didn't adjust correctly when position is below lower limit.");
  ts_assert_double_equal(wb_motor_get_max_position(slider_joint_lower_limit_motor), 0.1,
                         "'maxPosition' of SliderJoint (for motor) changed but shouldn't, lower limit being tested.");

  WbDeviceTag slider_joint_upper_limit_motor = wb_robot_get_device("slider_joint_upper_limit_motor");

  // for motor: [minPosition, maxPosition] was [-0.1, 0.1] in world, should be [-0.1, 0.2] after loading
  ts_assert_double_equal(wb_motor_get_min_position(slider_joint_upper_limit_motor), -0.1,
                         "'minPosition' of SliderJoint (for motor) changed but shouldn't, upper limit being tested.");
  ts_assert_double_equal(
    wb_motor_get_max_position(slider_joint_upper_limit_motor), 0.2,
    "'maxPosition' of SliderJoint (for motor) didn't adjust correctly when position is above upper limit.");

  // HingeJoint Tests --------------------------------------------------------------------------------------------------
  WbDeviceTag hinge_joint_lower_limit_motor = wb_robot_get_device("hinge_joint_lower_limit_motor");

  // for motor: [minPosition, maxPosition] was [-0.1, 0.1] in world, should be [-0.2, 0.1] after loading
  ts_assert_double_equal(wb_motor_get_min_position(hinge_joint_lower_limit_motor), -0.2,
                         "'minPosition' of HingeJoint (for motor) didn't adjust correctly when position is below lower limit.");
  ts_assert_double_equal(wb_motor_get_max_position(hinge_joint_lower_limit_motor), 0.1,
                         "'maxPosition' of HingeJoint (for motor) changed but shouldn't, lower limit being tested.");

  WbDeviceTag hinge_joint_upper_limit_motor = wb_robot_get_device("hinge_joint_upper_limit_motor");

  // for motor: [minPosition, maxPosition] was [-0.1, 0.1] in world, should be [-0.1, 0.2] after loading
  ts_assert_double_equal(wb_motor_get_min_position(hinge_joint_upper_limit_motor), -0.1,
                         "'minPosition' of HingeJoint (for motor) changed but shouldn't, upper limit being tested.");
  ts_assert_double_equal(wb_motor_get_max_position(hinge_joint_upper_limit_motor), 0.2,
                         "'maxPosition' of HingeJoint (for motor) didn't adjust correctly when position is above upper limit.");

  // Hinge2Joint Tests -------------------------------------------------------------------------------------------------
  WbDeviceTag hinge_2_joint_lower_limit_motor = wb_robot_get_device("hinge_2_joint_lower_limit_motor");
  WbDeviceTag hinge_2_joint_lower_limit_motor2 = wb_robot_get_device("hinge_2_joint_lower_limit_motor2");

  // for motor: [minPosition, maxPosition] was [-0.1, 0.1] in world, should be [-0.2, 0.1] after loading
  ts_assert_double_equal(
    wb_motor_get_min_position(hinge_2_joint_lower_limit_motor), -0.2,
    "'minPosition' of Hinge2Joint (for motor) didn't adjust correctly when position is below lower limit.");
  ts_assert_double_equal(wb_motor_get_max_position(hinge_2_joint_lower_limit_motor), 0.1,
                         "'maxPosition' of Hinge2Joint (for motor) changed but shouldn't, lower limit being tested.");

  // for motor2: [minPosition, maxPosition] was [-0.1, 0.1] in world, should be [-0.3, 0.1] after loading
  ts_assert_double_equal(
    wb_motor_get_min_position(hinge_2_joint_lower_limit_motor2), -0.3,
    "'minPosition' of Hinge2Joint (for motor2) didn't adjust correctly when position is below lower limit.");
  ts_assert_double_equal(wb_motor_get_max_position(hinge_2_joint_lower_limit_motor2), 0.1,
                         "'maxPosition' of Hinge2Joint (for motor2) changed but shouldn't, lower limit being tested.");

  WbDeviceTag hinge_2_joint_upper_limit_motor = wb_robot_get_device("hinge_2_joint_upper_limit_motor");
  WbDeviceTag hinge_2_joint_upper_limit_motor2 = wb_robot_get_device("hinge_2_joint_upper_limit_motor2");

  // for motor: [minPosition, maxPosition] was [-0.1, 0.1] in world, should be [-0.1, 0.2] after loading
  ts_assert_double_equal(wb_motor_get_min_position(hinge_2_joint_upper_limit_motor), -0.1,
                         "'minPosition' of Hinge2Joint (for motor) changed but shouldn't, upper limit being tested.");
  ts_assert_double_equal(
    wb_motor_get_max_position(hinge_2_joint_upper_limit_motor), 0.2,
    "'maxPosition' of Hinge2Joint (for motor) didn't adjust correctly when position is above lower limit.");

  // for motor2: [minPosition, maxPosition] was [-0.1, 0.1] in world, should be [-0.1, 0.3] after loading
  ts_assert_double_equal(wb_motor_get_min_position(hinge_2_joint_upper_limit_motor2), -0.1,
                         "'minPosition' of Hinge2Joint (for motor2) changed but shouldn't, upper limit being tested.");
  ts_assert_double_equal(
    wb_motor_get_max_position(hinge_2_joint_upper_limit_motor2), 0.3,
    "'maxPosition' of Hinge2Joint (for motor2) didn't adjust correctly when position is above lower limit.");

  // BallJoint Tests ---------------------------------------------------------------------------------------------------
  WbDeviceTag ball_joint_lower_limit_motor = wb_robot_get_device("ball_joint_lower_limit_motor");
  WbDeviceTag ball_joint_lower_limit_motor2 = wb_robot_get_device("ball_joint_lower_limit_motor2");
  WbDeviceTag ball_joint_lower_limit_motor3 = wb_robot_get_device("ball_joint_lower_limit_motor3");

  // for motor: [minPosition, maxPosition] was [-0.1, 0.1] in world, should be [-0.2, 0.1] after loading
  ts_assert_double_equal(wb_motor_get_min_position(ball_joint_lower_limit_motor), -0.2,
                         "'minPosition' of BallJoint (for motor) didn't adjust correctly when position is below lower limit.");
  ts_assert_double_equal(wb_motor_get_max_position(ball_joint_lower_limit_motor), 0.1,
                         "'maxPosition' of BallJoint (for motor) changed but shouldn't, lower limit being tested.");

  // for motor2: [minPosition, maxPosition] was [-0.1, 0.1] in world, should be [-0.3, 0.1] after loading
  ts_assert_double_equal(wb_motor_get_min_position(ball_joint_lower_limit_motor2), -0.3,
                         "'minPosition' of BallJoint (for motor2) didn't adjust correctly when position is below lower limit.");
  ts_assert_double_equal(wb_motor_get_max_position(ball_joint_lower_limit_motor2), 0.1,
                         "'maxPosition' of BallJoint (for motor2) changed but shouldn't, lower limit being tested.");

  // for motor3: [minPosition, maxPosition] was [-0.1, 0.1] in world, should be [-0.4, 0.1] after loading
  ts_assert_double_equal(wb_motor_get_min_position(ball_joint_lower_limit_motor3), -0.4,
                         "'minPosition' of BallJoint (for motor3) didn't adjust correctly when position is below lower limit.");
  ts_assert_double_equal(wb_motor_get_max_position(ball_joint_lower_limit_motor3), 0.1,
                         "'maxPosition' of BallJoint (for motor3) changed but shouldn't, lower limit being tested.");

  WbDeviceTag ball_joint_upper_limit_motor = wb_robot_get_device("ball_joint_upper_limit_motor");
  WbDeviceTag ball_joint_upper_limit_motor2 = wb_robot_get_device("ball_joint_upper_limit_motor2");
  WbDeviceTag ball_joint_upper_limit_motor3 = wb_robot_get_device("ball_joint_upper_limit_motor3");

  // for motor: [minPosition, maxPosition] was [-0.1, 0.1] in world, should be [-0.1, 0.2] after loading
  ts_assert_double_equal(wb_motor_get_min_position(ball_joint_upper_limit_motor), -0.1,
                         "'minPosition' of BallJoint (for motor) changed but shouldn't, upper limit being tested.");
  ts_assert_double_equal(wb_motor_get_max_position(ball_joint_upper_limit_motor), 0.2,
                         "'maxPosition' of BallJoint (for motor) didn't adjust correctly when position is above upper limit.");

  // for motor2: [minPosition, maxPosition] was [-0.1, 0.1] in world, should be [-0.1, 0.3] after loading
  ts_assert_double_equal(wb_motor_get_min_position(ball_joint_upper_limit_motor2), -0.1,
                         "'minPosition' of BallJoint (for motor2) changed but shouldn't, upper limit being tested.");
  ts_assert_double_equal(wb_motor_get_max_position(ball_joint_upper_limit_motor2), 0.3,
                         "'maxPosition' of BallJoint (for motor2) didn't adjust correctly when position is above upper limit.");

  // for motor3: [minPosition, maxPosition] was [-0.1, 0.1] in world, should be [-0.1, 0.4] after loading
  ts_assert_double_equal(wb_motor_get_min_position(ball_joint_upper_limit_motor3), -0.1,
                         "'minPosition' of BallJoint (for motor3) changed but shouldn't, upper limit being tested.");
  ts_assert_double_equal(wb_motor_get_max_position(ball_joint_upper_limit_motor3), 0.4,
                         "'maxPosition' of BallJoint (for motor3) didn't adjust correctly when position is above upper limit.");

  ts_send_success();
  return EXIT_SUCCESS;
}
