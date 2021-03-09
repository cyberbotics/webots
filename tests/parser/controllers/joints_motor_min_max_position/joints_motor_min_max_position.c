#include <webots/motor.h>
#include <webots/robot.h>
#include <webots/supervisor.h>

#include "../../../lib/ts_assertion.h"
#include "../../../lib/ts_utils.h"

#define TIME_STEP 32

int main(int argc, char **argv) {
  ts_setup(argv[0]);
  ts_disable_output_log();
  // This test suite verifies that if the position field, as set in the jointParameters, is outside of the bounds of the
  // minPosition and maxPosition of RotationalMotor, these bounds must adadpt accordingly.
  // This suite also checks that the correct pairing is done (device <-> jointParameters, device2 <-> jointParameters2)

  // SliderJoint Tests -------------------------------------------------------------------------------------------------
  WbDeviceTag slider_joint_motor_lower_limit = wb_robot_get_device("slider_joint_lower_limit_motor");

  // for motor: [minPosition, maxPosition] was [-0.1, 0.1] in world, should be [-0.2, 0.1] after loading
  ts_assert_double_equal(wb_motor_get_min_position(slider_joint_motor_lower_limit), -0.2,
                         "'minPosition' of SliderJoint (for motor) not adjusted correctly when position below lower limit.");
  ts_assert_double_equal(
    wb_motor_get_max_position(slider_joint_motor_lower_limit), 0.1,
    "'maxPosition' of SliderJoint (for motor) changed but shouldn't have (testing slider_joint_motor_lower).");

  WbDeviceTag slider_joint_motor_upper_limit = wb_robot_get_device("slider_joint_upper_limit_motor");

  // for motor: [minPosition, maxPosition] was [-0.1, 0.1] in world, should be [-0.1, 0.2] after loading
  ts_assert_double_equal(wb_motor_get_min_position(slider_joint_motor_upper_limit), -0.1,
                         "'minPosition' of SliderJoint (for motor) not adjusted correctly when position beyond upper limit.");
  ts_assert_double_equal(
    wb_motor_get_max_position(slider_joint_motor_upper_limit), 0.2,
    "'maxPosition' of SliderJoint (for motor) changed but shouldn't have (testing slider_joint_motor_upper).");

  // HingeJoint Tests --------------------------------------------------------------------------------------------------
  WbDeviceTag hinge_joint_motor_lower_limit = wb_robot_get_device("hinge_joint_lower_limit_motor");

  // for motor: [minPosition, maxPosition] was [-0.1, 0.1] in world, should be [-0.2, 0.1] after loading
  ts_assert_double_equal(wb_motor_get_min_position(hinge_joint_motor_lower_limit), -0.2,
                         "'minPosition' of HingeJoint (for motor) not adjusted correctly when position below lower limit.");
  ts_assert_double_equal(
    wb_motor_get_max_position(hinge_joint_motor_lower_limit), 0.1,
    "'maxPosition' of HingeJoint (for motor) changed but shouldn't have (testing hinge_joint_motor_lower).");

  WbDeviceTag hinge_joint_motor_upper_limit = wb_robot_get_device("hinge_joint_upper_limit_motor");

  // for motor: [minPosition, maxPosition] was [-0.1, 0.1] in world, should be [-0.1, 0.2] after loading
  ts_assert_double_equal(wb_motor_get_min_position(hinge_joint_motor_upper_limit), -0.1,
                         "'minPosition' of HingeJoint (for motor) not adjusted correctly when position beyond upper limit.");
  ts_assert_double_equal(
    wb_motor_get_max_position(hinge_joint_motor_upper_limit), 0.2,
    "'maxPosition' of HingeJoint (for motor) changed but shouldn't have (testing hinge_joint_motor_upper).");

  // Hinge2Joint Tests -------------------------------------------------------------------------------------------------
  WbDeviceTag hinge_2_joint_motor_lower_limit = wb_robot_get_device("hinge_2_joint_lower_limit_motor");
  WbDeviceTag hinge_2_joint_motor_lower_limit2 = wb_robot_get_device("hinge_2_joint_lower_limit_motor2");

  // for motor: [minPosition, maxPosition] was [-0.1, 0.1] in world, should be [-0.2, 0.1] after loading
  ts_assert_double_equal(wb_motor_get_min_position(hinge_2_joint_motor_lower_limit), -0.2,
                         "'minPosition' of Hinge2Joint (for motor) not adjusted correctly when position below lower limit.");
  ts_assert_double_equal(
    wb_motor_get_max_position(hinge_2_joint_motor_lower_limit), 0.1,
    "'maxPosition' of Hinge2Joint (for motor) changed but shouldn't have (testing hinge_2_joint_motor_lower).");

  // for motor2: [minPosition, maxPosition] was [-0.1, 0.1] in world, should be [-0.3, 0.1] after loading
  ts_assert_double_equal(wb_motor_get_min_position(hinge_2_joint_motor_lower_limit2), -0.3,
                         "'minPosition' of Hinge2Joint (for motor2) not adjusted correctly when position below lower limit.");
  ts_assert_double_equal(
    wb_motor_get_max_position(hinge_2_joint_motor_lower_limit2), 0.1,
    "'maxPosition' of Hinge2Joint (for motor2) changed but shouldn't have (testing hinge_2_joint_motor_lower).");

  WbDeviceTag hinge_2_joint_motor_upper_limit = wb_robot_get_device("hinge_2_joint_upper_limit_motor");
  WbDeviceTag hinge_2_joint_motor_upper_limit2 = wb_robot_get_device("hinge_2_joint_upper_limit_motor2");

  // for motor: [minPosition, maxPosition] was [-0.1, 0.1] in world, should be [-0.1, 0.2] after loading
  ts_assert_double_equal(wb_motor_get_min_position(hinge_2_joint_motor_upper_limit), -0.1,
                         "'minPosition' of Hinge2Joint (for motor) not adjusted correctly when position beyond upper limit.");
  ts_assert_double_equal(
    wb_motor_get_max_position(hinge_2_joint_motor_upper_limit), 0.2,
    "'maxPosition' of Hinge2Joint (for motor) changed but shouldn't have (testing hinge_2_joint_motor_upper).");

  // for motor2: [minPosition, maxPosition] was [-0.1, 0.1] in world, should be [-0.1, 0.3] after loading
  ts_assert_double_equal(wb_motor_get_min_position(hinge_2_joint_motor_upper_limit2), -0.1,
                         "'minPosition' of Hinge2Joint (for motor2) not adjusted correctly when position beyond upper limit.");
  ts_assert_double_equal(
    wb_motor_get_max_position(hinge_2_joint_motor_upper_limit2), 0.3,
    "'maxPosition' of Hinge2Joint (for motor2) changed but shouldn't have (testing hinge_2_joint_motor_upper).");

  // BallJoint Tests ---------------------------------------------------------------------------------------------------
  WbDeviceTag ball_joint_motor_lower_limit = wb_robot_get_device("ball_joint_lower_limit_motor");
  WbDeviceTag ball_joint_motor_lower_limit2 = wb_robot_get_device("ball_joint_lower_limit_motor2");
  WbDeviceTag ball_joint_motor_lower_limit3 = wb_robot_get_device("ball_joint_lower_limit_motor3");

  // for motor: [minPosition, maxPosition] was [-0.1, 0.1] in world, should be [-0.2, 0.1] after loading
  ts_assert_double_equal(wb_motor_get_min_position(ball_joint_motor_lower_limit), -0.2,
                         "'minPosition' of BallJoint (for motor) not adjusted correctly when position below lower limit.");
  ts_assert_double_equal(wb_motor_get_max_position(ball_joint_motor_lower_limit), 0.1,
                         "'maxPosition' of BallJoint (for motor) changed but shouldn't have (testing ball_joint_motor_lower).");

  // for motor2: [minPosition, maxPosition] was [-0.1, 0.1] in world, should be [-0.3, 0.1] after loading
  ts_assert_double_equal(wb_motor_get_min_position(ball_joint_motor_lower_limit2), -0.3,
                         "'minPosition' of BallJoint (for motor2) not adjusted correctly when position below lower limit.");
  ts_assert_double_equal(
    wb_motor_get_max_position(ball_joint_motor_lower_limit2), 0.1,
    "'maxPosition' of BallJoint (for motor2) changed but shouldn't have (testing ball_joint_motor_lower).");

  // for motor3: [minPosition, maxPosition] was [-0.1, 0.1] in world, should be [-0.4, 0.1] after loading
  ts_assert_double_equal(wb_motor_get_min_position(ball_joint_motor_lower_limit3), -0.4,
                         "'minPosition' of BallJoint (for motor3) not adjusted correctly when position below lower limit.");
  ts_assert_double_equal(wb_motor_get_max_position(ball_joint_motor_lower_limit3), 0.1,
                         "'maxPosition' of BallJoint (for motor3) but shouldn't have (testing ball_joint_motor_lower).");

  WbDeviceTag ball_joint_motor_upper_limit = wb_robot_get_device("ball_joint_upper_limit_motor");
  WbDeviceTag ball_joint_motor_upper_limit2 = wb_robot_get_device("ball_joint_upper_limit_motor2");
  WbDeviceTag ball_joint_motor_upper_limit3 = wb_robot_get_device("ball_joint_upper_limit_motor3");

  // for motor: [minPosition, maxPosition] was [-0.1, 0.1] in world, should be [-0.1, 0.2] after loading
  ts_assert_double_equal(wb_motor_get_min_position(ball_joint_motor_upper_limit), -0.1,
                         "'minPosition' of BallJoint (for motor) not adjusted correctly when position beyond upper limit.");
  ts_assert_double_equal(wb_motor_get_max_position(ball_joint_motor_upper_limit), 0.2,
                         "'maxPosition' of BallJoint (for motor) changed but shouldn't have (testing ball_joint_motor_upper).");

  // for motor2: [minPosition, maxPosition] was [-0.1, 0.1] in world, should be [-0.1, 0.3] after loading
  ts_assert_double_equal(wb_motor_get_min_position(ball_joint_motor_upper_limit2), -0.1,
                         "'minPosition' of BallJoint (for motor2) not adjusted correctly when position beyond upper limit.");
  ts_assert_double_equal(
    wb_motor_get_max_position(ball_joint_motor_upper_limit2), 0.3,
    "'maxPosition' of BallJoint (for motor2) changed but shouldn't have (testing ball_joint_motor_upper).");

  // for motor3: [minPosition, maxPosition] was [-0.1, 0.1] in world, should be [-0.1, 0.4] after loading
  ts_assert_double_equal(wb_motor_get_min_position(ball_joint_motor_upper_limit3), -0.1,
                         "'minPosition' of BallJoint (for motor3) not adjusted correctly when position beyond upper limit.");
  ts_assert_double_equal(
    wb_motor_get_max_position(ball_joint_motor_upper_limit3), 0.4,
    "'maxPosition' of BallJoint (for motor3) changed but shouldn't have (testing ball_joint_motor_upper).");

  ts_send_success();
  return EXIT_SUCCESS;
}
