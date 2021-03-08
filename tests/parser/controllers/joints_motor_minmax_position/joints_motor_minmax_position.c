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
  WbDeviceTag sj_motor_lower_lim = wb_robot_get_device("sj_lower_lim_device");

  // for device: [minPosition, maxPosition] was [-0.1, 0.1] in world, should be [-0.2, 0.1] after loading
  ts_assert_double_equal(wb_motor_get_min_position(sj_motor_lower_lim), -0.2,
                         "'minPosition' of SliderJoint (for device) not adjusted correctly when position below lower limit.");
  ts_assert_double_equal(wb_motor_get_max_position(sj_motor_lower_lim), 0.1,
                         "'maxPosition' of SliderJoint (for device) changed but shouldn't have (testing sj_motor_lower).");

  WbDeviceTag sj_motor_upper_lim = wb_robot_get_device("sj_upper_lim_device");

  // for device: [minPosition, maxPosition] was [-0.1, 0.1] in world, should be [-0.1, 0.2] after loading
  ts_assert_double_equal(wb_motor_get_min_position(sj_motor_upper_lim), -0.1,
                         "'minPosition' of SliderJoint (for device) not adjusted correctly when position beyond upper limit.");
  ts_assert_double_equal(wb_motor_get_max_position(sj_motor_upper_lim), 0.2,
                         "'maxPosition' of SliderJoint (for device) changed but shouldn't have (testing sj_motor_upper).");

  // HingeJoint Tests --------------------------------------------------------------------------------------------------
  WbDeviceTag hj_motor_lower_lim = wb_robot_get_device("hj_lower_lim_device");

  // for device: [minPosition, maxPosition] was [-0.1, 0.1] in world, should be [-0.2, 0.1] after loading
  ts_assert_double_equal(wb_motor_get_min_position(hj_motor_lower_lim), -0.2,
                         "'minPosition' of HingeJoint (for device) not adjusted correctly when position below lower limit.");
  ts_assert_double_equal(wb_motor_get_max_position(hj_motor_lower_lim), 0.1,
                         "'maxPosition' of HingeJoint (for device) changed but shouldn't have (testing hj_motor_lower).");

  WbDeviceTag hj_motor_upper_lim = wb_robot_get_device("hj_upper_lim_device");

  // for device: [minPosition, maxPosition] was [-0.1, 0.1] in world, should be [-0.1, 0.2] after loading
  ts_assert_double_equal(wb_motor_get_min_position(hj_motor_upper_lim), -0.1,
                         "'minPosition' of HingeJoint (for device) not adjusted correctly when position beyond upper limit.");
  ts_assert_double_equal(wb_motor_get_max_position(hj_motor_upper_lim), 0.2,
                         "'maxPosition' of HingeJoint (for device) changed but shouldn't have (testing hj_motor_upper).");

  // Hinge2Joint Tests -------------------------------------------------------------------------------------------------
  WbDeviceTag h2j_motor_lower_lim = wb_robot_get_device("h2j_lower_lim_device");
  WbDeviceTag h2j_motor_lower_lim2 = wb_robot_get_device("h2j_lower_lim_device2");

  // for device: [minPosition, maxPosition] was [-0.1, 0.1] in world, should be [-0.2, 0.1] after loading
  ts_assert_double_equal(wb_motor_get_min_position(h2j_motor_lower_lim), -0.2,
                         "'minPosition' of Hinge2Joint (for device) not adjusted correctly when position below lower limit.");
  ts_assert_double_equal(wb_motor_get_max_position(h2j_motor_lower_lim), 0.1,
                         "'maxPosition' of Hinge2Joint (for device) changed but shouldn't have (testing h2j_motor_lower).");

  // for device2: [minPosition, maxPosition] was [-0.1, 0.1] in world, should be [-0.3, 0.1] after loading
  ts_assert_double_equal(wb_motor_get_min_position(h2j_motor_lower_lim2), -0.3,
                         "'minPosition' of Hinge2Joint (for device2) not adjusted correctly when position below lower limit.");
  ts_assert_double_equal(wb_motor_get_max_position(h2j_motor_lower_lim2), 0.1,
                         "'maxPosition' of Hinge2Joint (for device2) changed but shouldn't have (testing h2j_motor_lower).");

  WbDeviceTag h2j_motor_upper_lim = wb_robot_get_device("h2j_upper_lim_device");
  WbDeviceTag h2j_motor_upper_lim2 = wb_robot_get_device("h2j_upper_lim_device2");

  // for device: [minPosition, maxPosition] was [-0.1, 0.1] in world, should be [-0.1, 0.2] after loading
  ts_assert_double_equal(wb_motor_get_min_position(h2j_motor_upper_lim), -0.1,
                         "'minPosition' of Hinge2Joint (for device) not adjusted correctly when position beyond upper limit.");
  ts_assert_double_equal(wb_motor_get_max_position(h2j_motor_upper_lim), 0.2,
                         "'maxPosition' of Hinge2Joint (for device) changed but shouldn't have (testing h2j_motor_upper).");

  // for device2: [minPosition, maxPosition] was [-0.1, 0.1] in world, should be [-0.1, 0.3] after loading
  ts_assert_double_equal(wb_motor_get_min_position(h2j_motor_upper_lim2), -0.1,
                         "'minPosition' of Hinge2Joint (for device2) not adjusted correctly when position beyond upper limit.");
  ts_assert_double_equal(wb_motor_get_max_position(h2j_motor_upper_lim2), 0.3,
                         "'maxPosition' of Hinge2Joint (for device2) changed but shouldn't have (testing h2j_motor_upper).");

  // BallJoint Tests ---------------------------------------------------------------------------------------------------
  WbDeviceTag bj_motor_lower_lim = wb_robot_get_device("bj_lower_lim_device");
  WbDeviceTag bj_motor_lower_lim2 = wb_robot_get_device("bj_lower_lim_device2");
  WbDeviceTag bj_motor_lower_lim3 = wb_robot_get_device("bj_lower_lim_device3");

  // for device: [minPosition, maxPosition] was [-0.1, 0.1] in world, should be [-0.2, 0.1] after loading
  ts_assert_double_equal(wb_motor_get_min_position(bj_motor_lower_lim), -0.2,
                         "'minPosition' of BallJoint (for device) not adjusted correctly when position below lower limit.");
  ts_assert_double_equal(wb_motor_get_max_position(bj_motor_lower_lim), 0.1,
                         "'maxPosition' of BallJoint (for device) changed but shouldn't have (testing bj_motor_lower).");

  // for device2: [minPosition, maxPosition] was [-0.1, 0.1] in world, should be [-0.3, 0.1] after loading
  ts_assert_double_equal(wb_motor_get_min_position(bj_motor_lower_lim2), -0.3,
                         "'minPosition' of BallJoint (for device2) not adjusted correctly when position below lower limit.");
  ts_assert_double_equal(wb_motor_get_max_position(bj_motor_lower_lim2), 0.1,
                         "'maxPosition' of BallJoint (for device2) changed but shouldn't have (testing bj_motor_lower).");

  // for device3: [minPosition, maxPosition] was [-0.1, 0.1] in world, should be [-0.4, 0.1] after loading
  ts_assert_double_equal(wb_motor_get_min_position(bj_motor_lower_lim3), -0.4,
                         "'minPosition' of BallJoint (for device3) not adjusted correctly when position below lower limit.");
  ts_assert_double_equal(wb_motor_get_max_position(bj_motor_lower_lim3), 0.1,
                         "'maxPosition' of BallJoint (for device3) but shouldn't have (testing bj_motor_lower).");

  WbDeviceTag bj_motor_upper_lim = wb_robot_get_device("bj_upper_lim_device");
  WbDeviceTag bj_motor_upper_lim2 = wb_robot_get_device("bj_upper_lim_device2");
  WbDeviceTag bj_motor_upper_lim3 = wb_robot_get_device("bj_upper_lim_device3");

  // for device: [minPosition, maxPosition] was [-0.1, 0.1] in world, should be [-0.1, 0.2] after loading
  ts_assert_double_equal(wb_motor_get_min_position(bj_motor_upper_lim), -0.1,
                         "'minPosition' of BallJoint (for device) not adjusted correctly when position beyond upper limit.");
  ts_assert_double_equal(wb_motor_get_max_position(bj_motor_upper_lim), 0.2,
                         "'maxPosition' of BallJoint (for device) changed but shouldn't have (testing bj_motor_upper).");

  // for device2: [minPosition, maxPosition] was [-0.1, 0.1] in world, should be [-0.1, 0.3] after loading
  ts_assert_double_equal(wb_motor_get_min_position(bj_motor_upper_lim2), -0.1,
                         "'minPosition' of BallJoint (for device2) not adjusted correctly when position beyond upper limit.");
  ts_assert_double_equal(wb_motor_get_max_position(bj_motor_upper_lim2), 0.3,
                         "'maxPosition' of BallJoint (for device2) changed but shouldn't have (testing bj_motor_upper).");

  // for device3: [minPosition, maxPosition] was [-0.1, 0.1] in world, should be [-0.1, 0.4] after loading
  ts_assert_double_equal(wb_motor_get_min_position(bj_motor_upper_lim3), -0.1,
                         "'minPosition' of BallJoint (for device3) not adjusted correctly when position beyond upper limit.");
  ts_assert_double_equal(wb_motor_get_max_position(bj_motor_upper_lim3), 0.4,
                         "'maxPosition' of BallJoint (for device3) changed but shouldn't have (testing bj_motor_upper).");
  ts_send_success();
  return EXIT_SUCCESS;
}
