#include <webots/device.h>
#include <webots/gps.h>
#include <webots/robot.h>

#include "../../../lib/ts_assertion.h"
#include "../../../lib/ts_utils.h"

#define TIME_STEP 64

int main(int argc, char **argv) {
  int deviceType = -1;
  const char *deviceName;
  const char *deviceModel;
  const double *values;
  const double EXPECTED_POSITION[3] = {0.2, 0.0, 0.2};

  ts_setup(argv[0]);

  WbDeviceTag forthDevice = wb_robot_get_device_by_index(3);

  deviceType = wb_device_get_node_type(forthDevice);
  ts_assert_int_equal(deviceType, WB_NODE_GPS, "Device type of forth device should be '%d' not '%d'.", WB_NODE_GPS, deviceType);

  deviceName = wb_device_get_name(forthDevice);
  ts_assert_string_equal(deviceName, "test_gps", "Device name of forth device should be '%s' not '%s'.", "test_gps",
                         deviceName);

  deviceModel = wb_device_get_model(forthDevice);
  ts_assert_string_equal(deviceModel, "gps model", "Device model of forth device should be '%s' not '%s'.", "gps model",
                         deviceModel);

  wb_gps_enable(forthDevice, TIME_STEP);

  wb_robot_step(TIME_STEP);

  values = wb_gps_get_values(forthDevice);

  ts_assert_doubles_in_delta(3, values, EXPECTED_POSITION, 0.0001, "Device 'test_gps' didn't return the expected value.");

  WbDeviceTag gps = wb_robot_get_device("test_gps");
  ts_assert_boolean_equal(forthDevice == gps,
                          "wb_robot_get_device_by_index doesn't return the same WbDeviceTag as wb_robot_get_device.");

  ts_send_success();
  return EXIT_SUCCESS;
}
