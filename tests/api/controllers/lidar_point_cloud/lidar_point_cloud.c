#include <webots/lidar.h>
#include <webots/robot.h>

#include "../../../lib/ts_assertion.h"
#include "../../../lib/ts_utils.h"

#include <math.h>
#include <string.h>

#define TIME_STEP 32

int main(int argc, char **argv) {
  ts_setup(argv[0]);

  WbDeviceTag lidar = wb_robot_get_device("lidar");

  const int resolution = wb_lidar_get_horizontal_resolution(lidar);
  const int number_of_layers = wb_lidar_get_number_of_layers(lidar);

  ts_assert_int_equal(resolution, 512, "The horizontal resolution of the lidar should be '512'.");
  ts_assert_int_equal(number_of_layers, 3, "The number of layers of the lidar should be '3'.");

  wb_lidar_enable(lidar, 2 * TIME_STEP);
  wb_lidar_enable_point_cloud(lidar);

  wb_robot_step(TIME_STEP);

  // verify that image and point cloud are not yet available
  const float *image_lidar = wb_lidar_get_range_image(lidar);
  ts_assert_pointer_not_null((void *)image_lidar, "Cannot retrieve lidar image pointer (at step 1).");
  const WbLidarPoint *point_cloud = wb_lidar_get_point_cloud(lidar);
  ts_assert_pointer_not_null((void *)point_cloud, "Cannot retrieve point cloud image pointer (at step 1).");
  int i = 0;
  for (i = 0; i < resolution * number_of_layers; ++i) {
    const float x = point_cloud[i].x;
    const float y = point_cloud[i].y;
    const float z = point_cloud[i].z;
    ts_assert_vec3_equal(x, y, z, 0.0, 0.0, 0.0, "Point cloud shouldn't be available one step after enabling the lidar.");
    ts_assert_double_equal(image_lidar[i], 0.0, "Range image shouldn't be available one step after enabling the lidar.");
  }

  wb_robot_step(TIME_STEP);

  // verify that image and point cloud are available
  image_lidar = wb_lidar_get_range_image(lidar);
  ts_assert_pointer_not_null((void *)image_lidar, "Cannot retrieve lidar image pointer (at step 2).");
  point_cloud = wb_lidar_get_point_cloud(lidar);
  ts_assert_pointer_not_null((void *)point_cloud, "Cannot retrieve point cloud image pointer (at step 2).");
  bool point_cloud_found = false;
  bool range_image_found = false;
  for (i = 0; i < resolution * number_of_layers; ++i) {
    float x = point_cloud[i].x;
    float y = point_cloud[i].y;
    float z = point_cloud[i].z;
    if (x != 0.0 || y != 0.0 || z != 0.0)
      point_cloud_found = true;
    if (image_lidar[i] != 0.0)
      range_image_found = true;
    if (point_cloud_found && range_image_found)
      break;
  }
  ts_assert_boolean_equal(point_cloud_found, "Point cloud should should be available two steps after enabling the lidar.");
  ts_assert_boolean_equal(range_image_found, "Range image should should be available two steps after enabling the lidar.");

  // verify that image and point cloud correspond
  image_lidar = wb_lidar_get_range_image(lidar);
  ts_assert_pointer_not_null((void *)image_lidar, "Cannot retrieve lidar image pointer (at step 3).");
  point_cloud = wb_lidar_get_point_cloud(lidar);
  ts_assert_pointer_not_null((void *)point_cloud, "Cannot retrieve point cloud image pointer (at step 3).");
  for (i = 0; i < resolution * number_of_layers; ++i) {
    const float x = point_cloud[i].x;
    const float y = point_cloud[i].y;
    const float z = point_cloud[i].z;
    if (!isinf(x) && !isinf(y) && !isinf(z)) {
      const float d = sqrt(x * x + y * y + z * z);
      ts_assert_double_in_delta(image_lidar[i], d, 0.05,
                                "Point cloud and range image does not correspond, Received value = %f, Expected value = %f",
                                image_lidar[i], d);
    }
  }

  // check layer getting
  const float *layer_image_lidar = wb_lidar_get_layer_range_image(lidar, 2);
  ts_assert_pointer_not_null((void *)layer_image_lidar, "Cannot retrieve lidar image pointer for layer 2.");
  const WbLidarPoint *layer_point_cloud = wb_lidar_get_layer_point_cloud(lidar, 2);
  ts_assert_pointer_not_null((void *)layer_point_cloud, "Cannot retrieve point cloud pointer for layer 2.");
  for (i = 0; i < resolution; ++i) {
    ts_assert_double_in_delta(layer_image_lidar[i], image_lidar[i + 2 * resolution], 0.001,
                              "Layer image is not equivalent to raw image.");
    ts_assert_double_in_delta(layer_point_cloud[i].x, point_cloud[i + 2 * resolution].x, 0.001,
                              "Layer point cloud is not equivalent to raw point cloud.");
  }

  // check layer 1 see the inclined plan farther than layer 0 is constant (because see only the ground)
  const float *layer0_image_lidar = wb_lidar_get_layer_range_image(lidar, 0);
  ts_assert_pointer_not_null((void *)layer0_image_lidar, "Cannot retrieve lidar image pointer for layer 0.");
  const float *layer1_image_lidar = wb_lidar_get_layer_range_image(lidar, 1);
  ts_assert_pointer_not_null((void *)layer1_image_lidar, "Cannot retrieve lidar image pointer for layer 1.");
  ts_assert_double_is_bigger(layer0_image_lidar[450], layer1_image_lidar[450],
                             "Lidar layer 0 should see the inclined plan farther than lidar 1.");

  ts_send_success();
  return EXIT_SUCCESS;
}
