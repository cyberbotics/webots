#include <stdio.h>
#include <webots/robot.h>
#include <webots/supervisor.h>
#include <webots/camera.h>

#define TIME_STEP 64

int main(int argc, char **argv) {
  wb_robot_init();

  WbDeviceTag cameras[3];
  char camera_name[10];

  for (int i = 0; i < 3; ++i) {
    sprintf(camera_name, "camera%d", i);
    cameras[i] = wb_robot_get_device(camera_name);
    wb_camera_enable(cameras[i], TIME_STEP);
  }
  
  wb_robot_step(TIME_STEP);

  const int width = wb_camera_get_width(cameras[0]);
  
  wb_robot_step(TIME_STEP);
  
  for (int i = 0; i < 3; ++i) {
    const unsigned char *image = wb_camera_get_image(cameras[i]);
    int r = wb_camera_image_get_red(image, width, 32, 32);
    int g = wb_camera_image_get_green(image, width, 32, 32);
    int b = wb_camera_image_get_blue(image, width, 32, 32);
    printf("%d %d %d\n", r, g, b);
  }
 
  wb_robot_step(TIME_STEP);
  wb_robot_step(TIME_STEP);

  wb_robot_cleanup();

  return 0;
}
