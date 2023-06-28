#include <unistd.h>
#include <webots/supervisor.h>
#include "../../../lib/ts_assertion.h"
#include "../../../lib/ts_utils.h"

#include <time.h>

bool file_exists(const char *filename) {
  FILE *file = fopen(filename, "r");
  if (file) {
    fclose(file);
    return true;
  }
  return false;
}

int main(int argc, char **argv) {
  int i;

  ts_setup(argv[0]);

  WbNodeRef node = wb_supervisor_node_get_from_def("VIEWPOINT");
  ts_assert_pointer_not_null(node, "wb_supervisor_node_get_from_def(\"VIEWPOINT\") failed");
  WbFieldRef position = wb_supervisor_node_get_field(node, "position");
  ts_assert_pointer_not_null(node, "wb_supervisor_node_get_field(\"position\") failed");
  WbFieldRef orientation = wb_supervisor_node_get_field(node, "orientation");
  ts_assert_pointer_not_null(node, "wb_supervisor_node_get_field(\"orientation\") failed");

  const int time_step = wb_robot_get_basic_time_step();

  remove("image0.png");
  remove("image1.png");
  remove("image2.png");

  ts_assert_boolean_not_equal(file_exists("image0.png"), "Impossible to remove a PNG file");
  ts_assert_boolean_not_equal(file_exists("image1.png"), "Impossible to remove a PNG file");
  ts_assert_boolean_not_equal(file_exists("image2.png"), "Impossible to remove a PNG file");

  wb_robot_step(time_step);

  wb_supervisor_export_image("image0.png", 100);

  const double NEW_POSITION[3] = {5, -3, 0.4};
  wb_supervisor_field_set_sf_vec3f(position, NEW_POSITION);

  // this is necessary to get the absolute position of the node updated by Webots
  wb_robot_step(time_step);

  wb_supervisor_export_image("image1.png", 100);
  const double NEW_ROTATION_FIELD_VALUE[4] = {0.5, 0.5, 1.0, M_PI / 3.0};
  wb_supervisor_field_set_sf_rotation(orientation, NEW_ROTATION_FIELD_VALUE);

  wb_robot_step(time_step);

  wb_supervisor_export_image("image2.png", 100);

  // We need to allow some time to check the existence of image files
  // as the test sometimes fails on Windows, probably because the system
  // needs some time to make the file available to other applications
  // after it was just created.
  wb_robot_step(time_step);

  time_t start_time = time(NULL);
  while (time(NULL) - start_time < 5) {
    if (file_exists("image0.png") && file_exists("image1.png") && file_exists("image2.png"))
      break;
    usleep(100000);
    wb_robot_step(time_step);
  }

  ts_assert_boolean_equal(file_exists("image0.png"), "wb_supervisor_export_image() failed to create the "
                                                     "image0.png"
                                                     " file.");
  ts_assert_boolean_equal(file_exists("image1.png"), "wb_supervisor_export_image() failed to create the "
                                                     "image1.png"
                                                     " file.");
  ts_assert_boolean_equal(file_exists("image2.png"), "wb_supervisor_export_image() failed to create the "
                                                     "image2.png"
                                                     " file.");

  FILE *file = fopen("image0.png", "r");
  fseek(file, 0L, SEEK_END);
  int size = ftell(file);
  fclose(file);
  file = fopen("image1.png", "r");
  fseek(file, 0L, SEEK_END);
  int mismatch = ftell(file) - size;
  fclose(file);
  ts_assert_int_equal(mismatch, 0, "File sizes differ between image0 and image1");
  file = fopen("image2.png", "r");
  fseek(file, 0L, SEEK_END);
  mismatch = ftell(file) - size;
  fclose(file);
  ts_assert_int_equal(mismatch, 0, "File sizes differ between image0 and image2");

  char *buffer[3];
  FILE *f[3];
  char *filenames[3];
  filenames[0] = "image0.png";
  filenames[1] = "image1.png";
  filenames[2] = "image2.png";

  for (i = 0; i < 3; i++) {
    buffer[i] = malloc(size);
    f[i] = fopen(filenames[i], "rb");
    ts_assert_int_equal(fread(buffer[i], sizeof(char), size, f[i]), size, "Cannot read image%d file.", i);
  }

  int difference = 0;
  for (i = 0; i < size; i++) {
    if (buffer[0][i] != buffer[1][i] || buffer[0][i] != buffer[2][i]) {
      difference = i;
      break;
    }
  }

  for (i = 0; i < 3; i++) {
    free(buffer[i]);
    fclose(f[i]);
  }

  ts_assert_int_equal(difference, 0, "Files differ at index %d", difference);

  unlink("image0.png");
  unlink("image1.png");
  unlink("image2.png");

  ts_send_success();
  return EXIT_SUCCESS;
}
