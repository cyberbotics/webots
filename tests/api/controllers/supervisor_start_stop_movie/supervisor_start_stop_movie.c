#include <unistd.h>
#include <webots/supervisor.h>
#include "../../../lib/ts_assertion.h"
#include "../../../lib/ts_utils.h"

#define FILE_NAME "movie.mp4"

#define TIME_STEP 32

bool fileExists() {
  FILE *file = fopen(FILE_NAME, "r");
  if (file)
    return true;

  ts_send_error_and_exit("wb_supervisor_start/stop_movie() failed to create the " FILE_NAME " file in 5 seconds.");
  return false;
}

int main(int argc, char **argv) {
  ts_setup(argv[0]);

  remove(FILE_NAME);

  ts_assert_boolean_equal(wb_supervisor_movie_is_ready(), "Movie should be ready before starting recording a video.");

  ts_assert_boolean_not_equal(wb_supervisor_movie_failed(), "Movie shouldn't be failed before starting recording a video.");

  wb_supervisor_movie_start_recording(FILE_NAME, 640, 480, 0, 50, 1, false);

  ts_assert_boolean_not_equal(wb_supervisor_movie_is_ready(),
                              "Movie shouldn't be ready just after starting recording a video.");

  int i;
  for (i = 0; i < 20; i++)
    wb_robot_step(TIME_STEP);

  wb_supervisor_movie_stop_recording();

  ts_assert_boolean_not_equal(wb_supervisor_movie_is_ready(),
                              "Movie shouldn't be ready just after stopping recording a video.");

  for (i = 0; i < 50; i++) {
    usleep(100000);

    if (wb_supervisor_movie_is_ready()) {
      ts_assert_boolean_not_equal(wb_supervisor_movie_failed(), "Error while recording the movie file.");

      if (fileExists()) {
        ts_send_success();
        return EXIT_SUCCESS;
      }

      wb_robot_cleanup();
      return EXIT_FAILURE;
    }
  }

  if (fileExists())
    ts_send_error_and_exit("wb_supervisor_movie_is_ready doesn't return TRUE "
                           "after 5 seconds from stop command.");

  wb_robot_cleanup();
  return EXIT_FAILURE;
}
