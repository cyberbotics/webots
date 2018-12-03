#include <webots/robot.h>

#include "../../../lib/ts_assertion.h"
#include "../../../lib/ts_utils.h"

#define TIME_STEP 32

int main(int argc, char **argv) {
  ts_setup(argv[0]);

  wb_robot_step(TIME_STEP);

  if (getenv("TEST_RELATIVE_PATHS") != NULL)
#ifdef _WIN32
    ts_assert_string_equal(getenv("TEST_RELATIVE_PATHS"), "test1\\test2;test3",
                           "The environment variable TEST_RELATIVE_PATHS was not set to the right value");
#else
    ts_assert_string_equal(getenv("TEST_RELATIVE_PATHS"), "test1/test2:test3",
                           "The environment variable TEST_RELATIVE_PATHS was not set to the right value");
#endif
  else
    ts_assert_pointer_not_null(getenv("TEST_RELATIVE_PATHS"), "TEST_RELATIVE_PATHS was not set");

  if (getenv("TEST") != NULL)
    ts_assert_string_equal(getenv("TEST"), "test1/test2:test3", "The environment variable TEST was not set to the right value");
  else
    ts_assert_pointer_not_null(getenv("TEST"), "TEST was not set");

  ts_assert_pointer_null(getenv("TEST_2"), "The environment variable TEST_2 was wrongly set");

  if (getenv("TEST_REFERENCE") != NULL)
    ts_assert_string_equal(getenv("TEST_REFERENCE"), getenv("WEBOTS_HOME"),
                           "The environment variable TEST_REFERENCE was not set to the right value");
  else
    ts_assert_pointer_not_null(getenv("TEST_REFERENCE"), "TEST_REFERENCE was not set");

#ifdef _WIN32
  if (getenv("TEST_WINDOWS") != NULL)
    ts_assert_string_equal(getenv("TEST_WINDOWS"), "test1\\test2;test3;C:\\Program Files;My Documents\\Images",
                           "The environment variable TEST_WINDOWS was not set to the right value");
  else
    ts_assert_pointer_not_null(getenv("TEST_WINDOWS"), "TEST_WINDOWS was not set");
#elif defined(__APPLE__)
  if (getenv("TEST_MAC_OS_X") != NULL)
    ts_assert_string_equal(getenv("TEST_MAC_OS_X"), "test1/test2:test3:test3",
                           "The environment variable TEST_MAC_OS_X was not set to the right value");
  else
    ts_assert_pointer_not_null(getenv("TEST_MAC_OS_X"), "TEST_MAC_OS_X was not set");
#elif defined(__linux__)
  if (getenv("TEST_LINUX") != NULL) {
    ts_assert_string_equal(getenv("TEST_LINUX"), "test1/test2:test3:test3",
                           "The environment variable TEST_LINUX was not set to the right value");
    if (ts_is_pointer_size_64_bits()) {
      if (getenv("TEST_LINUX_64") != NULL)
        ts_assert_string_equal(getenv("TEST_LINUX_64"), "test1/test2:test64",
                               "The environment variable TEST_LINUX_64 was not set to the right value");
      else
        ts_assert_pointer_not_null(getenv("TEST_LINUX_64"), "TEST_LINUX_64 was not set");
    } else {
      if (getenv("TEST_LINUX_32") != NULL)
        ts_assert_string_equal(getenv("TEST_LINUX_32"), "test1/test2:test32",
                               "The environment variable TEST_LINUX_32 was not set to the right value");
      else
        ts_assert_pointer_not_null(getenv("TEST_LINUX32"), "TEST_LINUX_32 was not set");
    }
  } else
    ts_assert_pointer_not_null(getenv("TEST_LINUX"), "TEST_LINUX was not set");
#else
  ts_assert_bool_equal(false, "No correct platform (either Windows, macOS or Linux) was defined");
#endif

  ts_send_success();
  return EXIT_SUCCESS;
}
