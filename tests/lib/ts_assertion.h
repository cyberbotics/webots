#ifndef TS_ASSERTION_H
#define TS_ASSERTION_H

#include <math.h>
#include <stdarg.h>
#include <string.h>
#include <webots/robot.h>
#include "ts_utils.h"

#define TS_FINAL_CHECK()                             \
  do {                                               \
    if (!correct) {                                  \
      va_list args;                                  \
      va_start(args, error_message);                 \
      ts_v_send_error_and_exit(error_message, args); \
      va_end(args);                                  \
    }                                                \
  } while (0)

#define TS_DOUBLE_IN_DELTA(value, expected, delta) \
  ((isnan(value) && isnan(expected)) || (isinf(value) && isinf(expected)) || (fabs((value) - (expected)) <= (delta)))

void ts_assert_boolean_equal(bool value, const char *error_message, ...) {
  bool correct = value;
  TS_FINAL_CHECK();
}

void ts_assert_boolean_not_equal(bool value, const char *error_message, ...) {
  bool correct = !value;
  TS_FINAL_CHECK();
}

void ts_assert_pointer_null(void *ptr, const char *error_message, ...) {
  bool correct = ptr == NULL;
  TS_FINAL_CHECK();
}

void ts_assert_pointer_not_null(void *ptr, const char *error_message, ...) {
  bool correct = ptr != NULL;
  TS_FINAL_CHECK();
}

void ts_assert_int_equal(int value, int expected, const char *error_message, ...) {
  bool correct = (value == expected);
  TS_FINAL_CHECK();
}

void ts_assert_int_not_equal(int value, int not_expected, const char *error_message, ...) {
  bool correct = (value != not_expected);
  TS_FINAL_CHECK();
}

void ts_assert_int_in_delta(int value, int expected, int delta, const char *error_message, ...) {
  bool correct = abs(value - expected) <= delta;
  TS_FINAL_CHECK();
}

void ts_assert_integers_in_delta(int size, const int *value, const int *expected, int delta, const char *error_message, ...) {
  int i;
  bool correct = true;
  for (i = 0; i < size; i++) {
    if (abs(value[i] - expected[i]) > delta) {
      correct = false;
      break;
    }
  }
  TS_FINAL_CHECK();
}

void ts_assert_int_is_bigger(int value, int compared_value, const char *error_message, ...) {
  bool correct = (value > compared_value);
  TS_FINAL_CHECK();
}

void ts_assert_double_equal(double value, double expected, const char *error_message, ...) {
  bool correct = TS_DOUBLE_IN_DELTA(value, expected, 0.0);
  TS_FINAL_CHECK();
}

void ts_assert_double_not_equal(double value, double not_expected, const char *error_message, ...) {
  bool correct = !TS_DOUBLE_IN_DELTA(value, not_expected, 0.0);
  TS_FINAL_CHECK();
}

void ts_assert_double_in_delta(double value, double expected, double delta, const char *error_message, ...) {
  bool correct = TS_DOUBLE_IN_DELTA(value, expected, delta);
  TS_FINAL_CHECK();
}

void ts_assert_double_is_bigger(double value, double compared_value, const char *error_message, ...) {
  bool correct = (value > compared_value);
  TS_FINAL_CHECK();
}

void ts_assert_vec3_equal(double v0, double v1, double v2, double e0, double e1, double e2, const char *error_message, ...) {
  bool correct = TS_DOUBLE_IN_DELTA(v0, e0, 0.0) && TS_DOUBLE_IN_DELTA(v1, e1, 0.0) && TS_DOUBLE_IN_DELTA(v2, e2, 0.0);
  TS_FINAL_CHECK();
}

void ts_assert_vec3_in_delta(double v0, double v1, double v2, double e0, double e1, double e2, double delta,
                             const char *error_message, ...) {
  bool correct = TS_DOUBLE_IN_DELTA(v0, e0, delta) && TS_DOUBLE_IN_DELTA(v1, e1, delta) && TS_DOUBLE_IN_DELTA(v2, e2, delta);
  TS_FINAL_CHECK();
}

void ts_assert_vec3_not_in_delta(double v0, double v1, double v2, double e0, double e1, double e2, double delta,
                                 const char *error_message, ...) {
  bool correct = !TS_DOUBLE_IN_DELTA(v0, e0, delta) || !TS_DOUBLE_IN_DELTA(v1, e1, delta) || !TS_DOUBLE_IN_DELTA(v2, e2, delta);
  TS_FINAL_CHECK();
}

void ts_assert_doubles_equal(int size, const double *value, const double *expected, const char *error_message, ...) {
  int i;
  bool correct = true;
  for (i = 0; i < size; i++) {
    if (!TS_DOUBLE_IN_DELTA(value[i], expected[i], 0.0)) {
      correct = false;
      break;
    }
  }
  TS_FINAL_CHECK();
}

// An euler and axis rotation equals to another.
void ts_assert_rotation_equals(const double *rot_a, const double *rot_b, double tolerance, const char *error_message, ...) {
  // Inspired from WbRotation::normalizeAngle()
  double angle_a = rot_a[3];
  while (angle_a < -M_PI)
    angle_a += 2.0 * M_PI;
  while (angle_a > M_PI)
    angle_a -= 2.0 * M_PI;
  double angle_b = rot_b[3];
  while (angle_b < -M_PI)
    angle_b += 2.0 * M_PI;
  while (angle_b > M_PI)
    angle_b -= 2.0 * M_PI;

  // Inspired from WbRotation::almostEquals()
  bool correct = false;
  if (fabs(angle_a - angle_b) < tolerance) {
    if (fabs(angle_a) < tolerance)
      correct = true;  // axis can be different but rotation is the same
    else
      correct =
        fabs(rot_a[0] - rot_b[0]) < tolerance && fabs(rot_a[1] - rot_b[1]) < tolerance && fabs(rot_a[2] - rot_b[2]) < tolerance;
  } else if (fabs(angle_a + angle_b) < tolerance) {
    if (fabs(angle_a) < tolerance)
      correct = true;  // axis can be different but rotation is the same
    else
      correct =
        fabs(rot_a[0] + rot_b[0]) < tolerance && fabs(rot_a[1] + rot_b[1]) < tolerance && fabs(rot_a[2] + rot_b[2]) < tolerance;
  }
  TS_FINAL_CHECK();
}

void ts_assert_doubles_in_delta(int size, const double *value, const double *expected, double delta, const char *error_message,
                                ...) {
  int i;
  bool correct = true;
  for (i = 0; i < size; i++) {
    if (!TS_DOUBLE_IN_DELTA(value[i], expected[i], delta)) {
      correct = false;
      break;
    }
  }
  TS_FINAL_CHECK();
}

void ts_assert_doubles_out_delta(int size, const double *value, const double *expected, double delta, const char *error_message,
                                 ...) {
  int i;
  bool correct = true;
  for (i = 0; i < size; i++) {
    if (!TS_DOUBLE_IN_DELTA(value[i], expected[i], delta)) {
      correct = false;
      break;
    }
  }
  correct = !correct;
  TS_FINAL_CHECK();
}

void ts_assert_string_equal(const char *value, const char *expected, const char *error_message, ...) {
  bool correct = false;
  // if string is NULL and the other isn't, false
  if ((value == NULL) != (expected == NULL))
    correct = false;
  // if they're both NULL, this still counts as a match
  else if (value == NULL && expected == NULL)
    correct = true;
  // otherwise compare normally
  else
    correct = (strcmp(value, expected) == 0);

  TS_FINAL_CHECK();
}

void ts_assert_string_contains(const char *haystack, const char *needle, const char *error_message, ...) {
  bool correct = false;
  // if string is NULL and the other isn't, false
  if ((haystack == NULL) != (needle == NULL))
    correct = false;
  // if they're both NULL, this still counts as a match
  else if (haystack == NULL && needle == NULL)
    correct = true;
  // the needle string has to be shorter than or equal to the haystack string
  else if (strlen(haystack) < strlen(needle))
    correct = false;
  // otherwise compare normally
  else
    correct = strstr(haystack, needle) != NULL;

  TS_FINAL_CHECK();
}

void ts_assert_color_in_delta(int red, int green, int blue, int expected_red, int expected_green, int expected_blue, int delta,
                              const char *error_message, ...) {
  bool correct = abs(red - expected_red) <= delta && abs(green - expected_green) <= delta && abs(blue - expected_blue) <= delta;
  TS_FINAL_CHECK();
}

#endif
