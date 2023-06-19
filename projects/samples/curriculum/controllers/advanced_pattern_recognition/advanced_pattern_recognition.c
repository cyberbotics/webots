/*
 * Copyright 1996-2023 Cyberbotics Ltd.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     https://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/**********************************************
 *
 * Controller of the curriculum exercise:
 * about pattern recognition
 *
 * The e-puck can learn a specific
 * pattern
 *
 **********************************************/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <webots/camera.h>
#include <webots/display.h>
#include <webots/distance_sensor.h>
#include <webots/keyboard.h>
#include <webots/led.h>
#include <webots/motor.h>
#include <webots/robot.h>

#include "ann.h"  // definition of the used ANN

// general defines
#define TRUE 1
#define FALSE 0
#define LEFT 0
#define RIGHT 1
#define TIME_STEP 64
#define SPEED_UNIT 0.00628
// states (for moving the robot)
#define BED 0
#define ROTATION 1
#define RANDOM_WALK 2
#define DANSE 3
// modes (mode of the ANN algorithm)
#define NO_MODE 0
#define LEARN 1
#define TEST 2
// patterns (a landmark)
#define NUMBER_OF_PATTERNS 5
#define NO_PATTERN 0
#define P1 1
#define P2 2
#define P3 3
#define P4 4
// samples (an aera of the camera image which correspond to a landmark)
#define SAMPLE_MAX_NUMBER 5  // how many samples the e-puck can see at the same time
// devices
#define NB_LEDS 8
#define NB_DIST_SENS 8
#define PS_RIGHT_10 0
#define PS_RIGHT_45 1
#define PS_RIGHT_90 2
#define PS_RIGHT_REAR 3
#define PS_LEFT_REAR 4
#define PS_LEFT_90 5
#define PS_LEFT_45 6
#define PS_LEFT_10 7

static int state = BED;           // current state of the robot
static int mode = NO_MODE;        // current mode of the ANN algorithm
static int current_pattern = P1;  // current pattern for learning
static float samples[SAMPLE_MAX_NUMBER][SAMPLE_WIDTH * SAMPLE_HEIGHT];
static int sample_counter = 0;
static WbDeviceTag led[NB_LEDS];
static WbDeviceTag ps[NB_DIST_SENS];
static WbDeviceTag cam;
static WbDeviceTag display;
static WbDeviceTag left_motor, right_motor;
static int speed[2] = {0, 0};
static int delta = 0;           // delta between the two wheels
static int internal_state = 0;  // used for the random move and the danse
static int old_key = -1;        // last pressed key stroke
static float error = 1.0;

/*
 * This function displays the usage information
 */
static void usage() {
  printf("Commands (from keyboard):\n");
  printf("\n");
  printf("  1          : Select the pattern 1 for learning\n");
  printf("  2          : Select the pattern 2 for learning\n");
  printf("  3          : Select the pattern 3 for learning\n");
  printf("  4          : Select the pattern 4 for learning\n");
  printf("\n");
  printf("  Up arrow   : Select the next pattern for learning\n");
  printf("  Down arrow : Select the previous pattern for learning\n");
  printf("\n");
  printf("  L          : Enable (or disable) the learning mode\n");
  printf("  T          : Enable (or disable) the testing mode\n");
  printf("\n");
  printf("  O          : Load the weights stored in the weights.w file\n");
  printf("  S          : Save the weights stored in the weights.w file\n");
  printf("\n");
  printf("  B          : Stop the motors of the e-puck\n");
  printf("  R          : Rotate the e-puck\n");
  printf("  W          : Random walk of the e-puck\n");
  printf("  D          : the e-puck danse\n");
}

/*
 * This function rounds a float
 * to an int
 */
static int my_round(float x) {
  if (x >= 0)
    return (int)(x + 0.5);
  return (int)(x - 0.5);
}

/*
 * This function manage the last pressed
 * keyboard stroke.
 */
static void keyboard_manager() {
  int key = wb_keyboard_get_key();
  if (key != old_key) {  // for not call this function every TIME_STEP
    old_key = key;
    switch (key) {
      case '1':
      case '2':
      case '3':
      case '4':
        current_pattern = key - '0';
        break;
      case 'B':
        state = BED;
        printf("State of the robot: Bed\n");
        break;
      case 'R':
        state = ROTATION;
        printf("State of the robot: Rotation\n");
        break;
      case 'W':
        state = RANDOM_WALK;
        delta = 0;
        internal_state = 0;
        printf("State of the robot: Random walk\n");
        break;
      case 'D':
        state = DANSE;
        delta = 0;
        internal_state = 0;
        printf("State of the robot: Danse\n");
        break;
      case 'L':
        if (mode == LEARN) {
          mode = NO_MODE;
          printf("Mode of the algorithm: No mode\n");
        } else {
          mode = LEARN;
          printf("Mode of the algorithm: Learn\n");
        }
        break;
      case 'T':
        if (mode == TEST) {
          mode = NO_MODE;
          printf("Mode of the algorithm: No mode\n");
        } else {
          mode = TEST;
          printf("Mode of the algorithm: Test\n");
        }
        break;
      case WB_KEYBOARD_UP:
        current_pattern++;
        current_pattern %= NUMBER_OF_PATTERNS;
        break;
      case WB_KEYBOARD_DOWN:
        current_pattern--;
        if (current_pattern < 0)
          current_pattern = NUMBER_OF_PATTERNS - 1;
        break;
      case 'S':
        SaveNetworkWeights(&network, "weights.w");
        printf("Weights of the ANN saved in weights.w\n");
        break;
      case 'O':
        LoadNetworkWeights(&network, "weights.w");
        printf("weights.w loaded in the weights of the ANN\n");
        break;
      case 'P':
        printf("Network is:\n");
        PrintNetwork(&network);
        break;
      default:
        break;
    }
    if (key == WB_KEYBOARD_UP || key == WB_KEYBOARD_DOWN || (key >= '1' && key <= '4')) {
      if (current_pattern == NO_PATTERN)
        printf("No Pattern selected for learning\n");
      else
        printf("Pattern %d is selected for learning\n", current_pattern);
    }
  }
}

/*
 * This function perform the appropriate
 * action according to the state of the
 * robot. So, it just set the robot
 * motor speeds...
 */
static void robot_state_manager() {
  switch (state) {
    case BED:
      speed[LEFT] = 0;
      speed[RIGHT] = 0;
      break;
    case ROTATION:
      speed[LEFT] = 200;
      speed[RIGHT] = -200;
      break;
    case RANDOM_WALK: {
      int left_obstacle =
        wb_distance_sensor_get_value(ps[5]) + wb_distance_sensor_get_value(ps[6]) + wb_distance_sensor_get_value(ps[7]);
      int right_obstacle =
        wb_distance_sensor_get_value(ps[0]) + wb_distance_sensor_get_value(ps[1]) + wb_distance_sensor_get_value(ps[2]);
      int d = right_obstacle - left_obstacle;
      if (internal_state == 0)
        delta += (int)(5.0 * rand() / (double)RAND_MAX);
      else
        delta -= (int)(5.0 * rand() / (double)RAND_MAX);
      if (delta > 100)
        internal_state = 1;
      if (delta < -100)
        internal_state = 0;
      speed[RIGHT] = 200 + d + delta;
      speed[LEFT] = 200 - d - delta;
    } break;
    case DANSE: {
      int turn = 25;
      int forward = 200;
      if (internal_state > 200)
        internal_state = 0;
      else if (internal_state > 180) {
        speed[LEFT] = turn;
        speed[RIGHT] = -turn;
      } else if (internal_state > 140) {
        speed[LEFT] = -turn;
        speed[RIGHT] = turn;
      } else if (internal_state > 120) {
        speed[LEFT] = turn;
        speed[RIGHT] = -turn;
      } else if (internal_state > 100) {
        speed[LEFT] = -forward;
        speed[RIGHT] = -forward;
      } else if (internal_state > 80) {
        speed[LEFT] = turn;
        speed[RIGHT] = -turn;
      } else if (internal_state > 40) {
        speed[LEFT] = -turn;
        speed[RIGHT] = turn;
      } else if (internal_state > 20) {
        speed[LEFT] = turn;
        speed[RIGHT] = -turn;
      } else if (internal_state > 0) {
        speed[LEFT] = forward;
        speed[RIGHT] = forward;
      }
    }
      internal_state++;
      break;
    default:
      break;
  }
}

/*
 * This function search blue blobs
 * on the camera image. For each blue blob
 * found, it sample it by using a nearest
 * neighbor algorithm. The resulted sample
 * has a size of SAMPLE_WIDTH*SAMPLE_HEIGHT.
 * All the samples of the image are stored
 * in the "samples" array.
 * Finaly, this function displays the sample
 * array on the top left of the camera image.
 */
static void detect_blobs(void) {
  int i, j, k, x, y;
  const unsigned char *image = wb_camera_get_image(cam);
  int width = wb_camera_get_width(cam);
  int height = wb_camera_get_height(cam);

  // copy camera image to the display
  WbImageRef iref = wb_display_image_new(display, width, height, image, WB_IMAGE_BGRA);
  wb_display_image_paste(display, iref, 0, 0, false);

  // if the algorithm saw a blue pixel since the last sample
  int blue_pixel_detected = FALSE;

  // properties of a sample
  int sample_right = 0;
  int sample_left = width;
  int sample_up = 0;
  int sample_down = height;
  sample_counter = 0;

  // parse the image vertically
  for (i = 0; i < width; i++) {
    int blue_pixel_on_the_column = FALSE;
    for (j = 0; j < height; j++) {
      int r = wb_camera_image_get_red(image, width, i, j);
      int g = wb_camera_image_get_green(image, width, i, j);
      int b = wb_camera_image_get_blue(image, width, i, j);
      if (r < 150 && g < 150 && b > 150) {  // blue pixel detected
        blue_pixel_detected = TRUE;
        blue_pixel_on_the_column = TRUE;
        if (sample_right < i)
          sample_right = i;
        if (sample_left > i)
          sample_left = i;
        if (sample_up < j)
          sample_up = j;
        if (sample_down > j)
          sample_down = j;
      }
    }
    // at the end of a column, maybe a sample was seen.
    if ((i == width - 1 || blue_pixel_on_the_column == FALSE) && blue_pixel_detected == TRUE) {
      int sample_width = sample_right - sample_left + 1;
      int sample_height = sample_up - sample_down;
      if (sample_width > 1 && sample_height > 1 && sample_counter < SAMPLE_MAX_NUMBER) {
        // here a sample was seen
        // It should be sampled to SAMPLE_WIDTH*SAMPLE_HEIGHT and
        // be stored in the "samples" array and be displayed
        for (x = 0; x < SAMPLE_WIDTH; x++) {
          for (y = 0; y < SAMPLE_HEIGHT; y++) {
            int tmp_i = sample_left + my_round((float)x * sample_width / (float)SAMPLE_WIDTH);
            int tmp_j = sample_down + my_round((float)y * sample_height / (float)SAMPLE_HEIGHT);
            int value = wb_camera_image_get_blue(image, width, tmp_i, tmp_j);
            samples[sample_counter][SAMPLE_WIDTH * y + x] = 1.0f / 255.0f * value;
            wb_display_set_color(display, (value << 16) | (value << 8) | value);
            wb_display_draw_pixel(display, x + sample_counter * (SAMPLE_WIDTH + 1), y);
          }
        }
        sample_counter++;
      }
      blue_pixel_detected = FALSE;
      sample_left = width;
      sample_right = 0;
      sample_up = 0;
      sample_down = height;
    }
  }

  // display the majenta bounds
  wb_display_set_color(display, 0xFF00FF);
  for (x = 0; x < sample_counter * (SAMPLE_WIDTH + 1); x++)
    wb_display_draw_pixel(display, x, SAMPLE_HEIGHT);
  for (k = 0; k < sample_counter; k++) {
    for (y = 0; y < SAMPLE_HEIGHT; y++)
      wb_display_draw_pixel(display, k + (1 + k) * SAMPLE_WIDTH, y);
  }
}

/*
 * Print an entire sample on the
 * log window
 */
/*
static void print_samples(void){
  int i,j,k;
  for (k=0;k<sample_counter;k++){
    char big_buffer[6*SAMPLE_WIDTH*SAMPLE_HEIGHT+SAMPLE_HEIGHT+1] = "\n";
    for (j=0;j<SAMPLE_HEIGHT;j++){
      for (i=0;i<SAMPLE_WIDTH;i++) {
        char mini_buffer[6];
        sprintf(mini_buffer,"%1.3f ",samples[k][i+j*SAMPLE_WIDTH]);
        strcat(big_buffer,mini_buffer);
      }
      strcat(big_buffer,"\n");
    }
    printf("%s",big_buffer);
  }
}
*/

/*
 * Run function...
 */
static int run(void) {
  // Manage the keyboard, move the robot,
  // and get the landmarks on the camera
  // (stored in the "samples" array)
  keyboard_manager();
  robot_state_manager();
  detect_blobs();
  // print_samples();

  /* Learning mode (only if the robot sees one (and only one) sample)
   *
   * 1. prepare the output (supervised learning) according to the selected pattern.
   *    It corresponds to the result one want to achieve
   *
   * 2. Put the sample values as input of the network
   *
   * 3. Run the backprop algorithm by specifing the desired output
   *
   * 4. Print the error
   */
  if (mode == LEARN && sample_counter == 1 && current_pattern >= P1 && current_pattern <= P4) {
    float output[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    output[current_pattern - 1] = 1.0f;

    InputToNetwork(&network, &samples[0][0]);
    ActivateNetwork(&network);
    error = TrainNetwork(&network, &output[0]);
    printf("Learned P%d. Resulted error: %f\n", current_pattern, error);
  }

  /* Testing mode (only if the robot sees at least one sample)
   *
   * For each sample:
   *
   *   1. Put the sample values as input of the network
   *
   *   2. Read the output (ouput of the last layer of the ANN) and print
   *      the results (the maximum peak is concidered as the answer)
   */
  if (mode == TEST && sample_counter > 0) {
    int k, i;
    for (k = 0; k < sample_counter; k++) {
      InputToNetwork(&network, &samples[k][0]);
      ActivateNetwork(&network);
      PrintNetworkOutput(&network);

      // get result
      layer_t output_layer = network.layers[network.size - 1];
      float max = -666;
      int max_index = -1;
      for (i = 0; i < output_layer.depth; i++) {
        if (output_layer.y[i] > max) {
          max_index = i;
          max = output_layer.y[i];
        }
      }

      // print result
      printf("I guess that I see the landmark P%d\n", max_index + 1);
    }
  }

  wb_motor_set_velocity(left_motor, SPEED_UNIT * speed[LEFT]);
  wb_motor_set_velocity(right_motor, SPEED_UNIT * speed[RIGHT]);

  return TIME_STEP;
}

/*
 * Main function
 */
int main() {
  wb_robot_init(); /* initialize the webots controller library */

  usage();

  RandomizeNetwork(&network);  // initiate the ANN with random numbers

  srand(time(0));  // initiate the random number generator

  // get the led devices
  int i;
  char text[5] = "led0";
  for (i = 0; i < NB_LEDS; i++) {
    led[i] = wb_robot_get_device(text);
    wb_led_set(led[i], FALSE);
    text[3]++;
  }

  // get the distance sensor devices
  char textPS[] = "ps0";
  for (i = 0; i < NB_DIST_SENS; i++) {
    ps[i] = wb_robot_get_device(textPS);
    wb_distance_sensor_enable(ps[i], TIME_STEP);
    textPS[2]++;
  }

  // get the camera device
  cam = wb_robot_get_device("camera");
  wb_camera_enable(cam, TIME_STEP);

  // get the display device
  display = wb_robot_get_device("display");

  // enable the keyboard
  wb_keyboard_enable(TIME_STEP);

  // get the motors and set target position to infinity (speed control)
  left_motor = wb_robot_get_device("left wheel motor");
  right_motor = wb_robot_get_device("right wheel motor");
  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);
  wb_motor_set_velocity(left_motor, 0.0);
  wb_motor_set_velocity(right_motor, 0.0);

  /* main loop */
  while (wb_robot_step(TIME_STEP) != -1)
    run();

  wb_robot_cleanup();

  return 0;
}
