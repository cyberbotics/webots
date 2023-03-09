// Copyright 1996-2023 Cyberbotics Ltd.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     https://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/*
 * Description: This controller demonstrates how to use openCV 2.4 to process the camera image.
 *              In order to execute and recompile this example, opencv must be installed.
 *              To run this controller, it is recommended to install the Webots development
 *              environment as explained here:
 * https://github.com/cyberbotics/webots/wiki#installation-of-the-webots-development-environment
 *
 */

#include <stdio.h>
#include <webots/camera.h>
#include <webots/display.h>
#include <webots/keyboard.h>
#include <webots/robot.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#define NB_FILTERS 6

#define RED 0
#define GREEN 1
#define BLUE 2
#define YELLOW 3
#define PURPLE 4
#define WHITE 5
#define NONE 6
#define ALL 7

using namespace cv;

/* The scalars correspond to HSV margin (In the first example, [0,5] is the accepted hue for the red filter,
   [150,255] the accepted saturation and [30,255] the accepted value). */
static Scalar lMargin[NB_FILTERS] = {Scalar(0, 150, 30),  Scalar(58, 150, 30),  Scalar(115, 150, 30),
                                     Scalar(28, 150, 30), Scalar(148, 150, 30), Scalar(0, 0, 50)};
static Scalar uMargin[NB_FILTERS] = {Scalar(5, 255, 255),  Scalar(62, 255, 255),  Scalar(120, 255, 255),
                                     Scalar(32, 255, 255), Scalar(152, 255, 255), Scalar(0, 0, 255)};

/* we use the last spot to quickly check if filters are used or not. */
static bool filters[NB_FILTERS + 1] = {false, false, false, false, false, false, true};
static int width;
static int height;
static unsigned char *processed_image;

void display_commands() {
  printf("Press R to apply/remove a red filter.\n");
  printf("Press G to apply/remove a green filter.\n");
  printf("Press B to apply/remove a blue filter.\n");
  printf("Press Y to apply/remove a yellow filter.\n");
  printf("Press P to apply/remove a purple filter.\n");
  printf("Press W to apply/remove a white filter.\n");
  printf("Press A to apply all filters.\n");
  printf("Press X to remove all filters.\n");
  printf("When one or several filter is applied, only the corresponding colors are considered in the image.\n");
  printf("The processed image consists of the entire image if no filter is used.\n");
}

/* Function to process the image from the camera and display the result.
   The only processing this function does is only displaying parts of the image which
   correspond to one of the predefined filters. */
void process_image(const unsigned char *image, int length) {
  /* Matrix which contains the BGRA image from Webots' camera */
  Mat img = Mat(Size(width, height), CV_8UC4);
  img.data = const_cast<uchar *>(image);

  /* Matrix which contains the HSV version of the previous image */
  Mat hsv = Mat(Size(width, height), CV_8UC3);
  cvtColor(img, hsv, COLOR_BGR2HSV);

  /* Temporary data corresponding to the HSV image filtered through one filter */
  Mat temp_filtered = Mat(Size(width, height), CV_8UC1);

  /* Matrix which will contain the post-processing image */
  Mat filtered = Mat(Size(width, height), CV_8UC4);

  if (filters[NB_FILTERS])
    filtered = img;
  else {
    /* Initialize the output matrix in the case we have to build it */
    for (int i = 0; i < height; ++i) {
      for (int j = 0; j < width; ++j) {
        filtered.at<Vec4b>(i, j)[0] = 0;
        filtered.at<Vec4b>(i, j)[1] = 0;
        filtered.at<Vec4b>(i, j)[2] = 0;
        filtered.at<Vec4b>(i, j)[3] = 255;
      }
    }

    for (int f = 0; f < NB_FILTERS; ++f) {
      if (filters[f]) {
        inRange(hsv, lMargin[f], uMargin[f], temp_filtered);
        /* Copy the value from the original image to the output if it's accepted by a filter */
        for (int i = 0; i < height; ++i) {
          for (int j = 0; j < width; ++j) {
            if (temp_filtered.at<uchar>(i, j) == 255) {
              filtered.at<Vec4b>(i, j) = img.at<Vec4b>(i, j);
            }
          }
        }
      }
    }
  }

  /* Refresh the picture to display */
  memcpy(processed_image, filtered.data, length);
}

/* This function handles which filters need to be used when the display function is called. */
void apply_filter(int filter) {
  if (filter > (NB_FILTERS + 1)) {
    printf("Error: Unknown filter.\n");
  } else {
    int cnt = 0;

    if (filter == NB_FILTERS) {
      for (int i = 0; i < NB_FILTERS; ++i)
        filters[i] = false;
    } else if (filter == (NB_FILTERS + 1)) {
      for (int i = 0; i < NB_FILTERS; ++i)
        filters[i] = true;
    } else {
      if (filters[filter])
        filters[filter] = false;
      else
        filters[filter] = true;
    }

    printf("Filters currently applied: ");
    for (int i = 0; i < NB_FILTERS; ++i) {
      if (filters[i]) {
        switch (i) {
          case 0:
            printf("red ");
            break;
          case 1:
            printf("green ");
            break;
          case 2:
            printf("blue ");
            break;
          case 3:
            printf("yellow ");
            break;
          case 4:
            printf("purple ");
            break;
          case 5:
            printf("white ");
            break;
          default:
            break;
        }
        ++cnt;
      }
    }

    if (!cnt) {
      printf("none (the entire image will be displayed).");
      filters[NB_FILTERS] = true;
    } else {
      filters[NB_FILTERS] = false;
    }

    printf("\n");
  }
}

int main() {
  /* Initialize Webots */
  wb_robot_init();
  int timestep = wb_robot_get_basic_time_step();

  printf("Vision module demo, using openCV.\n");

  display_commands();

  /* Initialize camera */
  WbDeviceTag camera = wb_robot_get_device("camera");
  wb_camera_enable(camera, timestep);
  width = wb_camera_get_width(camera);
  height = wb_camera_get_height(camera);

  /* Variables for the display */
  int length = 4 * width * height * sizeof(unsigned char);
  WbDeviceTag processed_image_display = wb_robot_get_device("proc_im_display");
  WbImageRef processed_image_ref = NULL;
  processed_image = static_cast<unsigned char *>(malloc(length));

  wb_keyboard_enable(timestep);

  int input = 0;
  bool key_pressed = false;

  while (wb_robot_step(timestep) != -1) {
    /* Process inputs */
    const int key = wb_keyboard_get_key();
    if (key >= 0 && !key_pressed) {
      key_pressed = true;
      input = key;
    } else if (key == -1 && key_pressed) {
      key_pressed = false;
      switch (input) {
        case 'X':
          apply_filter(NONE);
          break;
        case 'R':
          apply_filter(RED);
          break;
        case 'G':
          apply_filter(GREEN);
          break;
        case 'B':
          apply_filter(BLUE);
          break;
        case 'Y':
          apply_filter(YELLOW);
          break;
        case 'P':
          apply_filter(PURPLE);
          break;
        case 'W':
          apply_filter(WHITE);
          break;
        case 'A':
          apply_filter(ALL);
          break;
        default:
          break;
      }
    }

    /* Process the image */
    process_image(wb_camera_get_image(camera), length);

    if (processed_image_ref) {
      wb_display_image_delete(processed_image_display, processed_image_ref);
      processed_image_ref = NULL;
    }

    /* Display the image */
    processed_image_ref = wb_display_image_new(processed_image_display, width, height, processed_image, WB_IMAGE_ARGB);
    wb_display_image_paste(processed_image_display, processed_image_ref, 0, 0, false);
  }

  // clean up
  if (processed_image_ref)
    wb_display_image_delete(processed_image_display, processed_image_ref);
  free(processed_image);

  wb_robot_cleanup();

  return 0;
}
