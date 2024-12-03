/*
 * Copyright 1996-2024 Cyberbotics Ltd.
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

/**********************************************************************************/
/* Description:  Webots C programming interface for the Camera node               */
/**********************************************************************************/

#ifndef WB_CAMERA_H
#define WB_CAMERA_H

#define WB_USING_C_API
#include "camera_recognition_object.h"
#include "types.h"

#ifdef __cplusplus
extern "C" {
#endif

void wb_camera_enable(WbDeviceTag tag, int sampling_period);
void wb_camera_disable(WbDeviceTag tag);
int wb_camera_get_sampling_period(WbDeviceTag tag);

const unsigned char *wb_camera_get_image(WbDeviceTag tag);
int wb_camera_get_width(WbDeviceTag tag);
int wb_camera_get_height(WbDeviceTag tag);
double wb_camera_get_fov(WbDeviceTag tag);
double wb_camera_get_max_fov(WbDeviceTag tag);
double wb_camera_get_min_fov(WbDeviceTag tag);
void wb_camera_set_fov(WbDeviceTag tag, double fov);  // fov specified in rad
double wb_camera_get_exposure(WbDeviceTag tag);
void wb_camera_set_exposure(WbDeviceTag tag, double exposure);
double wb_camera_get_focal_length(WbDeviceTag tag);
double wb_camera_get_focal_distance(WbDeviceTag tag);
double wb_camera_get_max_focal_distance(WbDeviceTag tag);
double wb_camera_get_min_focal_distance(WbDeviceTag tag);
void wb_camera_set_focal_distance(WbDeviceTag tag, double focal_distance);
double wb_camera_get_near(WbDeviceTag tag);
int wb_camera_save_image(WbDeviceTag tag, const char *filename, int quality);

// smart camera
bool wb_camera_has_recognition(WbDeviceTag tag);
void wb_camera_recognition_enable(WbDeviceTag tag, int sampling_period);
void wb_camera_recognition_disable(WbDeviceTag tag);
int wb_camera_recognition_get_sampling_period(WbDeviceTag tag);
int wb_camera_recognition_get_number_of_objects(WbDeviceTag tag);
const WbCameraRecognitionObject *wb_camera_recognition_get_objects(WbDeviceTag tag);
bool wb_camera_recognition_has_segmentation(WbDeviceTag tag);
void wb_camera_recognition_enable_segmentation(WbDeviceTag tag);
void wb_camera_recognition_disable_segmentation(WbDeviceTag tag);
bool wb_camera_recognition_is_segmentation_enabled(WbDeviceTag tag);
const unsigned char *wb_camera_recognition_get_segmentation_image(WbDeviceTag tag);
int wb_camera_recognition_save_segmentation_image(WbDeviceTag tag, const char *filename, int quality);

#ifdef WB_MATLAB_LOADLIBRARY
// This function should be used only in the Matlab wrapper
const WbCameraRecognitionObject *wb_camera_recognition_get_object(WbDeviceTag tag, int index);
#endif

/* useful macros to get pixel colors from the image data, width and coords *
 *
 *  ^ y
 *  |    (height)
 *  |===============
 *  |=============== *: pixel@(x,y)
 *  |---------*=====
 *  |=========|===== (width)
 *  |=========|=====
 *  |=========|=====
 *  |=========|=====
 * -+-----------------> x
 * o|
 */

#define wb_camera_image_get_red(image, width, x, y) (image[4 * ((y) * (width) + (x)) + 2])
#define wb_camera_image_get_green(image, width, x, y) (image[4 * ((y) * (width) + (x)) + 1])
#define wb_camera_image_get_blue(image, width, x, y) (image[4 * ((y) * (width) + (x))])

#ifdef KROS_COMPILATION
#define wb_camera_image_get_gray(image, width, x, y) (image[(y) * (width) + (x)])
#else
#define wb_camera_image_get_gray(image, w, x, y) \
  ((image[4 * ((y) * (w) + (x)) + 2] + image[4 * ((y) * (w) + (x)) + 1] + image[4 * ((y) * (w) + (x))]) / 3)
#endif
// alias
#define wb_camera_image_get_grey(image, width, x, y) wb_camera_image_get_gray(image, width, x, y)

#ifdef __cplusplus
}
#endif

#endif /* WB_CAMERA_H */
