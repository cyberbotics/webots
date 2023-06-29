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

/*******************************************************************************************************/
/* Description:  Header file of the camera API for the e-puck crosscompilation                         */
/*******************************************************************************************************/

#ifndef CAMERA_H
#define CAMERA_H

#include "types.h"
#include <float.h>

void           wb_camera_enable(WbDeviceTag,int sampling_period);
void           wb_camera_disable(WbDeviceTag);
#define        wb_camera_get_width(dt) 52
#define        wb_camera_get_height(dt) 39
#define        wb_camera_get_fov(dt) 0.7
#define        wb_camera_get_type(dt) 'c'
#define        wb_camera_get_near(dt) 0.0
#define        wb_camera_get_far(dt) DBL_MAX

// The returned image is turned through 90° and encoded in RGB_565
// For passing through this, use the camera_image_get_* functions below.
const unsigned char *wb_camera_get_image(WbDeviceTag);

unsigned char wb_camera_image_get_red(const unsigned char* image,int width,int x,int y);
unsigned char wb_camera_image_get_green(const unsigned char* image,int width,int x,int y);
unsigned char wb_camera_image_get_blue(const unsigned char* image,int width,int x,int y);

#define wb_camera_image_get_gray(image,w,x,y) (wb_camera_image_get_red(image,w,x,y)/3 + wb_camera_image_get_green(image,w,x,y)/3 + wb_camera_image_get_blue(image,w,x,y)/3)
#define wb_camera_image_get_grey(image,w,x,y) wb_camera_image_get_gray(image,w,x,y)

#endif /* CAMERA_H */
