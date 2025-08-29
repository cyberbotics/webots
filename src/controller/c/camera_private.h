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

#ifndef CAMERA_PRIVATE_H
#define CAMERA_PRIVATE_H

#include <webots/camera.h>
#include <webots/types.h>

typedef struct {
  double min_fov;
  double max_fov;
  double exposure;
  double focal_length;
  double focal_distance;
  double min_focal_distance;
  double max_focal_distance;
  bool set_exposure;
  bool set_focal_distance;
  bool set_fov;
  bool has_recognition;
  bool enable_recognition;
  int recognition_sampling_period;
  int recognized_object_number;                   // number of object currrently recognized
  WbCameraRecognitionObject *recognized_objects;  // list of objects
  bool segmentation;
  bool segmentation_enabled;
  bool segmentation_changed;
  Image *segmentation_image;
} Camera;

void camera_allocate_segmentation_image(WbDeviceTag tag, int size);
const unsigned char *camera_get_segmentation_image_buffer(WbDeviceTag tag);

#endif  // CAMERA_PRIVATE_H
