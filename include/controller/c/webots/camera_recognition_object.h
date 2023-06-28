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

/***************************************************************************************************/
/* Description:  Common definition of the 'WbCameraRecognitionObject' for both the C and C++ APIs */
/**************************************************************************************************/

#ifndef WB_CAMERA_RECOGNITION_OBJECT_H
#define WB_CAMERA_RECOGNITION_OBJECT_H

typedef struct {
  int id;
  double position[3];
  double orientation[4];
  double size[2];
  int position_on_image[2];
  int size_on_image[2];
  int number_of_colors;
  double *colors;
  char *model;
} WbCameraRecognitionObject;

#endif /* WB_CAMERA_RECOGNITION_OBJECT_H */
