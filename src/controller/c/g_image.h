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

#ifndef G_IMAGE_H
#define G_IMAGE_H

#define G_IMAGE_NONE 0
#define G_IMAGE_PNG 1
#define G_IMAGE_JPEG 2
#define G_IMAGE_XPM 3
#define G_IMAGE_TIFF 4
#define G_IMAGE_HDR 5

#define G_IMAGE_DATA_FORMAT_F 0
#define G_IMAGE_DATA_FORMAT_RGB 1
#define G_IMAGE_DATA_FORMAT_ABGR 2
#define G_IMAGE_DATA_FORMAT_BGRA 3
#define G_IMAGE_DATA_FORMAT_RGBA 4

#ifdef __cplusplus
extern "C" {
#endif

typedef struct _GImage GImage;

struct _GImage {
  int width;
  int height;
  float *float_data;
  unsigned char *data;
  unsigned char data_format;
  char failed;
  char flipped;
};

GImage *g_image_new(const char *filename);
void g_image_flip(GImage *);
void g_image_delete(GImage *);
int g_image_save(GImage *, const char *filename, char quality);
// the caller is responsible to free *target_data
int g_image_save_to_jpeg_buffer(GImage *img, unsigned char **target_data, unsigned long *target_data_size, char quality);
int g_image_downscale(GImage *img, int new_width, int new_height, float max_range);
unsigned char g_image_get_type(const char *filename);

#ifdef __cplusplus
}
#endif

#endif  // G_IMAGE_H
