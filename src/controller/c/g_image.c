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

#include "g_image.h"

#include <assert.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#define STB_IMAGE_IMPLEMENTATION
#define STB_IMAGE_WRITE_IMPLEMENTATION
#define STBIW_WINDOWS_UTF8
#include <stb_image.h>
#include <stb_image_write.h>

#include <webots/types.h>

// create a 64x64 pixel chess board image
static void g_image_make_chess_board(GImage *image) {
  int i, j, idx;
  unsigned char chessboard;
  image->data = malloc(64 * 64 * 3);
  image->data_format = G_IMAGE_DATA_FORMAT_RGB;
  image->width = 64;
  image->height = 64;
  for (j = 0; j < 64; ++j) {
    for (i = 0; i < 64; ++i) {
      idx = i * 64 * 3 + j * 3;
      chessboard = ((((i & 0x8) == 0) ? 1 : 0) ^ (((j & 0x8) == 0) ? 1 : 0)) * 255;
      image->data[idx++] = chessboard;
      image->data[idx++] = chessboard;
      image->data[idx++] = chessboard;
    }
  }
  image->flipped = false;
  image->failed = true;
}

static int g_image_file_not_found(const char *filename, GImage *image) {
  fprintf(stderr, "Error: %s: file not found\n", filename);
  g_image_make_chess_board(image);
  return false;
}

static int g_image_png_load(const char *filename, GImage *image) {
  if (access(filename, F_OK) == -1)
    return g_image_file_not_found(filename, image);

  int number_of_components;
  image->data = stbi_load(filename, &image->width, &image->height, &number_of_components, 0);

  if (!image->data)
    return false;

  if (number_of_components == STBI_rgb)
    image->data_format = G_IMAGE_DATA_FORMAT_RGB;
  else
    image->data_format = G_IMAGE_DATA_FORMAT_RGBA;
  return true;
}

static int g_image_jpeg_load(const char *filename, GImage *image) {
  if (access(filename, F_OK) == -1)
    return g_image_file_not_found(filename, image);

  int number_of_components;
  image->data = stbi_load(filename, &image->width, &image->height, &number_of_components, 0);

  if (!image->data)
    return false;

  image->data_format = G_IMAGE_DATA_FORMAT_RGB;

  return true;
}

unsigned char g_image_get_type(const char *filename) {
  int l = strlen(filename);
  if (((filename[l - 3] == 'j' || filename[l - 3] == 'J') && (filename[l - 2] == 'p' || filename[l - 2] == 'P') &&
       (filename[l - 1] == 'g' || filename[l - 1] == 'G')) ||
      ((filename[l - 4] == 'j' || filename[l - 4] == 'J') && (filename[l - 3] == 'p' || filename[l - 3] == 'P') &&
       (filename[l - 2] == 'e' || filename[l - 2] == 'E') && (filename[l - 1] == 'g' || filename[l - 1] == 'G')))
    return G_IMAGE_JPEG;
  else if ((filename[l - 3] == 'p' || filename[l - 3] == 'P') && (filename[l - 2] == 'n' || filename[l - 2] == 'N') &&
           (filename[l - 1] == 'g' || filename[l - 1] == 'G'))
    return G_IMAGE_PNG;
  else if (((filename[l - 3] == 't' || filename[l - 3] == 'T') && (filename[l - 2] == 'i' || filename[l - 2] == 'I') &&
            (filename[l - 1] == 'f' || filename[l - 1] == 'F')) ||
           ((filename[l - 4] == 't' || filename[l - 4] == 'T') && (filename[l - 3] == 'i' || filename[l - 3] == 'I') &&
            (filename[l - 2] == 'f' || filename[l - 2] == 'F') && (filename[l - 1] == 'f' || filename[l - 1] == 'F')))
    return G_IMAGE_TIFF;
  else if ((filename[l - 3] == 'h' || filename[l - 3] == 'H') && (filename[l - 2] == 'd' || filename[l - 2] == 'D') &&
           (filename[l - 1] == 'r' || filename[l - 1] == 'R'))
    return G_IMAGE_HDR;
  else
    return G_IMAGE_NONE;
}

void g_image_delete(GImage *image) {
  free(image->data);
  free(image);
}

static int g_image_png_save(GImage *img, const char *filename) {
  FILE *fd = fopen(filename, "wb");
  if (!fd) {
    if (filename[0] == '/'
#ifdef _WIN32
        || (filename[1] == ':' && filename[2] == '\\')
#define DIR_SEPARATOR '\\'
#else
#define DIR_SEPARATOR '/'
#endif
    )
      fprintf(stderr, "Insufficient permissions to write file: %s\n", filename);
    else {
      char cwd[256];
      char *r = getcwd(cwd, 256);
      if (r)
        fprintf(stderr, "Insufficient permissions to write file: %s%c%s\n", cwd, DIR_SEPARATOR, filename);
      else
        fprintf(stderr, "Cannot get current directory for %s!\n", filename);
    }
    return -1;
  }
  fclose(fd);

  if (img->data_format == G_IMAGE_DATA_FORMAT_BGRA) {
    unsigned char *image = (unsigned char *)malloc(4 * img->width * img->height);
    int i;
    for (i = 0; i < img->width * img->height; ++i) {
      image[4 * i] = img->data[4 * i + 2];
      image[4 * i + 1] = img->data[4 * i + 1];
      image[4 * i + 2] = img->data[4 * i];
      image[4 * i + 3] = img->data[4 * i + 3];
    }
    const int ret = stbi_write_png(filename, img->width, img->height, STBI_rgb_alpha, image, img->width * STBI_rgb_alpha);
    free(image);
    if (ret != 1)
      return -1;
    return 0;
  }

  int number_of_components = STBI_rgb_alpha;
  if (img->data_format == G_IMAGE_DATA_FORMAT_RGB)
    number_of_components = STBI_rgb;
  if (stbi_write_png(filename, img->width, img->height, number_of_components, img->data, img->width * number_of_components) !=
      1)
    return -1;
  return 0;
}

static int g_image_jpeg_save(GImage *img, char quality, const char *filename) {
  FILE *fd = fopen(filename, "wb");
  if (!fd) {
    fprintf(stderr, "Error: could not open \"%s\" for writing\n", filename);
    return -1;
  }
  fclose(fd);

  if (img->data_format == G_IMAGE_DATA_FORMAT_BGRA) {
    unsigned char *image = (unsigned char *)malloc(3 * img->width * img->height);
    int i;
    for (i = 0; i < img->width * img->height; ++i) {
      image[3 * i] = img->data[4 * i + 2];
      image[3 * i + 1] = img->data[4 * i + 1];
      image[3 * i + 2] = img->data[4 * i];
    }
    const int ret = stbi_write_jpg(filename, img->width, img->height, STBI_rgb, image, quality);
    free(image);
    if (ret != 1)
      return -1;
    return 0;
  }

  if (stbi_write_jpg(filename, img->width, img->height, STBI_rgb, img->data, quality) != 1)
    return -1;
  return 0;
}

static int g_image_hdr_save(GImage *img, const char *filename) {
  FILE *fd = fopen(filename, "wb");
  if (!fd) {
    fprintf(stderr, "Error: could not open \"%s\" for writing\n", filename);
    return -1;
  }
  fclose(fd);

  if (stbi_write_hdr(filename, img->width, img->height, STBI_grey, img->float_data) != 1)
    return -1;
  return 0;
}

int g_image_save(GImage *img, const char *filename, char quality) {
  switch (g_image_get_type(filename)) {
    case G_IMAGE_JPEG:
      return g_image_jpeg_save(img, quality, filename);
    case G_IMAGE_PNG:
      return g_image_png_save(img, filename);
    case G_IMAGE_HDR:
      return g_image_hdr_save(img, filename);
    default:
      fprintf(stderr, "Cannot save: unsupported image type: %s\n", filename);
      return -1;
  }
}

struct ImageData {
  unsigned char **target_data;
  unsigned long *target_data_size;
};

void g_image_save_to_jpeg_buffer_callback(void *context, void *data, int size) {
  if (!*(((struct ImageData *)context)->target_data))
    *(((struct ImageData *)context)->target_data) = (unsigned char *)malloc(size);
  else
    *(((struct ImageData *)context)->target_data) = (unsigned char *)realloc(
      *(((struct ImageData *)context)->target_data), *(((struct ImageData *)context)->target_data_size) + size);
  memcpy(*(((struct ImageData *)context)->target_data) + *(((struct ImageData *)context)->target_data_size), data, size);
  *(((struct ImageData *)context)->target_data_size) += size;
}

int g_image_save_to_jpeg_buffer(GImage *img, unsigned char **target_data, unsigned long *target_data_size, char quality) {
  struct ImageData imageData;
  imageData.target_data = target_data;
  imageData.target_data_size = target_data_size;
  *target_data_size = 0;

  if (stbi_write_jpg_to_func((stbi_write_func *)&g_image_save_to_jpeg_buffer_callback, &imageData, img->width, img->height,
                             STBI_rgb, img->data, quality) != 1)
    return -1;

  return 0;
}

GImage *g_image_new(const char *filename) {
  GImage *image = malloc(sizeof(GImage));
  image->failed = true;
  switch (g_image_get_type(filename)) {
    case G_IMAGE_JPEG:
      image->failed = !g_image_jpeg_load(filename, image);
      break;
    case G_IMAGE_PNG:
      image->failed = !g_image_png_load(filename, image);
      break;
    default:
      g_image_make_chess_board(image);
      fprintf(stderr, "Unsupported image type: %s\n", filename);
  }
  return image;
}

void g_image_flip(GImage *im) {
  int i, j;
  const int width = im->width;
  const int height = im->height;
  unsigned char channel = im->data_format == G_IMAGE_DATA_FORMAT_RGB ? 3 : 4;
  if (im->data == NULL)
    return;
  unsigned char *flipped_im = malloc(width * height * channel);
  for (i = 0; i < height; ++i) {
    for (j = 0; j < width; ++j) {
      flipped_im[(i * width + width - 1 - j) * channel] = im->data[(i * width + j) * channel];
      flipped_im[(i * width + width - 1 - j) * channel + 1] = im->data[(i * width + j) * channel + 1];
      flipped_im[(i * width + width - 1 - j) * channel + 2] = im->data[(i * width + j) * channel + 2];
      if (channel == 4)
        flipped_im[(i * width + width - 1 - j) * channel + 3] = im->data[(i * width + j) * channel + 3];
    }
  }
  free(im->data);
  im->data = flipped_im;
  im->flipped = !im->flipped;
}

int g_image_downscale(GImage *img, int new_width, int new_height, float max_range) {
  // assumptions:
  // - img->data is cleared at the caller level.
  // - (G_IMAGE_DATA_FORMAT_BGRA || G_IMAGE_DATA_FORMAT_F) -> G_IMAGE_DATA_FORMAT_RGB conversion.
  // - new dimension is smaller than or equals to the old dimension.
  assert(img->data_format == G_IMAGE_DATA_FORMAT_BGRA || img->data_format == G_IMAGE_DATA_FORMAT_F);
  assert((new_width < img->width && new_height < img->height) || (new_width < img->width && new_height == img->height) ||
         (new_width == img->width && new_height < img->height) ||
         (new_width == img->width && new_height == img->height)  // do only a BGRA -> RGB conversion.
  );

  unsigned char *new_data = malloc(new_width * new_height * 3);
  if (!new_data)
    return -1;

  const float width_ratio = (float)img->width / (float)new_width;
  const float height_ratio = (float)img->height / (float)new_height;
  const float _255_over_max = 255.0f / max_range;
  int new_y, new_x, c;
  for (new_y = 0; new_y < new_height; ++new_y) {
    const int y = height_ratio * new_y + 0.5f;  // +0.5 to round it.
    const int line_index = y * img->width;
    const int new_line_index = new_y * new_width;
    for (new_x = 0; new_x < new_width; ++new_x) {
      const int x = width_ratio * new_x + 0.5f;  // +0.5 to round it.
      const int new_column_index = 3 * (new_line_index + new_x);
      if (img->data_format == G_IMAGE_DATA_FORMAT_BGRA) {
        const int column_index = 4 * (line_index + x);
        for (c = 0; c < 3; ++c)
          new_data[new_column_index + c] = img->data[column_index + (2 - c)];
      } else {  // img->data_format == G_IMAGE_DATA_FORMAT_F
        const int column_index = line_index + x;
        for (c = 0; c < 3; ++c)
          new_data[new_column_index + c] = img->float_data[column_index] * _255_over_max;
      }
    }
  }

  img->width = new_width;
  img->height = new_height;
  img->data = new_data;
  img->float_data = NULL;
  img->data_format = G_IMAGE_DATA_FORMAT_RGB;
  return 0;
}
