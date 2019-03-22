/*
 * Copyright 1996-2019 Cyberbotics Ltd.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
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
#include <tiffio.h>
#include <unistd.h>

#include <jpeglib.h>
#include <png.h>

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
  png_structp png_ptr;
  png_infop info_ptr;
  png_uint_32 w = 0, h = 0;
  int color_type, bit_depth;
  png_bytep *row_pointers = NULL;
  FILE *f = fopen(filename, (char *)"rb");
  if (f) {
    png_ptr = png_create_read_struct(PNG_LIBPNG_VER_STRING, NULL, NULL, NULL);
    if (png_ptr == NULL)
      fprintf(stderr, "Wrong version of libpng: %s\n", PNG_LIBPNG_VER_STRING);
    info_ptr = png_create_info_struct(png_ptr);

    image->data = NULL;
    if (setjmp(png_jmpbuf(png_ptr))) {
      fprintf(stderr, "Error while reading %s", filename);
      png_destroy_read_struct(&png_ptr, &info_ptr, NULL);
      fclose(f);
      free(row_pointers);
      free(image->data);
      g_image_make_chess_board(image);
      return false;
    }

    char buf[8];
    const size_t n = fread(buf, 1, 8, f);
    if (n == 0) {
      fclose(f);
      return false;  // should never happen
    }
    if (png_sig_cmp((unsigned char *)buf, (png_size_t)0, 8)) {
      fprintf(stderr, "%s is not a PNG file", filename);
      g_image_make_chess_board(image);
      fclose(f);
      return false;
    }
    png_init_io(png_ptr, f);
    png_set_sig_bytes(png_ptr, 8);
    png_read_info(png_ptr, info_ptr);
    png_get_IHDR(png_ptr, info_ptr, &w, &h, &bit_depth, &color_type, NULL, NULL, NULL);
    image->width = (int)w;
    image->height = (int)h;
    if ((color_type == PNG_COLOR_TYPE_GRAY_ALPHA) || (color_type == PNG_COLOR_TYPE_RGB_ALPHA))
      image->data_format = G_IMAGE_DATA_FORMAT_ABGR;
    else
      image->data_format = G_IMAGE_DATA_FORMAT_RGB;

    png_set_strip_16(png_ptr);  // 16 bits -> 8 bits per channel
    png_set_packing(png_ptr);
    png_set_expand(png_ptr);

    if (image->data_format == G_IMAGE_DATA_FORMAT_ABGR) {
      png_set_bgr(png_ptr);         // RGBA -> BGRA
      png_set_swap_alpha(png_ptr);  // BGRA -> ABGR
    }

    const int channel = image->data_format == G_IMAGE_DATA_FORMAT_RGB ? 3 : 4;
    image->data = malloc(image->width * image->height * channel);
    row_pointers = malloc(sizeof(png_bytep) * image->height);
    int i;
    for (i = 0; i < image->height; i++)
      row_pointers[i] = image->data + (i * image->width * channel);
    png_read_image(png_ptr, row_pointers);
    png_read_end(png_ptr, NULL);

    // g_print("width=%d height=%d channel=%d data=%p\n",
    //   width, height, channel, data);

    free(row_pointers);
    png_destroy_read_struct(&png_ptr, &info_ptr, NULL);
    fclose(f);
    return true;
  } else
    return g_image_file_not_found(filename, image);
}

struct custom_jpeg_error_mgr {
  struct jpeg_error_mgr pub;
  jmp_buf setjmp_buffer;
};

static void custom_jpeg_error_exit(j_common_ptr cinfo) {
  struct custom_jpeg_error_mgr *ptr = (struct custom_jpeg_error_mgr *)cinfo->err;
  fprintf(stderr, "JPEG error: ");
  (*cinfo->err->output_message)(cinfo);  // display the message
  longjmp(ptr->setjmp_buffer, 1);
}

static int g_image_jpeg_load(const char *filename, GImage *image) {
  struct jpeg_decompress_struct cinfo;
  struct custom_jpeg_error_mgr jerr;
  unsigned char *line[16], *ptr;
  FILE *f;

  f = fopen(filename, "rb");
  if (f) {
    cinfo.err = jpeg_std_error(&jerr.pub);
    jerr.pub.error_exit = custom_jpeg_error_exit;
    if (setjmp(jerr.setjmp_buffer)) {
      jpeg_destroy_decompress(&cinfo);
      fclose(f);
      g_image_make_chess_board(image);
      return false;
    }
    jpeg_create_decompress(&cinfo);
    jpeg_stdio_src(&cinfo, f);
    jpeg_read_header(&cinfo, true);
    cinfo.do_fancy_upsampling = false;
    cinfo.do_block_smoothing = false;
    jpeg_start_decompress(&cinfo);
    image->width = cinfo.output_width;
    image->height = cinfo.output_height;
    image->data = malloc(image->width * image->height * 3);
    ptr = image->data;
    if (cinfo.rec_outbuf_height > 16) {
      fprintf(stderr, "Error: JPEG uses line buffers > 16. Cannot load.\n");
      fclose(f);
      return false;
    }
    int i, y;
    if (cinfo.output_components == 3) {
      for (y = 0; y < image->height; y += cinfo.rec_outbuf_height) {
        for (i = 0; i < cinfo.rec_outbuf_height; ++i) {
          line[i] = ptr;
          ptr += image->width * 3;
        }
        jpeg_read_scanlines(&cinfo, line, cinfo.rec_outbuf_height);
      }
    } else if (cinfo.output_components == 1) {
      for (i = 0; i < cinfo.rec_outbuf_height; ++i) {
        if ((line[i] = (unsigned char *)malloc(image->width)) == NULL) {
          int t;
          for (t = 0; t < i; ++t)
            free(line[t]);
          jpeg_destroy_decompress(&cinfo);
          return false;
        }
      }
      for (y = 0; y < image->height; y += cinfo.rec_outbuf_height) {
        jpeg_read_scanlines(&cinfo, line, cinfo.rec_outbuf_height);
        for (i = 0; i < cinfo.rec_outbuf_height; ++i) {
          int x;
          for (x = 0; x < image->width; ++x) {
            *ptr++ = line[i][x];
            *ptr++ = line[i][x];
            *ptr++ = line[i][x];
          }
        }
      }
      for (i = 0; i < cinfo.rec_outbuf_height; ++i)
        free(line[i]);
    }
    jpeg_finish_decompress(&cinfo);
    jpeg_destroy_decompress(&cinfo);
    fclose(f);
    image->data_format = G_IMAGE_DATA_FORMAT_RGB;
    return true;
  } else
    return g_image_file_not_found(filename, image);
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
  else
    return G_IMAGE_NONE;
}

void g_image_delete(GImage *image) {
  free(image->data);
  free(image);
}

static int g_image_png_save(GImage *img, const char *filename) {
  png_color_8 sig_bit;
  const unsigned short w = img->width;
  const unsigned short h = img->height;
  png_byte **image = malloc(sizeof(png_byte *) * h);
  FILE *fd = fopen(filename, "wb");
  const int channel = img->data_format == G_IMAGE_DATA_FORMAT_RGB ? 3 : 4;
  if (fd) {
    png_structp png_ptr = png_create_write_struct(PNG_LIBPNG_VER_STRING, NULL, NULL, NULL);
    png_infop info_ptr = png_create_info_struct(png_ptr);
    int i;
    for (i = 0; i < h; ++i)
      image[i] = &(img->data[(i * w) * channel]);

    if (setjmp(png_jmpbuf(png_ptr))) {
      fprintf(stderr, "Error while writing %s", filename);
      png_destroy_write_struct(&png_ptr, &info_ptr);
      fclose(fd);
      free(image);
      return -1;
    }

    png_init_io(png_ptr, fd);
    png_set_IHDR(png_ptr, info_ptr, w, h, 8, channel == 3 ? PNG_COLOR_TYPE_RGB : PNG_COLOR_TYPE_RGBA, PNG_INTERLACE_NONE,
                 PNG_COMPRESSION_TYPE_BASE, PNG_FILTER_TYPE_BASE);
    sig_bit.red = 8;
    sig_bit.green = 8;
    sig_bit.blue = 8;
    if (channel == 4) {
      sig_bit.alpha = 8;
      png_set_bgr(png_ptr);  // ABGR->ARGB or BGRA->RGBA
      if (img->data_format == G_IMAGE_DATA_FORMAT_ABGR)
        png_set_swap_alpha(png_ptr);  // ARGB -> RGBA
    }
    png_set_sBIT(png_ptr, info_ptr, &sig_bit);
    png_write_info(png_ptr, info_ptr);
    png_write_image(png_ptr, image);
    png_write_end(png_ptr, info_ptr);
    png_destroy_write_struct(&png_ptr, &info_ptr);
    fclose(fd);
    free(image);
    return 0;  // OK
  } else {
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
    free(image);
    return -1;  // error
  }
}

static int g_image_jpeg_save(GImage *img, char quality, bool to_file, const char *filename, unsigned char **target_data,
                             unsigned long *target_data_size) {
  struct jpeg_compress_struct cinfo;
  struct jpeg_error_mgr jerr;
  JSAMPROW row_pointer[1];
  int row_stride;
  unsigned char *data;

  FILE *f = NULL;
  if (to_file) {
    f = fopen(filename, "wb");
    if (f == NULL) {
      fprintf(stderr, "Error: could not open \"%s\" for writing\n", filename);
      return -1;  // error
    }
  }

  if (img->data_format == G_IMAGE_DATA_FORMAT_RGB)
    data = img->data;
  else {
    const int max = img->width * img->height * 3;
    data = (unsigned char *)malloc(max);
    int i, j;
    if (img->data_format == G_IMAGE_DATA_FORMAT_BGRA) {
      for (i = 0, j = 0; i < max; i += 3, j += 4) {
        data[i] = img->data[j + 2];
        data[i + 1] = img->data[j + 1];
        data[i + 2] = img->data[j];
      }
    } else if (img->data_format == G_IMAGE_DATA_FORMAT_ABGR) {
      for (i = 0, j = 0; i < max; i += 3, j += 4) {
        data[i] = img->data[j + 3];
        data[i + 1] = img->data[j + 2];
        data[i + 2] = img->data[j + 1];
      }
    } else {
      printf("unkown image format\n");
      free(data);
      if (to_file)
        fclose(f);
      return -1;  // error
    }
  }
  cinfo.err = jpeg_std_error(&jerr);
  jpeg_create_compress(&cinfo);

  if (to_file)
    jpeg_stdio_dest(&cinfo, f);
  else  // to buffer
    jpeg_mem_dest(&cinfo, target_data, target_data_size);

  cinfo.image_width = img->width;
  cinfo.image_height = img->height;
  cinfo.input_components = 3;
  cinfo.in_color_space = JCS_RGB;
  jpeg_set_defaults(&cinfo);
  jpeg_set_quality(&cinfo, quality, true);
  jpeg_start_compress(&cinfo, true);
  row_stride = cinfo.image_width * 3;
  while (cinfo.next_scanline < cinfo.image_height) {
    row_pointer[0] = data + (cinfo.next_scanline * row_stride);
    jpeg_write_scanlines(&cinfo, row_pointer, 1);
  }
  jpeg_finish_compress(&cinfo);
  jpeg_destroy_compress(&cinfo);
  if (img->data_format != G_IMAGE_DATA_FORMAT_RGB)
    free(data);
  if (to_file)
    fclose(f);
  return 0;
}

static int g_image_tiff_save(GImage *img, const char *filename) {
  TIFF *image;

  image = TIFFOpen(filename, "wb");

  if (image == NULL)
    fprintf(stderr, "Unable to write TIFF file: %s\n", filename);

  TIFFSetField(image, TIFFTAG_SAMPLEFORMAT, SAMPLEFORMAT_IEEEFP);
  TIFFSetField(image, TIFFTAG_SMINSAMPLEVALUE, 0);
  TIFFSetField(image, TIFFTAG_SMAXSAMPLEVALUE, 1);
  TIFFSetField(image, TIFFTAG_IMAGEWIDTH, img->width);
  TIFFSetField(image, TIFFTAG_IMAGELENGTH, img->height);
  TIFFSetField(image, TIFFTAG_SAMPLESPERPIXEL, 1);
  TIFFSetField(image, TIFFTAG_BITSPERSAMPLE, 32);
  TIFFSetField(image, TIFFTAG_ROWSPERSTRIP, img->height);
  TIFFSetField(image, TIFFTAG_ORIENTATION, (int)ORIENTATION_TOPLEFT);
  TIFFSetField(image, TIFFTAG_PLANARCONFIG, PLANARCONFIG_CONTIG);
  TIFFSetField(image, TIFFTAG_COMPRESSION, COMPRESSION_NONE);
  TIFFSetField(image, TIFFTAG_PHOTOMETRIC, 1);

  TIFFWriteEncodedStrip(image, 0, img->float_data, img->width * img->height * sizeof(float));

  TIFFWriteDirectory(image);

  TIFFClose(image);

  return 0;
}

int g_image_save(GImage *img, const char *filename, char quality) {
  switch (g_image_get_type(filename)) {
    case G_IMAGE_JPEG:
      return g_image_jpeg_save(img, quality, true, filename, NULL, NULL);
    case G_IMAGE_PNG:
      return g_image_png_save(img, filename);
    case G_IMAGE_TIFF:
      return g_image_tiff_save(img, filename);
    default:
      fprintf(stderr, "Cannot save: unsupported image type: %s\n", filename);
      return -1;
  }
}

int g_image_save_to_jpeg_buffer(GImage *img, unsigned char **target_data, unsigned long *target_data_size, char quality) {
  return g_image_jpeg_save(img, quality, false, NULL, target_data, target_data_size);
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
