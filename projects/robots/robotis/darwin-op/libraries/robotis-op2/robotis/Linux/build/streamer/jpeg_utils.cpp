/*
 * jpeg_utils.cpp
 *
 *  Created on: 2011. 1. 5.
 *      Author: zerom
 */

#include <stdio.h>
#include <stdlib.h>

#include <string.h>

#include "jpeg_utils.h"

#define OUTPUT_BUF_SIZE  4096

/******************************************************************************
Description.:
Input Value.:
Return Value:
******************************************************************************/
void jpeg_utils::init_destination(j_compress_ptr cinfo) {
  mjpg_dest_ptr dest = (mjpg_dest_ptr) cinfo->dest;

  /* Allocate the output buffer --- it will be released when done with image */
  dest->buffer = (JOCTET *)(*cinfo->mem->alloc_small) ((j_common_ptr) cinfo, JPOOL_IMAGE, OUTPUT_BUF_SIZE * sizeof(JOCTET));

  *(dest->written) = 0;

  dest->pub.next_output_byte = dest->buffer;
  dest->pub.free_in_buffer = OUTPUT_BUF_SIZE;
}

/******************************************************************************
Description.: called whenever local jpeg buffer fills up
Input Value.:
Return Value:
******************************************************************************/
boolean jpeg_utils::empty_output_buffer(j_compress_ptr cinfo) {
  mjpg_dest_ptr dest = (mjpg_dest_ptr) cinfo->dest;

  memcpy(dest->outbuffer_cursor, dest->buffer, OUTPUT_BUF_SIZE);
  dest->outbuffer_cursor += OUTPUT_BUF_SIZE;
  *(dest->written) += OUTPUT_BUF_SIZE;

  dest->pub.next_output_byte = dest->buffer;
  dest->pub.free_in_buffer = OUTPUT_BUF_SIZE;

  return TRUE;
}

/******************************************************************************
Description.: called by jpeg_finish_compress after all data has been written.
              Usually needs to flush buffer.
Input Value.:
Return Value:
******************************************************************************/
void jpeg_utils::term_destination(j_compress_ptr cinfo) {
  mjpg_dest_ptr dest = (mjpg_dest_ptr) cinfo->dest;
  size_t datacount = OUTPUT_BUF_SIZE - dest->pub.free_in_buffer;

  /* Write any data remaining in the buffer */
  memcpy(dest->outbuffer_cursor, dest->buffer, datacount);
  dest->outbuffer_cursor += datacount;
  *(dest->written) += datacount;
}

/******************************************************************************
Description.: Prepare for output to a stdio stream.
Input Value.: buffer is the already allocated buffer memory that will hold
              the compressed picture. "size" is the size in bytes.
Return Value: -
******************************************************************************/
void jpeg_utils::dest_buffer(j_compress_ptr cinfo, unsigned char *buffer, int size, int *written) {
  mjpg_dest_ptr dest;

  if (cinfo->dest == NULL) {
    cinfo->dest = (struct jpeg_destination_mgr *)(*cinfo->mem->alloc_small) ((j_common_ptr) cinfo, JPOOL_PERMANENT, sizeof(mjpg_destination_mgr));
  }

  dest = (mjpg_dest_ptr) cinfo->dest;
  dest->pub.init_destination = init_destination;
  dest->pub.empty_output_buffer = empty_output_buffer;
  dest->pub.term_destination = term_destination;
  dest->outbuffer = buffer;
  dest->outbuffer_size = size;
  dest->outbuffer_cursor = buffer;
  dest->written = written;
}

int jpeg_utils::compress_yuyv_to_jpeg(Image *src, unsigned char* buffer, int size, int quality)
{
    struct jpeg_compress_struct cinfo;
    struct jpeg_error_mgr jerr;
    JSAMPROW row_pointer[1];
    unsigned char *line_buffer, *yuyv;
    int z;
    static int written;

    line_buffer = (unsigned char*)calloc (src->m_Width * 3, 1);
    yuyv = src->m_ImageData;

    cinfo.err = jpeg_std_error (&jerr);
    jpeg_create_compress (&cinfo);
    /* jpeg_stdio_dest (&cinfo, file); */
    dest_buffer(&cinfo, buffer, size, &written);

    cinfo.image_width = src->m_Width;
    cinfo.image_height = src->m_Height;
    cinfo.input_components = 3;
    cinfo.in_color_space = JCS_RGB;

    jpeg_set_defaults (&cinfo);
    jpeg_set_quality (&cinfo, quality, TRUE);

    jpeg_start_compress (&cinfo, TRUE);

    z = 0;
    while (cinfo.next_scanline < src->m_Height) {
        int x;
        unsigned char *ptr = line_buffer;

        for (x = 0; x < src->m_Width; x++) {
            int r, g, b;
            int y, u, v;

            if (!z)
                y = yuyv[0] << 8;
            else
                y = yuyv[2] << 8;
            u = yuyv[1] - 128;
            v = yuyv[3] - 128;

            r = (y + (359 * v)) >> 8;
            g = (y - (88 * u) - (183 * v)) >> 8;
            b = (y + (454 * u)) >> 8;

            *(ptr++) = (r > 255) ? 255 : ((r < 0) ? 0 : r);
            *(ptr++) = (g > 255) ? 255 : ((g < 0) ? 0 : g);
            *(ptr++) = (b > 255) ? 255 : ((b < 0) ? 0 : b);

            if (z++) {
                z = 0;
                yuyv += 4;
            }
        }

        row_pointer[0] = line_buffer;
        jpeg_write_scanlines (&cinfo, row_pointer, 1);
    }

    jpeg_finish_compress (&cinfo);
    jpeg_destroy_compress (&cinfo);

    free (line_buffer);

    return (written);
}

int jpeg_utils::compress_rgb_to_jpeg(Image *src, unsigned char* buffer, int size, int quality)
{
    struct jpeg_compress_struct cinfo;
    struct jpeg_error_mgr jerr;
    JSAMPROW row_pointer[1];
    unsigned char *line_buffer, *rgb;
    int z;
    static int written;

    line_buffer = (unsigned char*)calloc (src->m_Width * 3, 1);
    rgb = src->m_ImageData;

    cinfo.err = jpeg_std_error (&jerr);
    jpeg_create_compress (&cinfo);
    /* jpeg_stdio_dest (&cinfo, file); */
    dest_buffer(&cinfo, buffer, size, &written);

    cinfo.image_width = src->m_Width;
    cinfo.image_height = src->m_Height;
    cinfo.input_components = 3;
    cinfo.in_color_space = JCS_RGB;

    jpeg_set_defaults (&cinfo);
    jpeg_set_quality (&cinfo, quality, TRUE);

    jpeg_start_compress (&cinfo, TRUE);

    z = 0;
    while (cinfo.next_scanline < src->m_Height) {
        int x;
        unsigned char *ptr = line_buffer;

        for (x = 0; x < src->m_Width; x++) {
            int r, g, b;

            r = rgb[0];
            g = rgb[1];
            b = rgb[2];

            *(ptr++) = (r > 255) ? 255 : ((r < 0) ? 0 : r);
            *(ptr++) = (g > 255) ? 255 : ((g < 0) ? 0 : g);
            *(ptr++) = (b > 255) ? 255 : ((b < 0) ? 0 : b);

            rgb += 3;
        }

        row_pointer[0] = line_buffer;
        jpeg_write_scanlines (&cinfo, row_pointer, 1);
    }

    jpeg_finish_compress (&cinfo);
    jpeg_destroy_compress (&cinfo);

    free (line_buffer);

    return (written);
}
