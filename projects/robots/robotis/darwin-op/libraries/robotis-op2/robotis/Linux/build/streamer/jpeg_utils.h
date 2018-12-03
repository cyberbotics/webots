/*
 * jpeg_utils.h
 *
 *  Created on: 2011. 1. 5.
 *      Author: zerom
 */

#ifndef JPEG_UTILS_H_
#define JPEG_UTILS_H_

#include <jpeglib.h>

#include "Image.h"

using namespace Robot;

class jpeg_utils
{
private:
    typedef struct {
      struct jpeg_destination_mgr pub; /* public fields */

      JOCTET * buffer;    /* start of buffer */

      unsigned char *outbuffer;
      int outbuffer_size;
      unsigned char *outbuffer_cursor;
      int *written;

    } mjpg_destination_mgr;

    typedef mjpg_destination_mgr * mjpg_dest_ptr;

    static void init_destination(j_compress_ptr cinfo);
    static boolean empty_output_buffer(j_compress_ptr cinfo);
    static void term_destination(j_compress_ptr cinfo);
    static void dest_buffer(j_compress_ptr cinfo, unsigned char *buffer, int size, int *written);

public:
    static int compress_yuyv_to_jpeg(Image *src, unsigned char* buffer, int size, int quality);
    static int compress_rgb_to_jpeg(Image *src, unsigned char* buffer, int size, int quality);
};

#endif /* JPEG_UTILS_H_ */
