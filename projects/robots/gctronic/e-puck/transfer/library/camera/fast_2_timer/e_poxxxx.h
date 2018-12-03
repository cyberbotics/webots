/*! \file
 * \brief Camera library header
 * \author Philippe Rétornaz
 */

/*! \defgroup camera1 Camera fast two timers
 *
 * \section intro_sec Introduction
 *
 * This driver expose the subset features bewteen the po6030k and po3030k camera interfaces.
 *
 *
 */

#ifndef __POXXXX_H__
#define __POXXXX_H__

/*! If you set this at 0, you save about 168 bytes of memory
 * But you loose all advanced camera functions */
#define POXXXX_FULL 1

#define	ARRAY_WIDTH			640
#define	ARRAY_HEIGHT		480

#define GREY_SCALE_MODE		0
#define RGB_565_MODE		1
#define YUV_MODE			2

int e_poxxxx_config_cam(unsigned int sensor_x1,unsigned int sensor_y1,
			 unsigned int sensor_width,unsigned int sensor_height,
			 unsigned int zoom_fact_width,unsigned int zoom_fact_height,
			 int color_mode);

int e_poxxxx_init_cam(void);
int e_poxxxx_get_orientation(void);

void e_poxxxx_write_cam_registers(void);

void e_poxxxx_launch_capture(char * buf);

int  e_poxxxx_is_img_ready(void);

void e_poxxxx_set_mirror(int vertical, int horizontal);

int e_poxxxx_apply_timer_config(int pixel_row, int pixel_col, int bpp, int pbp, int bbl);

void e_poxxxx_set_awb_ae(int awb, int ae);
void e_poxxxx_set_rgb_gain(unsigned char r, unsigned char g, unsigned char b);
void e_poxxxx_set_exposure(unsigned long exp);

#endif
