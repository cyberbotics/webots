/*! \file
 * \ingroup camera1
 * \brief Calculate the timing for the camera (two timers)
 * \author Philippe Rétornaz
 */

#include "e_poxxxx.h"
#include "e_po3030k.h"
#include "../../motor_led/e_epuck_ports.h"
#include "../../I2C/e_I2C_protocol.h"
#include "../../motor_led/e_init_port.h"

#define 	ARRAY_ORIGINE_X	210
#define		ARRAY_ORIGINE_Y 7

#define IRQ_PIX_LAT 1

/*! This function setup the internal timing of the camera to match the
 * zoom, color mode and interest area.
 * \warning If the common deniminator of both zoom factor is 4 or 2, part of the
 * subsampling is done by the camera ( QQVGA = 4, QVGA = 2 ). This increase the
 * framerate by respectivly 4 or 2. Moreover greyscale is twice faster than color mode.
 * \param sensor_x1 The X coordinate of the window's corner
 * \param sensor_y1 The Y coordinate of the window's corner
 * \param sensor_width the Width of the interest area, in FULL sampling scale
 * \param sensor_height The Height of the insterest area, in FULL sampling scale
 * \param zoom_fact_width The subsampling to apply for the window's Width
 * \param zoom_fact_height The subsampling to apply for the window's Height
 * \param color_mode The color mode in which the camera should be configured
 * \return Zero if the settings are correct, non-zero if an error occur
 * \sa e_po3030k_write_cam_registers
 */

int e_po3030k_config_cam(unsigned int sensor_x1,unsigned int sensor_y1,
				unsigned int sensor_width,unsigned int sensor_height,
				unsigned int zoom_fact_width,unsigned int zoom_fact_height,
				int color_mode) {
	int pbp_h, pbp_w;
	int nb_pixels, nb_lines;
	int real_zoom_h, real_zoom_w;
	int sampl_mode;

	sensor_x1 += ARRAY_ORIGINE_X;
	sensor_y1 += ARRAY_ORIGINE_Y;
	/* testing if the mode is acceptable */
	if(zoom_fact_height < 1 || zoom_fact_width < 1)
		return -1;

	/* Check if the area is out of bound */
	if((sensor_x1 + sensor_width) > (ARRAY_ORIGINE_X + ARRAY_WIDTH))
		return -1;
	if((sensor_y1 + sensor_height) > (ARRAY_ORIGINE_Y + ARRAY_HEIGHT))
		return -1;

	/* Check if Sensor[Width|Height] is a multiple of Zoom */
	if(sensor_width % zoom_fact_width)
		return -1;
	if(sensor_height % zoom_fact_height)
		return -1;

	/* Search the best subsampling aviable */
	if(!(zoom_fact_height%4)) {
		if(!(zoom_fact_width%4)) {
			sampl_mode = MODE_QQVGA;
			real_zoom_h = zoom_fact_height >> 2;
			real_zoom_w = zoom_fact_width >> 2;
			/* this is done because we must have Vsync a real
			 * row before the real picture */
			sensor_y1 -= 4;
			/* We need Hsync some time before the first effective pixel */
			sensor_x1 -= IRQ_PIX_LAT*4;
		} else {
			if(!(zoom_fact_width%2)) {
				sampl_mode = MODE_QVGA;
				real_zoom_h = zoom_fact_height >> 1;
				real_zoom_w = zoom_fact_width >> 1;
				sensor_y1 -= 2;
				sensor_x1 -= 2*IRQ_PIX_LAT;
			} else {
				sampl_mode = MODE_VGA;
				real_zoom_h = zoom_fact_height;
				real_zoom_w = zoom_fact_width;
				sensor_y1--;
				sensor_x1 -= IRQ_PIX_LAT;
			}
		}
	} else if(!(zoom_fact_height%2)) {
		if(!(zoom_fact_width%2)) {
			sampl_mode = MODE_QVGA;
			real_zoom_h = zoom_fact_height >> 1;
			real_zoom_w = zoom_fact_width >> 1;
			sensor_y1 -= 2;
			sensor_x1 -= 2*IRQ_PIX_LAT;
		} else {
			sampl_mode = MODE_VGA;
			real_zoom_h = zoom_fact_height;
			real_zoom_w = zoom_fact_width;
			sensor_y1--;
			sensor_x1 -= IRQ_PIX_LAT;
		}
	} else {
		sampl_mode = MODE_VGA;
		real_zoom_w = zoom_fact_width;
		real_zoom_h = zoom_fact_height;
		sensor_y1--;
		sensor_x1 -= IRQ_PIX_LAT;
	}
	pbp_w = real_zoom_w - 1;
	pbp_h = real_zoom_h - 1;

	nb_pixels = sensor_width / zoom_fact_width;
	nb_lines = sensor_height/ zoom_fact_height;

	/* set camera configuration */
	if(e_po3030k_set_color_mode(color_mode))
		return -1;

	if(e_po3030k_set_sampling_mode(sampl_mode))
		return -1;

	if(color_mode == GREY_SCALE_MODE) {
		switch (sampl_mode) {
			case MODE_VGA:
				e_po3030k_set_speed(SPEED_8);
				break;
			case MODE_QVGA:
				e_po3030k_set_speed(SPEED_4);
				break;
			case MODE_QQVGA:
				e_po3030k_set_speed(SPEED_2);
				break;
		}
	} else {
		switch (sampl_mode) {
			case MODE_VGA:
				e_po3030k_set_speed(SPEED_16);
				break;
			case MODE_QVGA:
				e_po3030k_set_speed(SPEED_8);
				break;
			case MODE_QQVGA:
				e_po3030k_set_speed(SPEED_4);
				break;
			}
	}

	if(e_po3030k_set_wx(sensor_x1,ARRAY_ORIGINE_X + ARRAY_WIDTH + 1))
		return -1;

	if(e_po3030k_set_wy(sensor_y1,ARRAY_ORIGINE_Y + ARRAY_HEIGHT + 1))
		return -1;

	if(e_po3030k_set_vsync(sensor_y1,ARRAY_ORIGINE_Y + ARRAY_HEIGHT, sensor_x1 + 1))
		return -1;

	/* set timer configuration */
	if(e_poxxxx_apply_timer_config(nb_lines,nb_pixels,e_po3030k_get_bytes_per_pixel(color_mode),pbp_w,pbp_h))
		return -1;

	return 0;

}

/*! Return the number of bytes per pixel in the given color mode
 * \param color_mode The given color mode
 * \return The number of bytes per pixel in the given color mode
 */
int e_po3030k_get_bytes_per_pixel(int color_mode) {
	switch (color_mode) {
		case GREY_SCALE_MODE:
			return 1;
		case RGB_565_MODE:
			return 2;
		case YUV_MODE:
			return 2; /* YUV mode is not the best mode for subsampling */
	}
	/* UNKNOW ! */
	return 1;
}
