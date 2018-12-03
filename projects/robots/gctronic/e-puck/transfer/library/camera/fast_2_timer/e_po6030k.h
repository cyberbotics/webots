/*! \file
 * \ingroup camera1
 * \brief PO6030k library header
 * \author Philippe Rétornaz
 */

#ifndef __PO6030K_H__
#define __PO6030K_H__

#include "e_poxxxx.h"

#define PO_6030_MODE_VGA 	0x20
#define PO_6030_MODE_QVGA 	0x40
#define PO_6030_MODE_QQVGA 	0x80

#define BAYER_CLOCK_1 0x10
#define BAYER_CLOCK_2 0x50
#define BAYER_CLOCK_4 0x90
#define BAYER_CLOCK_8 0xB0

#define BANK_A 0x0
#define BANK_B 0x1
#define BANK_C 0x2
#define BANK_D 0x3

#define PO_6030_SPEED_1 BAYER_CLOCK_1
#define PO_6030_SPEED_2 BAYER_CLOCK_2
#define PO_6030_SPEED_4 BAYER_CLOCK_4
#define PO_6030_SPEED_8 BAYER_CLOCK_8


int e_po6030k_config_cam(unsigned int sensor_x1,unsigned int sensor_y1,
			 unsigned int sensor_width,unsigned int sensor_height,
			 unsigned int zoom_fact_width,unsigned int zoom_fact_height,
			 int color_mode);

int  e_po6030k_get_bytes_per_pixel(int color_mode);

void e_po6030k_set_color_mode(unsigned char format);

void e_po6030k_set_bank(unsigned char bank);
void e_po6030k_write_register(unsigned char bank, unsigned char reg, unsigned char value);
#define e_po6030k_set_speed(div) e_po6030k_set_bayer_clkdiv(div)
void e_po6030k_set_bayer_clkdiv(unsigned char div);

void e_po6030k_set_pclkdiv(unsigned char div);
int e_po6030k_set_mode(unsigned char format, unsigned char sampl_mode);


int e_po6030k_get_bytes_per_pixel(int color_mode);
int e_po6030k_set_wx(unsigned int start,unsigned int stop);

int e_po6030k_set_wy(unsigned int start, unsigned int stop);

int e_po6030k_set_vsync(unsigned int start,unsigned int stop);

#define E_PO6030K_SKETCH_BW 0
#define E_PO6030K_SKETCH_COLOR 1
void e_po6030k_set_sketch_mode(int mode);

void e_po6030k_set_mirror(int vertical, int horizontal);

void e_po6030k_set_awb_ae(int awb, int ae);
void e_po6030k_set_rgb_gain(unsigned char r, unsigned char g, unsigned char b);
void e_po6030k_set_exposure(unsigned long exp);

#endif
