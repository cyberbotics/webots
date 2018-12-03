/*! \file
 * \ingroup camera1
 * \brief PO3030k library header (two timers)
 * \author Philippe Rétornaz
 */

/*! \defgroup camera1 Camera fast two timers
 *
 * \section intro_sec Introduction
 *
 * This driver expose most of the Po3030k camera interfaces. Some functions
 * are usefull, some other not. But since this is not up to the driver to
 * decide if a function is needed, I've exported almost all.
 *
 * The architecture is quite simple. The driver keep a array where
 * every known camera register is keept in memory. The configuration function
 * only alter this array. When you call, for example e_po3030k_config_cam(), nothing
 * is written on the camera, but only in the internal register representation.
 *
 * To effectively write the register, you must call e_po3030k_write_cam_registers().
 * This is typically done after every configuration call and before acquire
 * the first picture.
 *
 * \section defset Default settings
 * The camera is, by default, configured with the followin settings:
 * - Automatic white balance control
 * - Automatic exposure control
 * - Automatic flicker detection ( 50Hz and 60Hz )
 * .
 * There is no default setting for the image size and color.
 *
 * \section perfsec Performances
 * The maximum framerate ( without doing anything else than acquiring the picture ) vary
 * with the subsampling and the color mode.
 * Here are some framerates:
 * - Size: 640x480, Subsampling: 16x16, RGB565: 4.3 fps
 * - Size: 16x480, Subsampling: 16x16, RGB565: 4.3 fps
 * - Size: 480x16, Subsampling: 16x16: RGB565: 4.3fps
 * - Size: 64x64, Subsampling: 4x4, RGB565: 4.3 fps
 * - Size: 32x32, Subsampling: 2x2, RGB565: 2.6 fps
 * - Size: 16x16, No Subsampling, RGB565: 1.3 fps
 * - Size: 640x480, Subsampling: 16x16, GREYSCALE: 8.6 fps
 * - Size: 16x480, Subsampling: 16x16, GREYSCALE: 8.6 fps
 * - Size: 480x16, Subsampling: 16x16, GREYSCALE: 8.6 fps
 * - Size: 64x64, Subsampling: 4x4, GREYSCALE: 8.6 fps
 * - Size: 32x32, Subsampling: 2x2, GREYSCALE: 4.3 fps
 * - Size: 16x16, No subsampling, GREYSCALE: 2.2 fps
 *
 * \section IntegrDet Important note
 * This driver is extremly sensible to interrupt latency, thus it use interrupt priority to
 * be sure that the latencies are kepts low. The Timer4 and Timer5 interrupt priority are set at
 * level 6 and interrupt nesting is enabled.
 * The Timer4 interrupt use the "push.s" and "pop.s" instructions. You should not have any
 * code using thoses two instructions when you use the camera. This include the _ISRFAST C
 * macro. If you use them, some random and really hard to debug behavior will happen.
 * You have been warned !
 *
 * \section example_sect Examples
 *
 * \subsection ex1_sect Basic example
 *
 * \code
#include "e_po3030k.h"

char buffer[2*40*40];
int main(void) {

        e_po3030k_init_cam();
        e_po3030k_config_cam((ARRAY_WIDTH -160)/2,(ARRAY_HEIGHT-160)/2,
                         160,160,4,4,RGB_565_MODE);
        e_po3030k_write_cam_registers();

        e_po3030k_launch_capture(buffer);
        while(!e_po3030k_is_img_ready());

        // buffer contain a 40*40 RGB picture now
        ( insert usefull code here )

        return 0;
}
\endcode

 * This example tell de driver to aquire 160x160 pixel picture from the camera
 * 4x subsampling, thus resulting with a 40x40 pixel. The buffer as a size of
 * 40*40*2 because RGB565 is a two bytes per pixel data format.
 *
 * \subsection ex2_sect More advanced example
\code
#include "e_po3030k.h"

char buffer[160*2];
int main(void) {
        e_po3030k_init_cam();
        e_po3030k_config_cam((ARRAY_WIDTH - 320)/2,(ARRAY_HEIGHT - 32)/2,
                        320,8,2,4,GREY_SCALE_MODE);
        e_po3030k_set_mirror(1,1);
        e_po3030ke_set_ref_exposure(100);

        e_po3030k_write_cam_registers();

        e_po3030k_launch_capture(buffer);
        while(!e_po3030k_is_img_ready());

        // Here buffer contain a 160x2 greyscale picture

        return 0;
}
\endcode
 * This example configure the camera to aquire a 320x8 pixel picture, but subsampled
 * 2x in width and 4x in heigth, thus resulting in a 160*2 linear
 * greyscale picture. It "emulate" a linear camera. This example tell the camera to
 * to enable the vertical and horizontal mirror, and to set the average exposure to
 * 100.
 */

#ifndef __PO3030K_H__
#define __PO3030K_H__

#include "e_poxxxx.h"

/*! If you set this at 0, you save about 168 bytes of memory
 * But you loose all advanced camera functions */
#define PO3030K_FULL 1

#define MODE_VGA 			0x44
#define MODE_QVGA 			0x11
#define MODE_QQVGA 			0x33

#define SPEED_2				0x00
#define SPEED_2_3			0x10
#define SPEED_4				0x20
#define SPEED_8				0x30
#define SPEED_16			0x40
#define SPEED_32			0x50
#define SPEED_64			0x60
#define SPEED_128			0x70

int e_po3030k_config_cam(unsigned int sensor_x1,unsigned int sensor_y1,
			 unsigned int sensor_width,unsigned int sensor_height,
			 unsigned int zoom_fact_width,unsigned int zoom_fact_height,
			 int color_mode);

int  e_po3030k_get_bytes_per_pixel(int color_mode);

void e_po3030k_init_cam(void);

void e_po3030k_write_cam_registers(void);

int  e_po3030k_set_color_mode(int mode);

int  e_po3030k_set_sampling_mode(int mode);

int  e_po3030k_set_speed(int mode);

int  e_po3030k_set_wx(unsigned int start, unsigned int stop);

int  e_po3030k_set_wy(unsigned int start, unsigned int stop);

int  e_po3030k_set_vsync(unsigned int start,unsigned int stop,unsigned int col);

void e_po3030k_set_mirror(int vertical, int horizontal);

#if PO3030K_FULL

void e_po3030k_read_cam_registers(void);

int  e_po3030k_set_register(unsigned char adr,unsigned char value);

int  e_po3030k_get_register(unsigned char adr,unsigned char * value);

void e_po3030k_set_bias(unsigned char pixbias, unsigned char opbias);

void e_po3030k_set_integr_time(unsigned long time);

void e_po3030k_set_adc_offset(unsigned char offset);

void e_po3030k_set_sepia(int status);

void e_po3030k_set_lens_gain(unsigned char red, unsigned char green, unsigned char blue);

void e_po3030k_set_edge_prop(unsigned char gain, unsigned char tresh);

void e_po3030k_set_gamma_coef(unsigned char array[12], char color);

void e_po3030k_write_gamma_coef(void);

int  e_po3030k_sync_register_array(unsigned char start, unsigned char stop);

void e_po3030k_set_color_matrix(unsigned char array[3*3]);

void e_po3030k_set_cb_cr_gain(unsigned char cg11c, unsigned char cg22c);

void e_po3030k_set_brigh_contr(unsigned char bright, unsigned char contrast);

void e_po3030k_set_sepia_tone(unsigned char cb, unsigned char cr);

void e_po3030k_set_ww(unsigned char ww);

void e_po3030k_set_awb_ae_tol(unsigned char awbm, unsigned char aem);

void e_po3030k_set_ae_speed(unsigned char b, unsigned char d);

void e_po3030k_set_exposure(long t);

void e_po3030k_set_ref_exposure(unsigned char exp);

void e_po3030k_set_max_min_exp(unsigned int min, unsigned int max);

void e_po3030k_set_max_min_awb(unsigned char minb, unsigned char maxb, unsigned char minr,
					unsigned char maxr, unsigned char ratior, unsigned char ratiob);

int  e_po3030k_set_weight_win(unsigned int x1, unsigned int x2, unsigned int y1, unsigned int y2);

void e_po3030k_set_awb_ae(int awb, int ae);

int  e_po3030k_set_color_gain(unsigned char global, unsigned char red,
						unsigned char green1, unsigned char green2,
						unsigned char blue);

void e_po3030k_set_flicker_mode(int manual);

void e_po3030k_set_flicker_detection(int hz50, int hz60);

int  e_po3030k_set_flicker_man_set(int hz50, int hz60, int fdm, int fk, int tol);

#endif

#endif
