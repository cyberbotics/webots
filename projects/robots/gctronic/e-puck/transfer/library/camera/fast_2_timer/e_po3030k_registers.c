
/*! \file
 * \brief Manage po3030k registers (two timers)
 * \author Philippe Rétornaz
 */



#include "../../I2C/e_I2C_protocol.h"
#include "e_po3030k.h"

#define MCLK				((long) 14745600)  /* 14.7456Mhz */
#define MCLK_P_NS			0.067816840278 /* Master clock period in ns */


/* these two define should go into a private header but, well,
 * I won't do it one for only two define ... */
#define 	ARRAY_ORIGINE_X	210
#define		ARRAY_ORIGINE_Y 7

/* Some preprocessor ugliness */
#if PO3030K_FULL
#define BASE_D1 0
#define BASE_D2 0
#define BASE_D3 0
#define BASE_D4 0
#else
#define BASE_D1 20
#define BASE_D2 (BASE_D1 + 4)
#define BASE_D3 (BASE_D2 + 8)
#define BASE_D4 (BASE_D3 + 76)
#endif

#define DEVICE_ID  			0xDC

static unsigned char cam_reg[] = {
/* format:
   address,   value */
#define FRAME_WIDTH 0x0353
	0x04,		(unsigned char) (FRAME_WIDTH >> 8),
	0x05,		(unsigned char) FRAME_WIDTH,
#define FRAME_HEIGTH 0x01e8
	0x06,		(unsigned char) (FRAME_WIDTH >> 8),
	0x07,		(unsigned char) FRAME_WIDTH,
#define WINDOW_X1_BASE 9
	0x08,		0x00,
	0x09,		0xd1,
#define WINDOW_Y1_BASE 13
	0x0a,		0x00,
	0x0b,		0x07,
#define WINDOW_X2_BASE 17
	0x0c,		0x03,
	0x0d,		0x51,
#define WINDOW_Y2_BASE 21
	0x0e,		0x01,
	0x0f,		0xe7,

#if PO3030K_FULL
	#define BIAS_BASE 25
		0x12,		0x02,
		0x13,		0x02,
	#define COLGAIN_BASE 29
		0x15,		0x10,
		0x16,		0x40,
		0x17,		0x40,
		0x18,		0x40,
		0x19,		0x40,
	#define INTEGR_BASE 39
		0x1a,		0x03,
		0x1b,		0xe6,
		0x1c,		0x00,
#endif

#define SPEED_ADDR (45 - BASE_D1)
	0x1d,		/*0x00*/ 0b01110000,

#define MIRROR_BASE 47
		0x1e,		0x0a,
		0x1f,		0x1b,


#define SAMPLING_ADDR ( 51 - BASE_D2 )
	0x20,		MODE_VGA,
#define ADCOFF_BASE ( 53 - BASE_D2 )
	0x3b,		0x00,
#define FLICKM_BASE ( 55 - BASE_D2 )
	0x46,		0x80,
/* see datasheet p. 30 */
	0x48,		(unsigned char) (1667./(MCLK_P_NS*256)),

#if PO3030K_FULL
	#define FLICKP_BASE 59
		0x49,		0x96,
		0x4a,		0x00,
		0x4b,		0x7d,
		0x4c,		0x00,
#endif
#define COLOR_M_ADDR (67 - BASE_D3)
#define MODE_R5G6B5 0x08
#define MODE_YUV 0x02
#define MODE_GRAYSCALE 0x0c
	0x4e,		MODE_R5G6B5,

#if PO3030K_FULL
	#define SEPIA_BASE 69
		0x4f,		0x0a,
		0x50,		0x70,
	#define LENSG_BASE 73
		0x59,		0x00,
		0x5a,		0x00,
		0x5b,		0x00,
	#define EDGE_BASE 79
		0x5c,		0xac,
		0x5d,		0x02,
	#define GAMMA_BASE 83
		0x76,		0x00,
		0x77,		0x1a,
		0x78,		0x2a,
		0x79,		0x37,
		0x7a,		0x42,
		0x7b,		0x56,
		0x7c,		0x68,
		0x7d,		0x87,
		0x7e,		0xa3,
		0x7f,		0xbc,
		0x80,		0xd4,
		0x81,		0xea,
	#define COLOR_COEF_BASE 107
		0x8e,		0x38,
		0x8f,		0xa5,
		0x90,		0x0d,
		0x91,		0x93,
		0x92,		0x2d,
		0x93,		0x06,
		0x94,		0x83,
		0x95,		0xaa,
		0x96,		0x4d,
	#define CBCRGAIN_BASE 125
		0x97,		0x25,
		0x98,		0x25,
	#define BRICTR_BASE 129
		0x9b,		0x00,
		0x9c,		0x96,
	#define SEPIATONE_BASE 133
		0x9f,		0x80,
		0xa0,		0x80,
		0xa1,		0xb6,
		0xa2,		0x9d,
		0xa3,		0xab,
		0xa4,		0x80,
#endif

#define VSYNCSTART_BASE (145 - BASE_D4)
	0xa5,		0x00,
	0xa6,		0x07,
#define VSYNCSTOP_BASE (149 - BASE_D4)
	0xa7,		0x01,
	0xa8,		0xe7,
#define VSYNCCOL_BASE (153 - BASE_D4)
	0xa9,		0x00,
	0xaa,		0x01,

#if PO3030K_FULL
	#define WW_BASE 157
		0xad,		0x9f,
	#define AWVAETOL_BASE 159
		0xae,		0x22,
	#define AESPEED_BASE 161
		0xaf,		0x8c,
	#define EXPOSURE_BASE 163
		0xb0,		0x07,
		0xb1,		0xcc,
		0xb2,		0x00,
	#define REFREPO_BASE 169
		0xb3,		0x70,
		0xb9,		0x03,
		0xba,		0xe6,
	#define MINMAXEXP_BASE 175
		0xbb,		0x07,
		0xbc,		0xcc,
		0xbd,		0x00,
		0xbe,		0x0c,
	#define MINMAXAWB_BASE 183
		0xc1,		0x10,
		0xc2,		0xe0,
		0xc3,		0x10,
		0xc4,		0xe0,
		0xc5,		0x80,
		0xc6,		0x80,
	#define WEIGHWIN_BASE 195
		0xc7,		0x01,
		0xc8,		0xa5,
		0xc9,		0x02,
		0xca,		0x7a,
		0xcb,		0x00,
		0xcc,		0xa7,
		0xcd,		0x01,
		0xce,		0x47,
	#define AWBAEENABLE_BASE 211
		0xd4,		0x3c,
		0xd5,		0x01,
	#define GAMMASELCOL_BASE 215
		0xd8,		0x80
#endif
};
#define NB_REGISTERS (sizeof(cam_reg)/(2*sizeof(cam_reg[0])))


/*! The Po3030k module keep in memory the state of each register
 * the camera has. When you configure the camera, it only alter
 * the internal register state, not the camera one.
 * This function write the internal register state in the camera.
 * \sa e_po3030k_read_cam_registers
 */
void e_po3030k_write_cam_registers(void) {
	int i;
	e_i2cp_enable();
	for(i=0;i < 2*NB_REGISTERS; i+=2 )
			e_i2cp_write(DEVICE_ID,cam_reg[i],cam_reg[i+1]);

	e_i2cp_disable();
}

/*! Read the camera register
 * \sa e_po3030k_write_cam_registers
 */
void e_po3030k_read_cam_registers(void) {
	int i;
	e_i2cp_enable();
	for(i=0;i < 2*NB_REGISTERS; i+=2 )
		cam_reg[i+1] = e_i2cp_read(DEVICE_ID,cam_reg[i]);
	e_i2cp_disable();
}

/*! Set the camera color mode
 * \warning This is an internal function, use e_po3030k_config_cam
 * \param mode The color mode
 * \return Zero if OK, non-zero if an error occur
 * \sa Datasheet p. 31, e_po3030k_write_cam_registers and e_po3030k_config_cam
 */
int e_po3030k_set_color_mode(int mode) {
	switch (mode) {
		case GREY_SCALE_MODE:
			cam_reg[COLOR_M_ADDR] = MODE_GRAYSCALE;
			break;
		case RGB_565_MODE:
			cam_reg[COLOR_M_ADDR] = MODE_R5G6B5;
			break;
		case YUV_MODE:
			cam_reg[COLOR_M_ADDR] = MODE_YUV;
			break;
		default:
			return -1;
	}
	return 0;
}

/*! Set the camera sampling mode
 * \warning This is an internal function, use e_po3030k_config_cam
 * \param mode The given sampling mode
 * \return Zero if OK, non-zero if an error occur
 * \sa Datasheet p. 28 and e_po3030k_config_cam
 */
int e_po3030k_set_sampling_mode(int mode) {
	if(mode == MODE_VGA || mode == MODE_QVGA ||
		mode == MODE_QQVGA )
	{
		cam_reg[SAMPLING_ADDR] = mode;
		return 0;
	}
	return -1;
}

/*! Set the camera speed
 * \warning This is an internal function, use e_po3030k_config_cam
 * \param mode The given speed
 * \return Zero if OK, non-zero if unknow mode
 * \sa Datasheet p. 26 and e_po3030k_config_cam
 */

int e_po3030k_set_speed(int mode) {
	if(mode == SPEED_2 || mode == SPEED_4 ||
		mode == SPEED_8 || mode == SPEED_16 ||
		mode == SPEED_32 || mode == SPEED_64 ||
		mode == SPEED_64 || mode == SPEED_2_3)
	{
		cam_reg[SPEED_ADDR] = mode;
		return 0;
	}

	return -1;
}
/*! Set the camera window X coordinate
 * \warning This is an internal function, use e_po3030k_ConfigCam
 * \param start The start column
 * \param stop The stop column
 * \return Zero if OK, non-zero if an error occur
 * \sa Datasheet p.20-21, e_po3030k_write_cam_registers, e_po3030k_set_wy and e_po3030k_config_cam
 */
int e_po3030k_set_wx(unsigned int start, unsigned int stop) {
	unsigned char start_h,start_l,stop_h,stop_l;
	if(start >= stop)
		return -1;
	if(stop > 899)
		return -1;
	start--;
	start_l = (unsigned char) start;
	start_h = (unsigned char) (start >> 8);
	stop_l = (unsigned char) stop;
	stop_h = (unsigned char) (stop >> 8);

	cam_reg[WINDOW_X1_BASE] = start_h;
	cam_reg[WINDOW_X1_BASE + 2] = start_l;

	cam_reg[WINDOW_X2_BASE] = stop_h;
	cam_reg[WINDOW_X2_BASE + 2] = stop_l;

	return 0;
}

/*! Set the camera window Y coordinate
 * \warning This is an internal function, use e_po3030k_ConfigCam
 * \param start The start row
 * \param stop The stop row
 * \return Zero if OK, non-zero if an error occur
 * \sa Datasheet p.20-21, e_po3030k_WriteCamRegisters, e_po3030k_SetWX and e_po3030k_ConfigCam
 */
int e_po3030k_set_wy(unsigned int start, unsigned int stop) {
	unsigned char start_h,start_l,stop_h,stop_l;
	if(start >= stop)
		return -1;
	if(stop > 499)
		return -1;
	start_l = (unsigned char) start;
	start_h = (unsigned char) (start >> 8);
	stop_l = (unsigned char) stop;
	stop_h = (unsigned char) (stop >> 8);

	cam_reg[WINDOW_Y1_BASE] = start_h;
	cam_reg[WINDOW_Y1_BASE + 2] = start_l;

	cam_reg[WINDOW_Y2_BASE] = stop_h;
	cam_reg[WINDOW_Y2_BASE + 2] = stop_l;

	return 0;
}

/*! Set the camera window VSYNC coordinate
 * \warning This is an internal function, use e_po3030k_ConfigCam
 * \param start The start row
 * \param stop The stop row
 * \param col The start/stop column
 * \return Zero if OK, non-zero if an error occur
 * \sa Datasheet p.42-43, e_po3030k_write_cam_registers and e_po3030k_config_cam
 */
int e_po3030k_set_vsync(unsigned int start,unsigned int stop,unsigned int col) {
	unsigned char start_h,start_l,stop_h,stop_l,col_l,col_h;
	if(start >= stop)
		return -1;
	if(stop > 499)
		return -1;
	if(col > 899)
		return -1;
	start_l = (unsigned char) start;
	start_h = (unsigned char) (start >> 8);
	stop_l = (unsigned char) stop;
	stop_h = (unsigned char) (stop >> 8);
	col_l = (unsigned char) col;
	col_h = (unsigned char) (col >> 8);

	cam_reg[VSYNCSTART_BASE] = start_h;
	cam_reg[VSYNCSTART_BASE + 2] = start_l;

	cam_reg[VSYNCSTOP_BASE] = stop_h;
	cam_reg[VSYNCSTOP_BASE + 2] = stop_l;

	cam_reg[VSYNCCOL_BASE] = col_h;
	cam_reg[VSYNCCOL_BASE + 2] = col_l;

	return 0;
}

/*! Enable/Disable horizontal or vertical mirror
 * \param vertical Set to 1 when vertical mirror is enabled, 0 if disabled
 * \param horizontal Set to 1 when horizontal mirror is enabled, 0 if disabled
 * \sa Datasheet p.27
 */

void e_po3030k_set_mirror(int vertical, int horizontal) {
	char val;
	if(vertical && horizontal) {
		val = 0b11001010;
	} else {
		if (vertical) {
			val = 0b01001010;
		} else {
			if(horizontal) {
				val = 0b10001010;
			} else {
				val = 0b00001010;
			}
		}
	}
	cam_reg[MIRROR_BASE] = val;
}

#if PO3030K_FULL

/*! Set the register \a adr to value \a value
 * \param adr The address
 * \param value The value
 * \return Zero if register found, non-zero if not found
 * \warning This function is sub-optimal, if you use it heavly add an internal function to register.c
 * \sa e_po3030k_get_register
 */
int e_po3030k_set_register(unsigned char adr,unsigned char value) {
	int i;
	for(i = 0; i < 2*NB_REGISTERS; i+=2) {
		if(cam_reg[i] == adr) {
			cam_reg[i+1] = value;
			return 0;
		}
	}
	return -1;
}

/*! Get the register \a adr value
 * \param adr The address
 * \param value The pointer to the value to write to
 * \return Zero if register found, non-zero if not found
 * \warning This function is sub-optimal, if you use it heavly add an internal function to register.c
 * \sa e_po3030k_set_register
 */
int e_po3030k_get_register(unsigned char adr,unsigned char * value) {
	int i;
	for(i = 0; i < 2*NB_REGISTERS; i+=2) {
		if(cam_reg[i] == adr) {
			*value = cam_reg[i+1];
			return 0;
		}
	}
	return -1;
}

/*! Set the Pixel and amplificator bias
 * Increasing the bias produce better image quality, but increase camera's power consumption
 * \param pixbias The pixel bias
 * \param opbias The Amplificator bias
 * \sa Datasheet p.22
 */
void e_po3030k_set_bias(unsigned char pixbias, unsigned char opbias) {
	cam_reg[BIAS_BASE] = opbias;
	cam_reg[BIAS_BASE + 2] = pixbias;
}

/*! Set the gains of the camera
 * \param global The global gain \f$\in\lbrace 0,79 \rbrace\f$
 * \param red The red pixel's gain (fixed point [2:6] format)
 * \param green1 The green pixel near read one gain ([2:6] format)
 * \param green2 The green pixel near blue one gain ([2:6] format)
 * \param blue The blue pixel's gain ([2:6] format)
 * \return Zero if OK, non-zero if an error occur
 * \sa Datasheet p.23-24
 */
int  e_po3030k_set_color_gain(unsigned char global, unsigned char red,
						unsigned char green1, unsigned char green2,
						unsigned char blue) {
	if(global > 79)
		return -1;

	cam_reg[COLGAIN_BASE] = global;
	cam_reg[COLGAIN_BASE + 2] = red;
	cam_reg[COLGAIN_BASE + 4] = green1;
	cam_reg[COLGAIN_BASE + 6] = blue;
	cam_reg[COLGAIN_BASE + 8] = green2;
	return 0;
}

/*! Set the pixel intergration time
 * This is counted in line-time interval. See dataseet p.25 for more information
 * \param time The integration time ( fixed point [14:6] format )
 * \sa Datasheet p.25
 */
void e_po3030k_set_integr_time(unsigned long time) {
	cam_reg[INTEGR_BASE + 2] = (unsigned char) (time >> 6);
	cam_reg[INTEGR_BASE] = (unsigned char) (time >> 14);
	cam_reg[INTEGR_BASE + 4] = (unsigned char) (time << 8);
}



/*! Set the Analog to Digital Converter offset
 * \param offset The offset
 * \sa Datasheet p.28
 */
void e_po3030k_set_adc_offset(unsigned char offset) {
	cam_reg[ADCOFF_BASE] = offset;
}

/*! Enable/Disable Sepia color
 * \param status Set \a status to 1 to enable, 0 to disable
 * \sa Datasheet p.34 and 74
 */
void e_po3030k_set_sepia(int status) {
	if(status)
		cam_reg[SEPIA_BASE] |= 0b10000000;
	else
		cam_reg[SEPIA_BASE] &= 0b01111111;
}

/*! Set lens shading gain
 * \param red Lens gain for red pixel \f$\in\lbrace 0,15 \rbrace\f$
 * \param green Lens gain for green pixel \f$\in\lbrace 0,15 \rbrace\f$
 * \param blue Lens gain for blue pixel \f$\in\lbrace 0,15 \rbrace\f$
 * \sa Datasheet p.36
 */
void e_po3030k_set_lens_gain(unsigned char red, unsigned char green, unsigned char blue) {
	cam_reg[LENSG_BASE] = red;
	cam_reg[LENSG_BASE + 2] = green;
	cam_reg[LENSG_BASE + 4] = blue;
}

/*! Set Edge properties
 * \param gain Edge gain & moire factor (fixed point [2:3] format)
 * \param tresh Edge Enhancement threshold
 * \sa Datasheet p.36
 */

void e_po3030k_set_edge_prop(unsigned char gain, unsigned char tresh) {
	cam_reg[EDGE_BASE] = 0b1010000 + (gain & 0b00011111);
	cam_reg[EDGE_BASE + 2] = tresh;
}

/*! Set gamma coefficient
 * \warning This feature need extra care from the user
 * \param array Gamma coefficient array
 * \param color First two bytes : - 0b01 => Green
 * - 0b00 => Red
 * - else => Blue
 * \sa Datasheet p. 38, 50, 57-58 and e_po3030k_WriteGammaCoef
 */

void e_po3030k_set_gamma_coef(unsigned char array[12], char color) {
	int i;
	cam_reg[GAMMASELCOL_BASE] = 0b10000000 + (( color & 0x3 ) << 5);
	for(i = 0; i < sizeof(array); i++)
		cam_reg[GAMMA_BASE + i*2] = array[i];
}

/*! This special function write directly the Gamma coefficient and Gamma color select
 * into camera register.
 * \warning This function need extra care from the user
 * \sa e_po3030k_set_gamma_coef
 */

void e_po3030k_write_gamma_coef(void) {
	e_i2cp_enable();
	e_i2cp_write(DEVICE_ID,cam_reg[GAMMASELCOL_BASE - 1],cam_reg[GAMMASELCOL_BASE]);
	e_i2cp_disable();
	e_po3030k_sync_register_array(cam_reg[GAMMA_BASE - 1], cam_reg[GAMMA_BASE + 11 * 2 - 1]);
}

/*! Write every known register between address start and stop (inclusivly).
 * \warning It's better to set the configuration with appropriate functions
 * and then write all registers with e_po3030k_WriteCamRegisters
 * \param start The beginning address of the write
 * \param stop The last write address
 * \return The number of register written
 * \sa e_po3030k_write_cam_registers
 */

int  e_po3030k_sync_register_array(unsigned char start, unsigned char stop) {
	int i;
	int ret = 0;
	e_i2cp_enable();
	for(i=0;i < 2*NB_REGISTERS; i+=2 ) {
			if(cam_reg[i] >= start && cam_reg[i] <= stop) {
				e_i2cp_write(DEVICE_ID,cam_reg[i],cam_reg[i+1]);
				ret++;
			}
	}

	e_i2cp_disable();

	return ret;
}

/*! Set color correction coefficient
 * \param array Color coefficient matrix (3x3), sign[7] | integer [6:5] | fractional [4:0]
 * \sa Datasheet p. 39
 */

void e_po3030k_SetColorMatrix(unsigned char array[3*3]) {
	int i;
	for(i = 0; i< sizeof(array); i++)
		cam_reg[COLOR_COEF_BASE + i*2] = array[i];
}

/*! Set The color gain ( Cb/Cr )
 * \param cg11c Cb gain ( Sign[7] | Integer[6:5] | fractional[4:0] )
 * \param cg22c Cr gain ( Sign[7] | Integer[6:5] | fractional[4:0] )
 * \sa Datasheet p. 40
 */

void e_po3030k_set_cb_cr_gain(unsigned char cg11c, unsigned char cg22c) {
	cam_reg[CBCRGAIN_BASE] = cg11c;
	cam_reg[CBCRGAIN_BASE + 2] = cg22c;
}

/*! Set the Brightness & Contrast
 * \param bright The Brightness ( signed 1+7bits fixed point format )
 * \param contrast The Contrast
 * \sa Datasheet p. 40
 */

void e_po3030k_set_brigh_contr(unsigned char bright, unsigned char contrast) {
	cam_reg[BRICTR_BASE] = bright;
	cam_reg[BRICTR_BASE + 2] = contrast;
}

/*! Set The color tone at sepia color condition
 * \param cb Cb tone
 * \param cr Cr tone
 * \sa e_po3030k_set_sepia and Datasheet p. 41 and 74
 */

void e_po3030k_set_sepia_tone(unsigned char cb, unsigned char cr) {
	cam_reg[SEPIATONE_BASE] = cb;
	cam_reg[SEPIATONE_BASE + 2] = cr;
}

/*! Set the Center weight (Back Light compensation) Control parameter
 * \param ww Center weight ( 4 lower bits only )
 * \sa Datasheet p.44
 */

void e_po3030k_set_ww(unsigned char ww) {
	cam_reg[WW_BASE] = (ww << 4) + 0b1111;
}

/*! Set AWB/AE tolerence margin
 * \param awbm AWV Margin ( 4 lower bits only )
 * \param aem AW Margin ( 4 lower bits only )
 * \sa Datasheet p.44
 */

void e_po3030k_set_awb_ae_tol(unsigned char awbm, unsigned char aem) {
	cam_reg[AWVAETOL_BASE] = (awbm << 4) + (aem & 0b1111);
}

/*! Set AE speed
 * \param b AE speed factor when exposure time is decreasing ( 4 lower bits only )
 * \param d AE speed factor when exposure time is increasing ( 4 lower bits only )
 * \sa Datasheet p.44
 */

void e_po3030k_set_ae_speed(unsigned char b, unsigned char d) {
	cam_reg[AESPEED_BASE] = (d << 4) + (b & 0b1111);
}

/*! Set exposure time
 * \param t Exposure time, LSB is in 1/64 line time
 * \warning Only writable if AE is disabled
 * \sa Datasheet p.45
 */

void e_po3030k_set_exposure(long t) {
	cam_reg[EXPOSURE_BASE] = (unsigned char) (t >> 16);
	cam_reg[EXPOSURE_BASE + 2] = (unsigned char) (t >> 8);
	cam_reg[EXPOSURE_BASE + 4] = (unsigned char ) t;
}

/*! Set the reference exposure. The average brightness which the AE should have
 * \param exp The target exposure level
 * \sa  Datasheet p.45
 */

void e_po3030k_set_ref_exposure(unsigned char exp) {
	cam_reg[REFREPO_BASE] = exp;
}

/*! Set the minimum and maximum exposure time in AE mode
 * \param min The minimum exposure time
 * \param max The maximum exposure time
 * \sa Datasheet p.46-47
 */

void e_po3030k_set_max_min_exp(unsigned int max, unsigned int min) {
	cam_reg[MINMAXEXP_BASE] = (unsigned char) (max >> 8);
	cam_reg[MINMAXEXP_BASE + 2] = (unsigned char) max;
	cam_reg[MINMAXEXP_BASE + 4] = (unsigned char) (min >> 8);
	cam_reg[MINMAXEXP_BASE + 6] = (unsigned char) min;
}

/*! Set the minimum and maximum red and blue gain in AWB mode
 * \param minb The minimum blue gain
 * \param maxb The maximum blue gain
 * \param minr The minimum red gain
 * \param maxr The maximum red gain
 * \param ratior The red gain ratio
 * \param ratiob The blue gain ratio
 * \sa Datasheet p. 47-48
 */
void e_po3030k_set_max_min_awb(unsigned char minb, unsigned char maxb, unsigned char minr,
					unsigned char maxr, unsigned char ratior, unsigned char ratiob) {
	cam_reg[MINMAXAWB_BASE] = minr;
	cam_reg[MINMAXAWB_BASE + 2] = maxr;
	cam_reg[MINMAXAWB_BASE + 4] = minb;
	cam_reg[MINMAXAWB_BASE + 6] = maxb;

	cam_reg[MINMAXAWB_BASE + 8] = ratior;
	cam_reg[MINMAXAWB_BASE + 10] = ratiob;
}

/*! Set the Weighting Window coordinate
 * \param x1 The X1 coordinate \f$\in\lbrace 211,x2 \rbrace\f$
 * \param x2 The X2 coordinate \f$\in\lbrace x1+1,423 \rbrace\f$
 * \param y1 The Y1 coordinate \f$\in\lbrace 160,y2 \rbrace\f$
 * \param y2 The Y2 coordinate \f$\in\lbrace y1+1,319 \rbrace\f$
 * \return Zero if OK, non-zero if an error occur
 * \sa Datasheet p. 49
 */
int  e_po3030k_set_weight_win(unsigned int x1, unsigned int x2, unsigned int y1, unsigned int y2) {
	x1 += ARRAY_ORIGINE_X;
	x2 += ARRAY_ORIGINE_X;
	y1 += ARRAY_ORIGINE_Y;
	y2 += ARRAY_ORIGINE_Y;

	if(x2 > ARRAY_ORIGINE_X + ARRAY_WIDTH)
		return -1;
	if(y2 > ARRAY_ORIGINE_Y + ARRAY_HEIGHT)
		return -1;

	if(x1 >= x2 || y1 >= y2)
		return -1;

	if(x1 <= 421 || x2 >= 634 || y1 <= 167 || y2 >= 327)
		return -1;

	cam_reg[WEIGHWIN_BASE] = (unsigned char) (x1 >> 8);
	cam_reg[WEIGHWIN_BASE + 2] = (unsigned char) x1;
	cam_reg[WEIGHWIN_BASE + 4] = (unsigned char) (x2 >> 8);
	cam_reg[WEIGHWIN_BASE + 6] = (unsigned char) x2;

	cam_reg[WEIGHWIN_BASE + 8] = (unsigned char) (y1 >> 8);
	cam_reg[WEIGHWIN_BASE + 10] = (unsigned char) y1;
	cam_reg[WEIGHWIN_BASE + 12] = (unsigned char) (y2 >> 8);
	cam_reg[WEIGHWIN_BASE + 14] = (unsigned char) y2;
	return 0;
}

/*! Enable/Disable AWB and AE
 * \param awb 1 mean AWB enabled, 0 mean disabled
 * \param ae 1 mean AE enabled, 0 mean disabled
 * \sa Datasheet p. 50
 */

void e_po3030k_set_awb_ae(int awb, int ae) {
	char val;
	if(awb && ae) {
		val = 0x3c;
	} else {
		if(awb) {
			val = 0x2c;
		} else {
			if(ae) {
				val = 0x1c;
			} else {
				val = 0x0c;
			}
		}
	}
	cam_reg[AWBAEENABLE_BASE] = val;
}

/*! Set flicker detection mode
 * \param manual Non-zero mean manual mode is enabled ( default automatic mode enabled )
 * \sa Datasheet p.29 e_po3030k_set_flicker_detection() e_po3030k_set_flicker_man_set()
 */

void e_po3030k_set_flicker_mode(int manual) {
	if(manual) {
		cam_reg[FLICKM_BASE] &= ~0x80;
	} else {
		cam_reg[FLICKM_BASE] |= 0x80;
	}
}

/*! Set the 50/60Hz flicker detection
 * \param hz50 Non-zero mean 50Hz flicker detection enabled (default disabled)
 * \param hz60 Non-zero mean 60Hz flicker detection enabled (default disabled)
 * \warning If Automatic mode is enabled and both 50Hz and 60Hz are disabled,
 * camera will enable both. By default, the camera automatically detect 50 and 60Hz
 * flicker.
 * \sa Datasheet p.29 e_po3030k_set_flicker_mode() e_po3030k_set_flicker_man_set()
 */

void e_po3030k_set_flicker_detection(int hz50, int hz60) {

	if(hz50) {
		cam_reg[FLICKM_BASE] &= ~0x40;
	} else {
		cam_reg[FLICKM_BASE] |= 0x40;
	}

	if(hz60) {
		cam_reg[FLICKM_BASE] &= ~0x20;
	} else {
		cam_reg[FLICKM_BASE] |= 0x20;
	}

}

/* Private internal function to get the real pixel clock */
static long po3030k_get_pixelclock(void) {
	long clk;
	switch (cam_reg[SPEED_ADDR]) {
		case SPEED_2:
			clk = MCLK/2;
			break;
		case SPEED_2_3:
			clk = (MCLK*2)/3;
			break;
		case SPEED_4:
			clk = MCLK/4;
			break;
		case SPEED_8:
			clk = MCLK/8;
			break;
		case SPEED_16:
			clk = MCLK/16;
			break;
		case SPEED_32:
			clk = MCLK/32;
			break;
		case SPEED_64:
			clk = MCLK/64;
			break;
		case SPEED_128:
			clk = MCLK/128;
			break;
		default:
			return 0;
	}
	if(cam_reg[COLOR_M_ADDR] == MODE_GRAYSCALE)
		clk *= 2;
	return clk;
}

/*! Set the camera's manual flicker's detection setting
 * \param hz50 The Hz for the 50Hz detection
 * \param hz60 The Hz for the 60Hz detection
 * \param fdm Flicker duration mode
 * \param fk Flicker count step
 * \param tol Flicker tolerance
 * \warning You must have set the mode ( image size, color ) before calling this function
 * \return Non-zero if an error occur, 0 if OK
 * \sa Datasheet p.29-30 e_po3030k_set_flicker_detection() e_po3030k_set_flicker_mode()
 */

int  e_po3030k_set_flicker_man_set(int hz50, int hz60, int fdm, int fk, int tol) {
	unsigned int p50, p60;
	/* checking if parameter are correct */
	if(fk < 0 || fk > 3)
		return -1;

	if(tol < 0 || tol > 3)
		return -1;

	/* The datasheet isn't clear about this formula
	 * At p.30 it's written:
	 * Period = PixelClock / (HZ * 2 * FrameWidth)
	 * But at p.60 it's written:
	 * Period = 256*(MasterClock /(FrameWidth * 2))/(HZ*2)
 	 *        = 64*MasterClock / (FrameWidth * HZ)
	 */

	p50 = ( po3030k_get_pixelclock() * 2 * FRAME_WIDTH) / (long) hz50;
	p60 = ( po3030k_get_pixelclock() * 2 * FRAME_WIDTH) / (long) hz60;

	cam_reg[FLICKP_BASE] = (unsigned char) (p50 >> 8);
	cam_reg[FLICKP_BASE + 2] = (unsigned char) p50;
	cam_reg[FLICKP_BASE + 4] = (unsigned char) (p60 >> 8);
	cam_reg[FLICKP_BASE + 6] = (unsigned char) p60;

	if(fdm) {
		cam_reg[FLICKM_BASE] |= 0x10;
	} else {
		cam_reg[FLICKM_BASE] &= ~0x10;
	}

	cam_reg[FLICKM_BASE] = (tol & 0x3) + ((fk & 0x3) << 2) + (cam_reg[FLICKM_BASE] & 0xF0);

	return 0;
}
#endif
