
/*! \file
 * \brief Manage po6030k registers (two timers)
 * \author Philippe Rétornaz
 */



#include "../../I2C/e_I2C_protocol.h"
#include "e_po6030k.h"

#define MCLK				((long) 14745600)  /* 14.7456Mhz */
#define MCLK_P_NS			0.067816840278 /* Master clock period in ns */


/* these two define should go into a private header but, well,
 * I won't do it one for only two define ... */
#define 	ARRAY_ORIGINE_X	80
#define		ARRAY_ORIGINE_Y 8

#define DEVICE_ID 0xDC
#define BANK_REGISTER 0x3

/*! Set the camera register bank to use
 * \param bank The bank used.
 * \sa BANK_A, BANK_B, BANK_C, BANK_D
 */
void e_po6030k_set_bank(unsigned char bank) {
	e_i2cp_enable();
	e_i2cp_write(DEVICE_ID, BANK_REGISTER, bank);
	e_i2cp_disable();
}

/*! Set the register reg to value
 * \param bank The register bank
 * \param reg The register address
 * \param value The value to write
 * \sa BANK_A, BANK_B, BANK_C, BANK_D
 */
void e_po6030k_write_register(unsigned char bank, unsigned char reg, unsigned char value) {
	e_po6030k_set_bank(bank);
	e_i2cp_enable();
	e_i2cp_write(DEVICE_ID, reg, value);
	e_i2cp_disable();
}

/*! Read the register reg.
 * \param bank The register bank
 * \param reg The register address
 * \return The register value
 * \sa BANK_A, BANK_B, BANK_C, BANK_D
 */
unsigned char e_po6030k_read_register(unsigned char bank, unsigned char reg) {
	unsigned char ret;
	e_po6030k_set_bank(bank);
	e_i2cp_enable();
	ret = e_i2cp_read(DEVICE_ID, reg);
	e_i2cp_disable();
	return ret;

}

/*! Set the bayer clock divider
 * \param div the register value
 * \sa BAYER_CLOCK_1, BAYER_CLOCK_2, BAYER_CLOCK_4, BAYER_CLOCK_8
 */
void e_po6030k_set_bayer_clkdiv(unsigned char div) {
	e_po6030k_write_register(BANK_A, 0x91, div);
}

/*! Set the pixel clock divider
 * \param div the register value
 * \warning The register value is dependant of the camera color and sample mode
 */
void e_po6030k_set_pclkdiv(unsigned char div) {
	e_po6030k_write_register(BANK_B, 0x68, div);
}

static int e_po6030k_set_sampl_gray(unsigned char sample) {
	switch (sample) {
		case PO_6030_MODE_VGA:
			e_po6030k_set_pclkdiv(1);
			e_po6030k_write_register(BANK_B, 0x80, sample); /* Scale X*/
			e_po6030k_write_register(BANK_B, 0x81, sample); /* Scale Y*/
			e_po6030k_write_register(BANK_B, 0x82, 1); /* WTF ?!! */
			return 0;
		case PO_6030_MODE_QVGA:
			e_po6030k_set_pclkdiv(3);
			e_po6030k_write_register(BANK_B, 0x80, sample); /* Scale X*/
			e_po6030k_write_register(BANK_B, 0x81, sample); /* Scale Y*/
			e_po6030k_write_register(BANK_B, 0x82, 1); /* WTF ?!! */
			return 0;
		case PO_6030_MODE_QQVGA:
			e_po6030k_set_pclkdiv(7);
			e_po6030k_write_register(BANK_B, 0x80, sample); /* Scale X*/
			e_po6030k_write_register(BANK_B, 0x81, sample); /* Scale Y*/
			e_po6030k_write_register(BANK_B, 0x82, 1); /* WTF ?!! */
			return 0;
	}
	return -1;
}

static int e_po6030k_set_sampl_color(unsigned char sample) {
	switch (sample) {
		case PO_6030_MODE_VGA:
			e_po6030k_set_pclkdiv(0);
			e_po6030k_write_register(BANK_B, 0x80, sample); /* Scale X*/
			e_po6030k_write_register(BANK_B, 0x81, sample); /* Scale Y*/
			e_po6030k_write_register(BANK_B, 0x82, 1); /* WTF ?!! */
			return 0;
		case PO_6030_MODE_QVGA:
			e_po6030k_set_pclkdiv(1);
			e_po6030k_write_register(BANK_B, 0x80, sample); /* Scale X*/
			e_po6030k_write_register(BANK_B, 0x81, sample); /* Scale Y*/
			e_po6030k_write_register(BANK_B, 0x82, 1); /* WTF ?!! */
			return 0;
		case PO_6030_MODE_QQVGA:
			e_po6030k_set_pclkdiv(3);
			e_po6030k_write_register(BANK_B, 0x80, sample); /* Scale X*/
			e_po6030k_write_register(BANK_B, 0x81, sample); /* Scale Y*/
			e_po6030k_write_register(BANK_B, 0x82, 1); /* WTF ?!! */
			return 0;
	}
	return -1;
}

/*! Set camera sampling and color mode
 * \param format The color format
 * \param sampl_mode The sampling mode
 * \sa GREY_SCALE_MODE, RGB_565_MODE, YUV_MODE, MODE_VGA, MODE_QVGA, MODE_QQVGA
 */
int e_po6030k_set_mode(unsigned char format, unsigned char sampl_mode) {
	switch(format) {
		case GREY_SCALE_MODE:
			e_po6030k_write_register(BANK_B, 0x38, 0x0D);
			return e_po6030k_set_sampl_gray(sampl_mode);
		case RGB_565_MODE:
			e_po6030k_write_register(BANK_B, 0x38, 0x08);
			return e_po6030k_set_sampl_color(sampl_mode);
		case YUV_MODE:
			e_po6030k_write_register(BANK_B, 0x38, 0x02);
			return e_po6030k_set_sampl_color(sampl_mode);
	}
	return -1;
}

/*! Set the camera window X coordinate
 * \param start The start column
 * \param stop The stop column
 * \return Zero if OK, non-zero if an error occur
 */
int e_po6030k_set_wx(unsigned int start,unsigned int stop) {
	unsigned char start_h,start_l,stop_h,stop_l;

	if(start >= stop)
		return -1;
	if(stop > 799)
		return -1;
	start--;
	start_l = (unsigned char) start;
	start_h = (unsigned char) (start >> 8);
	stop_l = (unsigned char) stop;
	stop_h = (unsigned char) (stop >> 8);

//	start_l = e_po6030k_read_register(BANK_B, 0x51);


	e_po6030k_write_register(BANK_B, 0x51, start_l);
	e_po6030k_write_register(BANK_B, 0x50, start_h);

//	e_po6030k_write_register(BANK_B, 0x54, stop_h);
//	e_po6030k_write_register(BANK_B, 0x55, stop_l);

	return 0;
}

/*! Set the camera window Y coordinate
 * \param start The start row
 * \param stop The stop row
 * \return Zero if OK, non-zero if an error occur
 */
int e_po6030k_set_wy(unsigned int start, unsigned int stop) {
	unsigned char start_h,start_l,stop_h,stop_l;

	return 0;
	if(start >= stop)
		return -1;
	if(stop > 499)
		return -1;
	start_l = (unsigned char) start;
	start_h = (unsigned char) (start >> 8);
	stop_l = (unsigned char) stop;
	stop_h = (unsigned char) (stop >> 8);

	e_po6030k_write_register(BANK_B, 0x52, start_h);
	e_po6030k_write_register(BANK_B, 0x53, start_l);

//	e_po6030k_write_register(BANK_B, 0x56, stop_h);
//	e_po6030k_write_register(BANK_B, 0x57, stop_l);

	return 0;
}

/*! Set the camera window VSYNC coordinate
 * \param start The start row
 * \param stop The stop row
 * \return Zero if OK, non-zero if an error occur
 */
int e_po6030k_set_vsync(unsigned int start,unsigned int stop) {
	unsigned char start_h,start_l,stop_h,stop_l;

	if(start >= stop)
		return -1;
	if(stop > 499)
		return -1;

	start_l = (unsigned char) start;
	start_h = (unsigned char) (start >> 8);
	stop_l = (unsigned char) stop;
	stop_h = (unsigned char) (stop >> 8);

	e_po6030k_write_register(BANK_B, 0x60, start_h);
	e_po6030k_write_register(BANK_B, 0x61, start_l);

	e_po6030k_write_register(BANK_B, 0x62, stop_h);
	e_po6030k_write_register(BANK_B, 0x63, stop_l);

//	e_po6030k_write_register(BANK_B, 0x64, col_l);
//	e_po6030k_write_register(BANK_B, 0x65, col_h);

	return 0;
}


/*! Set the camera sketch mode (aka, sobel-filter)
 * \param mode The sketch mode
 * \sa E_PO6030K_SKETCH_BW, E_PO6030K_SKETCH_COLOR
 */
void e_po6030k_set_sketch_mode(int mode) {
	e_po6030k_write_register(BANK_C, 0x5A, 0x01);
	e_po6030k_write_register(BANK_B, 0x32, 0x41);
	if(mode == E_PO6030K_SKETCH_BW) {
		e_po6030k_write_register(BANK_B, 0x88, 0xFF);
		e_po6030k_write_register(BANK_B, 0x89, 0xFF);
		e_po6030k_write_register(BANK_B, 0x8A, 0x08);
		e_po6030k_write_register(BANK_B, 0x8B, 0xFF);
	} else {
		e_po6030k_write_register(BANK_B, 0x88, 0x20);
		e_po6030k_write_register(BANK_B, 0x89, 0x80);
		e_po6030k_write_register(BANK_B, 0x8A, 0x08);
		e_po6030k_write_register(BANK_B, 0x8B, 0xFF);
	}
}

/*! Enable/Disable horizontal or vertical mirror
 * \param vertical Set to 1 when vertical mirror is enabled, 0 if disabled
 * \param horizontal Set to 1 when horizontal mirror is enabled, 0 if disabled
 */
void e_po6030k_set_mirror(int vertical, int horizontal) {
	unsigned char bc;
	bc = e_po6030k_read_register(BANK_A, 0x90);
	if(vertical)
		bc |= 0x80;
	else
		bc &= ~0x80;

	if(horizontal)
		bc |= 0x40;
	else
		bc &= ~0x40;
	e_po6030k_write_register(BANK_A, 0x90, bc);
}

/*! Enable/Disable AWB and AE
 * \param awb 1 mean AWB enabled, 0 mean disabled
 * \param ae 1 mean AE enabled, 0 mean disabled
 */
void e_po6030k_set_awb_ae(int awb, int ae) {
	unsigned char reg = 0x98;
	if(!awb)
		reg |= 1 << 2;
	if(!ae) {
		reg |= 1;
		e_po6030k_write_register(BANK_C, 0x55, 0);
		e_po6030k_write_register(BANK_C, 0x56, 0);
	} else {
		e_po6030k_write_register(BANK_C, 0x55, 0x8);
		e_po6030k_write_register(BANK_C, 0x56, 0xc);
	}

	e_po6030k_write_register(BANK_C, 0x4, reg);
}

// 0x40 == gain: 1x, 0x80 == gain: 2x
void e_po6030k_set_rgb_gain(unsigned char r, unsigned char g, unsigned char b) {
	e_po6030k_write_register(BANK_C, 0xa4, r);
	e_po6030k_write_register(BANK_C, 0xa5, g);
	e_po6030k_write_register(BANK_C, 0xa6, b);
}

/*! Set exposure time
 * \param t Exposure time, LSB is in 1/64 line time
 */
void e_po6030k_set_exposure(unsigned long exp) {
	e_po6030k_write_register(BANK_C, 0x2c, exp >> 24);
	e_po6030k_write_register(BANK_C, 0x2d, (exp >> 16) & 0xFF);
	e_po6030k_write_register(BANK_C, 0x2e, (exp >> 8) & 0xFF);
	e_po6030k_write_register(BANK_C, 0x2f, exp & 0xFF);
}
