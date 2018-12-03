
/*! \file
 * \brief Manage po8030d registers (two timers)
 * \author Stefano Morgani
 */



#include "../../I2C/e_I2C_protocol.h"
#include "e_po8030d.h"

#define MCLK				((long) 14745600)  /* 14.7456Mhz */
#define MCLK_P_NS			0.067816840278 /* Master clock period in ns */


/* these two define should go into a private header but, well,
 * I won't do it one for only two define ... */
#define 	ARRAY_ORIGINE_X	68
#define		ARRAY_ORIGINE_Y 4

#define DEVICE_ID 0xDC
#define BANK_REGISTER 0x3

/*! Set the camera register bank to use
 * \param bank The bank used.
 * \sa BANK_A, BANK_B, BANK_C, BANK_D
 */
void e_po8030d_set_bank(unsigned char bank) {
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
void e_po8030d_write_register(unsigned char bank, unsigned char reg, unsigned char value) {
    e_po8030d_set_bank(bank);
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
unsigned char e_po8030d_read_register(unsigned char bank, unsigned char reg) {
    unsigned char ret;
    e_po8030d_set_bank(bank);
    e_i2cp_enable();
    ret = e_i2cp_read(DEVICE_ID, reg);
    e_i2cp_disable();
    return ret;

}

int testWriteReg(unsigned char bank) {
    unsigned char ret;
    e_po8030d_set_bank(bank);
    e_i2cp_enable();
    ret = e_i2cp_read(DEVICE_ID, BANK_REGISTER);
    e_i2cp_disable();
    return ret;
}

int enablePO8030() {
    unsigned char ret, ret2;
    e_po8030d_set_bank(BANK_A);
    e_i2cp_enable();
    ret2 = e_i2cp_read(DEVICE_ID, 0x5B); // pad control
    ret = ret2;
    e_i2cp_disable();
    ret &= ~(1 << 7);
    ret &= ~(1 << 6);
    ret &= ~(1 << 3);
    e_i2cp_enable();
    e_i2cp_write(DEVICE_ID, 0x5B, ret); // pad control
    e_i2cp_disable();
    return ret2;
}

int init_po8030() {
    unsigned char ret = 0;
    //unsigned long exposure = 0;

    ret = enablePO8030();
    //e_po8030d_set_awb_ae(1, 0);
    //exposure = ((unsigned long)0x00<<24)|((unsigned long)0x08<<16)|(0x00<<8)|0x00;
    //e_po8030d_set_exposure(exposure);

    e_po8030d_set_brightness(0x28);

    return ret;
}

/*! Set the clock divider
 * \param div the register value
 * \sa BAYER_CLOCK_1, BAYER_CLOCK_2, BAYER_CLOCK_4, BAYER_CLOCK_8
 */
void e_po8030d_set_bayer_clkdiv(unsigned char div) {
    e_po8030d_write_register(BANK_A, 0x6A, div);
}

/*! Set the pixel clock divider
 * \param div the register value
 * \warning The register value is dependant of the camera color and sample mode
 */

void e_po8030d_set_pclkdiv(unsigned char div) {
    e_po8030d_write_register(BANK_B, 0xB7, div);
}

static int e_po8030d_set_sampl_gray(unsigned char sample) {
    switch (sample) {
        case PO_8030_MODE_VGA:
            e_po8030d_set_pclkdiv(1);
            e_po8030d_write_register(BANK_B, 0x93, sample); /* Scale X*/
            e_po8030d_write_register(BANK_B, 0x94, sample); /* Scale Y*/
            return 0;
        case PO_8030_MODE_QVGA:
            e_po8030d_set_pclkdiv(3);
            e_po8030d_write_register(BANK_B, 0x93, sample); /* Scale X*/
            e_po8030d_write_register(BANK_B, 0x94, sample); /* Scale Y*/
            return 0;
        case PO_8030_MODE_QQVGA:
            e_po8030d_set_pclkdiv(7);
            e_po8030d_write_register(BANK_B, 0x93, sample); /* Scale X*/
            e_po8030d_write_register(BANK_B, 0x94, sample); /* Scale Y*/
            return 0;
    }
    return -1;
}

static int e_po8030d_set_sampl_color(unsigned char sample) {
    switch (sample) {
        case PO_8030_MODE_VGA:
            e_po8030d_set_pclkdiv(0);
            e_po8030d_write_register(BANK_B, 0x93, sample); /* Scale X*/
            e_po8030d_write_register(BANK_B, 0x94, sample); /* Scale Y*/
            return 0;
        case PO_8030_MODE_QVGA:
            e_po8030d_set_pclkdiv(1);
            e_po8030d_write_register(BANK_B, 0x93, sample); /* Scale X*/
            e_po8030d_write_register(BANK_B, 0x94, sample); /* Scale Y*/
            return 0;
        case PO_8030_MODE_QQVGA:
            e_po8030d_set_pclkdiv(3);
            e_po8030d_write_register(BANK_B, 0x93, sample); /* Scale X*/
            e_po8030d_write_register(BANK_B, 0x94, sample); /* Scale Y*/
            return 0;
    }
    return -1;
}

/*! Set camera sampling and color mode
 * \param format The color format
 * \param sampl_mode The sampling mode
 * \sa GREY_SCALE_MODE, RGB_565_MODE, YUV_MODE, MODE_VGA, MODE_QVGA, MODE_QQVGA
 */
int e_po8030d_set_mode(unsigned char format, unsigned char sampl_mode) {
    switch (format) {
        case GREY_SCALE_MODE:
            e_po8030d_write_register(BANK_B, 0x4E, 0x43);
            return e_po8030d_set_sampl_gray(sampl_mode);
        case RGB_565_MODE:
            e_po8030d_write_register(BANK_B, 0x4E, 0x30);
            return e_po8030d_set_sampl_color(sampl_mode);
        case YUV_MODE:
            e_po8030d_write_register(BANK_B, 0x4E, 0x02);
            return e_po8030d_set_sampl_color(sampl_mode);
    }
    return -1;
}

/*! Set the camera window X coordinate
 * \param start The start column
 * \param stop The stop column
 * \return Zero if OK, non-zero if an error occur
 */
int e_po8030d_set_wx(unsigned int start, unsigned int stop) {
    unsigned char start_h, start_l, stop_h, stop_l;

    if (start >= stop)
        return -1;
    if (stop > 775)
        return -1;
    start--;
    start_l = (unsigned char) start;
    start_h = (unsigned char) (start >> 8);
    stop_l = (unsigned char) stop;
    stop_h = (unsigned char) (stop >> 8);

    //	start_l = e_po8030d_read_register(BANK_A, 0x09);

    e_po8030d_write_register(BANK_A, 0x09, start_l);
    e_po8030d_write_register(BANK_A, 0x08, start_h);

    //	e_po8030d_write_register(BANK_A, 0x0C, stop_h);
    //	e_po8030d_write_register(BANK_A, 0x0D, stop_l);

    return 0;
}

/*! Set the camera window Y coordinate
 * \param start The start row
 * \param stop The stop row
 * \return Zero if OK, non-zero if an error occur
 */
int e_po8030d_set_wy(unsigned int start, unsigned int stop) {
    unsigned char start_h, start_l, stop_h, stop_l;

    return 0;
    if (start >= stop)
        return -1;
    if (stop > 515)
        return -1;
    start_l = (unsigned char) start;
    start_h = (unsigned char) (start >> 8);
    stop_l = (unsigned char) stop;
    stop_h = (unsigned char) (stop >> 8);

    e_po8030d_write_register(BANK_A, 0x0A, start_h);
    e_po8030d_write_register(BANK_A, 0x0B, start_l);

    //	e_po8030d_write_register(BANK_A, 0x0E, stop_h);
    //	e_po8030d_write_register(BANK_A, 0x0F, stop_l);

    return 0;
}

/*! Set the camera window VSYNC coordinate
 * \param start The start row
 * \param stop The stop row
 * \return Zero if OK, non-zero if an error occur
 */
int e_po8030d_set_vsync(unsigned int start, unsigned int stop) {
    unsigned char start_h, start_l, stop_h, stop_l;

    if (start >= stop)
        return -1;
    if (stop > 515)
        return -1;

    start_l = (unsigned char) start;
    start_h = (unsigned char) (start >> 8);
    stop_l = (unsigned char) stop;
    stop_h = (unsigned char) (stop >> 8);

    e_po8030d_write_register(BANK_A, 0x10, start_h);
    e_po8030d_write_register(BANK_A, 0x11, start_l);

    e_po8030d_write_register(BANK_A, 0x12, stop_h);
    e_po8030d_write_register(BANK_A, 0x13, stop_l);

    return 0;
}

/*! Set the camera sketch mode (aka, sobel-filter)
 * \param mode The sketch mode
 * \sa E_PO8030D_SKETCH_BW, E_PO8030D_SKETCH_COLOR
 */
void e_po8030d_set_sketch_mode(int mode) {
    if (mode == E_PO8030D_SKETCH_BW) {
        e_po8030d_write_register(BANK_B, 0x04, 0x80); // enable lens shading compensation
        e_po8030d_write_register(BANK_B, 0x06, 0x02); // enable sketch effect
        //e_po8030d_write_register(BANK_B, 0x8F, 0x80); // sketch offset => applied to chrominance signal
    } else {
        return;
    }
}

/*! Enable/Disable horizontal or vertical mirror
 * \param vertical Set to 1 when vertical mirror is enabled, 0 if disabled
 * \param horizontal Set to 1 when horizontal mirror is enabled, 0 if disabled
 */
void e_po8030d_set_mirror(int vertical, int horizontal) {
    unsigned char bc;
    bc = e_po8030d_read_register(BANK_A, 0x6C);
    if (vertical)
        bc |= 0x80;
    else
        bc &= ~0x80;

    if (horizontal)
        bc |= 0x40;
    else
        bc &= ~0x40;
    e_po8030d_write_register(BANK_A, 0x6C, bc);
}

/*! Enable/Disable AWB and AE
 * \param awb 1 mean AWB enabled, 0 mean disabled
 * \param ae 1 mean AE enabled, 0 mean disabled
 */
void e_po8030d_set_awb_ae(int awb, int ae) {
    unsigned char reg = 0x98;
    if (!awb)
        reg |= 1 << 2;
    if (!ae) {
        reg |= 1;
        e_po8030d_write_register(BANK_C, 0x48, 0);
        e_po8030d_write_register(BANK_C, 0x49, 0);
    } else {
        e_po8030d_write_register(BANK_C, 0x48, 0x8);
        e_po8030d_write_register(BANK_C, 0x49, 0xc);
    }

    e_po8030d_write_register(BANK_C, 0x4, reg);
}

// 0x40 == gain: 1x, 0x80 == gain: 2x
void e_po8030d_set_rgb_gain(unsigned char r, unsigned char g, unsigned char b) {
    e_po8030d_write_register(BANK_A, 0x23, r);
    e_po8030d_write_register(BANK_A, 0x24, g);
    e_po8030d_write_register(BANK_A, 0x25, b);
}

/*! Set exposure time
 * \param t Exposure time, LSB is in 1/64 line time
 */
void e_po8030d_set_exposure(unsigned long exp) {
    e_po8030d_write_register(BANK_C, 0x12, exp >> 24);
    e_po8030d_write_register(BANK_C, 0x13, (exp >> 16) & 0xFF);
    e_po8030d_write_register(BANK_C, 0x14, (exp >> 8) & 0xFF);
    e_po8030d_write_register(BANK_C, 0x15, exp & 0xFF);
}

/*! Set brightness
 * \param value Brightness => [7]:[6:0] = Sign:Magnitude: luminance = Y*contrast + brightness
 */
void e_po8030d_set_brightness(signed char value) {
    e_po8030d_write_register(BANK_B, 0x9E, value);
}

