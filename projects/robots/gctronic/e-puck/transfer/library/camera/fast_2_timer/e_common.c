#include "e_poxxxx.h"
#include "e_po3030k.h"
#include "e_po6030k.h"
#include "e_po8030d.h"
#include "../../motor_led/e_epuck_ports.h"
#include "../../I2C/e_I2C_protocol.h"
#include "../../motor_led/e_init_port.h"

static unsigned int camera_version;


int e_poxxxx_config_cam(unsigned int sensor_x1,unsigned int sensor_y1,
				unsigned int sensor_width,unsigned int sensor_height,
				unsigned int zoom_fact_width,unsigned int zoom_fact_height,
				int color_mode) {
	switch(camera_version) {
		case 0x3030:
			return e_po3030k_config_cam(sensor_x1, sensor_y1,
				sensor_width, sensor_height,
				zoom_fact_width, zoom_fact_height,
				color_mode);
			break;
		case 0x6030:
			return e_po6030k_config_cam(sensor_x1, sensor_y1,
				sensor_width, sensor_height,
				zoom_fact_width, zoom_fact_height,
				color_mode);
			break;

                case 0x8030:
                        return e_po8030d_config_cam(sensor_x1, sensor_y1,
				sensor_width, sensor_height,
				zoom_fact_width, zoom_fact_height,
				color_mode);
                        break;

                default:
                    return 11;
                    return -1;
	}
}


void e_poxxxx_set_mirror(int vertical, int horizontal) {
	switch(camera_version) {
		case 0x3030:
			e_po3030k_set_mirror(vertical, horizontal);
			break;
		case 0x6030:
			e_po6030k_set_mirror(vertical, horizontal);
			break;
                case 0x8030:
                        e_po8030d_set_mirror(vertical, horizontal);
                        break;
	}
}

void e_poxxxx_write_cam_registers(void) {
	switch(camera_version) {
		case 0x3030:
			e_po3030k_write_cam_registers();
			break;
		case 0x6030:
			// Nothing to do
			break;
                case 0x8030:
                        // Nothing to do
                        break;
	}
}

#define DEVICE_ID 0xDC
/**
 * Initalize the camera, return the version in hexa, 0x3030 or 0x6030
 */
int e_poxxxx_init_cam(void) {
	int i;
	unsigned char reg0, reg1;
	e_init_port();
	e_i2cp_init();
	CAM_RESET=0;
	for(i=100;i;i--) __asm__ volatile ("nop");
	CAM_RESET=1;
	for(i=100;i;i--) __asm__ volatile ("nop");
	/* enable interrupt nesting */
	INTCON1bits.NSTDIS = 0;
	/* set a higher priority on camera's interrupts */
	IPC5 = (IPC5 & 0xF00F) + 0x0660;

	/* read the camera version */
	reg0 = e_i2cp_read(DEVICE_ID, 0x0);
	reg1 = e_i2cp_read(DEVICE_ID, 0x1);
	camera_version = reg0 << 8 | reg1;

        if(camera_version == 0x3030) {
            e_poxxxx_set_mirror(0,0);
        } else if(camera_version == 0x6030) {
            e_poxxxx_set_mirror(1,1);
        } else if(camera_version == 0x8030) {
            e_poxxxx_set_mirror(0,0);
            init_po8030();
        }
        e_poxxxx_write_cam_registers();

	return camera_version;
}

static unsigned readee(unsigned page, unsigned addr) {
	unsigned word;
	unsigned temp = TBLPAG;
	TBLPAG = page;
	word = __builtin_tblrdl(addr);
	TBLPAG = temp;
	return word;
}

/**
 * Return the camera orientation
 * \return 0: vertical 480x640, 1: horizontal 640x480, horizontal inverted, -1: unknown
 */
int e_poxxxx_get_orientation(void) {
	switch(camera_version) {
		case 0x3030:
		{
			// read last word of EEPROM to get camera orientation
			switch(readee(0x7F,0xFFFE) & 0x3) {
				case 1: return 1;
				case 2: return 2;
				case 3: return 0;
			}
		}
		case 0x6030:
			return 1;
                case 0x8030:
                        return 1;
                        break;
	}
	return -1;
}

/*! Enable/Disable AWB and AE
 * \param awb 1 means AWB enabled, 0 means disabled
 * \param ae 1 means AE enabled, 0 means disabled
 */
void e_poxxxx_set_awb_ae(int awb, int ae) {
	switch(camera_version) {
		case 0x3030:
			e_po3030k_set_awb_ae(awb, ae);
			break;
		case 0x6030:
			e_po6030k_set_awb_ae(awb, ae);
			break;
                case 0x8030:
                        e_po8030d_set_awb_ae(awb, ae);
                        break;
	}
}

/*! Set the gains of the camera
 * \param red The red pixels' gain (0..255)
 * \param green The green pixels' gain (0..255)
 * \param blue The blue pixels' gain (0..255)
 * \warning Only meaningful if AWB is disabled
 */
void e_poxxxx_set_rgb_gain(unsigned char r, unsigned char g, unsigned char b) {
	switch(camera_version) {
		case 0x3030:
			e_po3030k_set_color_gain(40, r, g, g, b);
			break;
		case 0x6030:
			e_po6030k_set_rgb_gain(r,g,b);
			break;
                case 0x8030:
                        e_po8030d_set_rgb_gain(r,g,b);
                        break;
	}
}

/*! Set exposure time
 * \param t Exposure time, LSB is in 1/64 line time
 * \warning Only meaningful if AE is disabled
 */
void e_poxxxx_set_exposure(unsigned long exp) {
	switch(camera_version) {
		case 0x3030:
			e_po3030k_set_exposure(exp);
			break;
		case 0x6030:
			e_po6030k_set_exposure(exp);
			break;
                case 0x8030:
                        e_po8030d_set_exposure(exp);
                        break;
	}
}
unsigned int getCameraVersion() {
    return camera_version;
}
