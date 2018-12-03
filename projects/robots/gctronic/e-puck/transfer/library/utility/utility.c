#include <motor_led/e_epuck_ports.h>
#include <motor_led/e_init_port.h>
#include <a_d/advance_ad_scan/e_acc.h>
#include <a_d/advance_ad_scan/e_ad_conv.h>

extern int e_acc_scan[3][ACC_SAMP_NB];
extern unsigned int e_last_acc_scan_id;
extern unsigned int tickAdcIsr;

#define MAX_BATT_VALUE 2560 // corresponds to 4.2 volts
#define MIN_BATT_VALUE 2070 // corresponds to 3.4 volts
#define BATT_VALUES_RANGE (MAX_BATT_VALUE-MIN_BATT_VALUE)

void wait(long num) {
	long i;
	for(i=0;i<num;i++);
}

int getselector() {
	return SELECTOR0 + 2*SELECTOR1 + 4*SELECTOR2 + 8*SELECTOR3;
}

unsigned int getBatteryValueRaw() {
    if(isEpuckVersion1_3()) {
        return e_acc_scan[2][e_last_acc_scan_id];   // the battery value on e-puck rev 1.3 is sampled on the same channel where the accelerometer z axis
                                                    // was sampled on e-puck 1.0
    } else {
        return 0;
    }
}

unsigned int getBatteryValuePercentage() {
    if(isEpuckVersion1_3()) {
        return 100*(float)(e_acc_scan[2][e_last_acc_scan_id]-MIN_BATT_VALUE)/(float)BATT_VALUES_RANGE;
    } else {
        return 0;
    }
}

void resetTime(void) {
    tickAdcIsr = 0;
}

// Based on ADC ISR interrupt frequency of 1/16384 (about 61 us). Each time the ISR is entered the tick
// is incremented, so we can compute the elapsed time from the last reset (knowing the ISR period)
float getDiffTimeMs(void) {
    return ((float)tickAdcIsr)*ADC_ISR_PERIOD_MS;   // the function itself takes 20-40 us (negligable)
}

float getDiffTimeMsAndReset(void) {
    float value = ((float)tickAdcIsr)*ADC_ISR_PERIOD_MS;   // the function itself takes 20-40 us (negligable)
    tickAdcIsr = 0;
    return value;
}

