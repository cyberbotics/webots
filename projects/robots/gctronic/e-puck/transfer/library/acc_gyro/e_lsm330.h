/*! \file
 * \brief LSM330 library header
 * \author Stefano Morgani
 */

#ifndef __LSM330_H__
#define __LSM330_H__


void initAccAndGyro(void);

void getAllAxesAccRaw(unsigned char *arr);

void getAllAxesAcc(signed int *x, signed int *y, signed int *z);

// do not remove the offsets computed during calibration
void getAllAxesGyroRaw(unsigned char *arr);

void getAllAxesGyro(signed int *x, signed int *y, signed int *z);

// degrees (two's complement)
signed char getTemperature(void);

int getXAxisAcc();

int getYAxisAcc();

int getZAxisAcc();

int getXAxisGyro();

int getYAxisGyro();

int getZAxisGyro();

void calibrateGyroscope();

float rawToDps(int value);

float rawToDpms(int value);

#endif
