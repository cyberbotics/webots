
/*! \file
 * \brief Manage LSM330 (accelerometer + gyro) registers
 * \author Stefano Morgani
 *
 * x, y axes representation on e-puck rev < 1.3 (analog accelerometer MMA7260):
 *      FORWARD
 *          ^
 *          | (-y)
 *          * - > (+x) RIGHT
 *
 * y, y axes representation on e-puck rev 1.3 (i2c device LSM330):
 *      FORWARD
 *          ^
 *          | (+x)
 *          * - > (-y) RIGHT
 *
 * Thus the x and y axes are exchanged to maintain compatibility.
 */

#include "../I2C/e_I2C_protocol.h"
#include "e_lsm330.h"
#include <uart/e_uart_char.h>
#include "string.h"
#include "stdio.h"

#define ACC_ADDR 0x3C
#define GYRO_ADDR 0xD4

signed int gyroOffsetX = 0;
signed int gyroOffsetY = 0;
signed int gyroOffsetZ = 0;

/*!\brief Set the register reg to value
 * \param addr The device address
 * \param reg The register address
 * \param value The value to write
 */
void writeReg(unsigned char addr, unsigned char reg, unsigned char value) {
    e_i2cp_enable();
    e_i2cp_write(addr, reg, value);
    e_i2cp_disable();
}

/*!\brief Read the register reg.
 * \param addr The device address
 * \param reg The register address
 * \return The register value
 */
unsigned char readReg(unsigned char addr, unsigned char reg) {
    unsigned char ret;
    e_i2cp_enable();
    ret = e_i2cp_read(addr, reg);
    e_i2cp_disable();
    return ret;

}
/*!\brief Continuous read (multiple register reading).
 * \param device_add The device address
 * \param read_buffer the buffer that will contain the values read from the registers
 * \param reg The register start address
 * \return 1 to confirme the oparation and 0 for an error
 */
char readRegMulti(char device_add, unsigned char *read_buffer, char start_address, unsigned char numBytes) {

    char error = 0;
    unsigned int i = 0;
    while (!error) {
        error = 1;
        error &= e_i2c_start();
        error &= e_i2c_write(device_add); // Device address
        error &= e_i2c_write(start_address | 0x80); // address of first register to be read
        error &= e_i2c_restart();
        error &= e_i2c_write(device_add + 1); // To change data direction ([bit 0]=1)

        for (i = 0; i < numBytes; i++) {
            error &= e_i2c_read((char*)&read_buffer[i]); // read the next byte
            if (i == (numBytes - 1)) { // the last byte to be read, must send nack
                error &= e_i2c_nack();
            } else {
                error &= e_i2c_ack(); // not the last byte, send ack
            }
        }
        e_i2c_stop(); // End read cycle
        if (error)
            break;
        e_i2c_reset();
    }
    return error;
}

/*!\brief Configure and turn on the device.
 */
void initAccAndGyro(void) {

    e_i2cp_enable();

    // REG5_A (0x20) =  ODR (4) | BDU (1)   | Axes enabling (3)
    //                  0 1 1 0 |   1       | 1 1 1             => 0x6F => 100 Hz | BDU | x, y, z axes enabled
    //                  0 1 1 1 |   1       | 1 1 1             => 0x7F => 400 Hz | BDU | x, y, z axes enabled
    //                  1 0 0 0 |   1       | 1 1 1             => 0x8F => 800 Hz | BDU | x, y, z axes enabled
    //                  1 0 0 1 |   1       | 1 1 1             => 0x9F => 1600 Hz | BDU | x, y, z axes enabled
    e_i2cp_write(ACC_ADDR, 0x20, 0x9F);
    // REG7_A (0x25) = 0 1 (enable fifo, needed?) 0 1 (auto increment address on multi read) 0 0 0 0 => 0x50
    e_i2cp_write(ACC_ADDR, 0x25, 0x50);
    // REG6_A (0x24) => default +- 2g
    // FIFO_CTRL_REG_A (0x2E) => default bypass mode

    // CTRL_REG1_G (0x20) = ODR and cut-off (4) | mode (1)  | Axes enabling (3)
    //                      0 0 1 1             |   1       | 1 1 1             => 0x3F => odr=95Hz, cut-off=25Hz | normal | x, y, z axes enabled
    //                      1 1 0 0             |   1       | 1 1 1             => 0xCF => odr=760Hz, cut-off=30Hz | normal | x, y, z axes enabled
    e_i2cp_write(GYRO_ADDR, 0x20, 0xCF);

    // CTRL_REG2_G (0x21) => normal mode; HPF=51.4 Hz (not used anyway)
    e_i2cp_write(GYRO_ADDR, 0x21, 0x20);

    // CTRL_REG4_G (0x23) => 250 dps (degrees per second) and continuous update
    e_i2cp_write(GYRO_ADDR, 0x23, 0x00);

    // CTRL_REG5_G (0x24) => enable fifo (needed?)
    //e_i2cp_write(GYRO_ADDR, 0x24, 0x40);
    //e_i2cp_write(GYRO_ADDR, 0x24, 0x50);    // LPF1
    //e_i2cp_write(GYRO_ADDR, 0x24, 0x51);    // LPF1 + HPF
    //e_i2cp_write(GYRO_ADDR, 0x24, 0x42);    // LPF1 + LPF2
    //e_i2cp_write(GYRO_ADDR, 0x24, 0x52);    // LPF1 + HPF + LPF2

    e_i2cp_disable();

}

void getAllAxesAccRaw(unsigned char *arr) {
    e_i2cp_enable();

    readRegMulti(ACC_ADDR, arr, 0x28, 6);
    /*
    arr[0] = readReg(ACC_ADDR, 0x28);   // X acc axis low byte
    arr[1] = readReg(ACC_ADDR, 0x29);   // X acc axis high byte
    arr[2] = readReg(ACC_ADDR, 0x2A);   // Y acc axis low byte
    arr[3] = readReg(ACC_ADDR, 0x2B);   // Y acc axis high byte
    arr[4] = readReg(ACC_ADDR, 0x2C);   // Z acc axis low byte
    arr[5] = readReg(ACC_ADDR, 0x2D);   // Z acc axis high byte
     */
    e_i2cp_disable();
}

void getAllAxesAcc(signed int *x, signed int *y, signed int *z) {
    unsigned char arr[6];
    e_i2cp_enable();
    readRegMulti(ACC_ADDR, arr, 0x28, 6);
    e_i2cp_disable();
    *x = (((signed int) arr[1] << 8) + (signed int) arr[0]);
    *y = (((signed int) arr[3] << 8) + (signed int) arr[2]);
    *z = (((signed int) arr[5] << 8) + (signed int) arr[4]);
}

int getXAxisAcc() {
    signed char arr[2];
    e_i2cp_enable();
    // 2's complement
    readRegMulti(ACC_ADDR, (unsigned char*)arr, 0x28, 2);
    //arr[0] = readReg(ACC_ADDR, 0x28); // X acc axis low byte
    //arr[1] = readReg(ACC_ADDR, 0x29); // X acc axis high byte
    e_i2cp_disable();
    return ((signed int) arr[1] << 8) + (signed int) arr[0];
}

int getYAxisAcc() {
    signed char arr[2];
    e_i2cp_enable();
    // 2's complement
    readRegMulti(ACC_ADDR, (unsigned char*)arr, 0x2A, 2);
    //arr[0] = readReg(ACC_ADDR, 0x2A); // Y acc axis low byte
    //arr[1] = readReg(ACC_ADDR, 0x2B); // Y acc axis high byte
    e_i2cp_disable();
    return ((signed int) arr[1] << 8) + (signed int) arr[0];
}

int getZAxisAcc() {
    signed char arr[2];
    e_i2cp_enable();
    // 2's complement
    readRegMulti(ACC_ADDR, (unsigned char*)arr, 0x2C, 2);
    //arr[0] = readReg(ACC_ADDR, 0x2C); // Z acc axis low byte
    //arr[1] = readReg(ACC_ADDR, 0x2D); // Z acc axis high byte
    e_i2cp_disable();
    return ((signed int) arr[1] << 8) + (signed int) arr[0];
}

void getAllAxesGyroRaw(unsigned char *arr) {
    e_i2cp_enable();
    readRegMulti(GYRO_ADDR, arr, 0x28, 6);
    /*
    arr[0] = readReg(GYRO_ADDR, 0x28);
    arr[1] = readReg(GYRO_ADDR, 0x29);
    arr[2] = readReg(GYRO_ADDR, 0x2A);
    arr[3] = readReg(GYRO_ADDR, 0x2B);
    arr[4] = readReg(GYRO_ADDR, 0x2C);
    arr[5] = readReg(GYRO_ADDR, 0x2D);
     */
    e_i2cp_disable();
}

void getAllAxesGyro(signed int *x, signed int *y, signed int *z) {
    unsigned char arr[6];
    e_i2cp_enable();
    readRegMulti(GYRO_ADDR, arr, 0x28, 6);
    e_i2cp_disable();
    *x = (((signed int) arr[1] << 8) + (signed int) arr[0]) - gyroOffsetX;
    *y = (((signed int) arr[3] << 8) + (signed int) arr[2]) - gyroOffsetY;
    *z = (((signed int) arr[5] << 8) + (signed int) arr[4]) - gyroOffsetZ;
}

int getXAxisGyro() {
    signed char arr[2];
    e_i2cp_enable();
    // 2's complement
    readRegMulti(GYRO_ADDR, (unsigned char*)arr, 0x28, 2);
    //arr[0] = readReg(GYRO_ADDR, 0x28);
    //arr[1] = readReg(GYRO_ADDR, 0x29);
    e_i2cp_disable();
    return (((signed int) arr[1] << 8) + (signed int) arr[0]) - gyroOffsetX;
}

int getYAxisGyro() {
    signed char arr[2];
    e_i2cp_enable();
    // 2's complement
    readRegMulti(GYRO_ADDR, (unsigned char*)arr, 0x2A, 2);
    //arr[0] = readReg(GYRO_ADDR, 0x2A);
    //arr[1] = readReg(GYRO_ADDR, 0x2B);
    e_i2cp_disable();
    return (((signed int) arr[1] << 8) + (signed int) arr[0]) - gyroOffsetY;
}

int getZAxisGyro() {
    signed char arr[2];
    e_i2cp_enable();
    // 2's complement
    readRegMulti(GYRO_ADDR, (unsigned char*)arr, 0x2C, 2);
    //arr[0] = readReg(GYRO_ADDR, 0x2C);
    //arr[1] = readReg(GYRO_ADDR, 0x2D);
    e_i2cp_disable();
    return (((signed int) arr[1] << 8) + (signed int) arr[0]) - gyroOffsetZ;
}

signed char getTemperature(void) {
    return (signed char) readReg(GYRO_ADDR, 0x26);  // 2's complement, 1 lsb/deg
}

// The values of the gyroscope have a noise that screw up the calibration
// (the offset values result too much different between various calibrations), thus avoid to use it.
void calibrateGyroscope(int numSamples) {
    unsigned char gyroData[6];
    signed int gyroSum[3] = {0, 0, 0};
    int i = 0;

    for(i=0; i<numSamples; i++) {
        getAllAxesGyroRaw(gyroData);
        gyroSum[0] += ((signed int) gyroData[1] << 8) + (signed int) gyroData[0];
        gyroSum[1] += ((signed int) gyroData[3] << 8) + (signed int) gyroData[2];
        gyroSum[2] += ((signed int) gyroData[5] << 8) + (signed int) gyroData[4];
    }

    gyroOffsetX = gyroSum[0]/numSamples;
    gyroOffsetY = gyroSum[1]/numSamples;
    gyroOffsetZ = gyroSum[2]/numSamples;

    //sprintf(buffer, "%d,%d,%d\r\n", gyroOffsetX, gyroOffsetY, gyroOffsetZ);
    //e_send_uart2_char(buffer, strlen(buffer));
    //while(e_uart2_sending());

}

/*! \brief Raw value to degrees per second.
 * Take as input the 2's complement value received by the device for a certain axis
 * and return the degrees per second (knowing the gyroscope is configured with sensitivity
 * of +- 250 dps).
 */
float rawToDps(int value) {
    // The gyroscope returns 16 bits 2's complement values with a full-scale of +-250 dps;
    // from the datasheet with full scale = +- 250 dps, then angular rate sensitivity is 8.75 mdps/digit
    return (float)value*0.00875;
}

/*! \brief Raw value to degrees per millisecond.
 * Take as input the 2's complement value received by the device for a certain axis
 * and return the degrees per millisecond (knowing the gyroscope is configured with sensitivity
 * of +- 250 dps).
 */
float rawToDpms(int value) {
    return (float)value*0.00000875;
}
