/*
 * Copyright 1996-2023 Cyberbotics Ltd.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     https://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/*
 * Description:  The code used for the Scan.
 *               This code is directly taken from the real one.
 */

/*
 * For more information about the functions define here, please refere to
 * the surveyor_scan.h file.
 */

#include <string.h>

#define index(x, y) (((63 - y) * 80) + x)  // hardwired for 80 x 64 resolution - IMJ3

static unsigned int tvect[80], tmax[10], tmean[10], tzeros[10];
static unsigned int tflag = 0;
static unsigned int y1, u1, v1;
static unsigned int ymin = 255;
static unsigned int umin = 255;
static unsigned int vmin = 255;
static unsigned int ymax = 0;
static unsigned int umax = 0;
static unsigned int vmax = 0;

/* I added this variable to know if the data of the Scan are ready. */
static int scan_ready = 0;

static unsigned char motor_cmd[128] = {
  // assign motor command to each 7-bit tflag pattern
  //      high bits represent clear areas for movement
  //  0000000, 0000001, 0000010, 0000011, 0000100, 0000101, 0000110, 0000111,
  '.', '.', '.', '.', '6', '6', '6', '6',
  //  0001000, 0001001, 0001010, 0001011, 0001100, 0001101, 0001110, 0001111,
  '8', '8', '9', '9', '8', '7', '6', '6',
  //  0010000, 0010001, 0010010, 0010011, 0010100, 0010101, 0010110, 0010111,
  '4', '4', '4', '.', '8', '6', '6', '6',
  //  0011000, 0011001, 0011010, 0011011, 0011100, 0011101, 0011110, 0011111,
  '4', '4', '4', '8', '8', '8', '8', '9',
  //  0100000, 0100001, 0100010, 0100011, 0100100, 0100101, 0100110, 0100111,
  '0', '.', '0', '.', '6', '6', '6', '6',
  //  0101000, 0101001, 0101010, 0101011, 0101100, 0101101, 0101110, 0101111,
  '4', '4', '4', '8', '8', '8', '9', '9',
  //  0110000, 0110001, 0110010, 0110011, 0110100, 0110101, 0110110, 0110111,
  '4', '4', '4', '4', '4', '4', '2', '6',
  //  0111000, 0111001, 0111010, 0111011, 0111100, 0111101, 0111110, 0111111,
  '4', '4', '4', '4', '4', '4', '8', '9',
  //  1000000, 1000001, 1000010, 1000011, 1000100, 1000101, 1000110, 1000111,
  '0', '0', '0', '.', '0', '6', '6', '6',
  //  1001000, 1001001, 1001010, 1001011, 1001100, 1001101, 1001110, 1001111,
  '7', '8', '8', '8', '8', '8', '6', '6',
  //  1010000, 1010001, 1010010, 1010011, 1010100, 1010101, 1010110, 1010111,
  '4', '4', '4', '4', '4', '2', '6', '6',
  //  1011000, 1011001, 1011010, 1011011, 1011100, 1011101, 1011110, 1011111,
  '7', '7', '7', '7', '8', '8', '8', '8',
  //  1100000, 1100001, 1100010, 1100011, 1100100, 1100101, 1100110, 1100111,
  '0', '0', '0', '2', '0', '6', '6', '6',
  //  1101000, 1101001, 1101010, 1101011, 1101100, 1101101, 1101110, 1101111,
  '4', '4', '4', '4', '7', '7', '6', '6',
  //  1110000, 1110001, 1110010, 1110011, 1110100, 1110101, 1110110, 1110111,
  '4', '4', '4', '4', '4', '4', '7', '8',
  //  1111000, 1111001, 1111010, 1111011, 1111100, 1111101, 1111110, 1111111,
  '7', '7', '7', '7', '8', '8', '8', '8'};

/*
 * This function needs to be called before any Wandering. It initialize
 * the needed variables
 */
void surveyor_init_wander(const unsigned char *decode_buf) {
  unsigned int jx, jy;

  ymin = 255;
  umin = 255;
  vmin = 255;
  ymax = 0;
  umax = 0;
  vmax = 0;
  for (jx = 20; jx < 60; jx++) {  // 6 x 40 point initial sample
    for (jy = 0; jy < 6; jy++) {
      y1 = (unsigned int)decode_buf[index(jx, jy) * 4];
      u1 = (unsigned int)decode_buf[index(jx, jy) * 4 + 1];
      v1 = (unsigned int)decode_buf[index(jx, jy) * 4 + 2];
      if (ymax < y1)
        ymax = y1;
      if (ymin > y1)
        ymin = y1;
      if (umax < u1)
        umax = u1;
      if (umin > u1)
        umin = u1;
      if (vmax < v1)
        vmax = v1;
      if (vmin > v1)
        vmin = v1;
    }
  }

  return;
}

/*
 * This function computes the distance to the obstacles using the image
 * contained in decode_buf and return the command which must be executed
 * using the table motor_cmd.
 */
unsigned char surveyor_wandering(unsigned char *decode_buf) {
  unsigned int jx, jy, ix, itmp;

  for (jx = 0; jx < 80; jx++) {  // for each column, measure distance to first mismatch from initial samples
    tvect[jx] = 64;              //      max distance will be 64 pixels (height of image)
    for (jy = 0; jy < 64; jy++) {
      y1 = (unsigned int)decode_buf[index(jx, jy) * 4];
      u1 = (unsigned int)decode_buf[index(jx, jy) * 4 + 1];
      v1 = (unsigned int)decode_buf[index(jx, jy) * 4 + 2];
      if ((y1 < ymin) || (y1 > ymax) || (u1 < umin) || (u1 > umax) || (v1 < vmin) || (v1 > vmax)) {
        tvect[jx] = jy;
        break;
      }
    }
  }

  for (ix = 0; ix < 7; ix++) {  // in groups of 20 columns, compute max, mean and # of zeros
    tmax[ix] = tmean[ix] = tzeros[ix] = 0;
    for (jx = 0; jx < 20; jx++) {
      itmp = tvect[(ix * 10) + jx];
      if (tmax[ix] < itmp)
        tmax[ix] = itmp;
      tmean[ix] += itmp;
      if (itmp == 0)
        tzeros[ix]++;
    }
    tmean[ix] /= 20;
  }

  tflag = 0;                    // flag the 7 open/blocked regions, and pack the flags into a single 7-bit word
  for (ix = 0; ix < 7; ix++) {  // a table lookup (motor_cmd[]) is used to set robot action
    if (tzeros[ix] < 8)
      tflag += (0x40 >> ix);
  }

  scan_ready = 1;

  return motor_cmd[tflag];
}

/*
 *  This function returns the Scan values, if they are ready, under
 *  the form defined by the protocol.
 */
void surveyor_get_scan_values(unsigned char *result) {
  if (scan_ready == 1) {
    memcpy(result, "##Scan - ", 9);
    int ix;
    for (ix = 0; ix < 80; ix++)
      result[9 + ix] = tvect[ix];
    result[9 + ix] = '\n';
  } else
    result[0] = 0;

  return;
}
