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

// indented with "indent -nut -i2 -npcs -nprs -npsl -nsaf -nss -br -brs -l80 -nbfda"

#include <p30F6014A.h>

#define FLOOR_SENSORS           // define to enable floor sensors
#define IR_RECEIVER

#include <string.h>
#include <ctype.h>
#include <stdio.h>

#include <motor_led/e_epuck_ports.h>
#include <motor_led/e_init_port.h>
#include <motor_led/advance_one_timer/e_led.h>
#include <motor_led/advance_one_timer/e_motors.h>
#include <uart/e_uart_char.h>
#include <a_d/advance_ad_scan/e_ad_conv.h>
#include <a_d/advance_ad_scan/e_acc.h>
#include <a_d/advance_ad_scan/e_prox.h>
#include <a_d/advance_ad_scan/e_micro.h>
#include <motor_led/advance_one_timer/e_agenda.h>
#include <camera/fast_2_timer/e_poxxxx.h>
#include <data_eeprom/data_eeprom.h>

#ifdef FLOOR_SENSORS
#include <I2C/e_I2C_protocol.h>
#endif

#ifdef IR_RECEIVER
#include <motor_led/advance_one_timer/e_remote_control.h>
#define SPEED_IR 600
#endif

#define uart_send_static_text(msg) do { e_send_uart1_char(msg,sizeof(msg)-1); while(e_uart1_sending()); } while(0)
#define uart_send_text(msg) do { e_send_uart1_char(msg,strlen(msg)); while(e_uart1_sending()); } while(0)

#define DEFAULT_WIDTH 52
#define DEFAULT_HEIGHT 39
static char __attribute__((far)) buffer[DEFAULT_WIDTH * DEFAULT_HEIGHT * 2 + 3 + 80];

extern int e_mic_scan[3][MIC_SAMP_NB];
extern unsigned int e_last_mic_scan_id;

void calibrate_ir_sensors(int ir_delta[8]) {
  int i = 0, j = 0;
  int long t;
  int long tmp[8];
  e_set_led(8, 1);
  for(t = 0; t < 2000000; ++t);
  e_led_clear();
  for(t = 0; t < 200000; ++t);
  for(; i < 8; ++i) {
    ir_delta[i] = 0;
    tmp[i] = 0;
  }
  for(; j < 100; ++j) {
    for(i = 0; i < 8; ++i) {
      tmp[i] += (e_get_prox(i));
      for(t = 0; t < 1000; ++t);
    }
  }
  for(i = 0; i < 8; ++i) {
    ir_delta[i] = (int) (tmp[i] / (j * 1.0));
  }
}

int main(void) {
  char c, c1, c2, wait_cam = 0;
  int i, j, n, speedr=0, speedl=0, positionr=0, positionl=0, LED_nbr, LED_action, accx=0,
    accy=0, accz=0, selector=0;
  int cam_mode, cam_width, cam_heigth, cam_zoom, cam_size, cam_x1, cam_y1;
  int ir_delta[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };
  char *address;
  char *ptr;
  int HWversion=0xFFFF;
#ifdef IR_RECEIVER
  char ir_move = 0, ir_address = 0, ir_last_move = 0;
#endif
  TypeAccSpheric accelero;
  e_init_port(); // configure port pins
  e_start_agendas_processing();
  e_init_motors();
  e_init_uart1(); // initialize UART to 115200 Kbaud
  e_init_ad_scan(ALL_ADC);
#ifdef FLOOR_SENSORS
  e_i2cp_init();
#endif
#ifdef IR_RECEIVER
  e_init_remote_control();
#endif
  if (RCONbits.POR) { // reset if power on (some problem for a few new robots)
    RCONbits.POR = 0;
    RESET();
  }
  // camera default parameters
  ReadEE(0x7F,0xFFFE,&HWversion, 1); //get camera model
  cam_mode = RGB_565_MODE;
  cam_width = 40;
  cam_heigth = 40;
  cam_zoom = 8;
  cam_size = cam_width * cam_heigth * 2;
  e_poxxxx_init_cam();
  e_poxxxx_config_cam((ARRAY_WIDTH - cam_width * cam_zoom) / 2,
                       (ARRAY_HEIGHT - cam_heigth * cam_zoom) / 2,
                       cam_width * cam_zoom, cam_heigth * cam_zoom,
                       cam_zoom, cam_zoom, cam_mode);
  e_poxxxx_write_cam_registers();
  e_acc_calibr();
  uart_send_static_text("\f\a"
                        "WELCOME to the SerCom protocol on e-Puck\r\n"
                        "the EPFL education robot type \"H\" for help\r\n");
  while (1) {
    while (e_getchar_uart1(&c) == 0)
#ifdef IR_RECEIVER
    {
      ir_move = e_get_data();
      ir_address = e_get_address();
      if (((ir_address == 0) || (ir_address == 8)) && (ir_move != ir_last_move)) {
        switch (ir_move) {
        case 1:
          speedr = SPEED_IR;
          speedl = SPEED_IR / 2;
          break;
        case 2:
          speedr = SPEED_IR;
          speedl = SPEED_IR;
          break;
        case 3:
          speedr = SPEED_IR / 2;
          speedl = SPEED_IR;
          break;
        case 4:
          speedr = SPEED_IR;
          speedl = -SPEED_IR;
          break;
        case 5:
          speedr = 0;
          speedl = 0;
          break;
        case 6:
          speedr = -SPEED_IR;
          speedl = SPEED_IR;
          break;
        case 7:
          speedr = -SPEED_IR;
          speedl = -SPEED_IR / 2;
          break;
        case 8:
          speedr = -SPEED_IR;
          speedl = -SPEED_IR;
          break;
        case 9:
          speedr = -SPEED_IR / 2;
          speedl = -SPEED_IR;
          break;
        //case 0:
          // sound case: not implemented
        //  break;
        default:
          speedr = speedl = 0;
        }
        ir_last_move = ir_move;
        e_set_speed_left(speedl);
        e_set_speed_right(speedr);
      }
    }
#else
      ;
#endif
    if (c < 0) {                // binary mode (big endian)
      i = 0;
      do {
        switch (-c) {
        case 'a': // Read acceleration sensors in a non
                  // filtered way, some as ASCII
          accx = e_get_acc_filtered(0, 1);
          accy = e_get_acc_filtered(1, 1);
          accz = e_get_acc_filtered(2, 1);

          buffer[i++] = accx & 0xff;
          buffer[i++] = accx >> 8;

          buffer[i++] = accy & 0xff;
          buffer[i++] = accy >> 8;

          buffer[i++] = accz & 0xff;
          buffer[i++] = accz >> 8;

          break;
        case 'A': // read acceleration sensors
          accelero = e_read_acc_spheric();
          ptr = (char *) &accelero.acceleration;
          buffer[i++] = (*ptr);
          ptr++;
          buffer[i++] = (*ptr);
          ptr++;
          buffer[i++] = (*ptr);
          ptr++;
          buffer[i++] = (*ptr);

          ptr = (char *) &accelero.orientation;
          buffer[i++] = (*ptr);
          ptr++;
          buffer[i++] = (*ptr);
          ptr++;
          buffer[i++] = (*ptr);
          ptr++;
          buffer[i++] = (*ptr);

          ptr = (char *) &accelero.inclination;
          buffer[i++] = (*ptr);
          ptr++;
          buffer[i++] = (*ptr);
          ptr++;
          buffer[i++] = (*ptr);
          ptr++;
          buffer[i++] = (*ptr);

          break;
        case 'D': // set motor speed
          while (e_getchar_uart1(&c1)==0);
          while (e_getchar_uart1(&c2)==0);
          speedl = (unsigned char) c1 + ((unsigned int) c2 << 8);
          while (e_getchar_uart1(&c1)==0);
          while (e_getchar_uart1(&c2)==0);
          speedr = (unsigned char) c1 + ((unsigned int) c2 << 8);
          e_set_speed_left(speedl);
          e_set_speed_right(speedr);
          break;
        case 'E': // get motor speed
          buffer[i++] = speedl & 0xff;
          buffer[i++] = speedl >> 8;
          buffer[i++] = speedr & 0xff;
          buffer[i++] = speedr >> 8;
          break;
        case 'I': // get camera image
          e_poxxxx_launch_capture(&buffer[i + 3]);
          wait_cam = 1;
          buffer[i++] = (char) cam_mode & 0xff; // send image parameter
          buffer[i++] = (char) cam_width & 0xff;
          buffer[i++] = (char) cam_heigth & 0xff;
          i += cam_size;
          break;
        case 'L': // set LED
          while (e_getchar_uart1(&c1)==0);
          while (e_getchar_uart1(&c2)==0);
          switch (c1) {
          case 8:
            e_set_body_led(c2);
            break;
          case 9:
            e_set_front_led(c2);
            break;
          default:
            e_set_led(c1, c2);
            break;
          }
          break;
        case 'M': // optional floor sensors
#ifdef FLOOR_SENSORS
          e_i2cp_enable();

          for (j=0; j<3; ++j) {
            buffer[i++] = e_i2cp_read(0xC0,2*j+1);
            buffer[i++] = e_i2cp_read(0xC0,2*j);
          }

          e_i2cp_disable();
#else
          for(j = 0; j < 6; j++) buffer[i++] = 0;
#endif
          break;
        case 'N': // read proximity sensors
          for(j = 0; j < 8; j++) {
            n = e_get_prox(j);
            buffer[i++] = n & 0xff;
            buffer[i++] = n >> 8;
          }
          break;
        case 'O': // read light sensors
          for(j = 0; j < 8; j++) {
            n = e_get_ambient_light(j);
            buffer[i++] = n & 0xff;
            buffer[i++] = n >> 8;
          }
          break;
        case 'P': // set encoders
          while (e_getchar_uart1(&c1)==0);
          while (e_getchar_uart1(&c2)==0);
          positionl = (unsigned char) c1 + ((unsigned int) c2 << 8);
          while (e_getchar_uart1(&c1)==0);
          while (e_getchar_uart1(&c2)==0);
          positionr = (unsigned char) c1 + ((unsigned int) c2 << 8);
          e_set_steps_left(positionl);
          e_set_steps_right(positionr);
          break;
        case 'Q': // read encoders
          n = e_get_steps_left();
          buffer[i++] = n & 0xff;
          buffer[i++] = n >> 8;
          n = e_get_steps_right();
          buffer[i++] = n & 0xff;
          buffer[i++] = n >> 8;
          break;
        case 'u': // get last micro volumes
          n = e_get_micro_volume(0);
          buffer[i++] = n & 0xff;
          buffer[i++] = n >> 8;

          n = e_get_micro_volume(1);
          buffer[i++] = n & 0xff;
          buffer[i++] = n >> 8;

          n = e_get_micro_volume(2);
          buffer[i++] = n & 0xff;
          buffer[i++] = n >> 8;
          break;
        case 'U': // get micro buffer
          address = (char *) e_mic_scan;
          e_send_uart1_char(address, 600); // send sound buffer
          n = e_last_mic_scan_id;          // send last scan
          buffer[i++] = n & 0xff;
          break;
        default: // silently ignored
          break;
        }
        while (e_getchar_uart1(&c)==0);
      }
      while (c);
      if (i != 0) {
        if (wait_cam) {
          wait_cam = 0;
          while (!e_poxxxx_is_img_ready());
        }
        e_send_uart1_char(buffer, i); // send answer
        while (e_uart1_sending());
      }
    }
    else if (c > 0) { // ascii mode
      while (c == '\n' || c == '\r') e_getchar_uart1(&c);
      buffer[0] = c;
      i = 1;
      do if (e_getchar_uart1(&c)) buffer[i++] = c;
      while (c != '\n' && c != '\r');
      buffer[i++] = '\0';
      buffer[0] = toupper(buffer[0]);   // we also accept lowercase letters
      switch (buffer[0]) {
      case 'A': // read accelerometer
        accx = e_get_acc(0);
        accy = e_get_acc(1);
        accz = e_get_acc(2);
        sprintf(buffer, "a,%d,%d,%d\r\n", accx, accy, accz);
        uart_send_text(buffer);
        break;
      case 'B': // set body led
        sscanf(buffer, "B,%d\r", &LED_action);
        e_set_body_led(LED_action);
        uart_send_static_text("b\r\n");
        break;
      case 'C': // read selector position
        selector = SELECTOR0 + 2 * SELECTOR1 + 4 * SELECTOR2 + 8 * SELECTOR3;
        sprintf(buffer, "c,%d\r\n", selector);
        uart_send_text(buffer);
        break;
      case 'D': // set motor speed
        sscanf(buffer, "D,%d,%d\r", &speedl, &speedr);
        e_set_speed_left(speedl);
        e_set_speed_right(speedr);
        uart_send_static_text("d\r\n");
        break;
      case 'E': // read motor speed
        sprintf(buffer, "e,%d,%d\r\n", speedl, speedr);
        uart_send_text(buffer);
        break;
      case 'F': // set front led
        sscanf(buffer, "F,%d\r", &LED_action);
        e_set_front_led(LED_action);
        uart_send_static_text("f\r\n");
        break;
#ifdef IR_RECEIVER
      case 'G':
        sprintf(buffer,
                "g IR check : 0x%x, address : 0x%x, data : 0x%x\r\n",
                e_get_check(), e_get_address(), e_get_data());
        uart_send_text(buffer);
        break;
#endif
      case 'H': // ask for help
        uart_send_static_text("\n");
        uart_send_static_text("\"A\"         Accelerometer\r\n");
        uart_send_static_text("\"B,#\"       Body led 0=off 1=on 2=inverse\r\n");
        uart_send_static_text("\"C\"         Selector position\r\n");
        uart_send_static_text("\"D,#,#\"     Set motor speed left,right\r\n");
        uart_send_static_text("\"E\"         Get motor speed left,right\r\n");
        uart_send_static_text("\"F,#\"       Front led 0=off 1=on 2=inverse\r\n");
#ifdef IR_RECEIVER
        uart_send_static_text("\"G\"         IR receiver\r\n");
#endif
        uart_send_static_text("\"H\"	     Help\r\n");
        uart_send_static_text("\"I\"         Get camera parameter\r\n");
        uart_send_static_text("\"J,#,#,#,#\" Set camera parameter mode,width,heigth,zoom(1,4 or 8)\r\n");
        uart_send_static_text("\"K\"         Calibrate proximity sensors\r\n");
        uart_send_static_text("\"L,#,#\"     Led number,0=off 1=on 2=inverse\r\n");
#ifdef FLOOR_SENSORS
        uart_send_static_text("\"M\"         Floor sensors\r\n");
#endif
        uart_send_static_text("\"N\"         Proximity\r\n");
        uart_send_static_text("\"O\"         Light sensors\r\n");
        uart_send_static_text("\"P,#,#\"     Set motor position left,right\r\n");
        uart_send_static_text("\"Q\"         Get motor position left,right\r\n");
        uart_send_static_text("\"R\"         Reset e-puck\r\n");
        uart_send_static_text("\"S\"         Stop e-puck and turn off leds\r\n");
        uart_send_static_text("\"U\"         Get microphone amplitude\r\n");
        uart_send_static_text("\"V\"         Version of SerCom\r\n");
        break;
      case 'I':
        sprintf(buffer, "i,%d,%d,%d,%d,%d\r\n", cam_mode,
                cam_width, cam_heigth, cam_zoom, cam_size);
        uart_send_text(buffer);
        break;
      case 'J':                // set camera parameter see also cam
        cam_x1 = -1;
        cam_y1 = -1;
        sscanf(buffer, "J,%d,%d,%d,%d,%d,%d\r", &cam_mode, &cam_width, &cam_heigth, &cam_zoom, &cam_x1, &cam_y1);
        if (cam_mode == GREY_SCALE_MODE)
          cam_size = cam_width * cam_heigth;
        else
          cam_size = cam_width * cam_heigth * 2;
        if (cam_size > DEFAULT_WIDTH * DEFAULT_HEIGHT) { // if desired settings too demanding set to a reasonable default
          cam_mode = RGB_565_MODE;
          cam_width = DEFAULT_WIDTH;
          cam_heigth = DEFAULT_HEIGHT;
          cam_size = cam_width * cam_heigth * 2;
        }
        e_poxxxx_init_cam();
        if (cam_x1 == -1) { // user did not specify: take default
          cam_x1 = (ARRAY_WIDTH - cam_width * cam_zoom) / 2;
        }
        if (cam_y1 == -1) { // user did not specify: take default
          cam_y1 = (ARRAY_HEIGHT - cam_heigth * cam_zoom) / 2;
        }
        e_poxxxx_config_cam(cam_x1, cam_y1, cam_width*cam_zoom, cam_heigth*cam_zoom, cam_zoom, cam_zoom, cam_mode);
        e_poxxxx_write_cam_registers();
        uart_send_static_text("j\r\n");
        break;
      case 'K': // calibrate proximity sensors
        uart_send_static_text
          ("k, Starting calibration - Remove any object in sensors range\r\n");
        calibrate_ir_sensors(ir_delta);
        uart_send_static_text("k, Calibration finished\r\n");
        break;
      case 'L': // set led
        sscanf(buffer, "L,%d,%d\r", &LED_nbr, &LED_action);
        e_set_led(LED_nbr, LED_action);
        uart_send_static_text("l\r\n");
        break;
      case 'M': // read floor sensors (optional)
#ifdef FLOOR_SENSORS
        e_i2cp_enable();
        for(j = 0; j < 6; j++) {
          if (j % 2 == 0) buffer[j] = e_i2cp_read(0xC0, j + 1);
          else            buffer[j] = e_i2cp_read(0xC0, j - 1);
        }
        e_i2cp_disable();
        sprintf(buffer, "m,%d,%d,%d\r\n",
                (unsigned int) (buffer[0] & 0xff) +
                ((unsigned int) buffer[1] << 8),
                (unsigned int) (buffer[2] & 0xff) +
                ((unsigned int) buffer[3] << 8),
                (unsigned int) (buffer[4] & 0xff) +
                ((unsigned int) buffer[5] << 8));
        uart_send_text(buffer);
#else
        uart_send_static_text("m,0,0,0\r\n");
#endif
        break;
      case 'N':                // read proximity sensors
        sprintf(buffer, "n,%d,%d,%d,%d,%d,%d,%d,%d\r\n",
                e_get_prox(0) - ir_delta[0],
                e_get_prox(1) - ir_delta[1],
                e_get_prox(2) - ir_delta[2],
                e_get_prox(3) - ir_delta[3],
                e_get_prox(4) - ir_delta[4],
                e_get_prox(5) - ir_delta[5],
                e_get_prox(6) - ir_delta[6],
                e_get_prox(7) - ir_delta[7]);
        uart_send_text(buffer);
        break;
      case 'O':                // read ambient light sensors
        sprintf(buffer, "o,%d,%d,%d,%d,%d,%d,%d,%d\r\n",
                e_get_ambient_light(0), e_get_ambient_light(1),
                e_get_ambient_light(2), e_get_ambient_light(3),
                e_get_ambient_light(4), e_get_ambient_light(5),
                e_get_ambient_light(6), e_get_ambient_light(7));
        uart_send_text(buffer);
        break;
      case 'P':                // set motor position
        sscanf(buffer, "P,%d,%d\r", &positionl, &positionr);
        e_set_steps_left(positionl);
        e_set_steps_right(positionr);
        uart_send_static_text("p\r\n");
        break;
      case 'Q':                // read motor position
        sprintf(buffer, "q,%d,%d\r\n", e_get_steps_left(), e_get_steps_right());
        uart_send_text(buffer);
        break;
      case 'R':                // reset
        uart_send_static_text("r\r\n");
        RESET();
        break;
      case 'S':                // stop
        speedr=0;
        speedl=0;
        e_set_speed_left(speedl);
        e_set_speed_right(speedr);
        e_set_led(8, 0);
        e_set_body_led(0);
        e_set_front_led(0);
        uart_send_static_text("s\r\n");
        break;
      // case 'T':  // play sound : Not implemented to save memory
      // break;
      case 'U':
        sprintf(buffer, "u,%d,%d,%d\r\n", e_get_micro_volume(0),
                e_get_micro_volume(1), e_get_micro_volume(2));
        uart_send_text(buffer);
        break;
      case 'V':                // get version information
        uart_send_static_text("v,Version 1.5.3 November 2016 (Webots)\r\n");
        sprintf(buffer,"HW version: %X\r\n",HWversion);
        uart_send_text(buffer);
        break;
      case 'W':                // read Devantec ultrasonic range sensor
        // or Sharp Ir sensor (optional)
        break;

      case 'X':                // Dummy command returning a number of
        // bytes given as parameter
        sscanf(buffer, "X,%d\r", &positionl);
        buffer[positionl + 2] = '\r';
        buffer[positionl + 3] = '\n';
        while (positionl > 0) {
          buffer[positionl + 1] = '1';
          positionl--;
        }
        buffer[0] = 'x';
        buffer[1] = ',';
        uart_send_text(buffer);
        break;
      default:
        uart_send_static_text("z,Command not found\r\n");
        break;
      }
    }
  }
}
