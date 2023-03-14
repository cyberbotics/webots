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

#include <stdlib.h>
#include <string.h>

#include <motor_led/e_epuck_ports.h>
#include <motor_led/e_init_port.h>
#include <motor_led/advance_one_timer/e_led.h>
#include <motor_led/advance_one_timer/e_motors.h>
#include <a_d/advance_ad_scan/e_ad_conv.h>
#include <a_d/advance_ad_scan/e_prox.h>
#include <a_d/advance_ad_scan/e_acc.h>   //accelerometer
#include <motor_led/advance_one_timer/e_agenda.h> //uses Timer2
#include <camera/fast_2_timer/e_poxxxx.h>        //camera - uses Timer 4 and 5
#include <I2C/e_I2C_protocol.h> // floor sensors

//Futur developments ?
//#include <a_d/advance_ad_scan/e_micro.h> //micro
//#include <codec/e_sound.h>               //sound
//#include <uart/e_uart_char.h>            //emitter - receiver

//Camera
#define CAM_WIDTH 52
#define CAM_HEIGHT 39
#define CAM_ZOOM 8

#define CAM_B1(x,y) (2 * (x + y * CAM_WIDTH))
#define CAM_B2(x,y) (1 + CAM_B1(x,y))

//DeviceTags
typedef unsigned char DeviceTag;
#define DT_NULL       (DeviceTag) 0
#define DT_MOTOR_L    (DeviceTag) 1
#define DT_MOTOR_R    (DeviceTag) 2
#define DT_POSITION_SENSOR_L  (DeviceTag) 3
#define DT_POSITION_SENSOR_R  (DeviceTag) 4
#define DT_LED0       (DeviceTag) 10
#define DT_LED1       (DeviceTag) 11
#define DT_LED2       (DeviceTag) 12
#define DT_LED3       (DeviceTag) 13
#define DT_LED4       (DeviceTag) 14
#define DT_LED5       (DeviceTag) 15
#define DT_LED6       (DeviceTag) 16
#define DT_LED7       (DeviceTag) 17
#define DT_BODY_LED   (DeviceTag) 18
#define DT_FRONT_LED  (DeviceTag) 19
#define DT_PS0        (DeviceTag) 20
#define DT_PS1        (DeviceTag) 21
#define DT_PS2        (DeviceTag) 22
#define DT_PS3        (DeviceTag) 23
#define DT_PS4        (DeviceTag) 24
#define DT_PS5        (DeviceTag) 25
#define DT_PS6        (DeviceTag) 26
#define DT_PS7        (DeviceTag) 27
#define DT_LS0        (DeviceTag) 30
#define DT_LS1        (DeviceTag) 31
#define DT_LS2        (DeviceTag) 32
#define DT_LS3        (DeviceTag) 33
#define DT_LS4        (DeviceTag) 34
#define DT_LS5        (DeviceTag) 35
#define DT_LS6        (DeviceTag) 36
#define DT_LS7        (DeviceTag) 37
#define DT_CAM        (DeviceTag) 40
#define DT_ACC        (DeviceTag) 50
#define DT_FS0        (DeviceTag) 61
#define DT_FS1        (DeviceTag) 62
#define DT_FS2        (DeviceTag) 63

static int last_duration = 0;
static int duration_counter = 0;
static int step_counter = 0;
static double s_counter = 0.0;

// Proximity Sensors
static int ps_enable[] = {0,0,0,0,0,0,0,0};
static int ps_sampling_period[] = {0,0,0,0,0,0,0,0};
static double ps_last_value[] = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};

// Floor Sensors
static int fs_enable[] = {0,0,0};
static int fs_sampling_period[] = {0,0,0};
static double fs_last_value[] = {0.0,0.0,0.0};

// Light Sensors
static int ls_enable[] = {0,0,0,0,0,0,0,0};
static int ls_sampling_period[] = {0,0,0,0,0,0,0,0};
static double ls_last_value[] = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};

// Position Sensors
static int position_sensor_enable[] = {0,0};
static int position_sensor_sampling_period[] = {0,0};
static double position_sensor_last_value[] = {0.0,0.0};

// Camera
static int cam_enable = 0;
static int cam_sampling_period = 0;
static unsigned char __attribute__((far)) buffer[2*CAM_WIDTH*CAM_HEIGHT]; //image

// Accelerometer
static int acc_enable = 0;
static int accelerometer_sampling_period = 0;
static double accelerometer_last_value[] = {0.0,0.0,0.0};

// Motors
static double motor_speed_last_value[] = {0.0,0.0};

/*******************************
 internal functions
********************************/

// Timer 1 is used for time step management
void initTimer(void)
{
  T1CON = 0;
  T1CONbits.TCKPS=3;        // prescsaler = 256
  TMR1 = 0;                 // clear timer 1
  PR1 = (MILLISEC)/256.0;   // interrupt every ms with 256 prescaler
  IFS0bits.T1IF = 0;        // clear interrupt flag
  IEC0bits.T1IE = 1;        // set interrupt enable bit
  T1CONbits.TON = 1;        // start Timer1
}

void __attribute__((interrupt, auto_psv, shadow)) _T1Interrupt(void)
{
  IFS0bits.T1IF = 0;        // clear interrupt flag
  TMR1 = 0;                 // reset timer 1
  duration_counter++;       // duration_counter is incremented every ms
}

extern void CamInterrupt();

void __attribute__((interrupt, auto_psv, shadow)) _T4Interrupt(void)
{
  CamInterrupt();
}

void init_cam(void)
{
  e_poxxxx_init_cam();
  e_poxxxx_config_cam((ARRAY_WIDTH - CAM_WIDTH * CAM_ZOOM) / 2,
                      (ARRAY_HEIGHT - CAM_HEIGHT * CAM_ZOOM) / 2,
                      CAM_WIDTH * CAM_ZOOM, CAM_HEIGHT * CAM_ZOOM,
                      CAM_ZOOM, CAM_ZOOM, RGB_565_MODE);
  e_poxxxx_write_cam_registers();
}

void wb_robot_init(void)
{
  e_init_port();
  e_start_agendas_processing();
  e_init_motors();
  e_init_ad_scan(ALL_ADC);
  e_acc_calibr();
  e_i2cp_init(); // floor sensors
  initTimer();
  init_cam();
}

void wb_robot_cleanup(void)
{
}

/*******************************
 robot functions
********************************/

int wb_robot_step(int duration)
{
  unsigned int delay = duration_counter; // 0
  while (delay < duration) {
     delay = duration_counter;
  }
  initTimer();
  duration_counter=0; // if not reseted, the e-puck restarts
                // when the counter reaches the upper bound
  last_duration = duration;
  step_counter++;
  if(last_duration !=0 && step_counter > 1000 / last_duration) {
    step_counter=0; // reseted every second
                    // if not reseted, the e-puck restarts
                    // when the counter reaches the upper bound
    s_counter+=1.0;
  }
  return delay;
}

DeviceTag wb_robot_get_device(const char *name)
{
  if (name==NULL) return DT_NULL;
  if (
    name[0]=='p' &&
    name[1]=='s' &&
    name[3]=='\0') {
      if (name[2]>='0' && name[2]<='7') return name[2]-'0'+DT_PS0;
  } else if (
    name[0]=='l' &&
    name[1]=='s' &&
    name[3]=='\0') {
      if (name[2]>='0' && name[2]<='7') return name[2]-'0'+DT_LS0;
  } else if (
    name[0]=='g' &&
    name[1]=='s' &&
    name[3]=='\0') {
      if (name[2]>='0' && name[2]<='2') return name[2]-'0'+DT_FS0;
  } else if (
    name[0]=='l' &&
    name[1]=='e' &&
    name[2]=='d' &&
    name[4]=='\0') {
      if (name[3]>='0' && name[3]<='9') return name[3]-'0'+DT_LED0;
  } else if (
    name[0]=='c' &&
    name[1]=='a' &&
    name[2]=='m' &&
    name[3]=='e' &&
    name[4]=='r' &&
    name[5]=='a' &&
    name[6]=='\0') {
      return DT_CAM;
  } else if (
    name[0]=='a' &&
    name[1]=='c' &&
    name[2]=='c' &&
    name[3]=='e' &&
    name[4]=='l' &&
    name[5]=='e' &&
    name[6]=='r' &&
    name[7]=='o' &&
    name[8]=='m' &&
    name[9]=='e' &&
    name[10]=='t' &&
    name[11]=='e' &&
    name[12]=='r' &&
    name[13]=='\0') {
      return DT_ACC;
  } else if (
    name[0]=='l' &&
    name[1]=='e' &&
    name[2]=='f' &&
    name[3]=='t' &&
    name[4]==' ' &&
    name[5]=='w' &&
    name[6]=='h' &&
    name[7]=='e' &&
    name[8]=='e' &&
    name[9]=='l' &&
    name[10]==' '
  ) {
    if (
      name[11]=='m' &&
      name[12]=='o' &&
      name[13]=='t' &&
      name[14]=='o' &&
      name[15]=='r' &&
      name[16]=='\0'
    ) {
      return DT_MOTOR_L;
    } else if (
      name[11]=='s' &&
      name[12]=='e' &&
      name[13]=='n' &&
      name[14]=='s' &&
      name[15]=='o' &&
      name[16]=='r' &&
      name[17]=='\0'
    ) {
      return DT_POSITION_SENSOR_L;
    }
  }  else if (
    name[0]=='r' &&
    name[1]=='i' &&
    name[2]=='g' &&
    name[3]=='h' &&
    name[4]=='t' &&
    name[5]==' ' &&
    name[6]=='w' &&
    name[7]=='h' &&
    name[8]=='e' &&
    name[9]=='e' &&
    name[10]=='l' &&
    name[11]==' '
  ) {
    if (
      name[12]=='m' &&
      name[13]=='o' &&
      name[14]=='t' &&
      name[15]=='o' &&
      name[16]=='r' &&
      name[17]=='\0'
    ) {
      return DT_MOTOR_R;
    } else if (
      name[12]=='s' &&
      name[13]=='e' &&
      name[14]=='n' &&
      name[15]=='s' &&
      name[16]=='o' &&
      name[17]=='r' &&
      name[18]=='\0'
    ) {
      return DT_POSITION_SENSOR_R;
    }
  }
  return DT_NULL;
}

double wb_robot_get_time(void){
  return s_counter;
}

/*******************************
 led functions
********************************/

void wb_led_set(DeviceTag dt,int value)
{
  if (value==0 || value==1){
    if (dt == DT_BODY_LED) e_set_body_led(value);
    if (dt == DT_FRONT_LED) e_set_front_led(value);
    if (dt>=DT_LED0 && dt <=DT_LED7) e_set_led(dt-DT_LED0,value);
  }
}

/*******************************
 distance_sensor functions
********************************/

void wb_distance_sensor_enable(DeviceTag dt,int sampling_period)
{
  if (dt>=DT_PS0 && dt<=DT_PS7) {
    ps_enable[dt-DT_PS0]=1;
    ps_sampling_period[dt-DT_PS0]=sampling_period;
  } else if (dt>=DT_FS0 && dt<=DT_FS2) {
    fs_enable[dt-DT_FS0]=1;
    fs_sampling_period[dt-DT_FS0]=sampling_period;
  }
}

void wb_distance_sensor_disable(DeviceTag dt)
{
  if (dt>=DT_PS0 && dt<=DT_PS7) {
    ps_enable[dt-DT_PS0]=0;
  } else if (dt>=DT_FS0 && dt<=DT_FS2) {
    fs_enable[dt-DT_FS0]=0;
  }
}

double wb_distance_sensor_get_value(DeviceTag dt)
{
  if (dt>=DT_PS0 && dt<=DT_PS7 && ps_enable[dt-DT_PS0]) {
    if (last_duration==0 || (step_counter%(ps_sampling_period[dt-DT_PS0]/last_duration))==0) {
        ps_last_value[dt-DT_PS0] = e_get_prox(dt-DT_PS0);
    }
    return ps_last_value[dt-DT_PS0];
  } else if (dt>=DT_FS0 && dt<=DT_FS2 && fs_enable[dt-DT_FS0]) {
    if (last_duration==0 || (step_counter%(fs_sampling_period[dt-DT_FS0]/last_duration))==0) {
      e_i2cp_enable();
      fs_last_value[dt-DT_FS0] =
        (double) ((unsigned int) e_i2cp_read(0xC0,2*(dt-DT_FS0)+1) +
                  (((unsigned int) e_i2cp_read(0xC0,2*(dt-DT_FS0)))<<8));
      e_i2cp_disable();
    }
    return fs_last_value[dt-DT_FS0];
  }
  return 0.0;
}

/*******************************
 light_sensor functions
********************************/

void wb_light_sensor_enable(DeviceTag dt,int sampling_period)
{
  if (dt>=DT_LS0 && dt<=DT_LS7) {
    ls_enable[dt-DT_LS0]=1;
    ls_sampling_period[dt-DT_LS0]=sampling_period;
  }
}

void wb_light_sensor_disable(DeviceTag dt)
{
  if (dt>=DT_LS0 && dt<=DT_LS7) {
    ls_enable[dt-DT_LS0]=0;
  }
}

double wb_light_sensor_get_value(DeviceTag dt)
{
  if (dt>=DT_LS0 && dt<=DT_LS7 && ls_enable[dt-DT_LS0]) {
    if (last_duration==0 || (step_counter%(ls_sampling_period[dt-DT_LS0]/last_duration))==0) {
        ls_last_value[dt-DT_LS0] = e_get_ambient_light(dt-DT_LS0);
    }
    return ls_last_value[dt-DT_LS0];
  }
  return 0.0;
}

/*******************************
 motor functions
********************************/

void wb_motor_set_velocity(DeviceTag dt, double velocity)
{
  if (dt == DT_MOTOR_L) {
    motor_speed_last_value[0] = velocity;
    e_set_speed_left(velocity / 0.00628);  // 0.00628 = ( 2 * pi) / encoder_resolution
  } else if (dt == DT_MOTOR_R) {
    motor_speed_last_value[1] = velocity;
    e_set_speed_right(velocity / 0.00628);
  }
}

double wb_motor_get_velocity(DeviceTag dt)
{
  if (dt == DT_MOTOR_L)
    return motor_speed_last_value[0];
  else if (dt == DT_MOTOR_R)
    return motor_speed_last_value[1];
  return 0.0;
}

/*******************************
 position sensor functions
********************************/

void wb_position_sensor_enable(DeviceTag dt, int sampling_period)
{
  if (dt == DT_POSITION_SENSOR_L) {
    position_sensor_enable[0] = 1;
    position_sensor_sampling_period[0] = sampling_period;
  } else if (dt == DT_POSITION_SENSOR_R) {
    position_sensor_enable[1] = 1;
    position_sensor_sampling_period[1] = sampling_period;
  }
}

void wb_position_sensor_disable(DeviceTag dt)
{
  if (dt == DT_POSITION_SENSOR_L)
    position_sensor_enable[0] = 0;
  else if (dt == DT_POSITION_SENSOR_R)
    position_sensor_enable[1] = 0;
}

double wb_position_sensor_get_value(DeviceTag dt)
{
  int index = -1;
  if (dt == DT_POSITION_SENSOR_L)
    index = 0;
  else if (dt == DT_POSITION_SENSOR_R)
    index = 1;
  if (index < 0)
    return 0.0;
  if (position_sensor_enable[index] && position_sensor_sampling_period[index]) {
    if (last_duration == 0 || (step_counter % (position_sensor_sampling_period[index] / last_duration)) == 0) {
      if (index == 0)
        position_sensor_last_value[0] = e_get_steps_left() * 0.00628;  // 0.00628 = ( 2 * pi) / encoder_resolution
      else if (index == 1)
        position_sensor_last_value[1] = e_get_steps_right() * 0.00628;
    }
    return position_sensor_last_value[index];
  }
  return 0.0;
}

/*******************************
 camera functions
********************************/

void wb_camera_enable(DeviceTag dt,int samping_period)
{
  if (dt==DT_CAM) {
    cam_sampling_period = samping_period;
    cam_enable = 1;
  }
}

void wb_camera_disable(DeviceTag dt)
{
  if (dt==DT_CAM) {
    cam_enable = 0;
  }
}

const unsigned char *wb_camera_get_image(DeviceTag dt)
{
  if (dt==DT_CAM && cam_enable && cam_sampling_period) {
    if (last_duration==0 || (step_counter%(cam_sampling_period/last_duration))==0) {
       e_poxxxx_launch_capture((char *) buffer);
       while(!e_poxxxx_is_img_ready());
    }
    return buffer;
  }
  return NULL;
}

unsigned char wb_camera_image_get_red(const unsigned char* image,int width,int x,int y)
{
  return image[CAM_B1(x,y)] & 0xF8;
}

unsigned char wb_camera_image_get_green(const unsigned char* image,int width,int x,int y)
{
  return (image[CAM_B1(x,y)] << 5) + ((image[CAM_B2(x,y)] & 0xF8)>>3);
}

unsigned char wb_camera_image_get_blue(const unsigned char* image,int width,int x,int y)
{
  return image[CAM_B2(x,y)] << 3;
}

/*******************************
 accelerometer functions
********************************/

void wb_accelerometer_enable(DeviceTag dt,int samping_period)
{
  if (dt==DT_ACC) {
    accelerometer_sampling_period = samping_period;
    acc_enable = 1;
  }
}

void wb_accelerometer_disable(DeviceTag dt)
{
  if (dt==DT_ACC) {
    acc_enable = 0;
  }
}

const double *wb_accelerometer_get_values(DeviceTag dt)
{
  if (dt==DT_ACC && acc_enable && accelerometer_sampling_period) {
    if (last_duration==0 || (step_counter%(accelerometer_sampling_period/last_duration))==0) {
      int acc[3] = {e_read_acc_x(),e_read_acc_y(),e_read_acc_z()};
      double calibr[3] = {9.81/800.0,9.81/800.0,9.81/800.0}; // same calibration as in webots
      accelerometer_last_value[0]=-calibr[0]*acc[0]; // for having an orthonormal basis
      accelerometer_last_value[1]=calibr[1]*acc[1];
      accelerometer_last_value[2]=calibr[2]*acc[2];
    }
    return accelerometer_last_value;
  }
  accelerometer_last_value[0]=0.0;
  accelerometer_last_value[1]=0.0;
  accelerometer_last_value[2]=0.0;
  return accelerometer_last_value;
}

/*******************************
 other functions
********************************/

int printf_override(const char *fmt,...)
{
  return strlen(fmt);
}
