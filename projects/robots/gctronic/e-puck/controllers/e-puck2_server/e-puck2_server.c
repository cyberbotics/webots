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
 * Description:  A simple program implementing a TCP/IP server emulating the
 *               behavior of the e-puck2 robot, so that you can connect the
 *               EPuckMonitor (Wi-Fi version) and e-puck2.vi LabVIEW panel on it.
 */

#include <math.h>
#include <stdio.h>
#include <string.h>
#include <sys/types.h>
#include <time.h>
#ifdef _WIN32
#include <winsock.h>
#else
#include <arpa/inet.h> /* definition of inet_ntoa */
#include <errno.h>
#include <fcntl.h>
#include <netdb.h>      /* definition of gethostbyname */
#include <netinet/in.h> /* definition of struct sockaddr_in */
#include <stdlib.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h> /* definition of close */
#endif

#include <webots/accelerometer.h>
#include <webots/distance_sensor.h>
#include <webots/led.h>
#include <webots/light_sensor.h>
#include <webots/motor.h>
#include <webots/robot.h>
#include <webots/speaker.h>

#include "play_melody.h"

#define MOTOR_RATIO 0.00628

#ifdef _WIN32 /* initialize the socket API */
static bool socket_init() {
  WSADATA info;
  if (WSAStartup(MAKEWORD(1, 1), &info) != 0) {
    fprintf(stderr, "Cannot initialize Winsock.\n");
    return false;
  }
  return true;
}
#endif

static bool socket_set_non_blocking(int fd) {
  if (fd < 0)
    return false;
#ifdef _WIN32
  unsigned long mode = 1;
  return (ioctlsocket(fd, FIONBIO, &mode) == 0) ? true : false;
#else
  int flags = fcntl(fd, F_GETFL, 0) | O_NONBLOCK;
  return (fcntl(fd, F_SETFL, flags) == 0) ? true : false;
#endif
}

static int socket_accept(int server_fd) {
  int cfd;
  struct sockaddr_in client;
  struct hostent *client_info;
#ifndef _WIN32
  socklen_t asize;
#else
  int asize;
#endif
  asize = sizeof(struct sockaddr_in);
  cfd = accept(server_fd, (struct sockaddr *)&client, &asize);
  if (cfd == -1) {
#ifdef _WIN32
    int e = WSAGetLastError();
    if (e == WSAEWOULDBLOCK)
      return 0;
    fprintf(stderr, "Accept error: %d.\n", e);
#else
    if (errno == EWOULDBLOCK)
      return 0;
    fprintf(stderr, "Accept error: %d.\n", errno);
#endif
    return -1;
  }
  client_info = gethostbyname((char *)inet_ntoa(client.sin_addr));
  printf("Accepted connection from: %s.\n", client_info->h_name);
  return cfd;
}

static bool socket_close(int fd) {
#ifdef _WIN32
  return (closesocket(fd) == 0) ? true : false;
#else
  return (close(fd) == 0) ? true : false;
#endif
}

static bool socket_cleanup() {
#ifdef _WIN32
  return (WSACleanup() == 0) ? true : false;
#else
  return true;
#endif
}

static int create_socket_server(int port) {
  int sfd, rc;
  struct sockaddr_in address;
#ifdef _WIN32
  if (!socket_init())
    return -1;
#endif
  sfd = socket(AF_INET, SOCK_STREAM, 0);
  if (sfd == -1) {
    fprintf(stderr, "Cannot create socket.\n");
    return -1;
  }
  memset(&address, 0, sizeof(struct sockaddr_in));
  address.sin_family = AF_INET;
  address.sin_port = htons((unsigned short)port);
  address.sin_addr.s_addr = INADDR_ANY;
  rc = bind(sfd, (struct sockaddr *)&address, sizeof(struct sockaddr));
  if (rc == -1) {
    fprintf(stderr, "Cannot bind port %d.\n", port);
    socket_close(sfd);
    return -1;
  }
  if (listen(sfd, 1) == -1) {
    fprintf(stderr, "Cannot listen for connections.\n");
    socket_close(sfd);
    return -1;
  }
  return sfd;
}

int main(int argc, char *argv[]) {
  int fd = 0;
  fd_set rfds;
  int port;
  if (argc > 1)
    sscanf(argv[1], "%d", &port);
  else
    port = 1000;  // default port on the e-puck2 robot
  int sfd = create_socket_server(port);
  socket_set_non_blocking(sfd);
  wb_robot_init();
  int time_step = wb_robot_get_basic_time_step();
  WbDeviceTag left_motor = wb_robot_get_device("left wheel motor");
  WbDeviceTag right_motor = wb_robot_get_device("right wheel motor");
  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);
  wb_motor_set_velocity(left_motor, 0.0);
  wb_motor_set_velocity(right_motor, 0.0);
  const int number_of_ir_sensors = 8;
  WbDeviceTag distance_sensor[number_of_ir_sensors];
  WbDeviceTag light_sensor[number_of_ir_sensors];
  for (int i = 0; i < number_of_ir_sensors; i++) {
    char device_name[8];
    sprintf(device_name, "ps%d", i); /* distance sensor */
    distance_sensor[i] = wb_robot_get_device(device_name);
    wb_distance_sensor_enable(distance_sensor[i], time_step);
    device_name[0] = 'l'; /* light sensor */
    light_sensor[i] = wb_robot_get_device(device_name);
    wb_light_sensor_enable(light_sensor[i], time_step);
  }
  WbDeviceTag speaker = wb_robot_get_device("speaker");
  play_melody_set_speaker(speaker);
  WbDeviceTag accelerometer = wb_robot_get_device("accelerometer");
  wb_accelerometer_enable(accelerometer, time_step);
  const int number_of_leds = 10;
  WbDeviceTag led[number_of_leds];
  for (int i = 0; i < number_of_leds; i++) {
    char device_name[8];
    sprintf(device_name, "led%d", i); /* LED */
    led[i] = wb_robot_get_device(device_name);
  }
  bool stream_image = false;
  bool stream_sensors = false;
  printf("Waiting for a connection on port %d...\n", port);
  unsigned char command_buffer[21];
  unsigned char sensors_buffer[105] = {0};
  sensors_buffer[0] = 0x02;
  unsigned char *image_buffer = (unsigned char *)malloc(38401);
  image_buffer[0] = 0x01;
  while (wb_robot_step(time_step) != -1) {
    play_melody_step(time_step);
    struct timeval tv = {0, 0};
    if (fd == 0) {
      fd = socket_accept(sfd);
      if (fd > 0)
        socket_set_non_blocking(fd);
      else if (fd < 0)
        break;
    }
    if (fd) {
      FD_ZERO(&rfds);
      FD_SET(fd, &rfds);
      int number = select(fd + 1, &rfds, NULL, NULL, &tv);
      if (number != 0) {
        int n = recv(fd, (char *)command_buffer, 21, 0);
        if (n < 0) {
#ifdef _WIN32
          int e = WSAGetLastError();
          if (e == WSAECONNABORTED)
            fprintf(stderr, "Connection aborted.\n");
          else if (e == WSAECONNRESET)
            fprintf(stderr, "Connection reset.\n");
          else
            fprintf(stderr, "Error reading from socket: %d.\n", e);
#else
          if (errno)
            fprintf(stderr, "Error reading from socket: %d.\n", errno);
#endif
          break;
        }
        if (n == 21 && command_buffer[0] == 0x80) {
          double left_speed = MOTOR_RATIO * (command_buffer[3] + ((char)command_buffer[4] << 8));
          double right_speed = MOTOR_RATIO * (command_buffer[5] + ((char)command_buffer[6] << 8));
          wb_motor_set_velocity(left_motor, left_speed);
          wb_motor_set_velocity(right_motor, right_speed);
          stream_image = ((command_buffer[1] & 1) == 1);
          stream_sensors = ((command_buffer[1] & 2) == 2);
          wb_led_set(led[0], (command_buffer[7] & 0x01) ? 1 : 0);
          wb_led_set(led[2], (command_buffer[7] & 0x02) ? 1 : 0);
          wb_led_set(led[4], (command_buffer[7] & 0x04) ? 1 : 0);
          wb_led_set(led[6], (command_buffer[7] & 0x08) ? 1 : 0);
          wb_led_set(led[8], (command_buffer[7] & 0x10) ? 1 : 0);
          wb_led_set(led[9], (command_buffer[7] & 0x20) ? 1 : 0);
          wb_led_set(led[1], ((int)command_buffer[8] << 16) + ((int)command_buffer[9] << 8) + command_buffer[10]);
          wb_led_set(led[3], ((int)command_buffer[11] << 16) + ((int)command_buffer[12] << 8) + command_buffer[13]);
          wb_led_set(led[5], ((int)command_buffer[14] << 16) + ((int)command_buffer[15] << 8) + command_buffer[16]);
          wb_led_set(led[7], ((int)command_buffer[17] << 16) + ((int)command_buffer[18] << 8) + command_buffer[19]);
          switch (command_buffer[20]) {
            case 0x01:
              play_melody_set_song(MARIO);
              break;
            case 0x02:
              play_melody_set_song(UNDERWORLD);
              break;
            case 0x04:
              play_melody_set_song(STARWARS);
              break;
            case 0x08:
              play_melody_stop();
              wb_speaker_play_sound(speaker, speaker, "sounds/4KHz.wav", 1.0, 1.0, 0, true);
              break;
            case 0x10:
              play_melody_stop();
              wb_speaker_play_sound(speaker, speaker, "sounds/10KHz.wav", 1.0, 1.0, 0, true);
              break;
            case 0x20:
              play_melody_stop();
              break;
          }
        } else if (n == 0) {
          wb_motor_set_velocity(left_motor, 0);
          wb_motor_set_velocity(right_motor, 0);
          printf("Connection closed, waiting for new connection on port %d...\n", port);
          socket_close(fd);
          fd = 0;
        } else {
          printf("Received %d bytes:\n", n);
          for (int i = 0; i < n; i++)
            printf("0x%02x ", (unsigned char)command_buffer[i]);
          printf("\n");
        }
      }
      if (stream_sensors) {
        memset(sensors_buffer + 1, 0, sizeof(sensors_buffer) - 1);
        for (int i = 0; i < 8; i++) {
          double value = wb_distance_sensor_get_value(distance_sensor[i]);
          if (value < 0)
            value = 0;
          unsigned short int v = value;
          sensors_buffer[i * 2 + 38] = *((const unsigned char *)&v);
          sensors_buffer[i * 2 + 39] = *((const unsigned char *)(&v) + 1);
          value = wb_light_sensor_get_value(light_sensor[i]);
          if (value < 0)
            value = 0;
          v = value;
          sensors_buffer[i * 2 + 54] = *((const unsigned char *)&v);
          sensors_buffer[i * 2 + 55] = *((const unsigned char *)(&v) + 1);
        }
        const double *values = wb_accelerometer_get_values(accelerometer);
        const double calibration_k[3] = {-9.81 / 760.0, 9.81 / 760.0, 9.81 / 760.0};
        short int acc[3];
        for (int i = 0; i < 3; i++) {
          acc[i] = values[i] / calibration_k[i];
          if (acc[i] > 1500)
            acc[i] = 1500;
          else if (acc[i] < -1500)
            acc[i] = -1500;
        }
        /* The accelerometer maths are taken from the e-puck2 wifi server source code in e_acc.c */
        for (int i = 0; i < 3 * sizeof(short int); i++)
          sensors_buffer[1 + i] = *((const unsigned char *)acc + i);
        float acceleration = sqrtf((float)(acc[0] * acc[0] + acc[1] * acc[1] + acc[2] * acc[2]));
        float inclination = 90.0 - atan2f((float)(acc[2]), sqrtf((float)(acc[0] * acc[0] + acc[1] * acc[1]))) * 180.0 / M_PI;
        float orientation;
        if (inclination < 5 || inclination > 160)
          orientation = 0;
        else
          orientation = atan2f((float)(acc[0]), (float)(acc[1])) * 180.0 / M_PI + 180.0;
        for (int i = 0; i < sizeof(float); i++)
          sensors_buffer[7 + i] = *((const unsigned char *)&acceleration + i);
        for (int i = 0; i < sizeof(float); i++)
          sensors_buffer[11 + i] = *((const unsigned char *)&orientation + i);
        for (int i = 0; i < sizeof(float); i++)
          sensors_buffer[15 + i] = *((const unsigned char *)&inclination + i);
        send(fd, (char *)sensors_buffer, sizeof(sensors_buffer), 0);
      }
      if (stream_image)
        send(fd, (char *)image_buffer, 38401, 0);
    }
    fflush(stdout);
  }
  free(image_buffer);
  socket_cleanup();
  wb_robot_cleanup();
  return 0;
}
