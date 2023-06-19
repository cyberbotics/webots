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
 * Description:  A simple program implementing a TCP/IP relay controller for
 *               interfacing Webots with any development environment able to
 *               use TCP/IP, including MathLab, Lisp, Java, C, C++, etc.
 * Author:       Darren Smith
 */

/* The protocole used in this example is taken from the Khepera serial
 * communication protocole. Hence, if you already have developed an
 * application which uses this protocole (and send the data over the serial
 * port), you will just need to redirect the data to the TCP/IP socket of
 * this controller to make it work with Webots.
 *
 * Currently supported Khepera protocole commands include:
 *
 * B: read software version
 * D: set speed
 * G: set position counter
 * H: read position
 * L: change LED state
 * N: read proximity sensors
 * O: read ambient light sensors
 *
 * A sample client program, written in C is included in this directory.
 * See client.c for the source code
 * compile it with gcc client.c -o client
 *
 * Everything relies on standard POSIX TCP/IP sockets.
 */

#include <math.h>
#include <stdio.h>
#include <string.h>
#include <sys/types.h>
#include <time.h>
#include <webots/distance_sensor.h>
#include <webots/led.h>
#include <webots/light_sensor.h>
#include <webots/motor.h>
#include <webots/position_sensor.h>
#include <webots/robot.h>
#ifdef _WIN32
#include <winsock.h>
#else
#include <arpa/inet.h>  /* definition of inet_ntoa */
#include <netdb.h>      /* definition of gethostbyname */
#include <netinet/in.h> /* definition of struct sockaddr_in */
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h> /* definition of close */
#endif

#define SOCKET_PORT 10020
#define NB_IR_SENSOR 8
#define TIMESTEP 250

static WbDeviceTag distance[NB_IR_SENSOR];
static WbDeviceTag ambient[NB_IR_SENSOR];
static WbDeviceTag led[2];
static WbDeviceTag left_motor, right_motor, left_position_sensor, right_position_sensor;
static int fd;
static fd_set rfds;

static int accept_client(int server_fd) {
  int cfd;
  struct sockaddr_in client;
#ifndef _WIN32
  socklen_t asize;
#else
  int asize;
#endif
  struct hostent *client_info;

  asize = sizeof(struct sockaddr_in);

  cfd = accept(server_fd, (struct sockaddr *)&client, &asize);
  if (cfd == -1) {
    printf("cannot accept client\n");
    return -1;
  }
  client_info = gethostbyname((char *)inet_ntoa(client.sin_addr));
  printf("Accepted connection from: %s \n", client_info->h_name);

  return cfd;
}

static int create_socket_server(int port) {
  int sfd, rc;
  struct sockaddr_in address;

#ifdef _WIN32
  /* initialize the socket api */
  WSADATA info;

  rc = WSAStartup(MAKEWORD(1, 1), &info); /* Winsock 1.1 */
  if (rc != 0) {
    printf("cannot initialize Winsock\n");
    return -1;
  }
#endif
  /* create the socket */
  sfd = socket(AF_INET, SOCK_STREAM, 0);
  if (sfd == -1) {
    printf("cannot create socket\n");
    return -1;
  }

  /* fill in socket address */
  memset(&address, 0, sizeof(struct sockaddr_in));
  address.sin_family = AF_INET;
  address.sin_port = htons((unsigned short)port);
  address.sin_addr.s_addr = INADDR_ANY;

  /* bind to port */
  rc = bind(sfd, (struct sockaddr *)&address, sizeof(struct sockaddr));
  if (rc == -1) {
    printf("cannot bind port %d\n", port);
#ifdef _WIN32
    closesocket(sfd);
#else
    close(sfd);
#endif
    return -1;
  }

  /* listen for connections */
  if (listen(sfd, 1) == -1) {
    printf("cannot listen for connections\n");
#ifdef _WIN32
    closesocket(sfd);
#else
    close(sfd);
#endif
    return -1;
  }
  printf("Waiting for a connection on port %d...\n", port);

  return accept_client(sfd);
}

static void initialize() {
  int i;
  char text[32];

  for (i = 0; i < NB_IR_SENSOR; i++) {
    sprintf(text, "ds%d", i);
    distance[i] = wb_robot_get_device(text);
    wb_distance_sensor_enable(distance[i], TIMESTEP);
  }

  for (i = 0; i < NB_IR_SENSOR; i++) {
    sprintf(text, "ls%d", i);
    ambient[i] = wb_robot_get_device(text);
    wb_light_sensor_enable(ambient[i], TIMESTEP);
  }

  led[0] = wb_robot_get_device("led0");
  led[1] = wb_robot_get_device("led1");

  left_motor = wb_robot_get_device("left wheel motor");
  right_motor = wb_robot_get_device("right wheel motor");
  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);
  wb_motor_set_velocity(left_motor, 0.0);
  wb_motor_set_velocity(right_motor, 0.0);

  left_position_sensor = wb_robot_get_device("left wheel sensor");
  right_position_sensor = wb_robot_get_device("right wheel sensor");
  wb_position_sensor_enable(left_position_sensor, TIMESTEP);
  wb_position_sensor_enable(right_position_sensor, TIMESTEP);

  printf("Khepera robot has been initialized by Webots\n");
  fd = create_socket_server(SOCKET_PORT);
  FD_ZERO(&rfds);
  FD_SET(fd, &rfds);
}

static void run() {
  int n;
  int ret;
  char buffer[256];
  int left_speed, right_speed;
  short led_number, led_action;
  static short led_value[2] = {0, 0}; /* initially off */
  struct timeval tv = {0, 0};
  int number;
  int leftWheelEncoding;
  int rightWheelEncoding;
  int left_encoder_offset = 0;
  int right_encoder_offset = 0;

  /* Set up the parameters used for the select statement */

  FD_ZERO(&rfds);
  FD_SET(fd, &rfds);

  /*
   * Watch TCPIP file descriptor to see when it has input.
   * No wait - polling as fast as possible
   */
  number = select(fd + 1, &rfds, NULL, NULL, &tv);

  /* If there is no data at the socket, then redo loop */
  if (number == 0)
    return;

  /* ...otherwise, there is data to read, so read & process. */
  n = recv(fd, buffer, 256, 0);
  if (n < 0) {
    printf("error reading from socket\n");
    return;
  }
  buffer[n] = '\0';
  printf("Received %d bytes: %s\n", n, buffer);

  if (buffer[0] == 'D') { /* set the speed of the motors */
    sscanf(buffer, "D,%d,%d", &left_speed, &right_speed);
    wb_motor_set_velocity(left_motor, left_speed);
    wb_motor_set_velocity(right_motor, right_speed);
    send(fd, "d\r\n", 3, 0);

  } else if (buffer[0] == 'L') {
    sscanf(buffer, "L,%hd,%hd", &led_number, &led_action);
    if (led_number >= 0 && led_number <= 1 && led_action >= 0 && led_action <= 2) {
      if (led_action == 2) { /* change status */
        if (led_value[led_number] == 1)
          led_action = 0; /* switch off */
        else
          led_action = 1; /* switch on */
      }
      led_value[led_number] = led_action;
      printf("set led %d to %d\n", led_number, led_action);
      wb_led_set(led[led_number], (unsigned char)led_action);
    }
    send(fd, "l\r\n", 3, 0);

  } else if (buffer[0] == 'G') { /* set the position counter */
    int left, right;
    sscanf(buffer, "G,%d,%d", &left, &right);
    send(fd, "g\r\n", 3, 0);

  } else if (buffer[0] == 'B') { /* return a pretend version string */
    sprintf(buffer, "b,0,0\r\n");
    send(fd, buffer, strlen(buffer), 0);

  } else if (buffer[0] == 'N') { /* read distance sensor values */
    sprintf(buffer, "n,%d,%d,%d,%d,%d,%d,%d,%d\r\n", (int)wb_distance_sensor_get_value(distance[0]),
            (int)wb_distance_sensor_get_value(distance[1]), (int)wb_distance_sensor_get_value(distance[2]),
            (int)wb_distance_sensor_get_value(distance[3]), (int)wb_distance_sensor_get_value(distance[4]),
            (int)wb_distance_sensor_get_value(distance[5]), (int)wb_distance_sensor_get_value(distance[6]),
            (int)wb_distance_sensor_get_value(distance[7]));
    send(fd, buffer, strlen(buffer), 0);

  } else if (buffer[0] == 'H') {
    /* return the position counters of the pair of wheels */
    leftWheelEncoding = 50.0 * (wb_position_sensor_get_value(left_position_sensor) / M_PI) - left_encoder_offset;
    rightWheelEncoding = 50.0 * (wb_position_sensor_get_value(right_position_sensor) / M_PI) - right_encoder_offset;

    sprintf(buffer, "h,%d,%d\r\n", leftWheelEncoding, rightWheelEncoding);
    send(fd, buffer, strlen(buffer), 0);

  } else if (buffer[0] == 'O') { /* read the ambient light sensor */
    sprintf(buffer, "o,%d,%d,%d,%d,%d,%d,%d,%d\r\n", (int)wb_light_sensor_get_value(ambient[0]),
            (int)wb_light_sensor_get_value(ambient[1]), (int)wb_light_sensor_get_value(ambient[2]),
            (int)wb_light_sensor_get_value(ambient[3]), (int)wb_light_sensor_get_value(ambient[4]),
            (int)wb_light_sensor_get_value(ambient[5]), (int)wb_light_sensor_get_value(ambient[6]),
            (int)wb_light_sensor_get_value(ambient[7]));
    send(fd, buffer, strlen(buffer), 0);

  } else if (strncmp(buffer, "exit", 4) == 0) {
    printf("connection closed\n");
#ifdef _WIN32
    closesocket(fd);
    ret = WSACleanup();
#else
    ret = close(fd);
#endif
    if (ret != 0) {
      printf("Cannot close socket\n");
    }
    fd = 0;
  } else {
    send(fd, "\n", 1, 0);
  }
}

int main() {
  wb_robot_init();

  initialize();

  while (1) {
    wb_robot_step(TIMESTEP);
    run();
  }

  wb_robot_cleanup();

  return 0;
}
