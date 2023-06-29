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

#include <assert.h>
#include <stdio.h>
#include <winsock.h>

// e-puck wifi remote control test
// compile on Windows with: gcc -Wall test.c -o test -lws2_32
// mkdir images (to receive the BMP images)
// usage: test.exe <IP> where <IP> is the IP address of the e-puck robot
// author: Olivier.Michel@cyberbotics.com
// date: 2018.09.10

static int fd = 0;
static const int SOCKET_PORT = 1000;

static void save_bmp_image(const char *filename, const unsigned char *image, int width, int height) {
  int filesize = 54 + 3 * width * height;
  unsigned char bmpfileheader[14] = {'B', 'M', 0, 0, 0, 0, 0, 0, 0, 0, 54, 0, 0, 0};
  unsigned char bmpinfoheader[40] = {40, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 24, 0};
  unsigned char bmppad[3] = {0, 0, 0};
  bmpfileheader[2] = (unsigned char)(filesize);
  bmpfileheader[3] = (unsigned char)(filesize >> 8);
  bmpfileheader[4] = (unsigned char)(filesize >> 16);
  bmpfileheader[5] = (unsigned char)(filesize >> 24);
  bmpinfoheader[4] = (unsigned char)(width);
  bmpinfoheader[5] = (unsigned char)(width >> 8);
  bmpinfoheader[6] = (unsigned char)(width >> 16);
  bmpinfoheader[7] = (unsigned char)(width >> 24);
  bmpinfoheader[8] = (unsigned char)(height);
  bmpinfoheader[9] = (unsigned char)(height >> 8);
  bmpinfoheader[10] = (unsigned char)(height >> 16);
  bmpinfoheader[11] = (unsigned char)(height >> 24);
  FILE *f = fopen(filename, "wb");
  fwrite(bmpfileheader, 1, 14, f);
  fwrite(bmpinfoheader, 1, 40, f);
  for (int i = 0; i < height; i++) {
    fwrite(image + (width * (height - i - 1) * 3), 3, width, f);
    fwrite(bmppad, 1, (4 - (width * 3) % 4) % 4, f);
  }
  fclose(f);
}

static void rgb565_to_brg888(const unsigned char *rgb565, unsigned char *brg888, int width, int height) {
  int counter = 0;
  for (int j = 0; j < height; j++) {
    for (int i = 0; i < width; i++) {
      int index = 3 * (i + j * width);
      unsigned char red = rgb565[counter] & 0xf8;
      unsigned char green = (rgb565[counter++] << 5);
      green += (rgb565[counter] & 0xf8) >> 3;
      unsigned char blue = rgb565[counter++] << 3;
      brg888[index] = blue;
      brg888[index + 1] = green;
      brg888[index + 2] = red;
    }
  }
}

static void cleanup() {
  closesocket(fd);
}

int main(int argc, const char *argv[]) {
  struct sockaddr_in address;
  struct hostent *server;
  int rc;
  const char *ip = argv[1];

  // initialize the socket api
  WSADATA info;
  rc = WSAStartup(MAKEWORD(1, 1), &info);  // Winsock 1.1
  if (rc != 0) {
    fprintf(stderr, "Cannot initialize Winsock\n");
    exit(1);
  }
  fd = socket(AF_INET, SOCK_STREAM, 0);
  if (fd == -1) {
    fprintf(stderr, "Cannot create socket\n");
    exit(1);
  }
  memset(&address, 0, sizeof(struct sockaddr_in));
  address.sin_family = AF_INET;
  address.sin_port = htons(SOCKET_PORT);
  server = gethostbyname(ip);
  if (server)
    memcpy((char *)&address.sin_addr.s_addr, (char *)server->h_addr, server->h_length);
  else {
    fprintf(stderr, "Cannot resolve server name: %s\n", ip);
    cleanup();
    exit(1);
  }
  rc = connect(fd, (struct sockaddr *)&address, sizeof(struct sockaddr));
  if (rc == -1) {
    fprintf(stderr, "Cannot connect to the server\n");
    cleanup();
    exit(1);
  }
  unsigned char command[21];
  int i = 0;
  command[i++] = 0x80;
  command[i++] = 3;  // sensor & image
  command[i++] = 0;
  command[i++] = 100;   // left motor LSB
  command[i++] = 0;     // left motor MSB
  command[i++] = 100;   // right motor LSB
  command[i++] = 0;     // right motor MSB
  command[i++] = 0xf1;  // lEDs
  command[i++] = 0;     // LED2 red
  command[i++] = 0;     // LED2 green
  command[i++] = 0;     // LED2 blue
  command[i++] = 0;     // LED4 red
  command[i++] = 0;     // LED4 green
  command[i++] = 0;     // LED4 blue
  command[i++] = 0;     // LED6 red
  command[i++] = 0;     // LED6 green
  command[i++] = 0;     // LED6 blue
  command[i++] = 0;     // LED8 red
  command[i++] = 0;     // LED8 green
  command[i++] = 0;     // LED8 blue
  command[i++] = 0;     // speaker
  // cppcheck-suppress knownArgument
  // cppcheck-suppress constArgument
  assert(i == sizeof(command));
  for (int j = 0; j < 3000; j++) {
    fflush(stdout);
    if (j == 50) {  // stop motors
      printf("=> stop motors, camera and sensors ---------\n");
      command[1] = 0;
      command[3] = 0;
      command[5] = 0;
      command[7] = 0;
    } else if (j == 100) {
      printf("=> enable camera ---------------------------\n");
      command[1] = 1;
    } else if (j == 150) {
      printf("=> enable sensor ---------------------------\n");
      command[1] = 2;
    } else if (j == 200) {
      printf("=> enable camera and sensor ----------------\n");
      command[1] = 3;
    }

    const int n = send(fd, (char *)command, sizeof(command), 0);
    // Sleep(100);
    assert(n == sizeof(command));
    unsigned char header;
    unsigned char headers = 0;
    unsigned char sensor[104];
    const int width = 160;
    const int height = 120;
    unsigned char rgb565[width * height * 2];
    unsigned char brg888[width * height * 3];
    do {
      int r = recv(fd, (char *)&header, 1, 0);
      if (header != 0x03)
        headers |= header;
      if (r != 1) {
        fprintf(stderr, "Failed to receive header: %d\n", r);
        exit(1);
      }
      r = 0;
      switch (header) {
        case 0x01:
          // printf("received camera\n");
          do
            r += recv(fd, (char *)&rgb565[r], sizeof(rgb565) - r, 0);
          while (r < sizeof(rgb565));
          if (r != sizeof(rgb565)) {
            fprintf(stderr, "Cannot read camera image: %d != %d\n", r, (int)sizeof(rgb565));
            exit(1);
          }
          // save camera image
          rgb565_to_brg888(rgb565, brg888, width, height);
          static int image_counter = 0;
          char filename[32];
          sprintf(filename, "images/image%03d.bmp", image_counter++);
          save_bmp_image(filename, brg888, width, height);
          break;
        case 0x02:
          // printf("received sensor\n");
          do
            r += recv(fd, (char *)&sensor[r], sizeof(sensor) - r, 0);
          while (r < sizeof(sensor));
          if (r != sizeof(sensor)) {
            fprintf(stderr, "Cannot read sensor: %d != %d\n", r, (int)sizeof(sensor));
            exit(1);
          }
          /*
          printf("Acc: %d %d %d\n", *(short int *)(&sensor[0]), *(short int *)(&sensor[2]), *(short int *)(&sensor[4]));
          printf("Acceleration: %g\n", *(float *)(&sensor[6]));
          printf("Orientation: %g\n", *(float *)(&sensor[10]));
          printf("Inclination: %g\n", *(float *)(&sensor[14]));
          printf("Gyro: %d %d %d\n", *(short int *)&sensor[18], *(short int *)(&sensor[20]), *(short int *)(&sensor[22]));
          printf("Magnetometer: %g %g %g\n", *(float *)(&sensor[24]), *(float *)(&sensor[28]), *(float *)(&sensor[32]));
          printf("Temp: %d\n", (int)sensor[36]);
          printf("Proximity: %d %d %d %d %d %d %d %d\n", *(short int *)(&sensor[37]), *(short int *)(&sensor[39]),
                 *(short int *)(&sensor[41]), *(short int *)(&sensor[43]), *(short int *)(&sensor[45]),
                 *(short int *)(&sensor[47]), *(short int *)(&sensor[49]), *(short int *)(&sensor[51]));
          printf("Ambient: %d %d %d %d %d %d %d %d\n", *(short int *)(&sensor[53]), *(short int *)(&sensor[55]),
                 *(short int *)(&sensor[57]), *(short int *)(&sensor[59]), *(short int *)(&sensor[61]),
                 *(short int *)(&sensor[63]), *(short int *)(&sensor[65]), *(short int *)(&sensor[67]));
          printf("Distance: %d\n", *(short int *)(&sensor[69]));
          printf("Microphones: %d %d %d %d\n", *(short int *)(&sensor[71]), *(short int *)(&sensor[73]),
                 *(short int *)(&sensor[75]), *(short int *)(&sensor[77]));
          printf("Motors: %d %d\n", *(short int *)(&sensor[79]), *(short int *)(&sensor[81]));
          printf("Battery: %d\n", *(short int *)(&sensor[83]));
          printf("uSD: %d - TV toggle: %d - TV addr: %d - TV data: %d\n", sensor[85], sensor[86], sensor[87], sensor[88]);
          printf("Selector: %d\n", sensor[89]);
          printf("Ground proximity: %d %d %d\n", *(short int *)(&sensor[90]), *(short int *)(&sensor[92]),
                 *(short int *)(&sensor[94]));
          printf("Ground ambient: %d %d %d\n", *(short int *)(&sensor[96]), *(short int *)(&sensor[98]),
                 *(short int *)(&sensor[100]));
          printf("Button state: %d %d\n", sensor[102], sensor[103]);
          */
          break;
        case 0x03:
          break;
        default:
          break;
      }
      if (command[1] == 0 && header == 0x03)
        break;
      if (command[1] == 1 && headers == 0x01)
        break;
      if (command[1] == 2 && headers == 0x02)
        break;
      if (command[1] == 3 && headers == 0x03)
        break;
      if (headers == 0x03)
        headers = 0x00;
    } while (1);
    if (header == 0x03)
      printf("%04d) Received empty\n", j);
    else if (headers == 0x03)
      printf("%04d) Received camera and sensor\n", j);
    else if (headers == 0x01)
      printf("%04d) Received camera\n", j);
    else if (headers == 0x02)
      printf("%04d) Received sensor\n", j);
    else
      printf("%04d) Received unknown: %d\n", j, header);
  }
  printf("quitting\n");
  cleanup();
  return 0;
}
