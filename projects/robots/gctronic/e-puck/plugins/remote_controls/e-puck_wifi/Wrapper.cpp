// Copyright 1996-2023 Cyberbotics Ltd.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     https://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "Wrapper.hpp"

#include "Camera.hpp"
#include "Communication.hpp"
#include "Device.hpp"
#include "DeviceManager.hpp"
#include "EPuckCommandPacket.hpp"
#include "Led.hpp"
#include "Motor.hpp"
#include "Sensor.hpp"
#include "SingleValueSensor.hpp"
#include "Time.hpp"
#include "TripleValuesSensor.hpp"

#include <webots/camera.h>
#include <webots/remote_control.h>

#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

// Uncomment these line to save camera images and/or log communication times
// #define SAVE_CAMERA_IMAGES
// #define LOG_COMMUNICATION_TIME

using namespace std;

static const int IMAGE_SIZE = 160 * 120 * 2;
static const int SENSOR_DATA_SIZE = 104;

Communication *Wrapper::cCommunication = NULL;
Time *Wrapper::cTime = NULL;
bool Wrapper::cSuccess = true;

void Wrapper::init() {
  DeviceManager::instance();
  cCommunication = new Communication;
}

void Wrapper::cleanup() {
  delete cCommunication;
  DeviceManager::cleanup();
}

bool Wrapper::start(const char *args) {
  if (!args)
    return false;
  cCommunication->initialize(std::string(args));
  cTime = new Time();
  cSuccess = cCommunication->isInitialized();
  return cSuccess;
}

void Wrapper::stop() {
  stopActuators();
  cCommunication->cleanup();
  if (cTime) {
    delete cTime;
    cTime = NULL;
  }
}

#ifdef LOG_COMMUNICATION_TIME
#define log(...)                           \
  {                                        \
    FILE *_logfd = fopen("log.txt", "a+"); \
    fprintf(_logfd, __VA_ARGS__);          \
    fflush(_logfd);                        \
    fclose(_logfd);                        \
  }
#else
#define log(...) \
  {}
#endif

#ifdef SAVE_CAMERA_IMAGES
static void rgb565_to_brg888(const unsigned char *rgb565, unsigned char *brg888, int width, int height) {
  int rgb565_index = 0;
  int brg888_index = 0;
  for (int j = 0; j < height; j++) {
    for (int i = 0; i < width; i++) {
      unsigned char red = rgb565[rgb565_index] & 0xf8;
      unsigned char green = (rgb565[rgb565_index++] << 5);
      green += (rgb565[rgb565_index] & 0xf8) >> 3;
      unsigned char blue = rgb565[rgb565_index++] << 3;
      brg888[brg888_index++] = blue;
      brg888[brg888_index++] = green;
      brg888[brg888_index++] = red;
    }
  }
  assert(rgb565_index == 160 * 120 * 2);
  assert(brg888_index == 160 * 120 * 3);
}

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
#endif

int Wrapper::robotStep(int step) {
#ifdef LOG_COMMUNICATION_TIME
  static clock_t startTime = 0, endTime = 0;
  int cpu_time_used;
#endif

  if (step == 0)
    return 0;
  // get simulation time at the beginning of this step
  int beginStepTime = cTime->elapsedTime();
  // apply to sensors
  DeviceManager::instance()->apply(beginStepTime);
  // setup the command packet
  EPuckCommandPacket commandPacket;
  int command = commandPacket.apply(beginStepTime);
#ifdef LOG_COMMUNICATION_TIME
  startTime = clock();
#endif
  cSuccess = cCommunication->send(commandPacket.data(), EPUCK_COMMAND_PACKET_SIZE);
#ifdef LOG_COMMUNICATION_TIME
  endTime = clock();
  cpu_time_used = (endTime - startTime) * 1000 / CLOCKS_PER_SEC;
  log("Took %d ms to send 0x%02x command (%d bytes) \n", cpu_time_used, command, EPUCK_COMMAND_PACKET_SIZE);
#endif
  if (!cSuccess) {
    log("Failed to send packet to the e-puck.\n");
    return 0;
  }
  unsigned char *image = static_cast<unsigned char *>(malloc(IMAGE_SIZE));
  unsigned char *sensor_data = static_cast<unsigned char *>(malloc(SENSOR_DATA_SIZE));
  int got_camera_image = 0;
  int got_sensor_data = 0;
  bool all_read = false;
  do {
    // log("waiting for header\n");
    unsigned char header = 0;
    if (command == 3 && got_camera_image && got_sensor_data)
      break;
    if (command == 2 && got_sensor_data)
      break;
    if (command == 1 && got_camera_image)
      break;
    if (command == 0 && header == 3)
      break;

#ifdef LOG_COMMUNICATION_TIME
    startTime = clock();
#endif
    int n = cCommunication->receive(reinterpret_cast<char *>(&header), 1, true);
#ifdef LOG_COMMUNICATION_TIME
    endTime = clock();
    cpu_time_used = (endTime - startTime) * 1000 / CLOCKS_PER_SEC;
    log("Took %d ms to receive 0x%02x header (%d byte)\n", cpu_time_used, header, n);
#endif
    if (n == 0)
      all_read = true;
    if (command == 3 && got_camera_image > 0 && got_sensor_data > 0 && all_read)
      break;
    if (command == 2 && got_sensor_data > 0 && all_read)
      break;
    if (command == 1 && got_camera_image > 0 && all_read)
      break;
    if (command == 0 && header == 3 && all_read)
      break;
    if (n == -1) {
      log("failed to received header\n");
      exit(1);
    }
    if (n > 0) {
      // log("received %d header\n", header);
      if (header == 0x01) {  // camera image
#ifdef LOG_COMMUNICATION_TIME
        startTime = clock();
#endif
        n = cCommunication->receive(reinterpret_cast<char *>(image), IMAGE_SIZE, true);
        if (n == -1) {
          log("Failed to receive camera image.\n");
          exit(1);
        }
#ifdef LOG_COMMUNICATION_TIME
        endTime = clock();
        cpu_time_used = (endTime - startTime) * 1000 / CLOCKS_PER_SEC;
        log("Took %d ms to receive image (%d bytes)\n", cpu_time_used, n);
#endif
#ifdef SAVE_CAMERA_IMAGES
        // save the image as BMP file to debug
        static int image_counter = 0;
        char filename[32];
        sprintf(filename, "image%03d.png", image_counter);
        unsigned char *rgb888 = static_cast<unsigned char *>(malloc(160 * 120 * 3));
        rgb565_to_brg888(image, rgb888, 160, 120);
        save_bmp_image(filename, rgb888, 160, 120);
        free(rgb888);
        image_counter++;
#endif
        got_camera_image++;
      } else if (header == 0x02) {  // sensor data
#ifdef LOG_COMMUNICATION_TIME
        startTime = clock();
#endif
        n = cCommunication->receive(reinterpret_cast<char *>(sensor_data), SENSOR_DATA_SIZE, true);
        if (n == -1) {
          log("Failed to receive sensor data,\n");
          exit(1);
        }
#ifdef LOG_COMMUNICATION_TIME
        endTime = clock();
        cpu_time_used = (endTime - startTime) * 1000 / CLOCKS_PER_SEC;
        log("Took %d ms to receive sensor data (%d bytes)\n", cpu_time_used, n);
#endif
        got_sensor_data++;
      } else if (header == 0x03) {
        log("Got empty packet\n");
        if (command == 0)
          break;
      } else {
        log("Unknown header: 0x%02x\n", header);
        exit(1);
      }
    }
  } while (true);
  Camera *camera = DeviceManager::instance()->camera();
  if (camera->isEnabled()) {
    unsigned char *bgraImage = static_cast<unsigned char *>(malloc(160 * 120 * 4));
    if (camera->rawToBgraImage(bgraImage, static_cast<const unsigned char *>(image))) {
      wbr_camera_set_image(camera->tag(), static_cast<const unsigned char *>(bgraImage));
      // log("Got camera image (size = %d)\n", IMAGE_SIZE);
    } else
      log("Cannot rawToBgraImage\n");
    free(bgraImage);
  }
  TripleValuesSensor *accelerometer = DeviceManager::instance()->accelerometer();
  if (accelerometer->isEnabled()) {
    const double calibration_k[3] = {-9.81 / 800.0, 9.81 / 800.0, 9.81 / 800.0};
    const double calibration_offset = -2000.0;
    double values[3];
    for (int i = 0; i < 3; i++) {
      short int acc = sensor_data[2 * i] + 256 * sensor_data[2 * i + 1];
      values[i] = calibration_k[i] * (acc + calibration_offset);
    }
    wbr_accelerometer_set_values(accelerometer->tag(), values);
  }
  for (int i = 0; i < 3; i++) {
    SingleValueSensor *gs = DeviceManager::instance()->groundSensor(i);
    if (gs && gs->isSensorRequested()) {
      const double value = sensor_data[90 + 2 * i] + 256 * sensor_data[91 + 2 * i];
      wbr_distance_sensor_set_value(gs->tag(), value);
      gs->resetSensorRequested();
      gs->setLastRefreshTime(beginStepTime);
    }
  }
  for (int i = 0; i < 8; i++) {
    SingleValueSensor *ds = DeviceManager::instance()->distanceSensor(i);
    if (ds && ds->isSensorRequested()) {
      const double value = sensor_data[37 + 2 * i] + 256 * sensor_data[38 + 2 * i];
      wbr_distance_sensor_set_value(ds->tag(), value);
      ds->resetSensorRequested();
      ds->setLastRefreshTime(beginStepTime);
    }
  }
  SingleValueSensor *tof = DeviceManager::instance()->tofSensor();
  if (tof && tof->isSensorRequested()) {
    const double value = sensor_data[69] + 256 * sensor_data[70];
    wbr_distance_sensor_set_value(tof->tag(), value);
    tof->resetSensorRequested();
    tof->setLastRefreshTime(beginStepTime);
  }
  for (int i = 0; i < 8; i++) {
    SingleValueSensor *ls = DeviceManager::instance()->lightSensor(i);
    if (ls && ls->isSensorRequested()) {
      const double value = sensor_data[53 + 2 * i] + 256 * sensor_data[54 + 2 * i];
      wbr_light_sensor_set_value(ls->tag(), value);
      ls->resetSensorRequested();
      ls->setLastRefreshTime(beginStepTime);
    }
  }
  for (int i = 0; i < 2; i++) {
    SingleValueSensor *ps = DeviceManager::instance()->positionSensor(i);
    if (ps && ps->isSensorRequested()) {
      // 159.23 = encoder_resolution / ( 2 * pi)
      double value = (sensor_data[79 + 2 * i] + 256 * sensor_data[80 + 2 * i]) / 159.23;
      wbr_position_sensor_set_value(ps->tag(), value);
      ps->resetSensorRequested();
      ps->setLastRefreshTime(beginStepTime);
    }
  }
  free(image);
  free(sensor_data);
  // get simulation time at the end of this step
  int endStepTime = cTime->elapsedTime();

  // according to the step duration, either wait
  // or returns the delay
  int deltaStepTime = endStepTime - beginStepTime;
  int dt = step - deltaStepTime - 10;  // these 10 milliseconds were determinted empirically to get closer to real time
  log("Waiting %d ms to be in sync with the %d ms time step...\n", (dt > 0) ? dt : 0, step);
  if (dt > 0) {  // the packet is sent at time
    Time::wait(dt);
    return 0;
  } else  // the delay asked is not fulfilled
    return deltaStepTime - step;
}

void Wrapper::stopActuators() {
  unsigned char command[21] = {0x80, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0x20};
  do {
    unsigned char header;
    cCommunication->send(reinterpret_cast<const char *>(command), sizeof(command));
    cCommunication->receive(reinterpret_cast<char *>(&header), 1, true);
    if (header == 0x01) {
      char *image = static_cast<char *>(malloc(IMAGE_SIZE));
      cCommunication->receive(image, IMAGE_SIZE, true);
      free(image);
    } else if (header == 0x02) {
      char *sensor_data = static_cast<char *>(malloc(SENSOR_DATA_SIZE));
      cCommunication->receive(sensor_data, SENSOR_DATA_SIZE, true);
      free(sensor_data);
    } else if (header == 0x03)
      break;
    else
      log("Wrong header\n");
  } while (true);
}

void Wrapper::setSamplingPeriod(WbDeviceTag tag, int samplingPeriod) {
  Device *device = DeviceManager::instance()->findDeviceFromTag(tag);
  Sensor *sensor = dynamic_cast<Sensor *>(device);
  if (sensor) {
    sensor->setLastRefreshTime(0);
    sensor->setSamplingPeriod(samplingPeriod);
  } else
    log("Wrapper::setSamplingPeriod: unknown device.\n");
}

void Wrapper::motorSetVelocity(WbDeviceTag tag, double velocity) {
  Device *device = DeviceManager::instance()->findDeviceFromTag(tag);
  Motor *motor = dynamic_cast<Motor *>(device);
  if (motor)
    motor->setVelocity(velocity);
}

void Wrapper::ledSet(WbDeviceTag tag, int state) {
  Device *device = DeviceManager::instance()->findDeviceFromTag(tag);
  Led *led = dynamic_cast<Led *>(device);
  if (led)
    led->setState(state);
}

void *Wrapper::callCustomFunction(void *args) {
  return NULL;
}
