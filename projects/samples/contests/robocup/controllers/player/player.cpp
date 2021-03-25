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
// #define TURBOJPEG 1
// It turns out that the libjpeg interface to turbojpeg runs faster than the native turbojpeg interface
// Alternatives to be considered: NVIDIA CUDA nvJPEG Encoder, Intel IPP JPEG encoder
#ifdef TURBOJPEG
#include <turbojpeg.h>
#else
#include <jpeglib.h>
#endif
#include <assert.h>

#include <google/protobuf/text_format.h>

#include <webots/Accelerometer.hpp>
#include <webots/Camera.hpp>
#include <webots/Gyro.hpp>
#include <webots/Motor.hpp>
#include <webots/Node.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/Robot.hpp>
#include <webots/TouchSensor.hpp>
#include "messages.pb.h"

static int server_fd = -1;
static fd_set rfds;

static bool set_blocking(int fd, bool blocking) {
#ifdef _WIN32
  unsigned long mode = blocking ? 0 : 1;
  return (ioctlsocket(fd, FIONBIO, &mode) == 0) ? true : false;
#else
  int flags = fcntl(fd, F_GETFL, 0);
  if (flags == -1)
    return false;
  flags = blocking ? (flags & ~O_NONBLOCK) : (flags | O_NONBLOCK);
  return (fcntl(fd, F_SETFL, flags) == 0) ? true : false;
#endif
}

static int accept_client(int server_fd) {
  int cfd;
  struct sockaddr_in client;
  int size;
  size = sizeof(struct sockaddr_in);
  cfd = accept(server_fd, (struct sockaddr *)&client, &size);
  if (cfd != -1) {
    struct hostent *client_info = gethostbyname((char *)inet_ntoa(client.sin_addr));
    printf("Accepted connection from: %s \n", client_info->h_name);
  }
  return cfd;
}

static void close_socket(int fd) {
#ifdef _WIN32
  closesocket(fd);
#else
  close(fd);
#endif
}

static int create_socket_server(int port) {
  int rc;
  int server_fd;
  struct sockaddr_in address;

#ifdef _WIN32
  WSADATA info;
  rc = WSAStartup(MAKEWORD(2, 2), &info);  // Winsock 2.2
  if (rc != 0) {
    fprintf(stderr, "Cannot initialize Winsock\n");
    return -1;
  }
#endif

  server_fd = socket(AF_INET, SOCK_STREAM, 0);
  if (server_fd == -1) {
    fprintf(stderr, "Cannot create socket\n");
    return -1;
  }
  memset(&address, 0, sizeof(struct sockaddr_in));
  address.sin_family = AF_INET;
  address.sin_port = htons((unsigned short)port);
  address.sin_addr.s_addr = INADDR_ANY;
  rc = bind(server_fd, (struct sockaddr *)&address, sizeof(struct sockaddr));
  if (rc == -1) {
    fprintf(stderr, "Cannot bind port %d\n", port);
    close_socket(server_fd);
    return -1;
  }
  if (listen(server_fd, 1) == -1) {
    fprintf(stderr, "Cannot listen for connections\n");
    close_socket(server_fd);
    return -1;
  }
  return server_fd;
}

static void encode_jpeg(const unsigned char *image, int width, int height, int quality, unsigned long *size,
                        unsigned char **buffer) {
#ifdef TURBOJPEG
  tjhandle compressor = tjInitCompress();
  tjCompress2(compressor, image, width, 0, height, TJPF_RGB, buffer, size, TJSAMP_444, quality, TJFLAG_FASTDCT);
  tjDestroy(compressor);
#else
  struct jpeg_compress_struct cinfo;
  struct jpeg_error_mgr jerr;
  JSAMPROW row_pointer[1];
  cinfo.err = jpeg_std_error(&jerr);
  jpeg_create_compress(&cinfo);
  cinfo.image_width = width;
  cinfo.image_height = height;
  cinfo.input_components = 3;
  cinfo.in_color_space = JCS_RGB;
  jpeg_set_defaults(&cinfo);
  jpeg_set_quality(&cinfo, quality, TRUE);
  jpeg_mem_dest(&cinfo, buffer, size);
  jpeg_start_compress(&cinfo, TRUE);
  while (cinfo.next_scanline < cinfo.image_height) {
    row_pointer[0] = (unsigned char *)&image[cinfo.next_scanline * width * 3];
    jpeg_write_scanlines(&cinfo, row_pointer, 1);
  }
  jpeg_finish_compress(&cinfo);
  jpeg_destroy_compress(&cinfo);
#endif
}

static void free_jpeg(unsigned char *buffer) {
#ifdef TURBOJPEG
  tjFree(buffer);
#else
  free(buffer);
#endif
}

static void warn(SensorMeasurements &sensorMeasurements, std::string text) {
  Message *message = sensorMeasurements.add_message();
  message->set_message_type(Message::WARNING_MESSAGE);
  message->set_text(text);
}

int main(int argc, char *argv[]) {
  const std::string player_names[] = {
    "red player 1",  "red player 2",  "red player 3",  "red player 4",
    "blue player 1", "blue player 2", "blue player 3", "blue player 4",
  };
  const int ports[] = {10001, 10002, 10003, 10004, 10021, 100022, 100023, 100024};
  webots::Robot *robot = new webots::Robot();
  const double timeStep = robot->getBasicTimeStep();
  const size_t size = sizeof(player_names) / sizeof(player_names[0]);
  const std::string name = robot->getName();
  int client_fd = -1;
  int player_id = -1;
  int port = -1;
  for (unsigned int i = 0; i < size; i++)
    if (name.compare(player_names[i]) == 0) {
      player_id = i;
      port = ports[i];
      break;
    }
  std::cout << name << ": player ID = " << player_id << " running on port " << port << std::endl;
  server_fd = create_socket_server(port);
  set_blocking(server_fd, false);

  std::set<webots::Device *> sensors;

  webots::Camera *camera = robot->getCamera("Camera");
  camera->enable(timeStep);
  int width = camera->getWidth();
  int height = camera->getHeight();
  unsigned char *buffer = NULL;
  unsigned long bufferSize = 0;
  while (robot->step(timeStep) != -1) {
    const unsigned char *image = camera->getImage();
    encode_jpeg(image, width, height, 95, &bufferSize, &buffer);
    free_jpeg(buffer);
    buffer = NULL;
    if (client_fd == -1)
      client_fd = accept_client(server_fd);
    else {
      FD_ZERO(&rfds);
      FD_SET(client_fd, &rfds);
      struct timeval tv = {0, 0};
      int number = select(client_fd + 1, &rfds, NULL, NULL, &tv);
      if (number) {  // some data is available from the socket
        char data[256];
        int n = recv(client_fd, data, sizeof(int), 0);
        if (n <= 0) {
          printf("Closed connection\n");
          close_socket(client_fd);
          client_fd = -1;
        } else {
          assert(n == sizeof(int));
          int l = ntohl(*((int *)data));
          printf("packet size = %d %d\n", l, n);
          MotorPosition motorPosition;
          // motorPosition.ParseFromFileDescriptor(client_fd);
          int n = recv(client_fd, data, l, 0);
          assert(n == l);
          if (n <= 0) {
            printf("Broke connection\n");
            close_socket(client_fd);
            client_fd = -1;
          } else {
            data[n] = '\0';
            printf("Received %d bytes\n", n);
            ActuatorRequests actuatorRequests;
            actuatorRequests.ParseFromArray(data, n);
            SensorMeasurements sensorMeasurements;
            for (int i = 0; i < actuatorRequests.motor_position_size(); i++) {
              const MotorPosition motorPosition = actuatorRequests.motor_position(i);
              webots::Motor *motor = robot->getMotor(motorPosition.name());
              if (motor)
                motor->setPosition(motorPosition.position());
              else
                warn(sensorMeasurements, "Motor \"" + motorPosition.name() + "\" not found, position command ignored.");
            }
            for (int i = 0; i < actuatorRequests.motor_velocity_size(); i++) {
              const MotorVelocity motorVelocity = actuatorRequests.motor_velocity(i);
              webots::Motor *motor = robot->getMotor(motorVelocity.name());
              if (motor)
                motor->setVelocity(motorVelocity.velocity());
              else
                warn(sensorMeasurements, "Motor \"" + motorVelocity.name() + "\" not found, velocity command ignored.");
            }
            for (int i = 0; i < actuatorRequests.motor_force_size(); i++) {
              const MotorForce motorForce = actuatorRequests.motor_force(i);
              webots::Motor *motor = robot->getMotor(motorForce.name());
              if (motor)
                motor->setForce(motorForce.force());
              else
                warn(sensorMeasurements, "Motor \"" + motorForce.name() + "\" not found, force command ignored.");
            }
            for (int i = 0; i < actuatorRequests.motor_torque_size(); i++) {
              const MotorTorque motorTorque = actuatorRequests.motor_torque(i);
              webots::Motor *motor = robot->getMotor(motorTorque.name());
              if (motor)
                motor->setTorque(motorTorque.torque());
              else
                warn(sensorMeasurements, "Motor \"" + motorTorque.name() + "\" not found, torque command ignored.");
            }
            for (int i = 0; i < actuatorRequests.motor_pid_size(); i++) {
              const MotorPID motorPID = actuatorRequests.motor_pid(i);
              webots::Motor *motor = robot->getMotor(motorPID.name());
              if (motor)
                motor->setControlPID(motorPID.pid().x(), motorPID.pid().y(), motorPID.pid().z());
              else
                warn(sensorMeasurements, "Motor \"" + motorPID.name() + "\" not found, PID command ignored.");
            }
            for (int i = 0; i < actuatorRequests.sensor_time_step_size(); i++) {
              const SensorTimeStep sensorTimeStep = actuatorRequests.sensor_time_step(i);
              webots::Device *device = robot->getDevice(sensorTimeStep.name());
              if (device) {
                const int timeStep = sensorTimeStep.timestep();
                if (timeStep)
                  sensors.insert(device);
                else
                  sensors.erase(device);
                const int basicTimeStep = robot->getBasicTimeStep();
                if (timeStep != 0 && timeStep < basicTimeStep)
                  warn(sensorMeasurements, "Time step for \"" + sensorTimeStep.name() + "\" should be greater or equal to " +
                                             std::to_string(basicTimeStep) + ", ignoring " + std::to_string(timeStep) +
                                             " value.");
                else if (timeStep % basicTimeStep != 0)
                  warn(sensorMeasurements, "Time step for \"" + sensorTimeStep.name() + "\" should be a multiple of " +
                                             std::to_string(basicTimeStep) + ", ignoring " + std::to_string(timeStep) +
                                             " value.");
                else
                  switch (device->getNodeType()) {
                    case webots::Node::ACCELEROMETER: {
                      webots::Accelerometer *accelerometer = (webots::Accelerometer *)device;
                      accelerometer->enable(timeStep);
                      break;
                    }
                    case webots::Node::CAMERA: {
                      webots::Camera *camera = (webots::Camera *)device;
                      camera->enable(timeStep);
                      break;
                    }
                    case webots::Node::GYRO: {
                      webots::Gyro *gyro = (webots::Gyro *)device;
                      gyro->enable(timeStep);
                      break;
                    }
                    case webots::Node::POSITION_SENSOR: {
                      webots::PositionSensor *positionSensor = (webots::PositionSensor *)device;
                      positionSensor->enable(timeStep);
                      break;
                    }
                    case webots::Node::TOUCH_SENSOR: {
                      webots::TouchSensor *touchSensor = (webots::TouchSensor *)device;
                      touchSensor->enable(timeStep);
                      break;
                    }
                    default:
                      warn(sensorMeasurements,
                           "Device \"" + sensorTimeStep.name() + "\" is not supported, time step command, ignored.");
                  }
              } else
                warn(sensorMeasurements, "Device \"" + sensorTimeStep.name() + "\" not found, time step command, ignored.");
            }
            for (int i = 0; i < actuatorRequests.camera_quality_size(); i++) {
              const CameraQuality cameraQuality = actuatorRequests.camera_quality(i);
              webots::Camera *camera = robot->getCamera(cameraQuality.name());
              if (camera)
                warn(sensorMeasurements, "CameraQuality is not yet implemented, ignored.");
              else
                warn(sensorMeasurements, "Camera \"" + cameraQuality.name() + "\" not found, quality command ignored.");
            }
            for (int i = 0; i < actuatorRequests.camera_exposure_size(); i++) {
              const CameraExposure cameraExposure = actuatorRequests.camera_exposure(i);
              webots::Camera *camera = robot->getCamera(cameraExposure.name());
              if (camera)
                camera->setExposure(cameraExposure.exposure());
              else
                warn(sensorMeasurements, "Camera \"" + cameraExposure.name() + "\" not found, exposure command ignored.");
            }

            std::string printout;
            google::protobuf::TextFormat::PrintToString(actuatorRequests, &printout);
            std::cout << printout << std::endl;

            for (std::set<webots::Device *>::iterator it = sensors.begin(); it != sensors.end(); it++) {
              webots::Accelerometer *accelerometer = dynamic_cast<webots::Accelerometer *>(*it);
              if (accelerometer) {
                AccelerometerMeasurement *measurement = sensorMeasurements.add_accelerometer();
                measurement->set_name(accelerometer->getName());
                const double *values = accelerometer->getValues();
                Vector3 *vector3 = measurement->mutable_value();
                vector3->set_x(values[0]);
                vector3->set_y(values[1]);
                vector3->set_z(values[2]);
                continue;
              }
              webots::Camera *camera = dynamic_cast<webots::Camera *>(*it);
              if (camera) {
                CameraMeasurement *measurement = sensorMeasurements.add_camera();
                measurement->set_name(camera->getName());
                measurement->set_width(camera->getWidth());
                measurement->set_height(camera->getHeight());
                measurement->set_quality(-1);  // raw image (JPEG compression not yet supported)
                measurement->set_image((const char *)camera->getImage());
                continue;
              }
              webots::Gyro *gyro = dynamic_cast<webots::Gyro *>(*it);
              if (gyro) {
                GyroMeasurement *measurement = sensorMeasurements.add_gyro();
                measurement->set_name(gyro->getName());
                const double *values = gyro->getValues();
                Vector3 *vector3 = measurement->mutable_value();
                vector3->set_x(values[0]);
                vector3->set_y(values[1]);
                vector3->set_z(values[2]);
                continue;
              }
              webots::PositionSensor *position_sensor = dynamic_cast<webots::PositionSensor *>(*it);
              if (position_sensor) {
                PositionSensorMeasurement *measurement = sensorMeasurements.add_position_sensor();
                measurement->set_name(position_sensor->getName());
                measurement->set_value(position_sensor->getValue());
                continue;
              }
              webots::TouchSensor *touch_sensor = dynamic_cast<webots::TouchSensor *>(*it);
              if (touch_sensor) {
                webots::TouchSensor::Type type = touch_sensor->getType();
                switch (type) {
                  case webots::TouchSensor::BUMPER: {
                    BumperMeasurement *measurement = sensorMeasurements.add_bumper();
                    measurement->set_name(touch_sensor->getName());
                    measurement->set_value(touch_sensor->getValue() == 1.0);
                    continue;
                  }
                  case webots::TouchSensor::FORCE: {
                    ForceMeasurement *measurement = sensorMeasurements.add_force();
                    measurement->set_name(touch_sensor->getName());
                    measurement->set_value(touch_sensor->getValue());
                    continue;
                  }
                  case webots::TouchSensor::FORCE3D: {
                    Force3DMeasurement *measurement = sensorMeasurements.add_force3d();
                    measurement->set_name(touch_sensor->getName());
                    const double *values = touch_sensor->getValues();
                    Vector3 *vector3 = measurement->mutable_value();
                    vector3->set_x(values[0]);
                    vector3->set_y(values[1]);
                    vector3->set_z(values[2]);
                    continue;
                  }
                }
              }
            }
            const int size = sensorMeasurements.ByteSizeLong();
            char *output = (char *)malloc(sizeof(int) + size);
            int *output_size = (int *)output;
            *output_size = htonl(size);
            sensorMeasurements.SerializeToArray(&output[sizeof(int)], size);
            send(client_fd, output, sizeof(int) + size, 0);
            free(output);
          }
        }
      }
    }
  }
  delete robot;
  return 0;
}
