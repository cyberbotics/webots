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
#include <jpeglib.h>
#include <webots/Accelerometer.hpp>
#include <webots/Camera.hpp>
#include <webots/Gyro.hpp>
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/Robot.hpp>
#include <webots/TouchSensor.hpp>
#include "messages.pb.h"

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

static void encode_jpeg(const unsigned char *image, int width, int height, int quality, unsigned long *size,
                        unsigned char **buffer) {
  struct jpeg_compress_struct cinfo;
  struct jpeg_error_mgr jerr;
  JSAMPROW row_pointer[1];
  cinfo.err = jpeg_std_error(&jerr);
  jpeg_create_compress(&cinfo);
  cinfo.image_width = width;
  cinfo.image_height = height;
  cinfo.input_components = 3;
  cinfo.in_color_space = JCS_GRAYSCALE;
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
  int player_id = -1;
  int port = -1;
  for (unsigned int i = 0; i < size; i++)
    if (name.compare(player_names[i]) == 0) {
      player_id = i;
      port = ports[i];
      break;
    }
  std::cout << name << ": player ID = " << player_id << " running on port " << port << std::endl;
  fd = create_socket_server(port);
  FD_ZERO(&rfds);
  FD_SET(fd, &rfds);

  webots::Camera *camera = robot->getCamera("Camera");
  camera->enable(timeStep);
  int width = camera->getWidth();
  int height = camera->getHeight();
  unsigned char *buffer = NULL;
  unsigned long bufferSize = 0;
  while (robot->step(timeStep) != -1) {
    const unsigned char *image = camera->getImage();
    encode_jpeg(image, width, height, 95, &bufferSize, &buffer);
  }
  delete robot;
  return 0;
}
