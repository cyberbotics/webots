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
#include <webots/Accelerometer.hpp>
#include <webots/Camera.hpp>
#include <webots/Gyro.hpp>
#include <webots/Motor.hpp>
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
  /* initialize the socket api */
  WSADATA info;

  rc = WSAStartup(MAKEWORD(2, 2), &info);  // Winsock 2.2
  if (rc != 0) {
    printf("cannot initialize Winsock\n");
    return -1;
  }
#endif
  /* create the socket */
  server_fd = socket(AF_INET, SOCK_STREAM, 0);
  if (server_fd == -1) {
    printf("cannot create socket\n");
    return -1;
  }

  /* fill in socket address */
  memset(&address, 0, sizeof(struct sockaddr_in));
  address.sin_family = AF_INET;
  address.sin_port = htons((unsigned short)port);
  address.sin_addr.s_addr = INADDR_ANY;

  /* bind to port */
  rc = bind(server_fd, (struct sockaddr *)&address, sizeof(struct sockaddr));
  if (rc == -1) {
    printf("cannot bind port %d\n", port);
    close_socket(server_fd);
    return -1;
  }

  /* listen for connections */
  if (listen(server_fd, 1) == -1) {
    printf("cannot listen for connections\n");
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
        int n = recv(client_fd, data, 256, 0);
        if (n < 0) {
          printf("Closed connection\n");
          close_socket(client_fd);
          client_fd = -1;
        } else {
          data[n] = '\0';
          printf("Received %d bytes: %s\n", n, data);
          send(client_fd, "OK\r\n", 3, 0);
        }
      }
    }
  }
  delete robot;
  return 0;
}
