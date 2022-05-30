/*
 * Copyright 1996-2022 Cyberbotics Ltd.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <stdlib.h>  // atoi
#ifdef _WIN32
#include <windows.h>
#endif
#include <stdio.h>
#include <string.h>  // strlen, memcpy
#include <unistd.h>
#include <webots/camera.h>
#include <webots/types.h>
#include "abstract_camera.h"
#include "device_private.h"
#include "g_pipe.h"
#include "messages.h"
#include "robot_private.h"
#include "scheduler.h"
#include "tcp_client.h"
#ifdef _WIN32
#include <wininet.h>
#else  // __APPLE__ || __linux__
#include <arpa/inet.h>
#include <netdb.h>
#include <unistd.h>
#ifndef __APPLE__
extern int gethostname(char *n, size_t l);  // fixes problem in unistd.h on Linux
#endif  // __APPLE__
#endif  // _WIN32

#define SCHEDULER_URL_OK 0
#define SCHEDULER_URL_SYNTAX_ERROR 1
#define SCHEDULER_DATA_CHUNK 4096

unsigned int scheduler_data_size = 0;
unsigned int scheduler_actual_step = 0;
char *scheduler_data = NULL;
char *scheduler_meta = NULL;
GPipe *scheduler_pipe = NULL;
TcpClient *scheduler_client = NULL;

int scheduler_init_remote(const char *host, int port, const char *robot_name) {
  scheduler_client = tcp_client_new(host, port);
  if (scheduler_client == NULL)
    return false;

  int length = robot_name ? strlen(robot_name) + 20 : 4;
  char *init_msg = malloc(length);
  memcpy(init_msg, "CTR", 3);
  if (robot_name) {  // send robot name
    memcpy(init_msg + 3, "\nRobot-Name: ", 13);
    memcpy(init_msg + 16, &robot_name[1], strlen(robot_name));
  }
  tcp_client_send(scheduler_client, init_msg, strlen(init_msg));
  free(init_msg);

  char *ack_msg = malloc(10);
  tcp_client_receive(scheduler_client, ack_msg, 10);  // wait for ack message from Webots
  if (strncmp(ack_msg, "FAILED", 6) == 0) {
    fprintf(stderr, "%s",
            robot_name == NULL ? "Could not find any robot with an <extern> controller in the Webots simulation" :
                                 "The specified robot is not in the list of robots with <extern> controllers");
    return false;
  } else if (strncmp(ack_msg, "CONNECTED", 9) != 0) {
    fprintf(stderr, "Error: Unknown Webots response %s.\n", ack_msg);
    exit(EXIT_FAILURE);
  }
  free(ack_msg);

  scheduler_data = malloc(SCHEDULER_DATA_CHUNK);
  scheduler_data_size = SCHEDULER_DATA_CHUNK;
  return true;
}

int scheduler_init_local(const char *pipe) {
  scheduler_pipe = g_pipe_new(pipe);
  if (scheduler_pipe == NULL)
    return false;
  // set WEBOTS_PIPE_IN to facilitate the robot window pipe listening
  char pipe_buffer[64];
#ifdef _WIN32
  sprintf(pipe_buffer, "WEBOTS_PIPE_IN=%d", scheduler_get_pipe_handle());
  putenv(pipe_buffer);
#else
  sprintf(pipe_buffer, "%d", scheduler_get_pipe_handle());
  setenv("WEBOTS_PIPE_IN", pipe_buffer, true);
#endif
  scheduler_data = malloc(SCHEDULER_DATA_CHUNK);
  scheduler_data_size = SCHEDULER_DATA_CHUNK;
  return true;
}

void scheduler_cleanup() {
  int c = 0;
  if (scheduler_is_ipc())
    g_pipe_send(scheduler_pipe, (const char *)&c, 4);  // to make the Webots pipe reading thread exit
  if (scheduler_is_tcp())
    tcp_client_send(scheduler_client, (const char *)&c, 4);  // to make the Webots pipe reading thread exit
  free(scheduler_data);
  free(scheduler_meta);

  if (scheduler_pipe)
    g_pipe_delete(scheduler_pipe);
  if (scheduler_client)
    tcp_client_close(scheduler_client);
}

void scheduler_send_request(WbRequest *r) {
  if (scheduler_is_ipc())
    g_pipe_send(scheduler_pipe, r->data, r->pointer);
  if (scheduler_is_tcp())
    tcp_client_send(scheduler_client, r->data, r->pointer);

  // fprintf(stderr, "@ Send request:\n");
  // request_print(stderr, r);
}

// extern FILE *fd; TODO: to show possible memory leak

WbRequest *scheduler_read_data() {
  WbRequest *r = NULL;
  if (scheduler_is_ipc())
    r = scheduler_read_data_local();
  else if (scheduler_is_tcp())
    r = scheduler_read_data_remote();

  return r;
}

WbRequest *scheduler_read_data_remote() {
  WbRequest *r = NULL;

  int delay = 0, meta_size = 0, data_size = sizeof(int);
  scheduler_meta = malloc(sizeof(unsigned short) + sizeof(int));

  // receive and read the number of chunks
  meta_size += scheduler_receive_meta(meta_size, sizeof(unsigned short));
  int nb_chunks = scheduler_read_short(scheduler_meta);

  // receive and read the total data size (excluding image data)
  meta_size += scheduler_receive_meta(meta_size, sizeof(unsigned int));
  int tot_data_size = scheduler_read_int32(&scheduler_meta[sizeof(unsigned short)]) + sizeof(int);

  // set size at beginning of data array for request
  *((int *)(scheduler_data)) = tot_data_size;

  // if more than 1KB needs to be downloaded, show a progress bar
  // reallocate the scheduler data buffer if necessary
  if ((int)scheduler_data_size < tot_data_size) {
    scheduler_data_size = tot_data_size;
    scheduler_data = realloc(scheduler_data, scheduler_data_size);
    if (scheduler_data == NULL) {
      fprintf(stderr, "Error reading Webots TCP socket messages: not enough memory.\n");
      exit(EXIT_FAILURE);
    }
  }

  // iterate over each chunk
  for (int c = 0; c < nb_chunks; c++) {
    // read chunk size and chunk type
    scheduler_meta = realloc(scheduler_meta, meta_size + sizeof(unsigned int) + sizeof(unsigned char));
    if (scheduler_meta == NULL) {
      fprintf(stderr, "Error receiving Webots request: not enough memory.\n");
      exit(EXIT_FAILURE);
    }
    int chunk_info_size = scheduler_receive_meta(meta_size, sizeof(unsigned int) + sizeof(unsigned char));
    int chunk_size = scheduler_read_int32(scheduler_meta + meta_size);
    unsigned char chunk_type = scheduler_read_char(scheduler_meta + meta_size + sizeof(unsigned int));
    meta_size += chunk_info_size;

    switch (chunk_type) {
      case TCP_DATA_TYPE:
        data_size += scheduler_receive_data(data_size, chunk_size);

        // save the time step from the first chunk
        if (c == 0) {
          delay = scheduler_read_int32(&scheduler_data[sizeof(unsigned int)]);
          if (delay >= 0)  // not immediate
            scheduler_actual_step = delay;
        }
        break;

      case TCP_IMG_TYPE:
        // read the rendering device tag and command
        scheduler_meta = realloc(scheduler_meta, meta_size + sizeof(short unsigned int) + sizeof(unsigned char));
        if (scheduler_meta == NULL) {
          fprintf(stderr, "Error receiving Webots request: not enough memory.\n");
          exit(EXIT_FAILURE);
        }
        int img_info_size = scheduler_receive_meta(meta_size, sizeof(short unsigned int) + sizeof(unsigned char));
        short unsigned int tag = scheduler_read_short(scheduler_meta + meta_size);
        unsigned char cmd = scheduler_read_char(scheduler_meta + meta_size + sizeof(short unsigned int));
        meta_size += img_info_size;

        WbDevice *dev = robot_get_device(tag);
        if (!dev) {
          fprintf(stderr, "Error: Device doesn't no exist.\n");
          exit(EXIT_FAILURE);
        }

        switch (cmd) {
          case C_ABS_CAMERA_SERIAL_IMG:
            wb_abstract_camera_allocate_image(dev, chunk_size);
            const unsigned char *img = wbr_abstract_camera_get_image_buffer(dev);
            if (img == NULL) {
              fprintf(stderr, "Error: Cannot write the image to the rendering device memory.\n");
              exit(EXIT_FAILURE);
            }
            scheduler_receive_img(img, chunk_size);
            break;
          case C_CAMERA_SERIAL_SEGM_IMG:
            camera_allocate_segmentation_image(tag, chunk_size);
            const unsigned char *img_segm = camera_get_segmentation_image_buffer(tag);
            if (img_segm == NULL) {
              fprintf(stderr, "Error: Cannot write the segmentation image to the camera memory.\n");
              exit(EXIT_FAILURE);
            }
            scheduler_receive_img(img_segm, chunk_size);
            break;
          default:
            fprintf(stderr, "Error: Unsupported image data received on TCP connection.\n");
            exit(EXIT_FAILURE);
            break;
        }
        break;
      default:
        fprintf(stderr, "Error: Unsupported chunk type on TCP connection.\n");
        exit(EXIT_FAILURE);
        break;
    }
  }

  free(scheduler_meta);
  scheduler_meta = NULL;

  // create a request to hold the data
  r = request_new_from_data(scheduler_data, scheduler_data_size);
  request_set_immediate(r, (delay < 0));

  // skip size and delay
  request_set_position(r, 2 * sizeof(unsigned int));

  // fprintf(stderr, "@ Read request:\n");
  // request_print(stderr, r);

  return r;
}

WbRequest *scheduler_read_data_local() {
  WbRequest *r = NULL;

  int delay = 0, size = 0, socket_size = 0;

  do
    size += g_pipe_receive(scheduler_pipe, scheduler_data + size, sizeof(int) - size);
  while (size != sizeof(int));

  // read the size of the socket chunk
  socket_size = scheduler_read_int32(scheduler_data);
  // if more than 1KB needs to be downloaded, show a progress bar
  // reallocate the scheduler data buffer if necessary
  if ((int)scheduler_data_size < socket_size) {
    scheduler_data_size = socket_size;
    scheduler_data = realloc(scheduler_data, scheduler_data_size);
    if (scheduler_data == NULL) {
      fprintf(stderr, "Error reading Webots socket messages: not enough memory.\n");
      exit(EXIT_FAILURE);
    }
  }
  // read all the remaining data from the packet
  size += scheduler_receive_data(size, socket_size - size);

  // save the time step
  delay = scheduler_read_int32(&scheduler_data[sizeof(unsigned int)]);
  if (delay >= 0)  // not immediate
    scheduler_actual_step = delay;

  // create a request to hold the data
  // printf("Message: Local (size=%d)\n", socket_size);
  // printf("scheduler_data_size = %d\n", scheduler_data_size);
  r = request_new_from_data(scheduler_data, scheduler_data_size);
  request_set_immediate(r, (delay < 0));

  // skip size and delay
  request_set_position(r, 2 * sizeof(unsigned int));

  // fprintf(stderr, "@ Read request:\n");
  // request_print(stderr, r);

  return r;
}

int scheduler_receive_meta(int pointer, size_t type_size) {
  int curr_size = 0;
  do
    curr_size += tcp_client_receive(scheduler_client, scheduler_meta + pointer + curr_size, type_size - curr_size);
  while (curr_size != type_size);

  return curr_size;
}

int scheduler_receive_data(int pointer, int chunk_size) {
  int curr_size = 0;
  while (curr_size < chunk_size) {
    int block_size = chunk_size - curr_size;
    if (block_size > 4096)
      block_size = 4096;

    if (scheduler_is_ipc())
      curr_size += g_pipe_receive(scheduler_pipe, scheduler_data + pointer + curr_size, block_size);
    else if (scheduler_is_tcp())
      curr_size += tcp_client_receive(scheduler_client, scheduler_data + pointer + curr_size, block_size);
  }

  return curr_size;
}

void scheduler_receive_img(const unsigned char *img_buffer, int img_size) {
  int curr_size = 0;
  while (curr_size < img_size) {
    int block_size = img_size - curr_size;
    if (block_size > 4096)
      block_size = 4096;

    curr_size += tcp_client_receive(scheduler_client, (char *)img_buffer + curr_size, block_size);
  }
}

bool scheduler_is_ipc() {
  return (scheduler_pipe != NULL);
}

bool scheduler_is_tcp() {
  return (scheduler_client != NULL);
}

int scheduler_get_pipe_handle() {
  return g_pipe_get_handle(scheduler_pipe);
}
