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
#define SCHEDULER_DATA_TYPE 0
#define SCHEDULER_IMG_TYPE 1

unsigned int scheduler_data_size = 0;
unsigned int scheduler_actual_step = 0;
char *scheduler_data = NULL;
GPipe *scheduler_pipe = NULL;
TcpClient *scheduler_client = NULL;

int scheduler_init_remote(const char *host, int port, const char *robot_name) {
  scheduler_client = tcp_client_new(host, port);
  if (scheduler_client == NULL)
    return false;

  char *init_msg = strdup("CTR");
  if (robot_name != NULL) {
    strcat(init_msg, "\nRobot-Name: ");
    strcat(init_msg, &robot_name[1]);
  }
  tcp_client_send(scheduler_client, init_msg, strlen(init_msg));
  free(init_msg);

  char ack_msg[12];
  tcp_client_receive(scheduler_client, ack_msg, 12);  // wait for ack message from Webots
  if (strncmp(ack_msg, "FAILED", 6) == 0) {
    fprintf(stderr, "%s",
            robot_name == NULL ? "Could not find any robot with an <extern> controller in the Webots simulation" :
                                 "The specified robot is not in the list of robots with <extern> controllers");
    return false;
  } else if (strncmp(ack_msg, "CONNECTED", 9) != 0) {
    fprintf(stderr, "Error: Unknown Webots response %s.\n", ack_msg);
    exit(EXIT_FAILURE);
  }

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
  /*for (int tag = 0; tag < scheduler_nb_devices; tag++) {
    free(scheduler_devices[tag]);
  }
  free(scheduler_devices);*/
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

  int delay = 0, meta_size = 0, curr_meta_size = 0, data_size = sizeof(int), curr_data_size = 0;
  char *scheduler_meta = malloc(sizeof(unsigned short) + sizeof(int));

  // receive and read the number of chunks
  do
    meta_size += tcp_client_receive(scheduler_client, scheduler_meta + meta_size, sizeof(unsigned short) - meta_size);
  while (meta_size != sizeof(unsigned short));

  int nb_chunks = scheduler_read_short(scheduler_meta);
  // printf("nb_chunks = %d\n", nb_chunks);

  // receive and read the total data size (excluding image data)
  do
    curr_meta_size +=
      tcp_client_receive(scheduler_client, scheduler_meta + meta_size + curr_meta_size, sizeof(unsigned int) - curr_meta_size);
  while (curr_meta_size != sizeof(unsigned int));
  meta_size += curr_meta_size;

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
    curr_meta_size = 0;
    do
      curr_meta_size += tcp_client_receive(scheduler_client, scheduler_meta + meta_size + curr_meta_size,
                                           sizeof(unsigned int) + sizeof(unsigned char) - curr_meta_size);
    while (curr_meta_size != sizeof(unsigned int) + sizeof(unsigned char));
    int chunk_size = scheduler_read_int32(scheduler_meta + meta_size);
    unsigned char chunk_type = scheduler_read_char(scheduler_meta + meta_size + sizeof(unsigned int));
    meta_size += curr_meta_size;

    // printf("chunk_size = %d\n", chunk_size);
    // printf("chunk_type = %d\n", chunk_type);
    switch (chunk_type) {
      case SCHEDULER_DATA_TYPE:
        curr_data_size = 0;
        while (curr_data_size < chunk_size) {
          int block_size = chunk_size - curr_data_size;
          if (block_size > 4096)
            block_size = 4096;

          curr_data_size += tcp_client_receive(scheduler_client, scheduler_data + data_size + curr_data_size, block_size);
        }
        data_size += curr_data_size;

        // save the time step from the first chunk
        if (c == 0) {
          delay = scheduler_read_int32(&scheduler_data[sizeof(unsigned int)]);
          if (delay >= 0)  // not immediate
            scheduler_actual_step = delay;
        }
        break;

      case SCHEDULER_IMG_TYPE:
        // read the device tag and command
        scheduler_meta = realloc(scheduler_meta, meta_size + sizeof(short unsigned int) + sizeof(unsigned char));
        if (scheduler_meta == NULL) {
          fprintf(stderr, "Error receiving Webots request: not enough memory.\n");
          exit(EXIT_FAILURE);
        }
        curr_meta_size = 0;
        do
          curr_meta_size += tcp_client_receive(scheduler_client, scheduler_meta + meta_size + curr_meta_size,
                                               sizeof(short unsigned int) + sizeof(unsigned char) - curr_meta_size);
        while (curr_meta_size != sizeof(short unsigned int) + sizeof(unsigned char));
        short unsigned int tag = scheduler_read_short(scheduler_meta + meta_size);
        unsigned char cmd = scheduler_read_char(scheduler_meta + meta_size + sizeof(short unsigned int));
        meta_size += curr_meta_size;

        WbDevice *dev = robot_get_device(tag);
        if (!dev) {
          fprintf(stderr, "Error: Device doesn't no exist.\n");
          exit(EXIT_FAILURE);
        }

        int curr_img_size = 0;
        switch (cmd) {
          case C_ABS_CAMERA_SERIAL_IMG:
            wb_abstract_camera_allocate_image(dev, chunk_size);
            const unsigned char *img = wbr_abstract_camera_get_image_buffer(dev);
            if (img == NULL) {
              fprintf(stderr, "Error: Can't write the image to the rendering device memory.\n");
              exit(EXIT_FAILURE);
            }
            while (curr_img_size < chunk_size) {
              int block_size = chunk_size - curr_img_size;
              if (block_size > 4096)
                block_size = 4096;

              curr_img_size += tcp_client_receive(scheduler_client, (char *)(img + curr_img_size), block_size);
            }
            break;
          case C_CAMERA_SERIAL_SEGM_IMG:
            camera_allocate_segmentation_image(tag, chunk_size);
            const unsigned char *img_segm = camera_get_segmentation_image_buffer(tag);
            if (img_segm == NULL) {
              fprintf(stderr, "Error: Can't write the segmentation image to the camera memory.\n");
              exit(EXIT_FAILURE);
            }
            while (curr_img_size < chunk_size) {
              int block_size = chunk_size - curr_img_size;
              if (block_size > 4096)
                block_size = 4096;

              curr_img_size += tcp_client_receive(scheduler_client, (char *)(img_segm + curr_img_size), block_size);
            }
            break;
          default:
            fprintf(stderr, "Error: Unsupported image data received on TCP connection.\n");
            exit(EXIT_FAILURE);
            break;
        }
        break;
      default:
        fprintf(stderr, "Error: Unsupported data type.\n");
        exit(EXIT_FAILURE);
        break;
    }
  }

  free(scheduler_meta);

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
  while (size < socket_size) {
    int chunk_size = socket_size - size;
    if (chunk_size > 4096)
      chunk_size = 4096;

    size += g_pipe_receive(scheduler_pipe, scheduler_data + size, chunk_size);
  }
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

/*void scheduler_init_devices(WbDevice **dev_list, int nb_dev) {
  scheduler_devices = malloc(sizeof(WbDevice *) * nb_dev);
  for (int tag = 0; tag < nb_dev; tag++) {
    scheduler_devices[tag] = malloc(sizeof(WbDevice));
    scheduler_devices[tag] = dev_list[tag];
  }
  scheduler_nb_devices = nb_dev;
}

void scheduler_update_devices(WbDevice **dev_list, int nb_dev) {
  for (int tag = 0; tag < scheduler_nb_devices; tag++) {
    free(scheduler_devices[tag]);
  }
  scheduler_devices = realloc(scheduler_devices, sizeof(WbDevice *) * nb_dev);
  for (int tag = 0; tag < nb_dev; tag++) {
    scheduler_devices[tag] = malloc(sizeof(WbDevice));
    scheduler_devices[tag] = dev_list[tag];
  }
  scheduler_nb_devices = nb_dev;
}
*/
bool scheduler_is_ipc() {
  return (scheduler_pipe != NULL);
}

bool scheduler_is_tcp() {
  return (scheduler_client != NULL);
}

int scheduler_get_pipe_handle() {
  return g_pipe_get_handle(scheduler_pipe);
}
