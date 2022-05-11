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
#include <webots/types.h>
#include "g_pipe.h"
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
GPipe *scheduler_pipe = NULL;
TcpClient *scheduler_client = NULL;
char *scheduler_protocol = NULL;

int scheduler_init(const char *pipe) {
  if (scheduler_protocol == NULL) {
    fprintf(stderr, "Impossible to connect the controller to Webots: no connection protocol given.\n");
    exit(EXIT_FAILURE);
  }
  if (strncmp(scheduler_protocol, "IPC", 3) == 0) {
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
  } else if (strncmp(scheduler_protocol, "TCP", 3) == 0) {
    const char *url_suffix = strstr(pipe, ":");
    int host_length = strlen(pipe) - strlen(url_suffix);
    char *host = malloc(host_length);
    int port;

    memcpy(host, pipe, host_length);
    sscanf(url_suffix, ":%d", &port);
    scheduler_client = tcp_client_new(host, port);
    free(host);

    if (scheduler_client == NULL)
      return false;

    const char *robot_name = strstr(url_suffix, "/");

    char *init_msg = strdup("CTR");
    if (robot_name != NULL) {
      strcat(init_msg, "\nRobot-Name: ");
      strcat(init_msg, &robot_name[1]);
    }
    tcp_client_send(scheduler_client, init_msg, strlen(init_msg));
    free(init_msg);

    char ack_msg[20];
    fprintf(stdout, "Waiting for Webots...\n");
    tcp_client_receive(scheduler_client, ack_msg, 20);  // wait for ack message from Webots
    if (strcmp(ack_msg, "CONNECTED") == 0) {
      fprintf(stdout, "Connection to robot established.\n");
    } else if (strcmp(ack_msg, "FAILED") == 0) {
      fprintf(stderr, "%s.\n",
              robot_name == NULL ? "No robot with <extern> controllers is specified in the Webots simulation.\n" :
                                   "The specified robot is not in the list of robots with <extern> controllers.\n");
      exit(EXIT_FAILURE);
    } else {
      fprintf(stderr, "Unknown Webots response %s.\n", ack_msg);
      exit(EXIT_FAILURE);
    }

    char *test_msg = strdup("TST");
    tcp_client_send(scheduler_client, test_msg, strlen(test_msg));

  } else {
    fprintf(stderr, "Impossible to connect the controller to Webots: unknown protocol %s.\n", scheduler_protocol);
    exit(EXIT_FAILURE);
  }

  scheduler_data = malloc(SCHEDULER_DATA_CHUNK);
  scheduler_data_size = SCHEDULER_DATA_CHUNK;

  return true;
}

void scheduler_cleanup() {
  int c = 0;
  g_pipe_send(scheduler_pipe, (const char *)&c, 4);  // to make the Webots pipe reading thread exit
  free(scheduler_data);
  if (scheduler_pipe)
    g_pipe_delete(scheduler_pipe);
}

void scheduler_send_request(WbRequest *r) {
  if (scheduler_pipe)
    g_pipe_send(scheduler_pipe, r->data, r->pointer);

  // fprintf(stderr, "@ Send request:\n");
  // request_print(stderr, r);
}

// extern FILE *fd; TODO: to show possible memory leak

WbRequest *scheduler_read_data() {
  int delay = 0;
  if (scheduler_pipe) {
    int size = 0, socket_size = 0;
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
      if (scheduler_pipe)
        size += g_pipe_receive(scheduler_pipe, scheduler_data + size, chunk_size);
    }
    // save the time step
    delay = scheduler_read_int32(&scheduler_data[sizeof(unsigned int)]);
    if (delay >= 0)  // not immediate
      scheduler_actual_step = delay;
  }

  WbRequest *r = NULL;
  if (scheduler_pipe) {
    // create a request to hold the data
    // printf("Message: Local (size=%d)\n", socket_size);
    r = request_new_from_data(scheduler_data, scheduler_data_size);
    request_set_immediate(r, (delay < 0));

    // skip size and delay
    request_set_position(r, 2 * sizeof(unsigned int));
  }

  // fprintf(stderr, "@ Read request:\n");
  // request_print(stderr, r);

  return r;
}

bool scheduler_is_local() {
  return scheduler_pipe != NULL;
}

int scheduler_get_pipe_handle() {
  return g_pipe_get_handle(scheduler_pipe);
}
