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

/*
 * inpired from epuckupload perl script by Xavier Raemy and Thomas Lochmatter
 */

#include "Uploader.hpp"

#include "Serial.hpp"
#include "Time.hpp"

#include <assert.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fstream>
#include <stdexcept>

using namespace std;

typedef struct MemoryChunkStruct {  // size is 64KB = 0x10000 bytes
  char order;
  unsigned char *data;
  int address;
  int len;
  struct MemoryChunkStruct *next;
} MemoryChunk;

typedef struct PacketStruct {
  unsigned char data[101];  // 4+32*3+1
  struct PacketStruct *next;
} Packet;

Serial *Uploader::cSerial = NULL;
static void (*gUpdateProgressFunction)(int, int) = NULL;
static MemoryChunk *gMemoryChunk = NULL;
static Packet *gPacket = NULL;
static int gNumberOfPackets = 0;
static bool gUploadCanceled = false;

static MemoryChunk *get_memory_chunk(int address, char order) {
  MemoryChunk *m;

  if (gMemoryChunk) {
    for (m = gMemoryChunk; m->next; m = m->next) {
      if (address == m->address)
        return m;
    }
    if (address == m->address)
      return m;

    m->next = static_cast<MemoryChunk *>(malloc(sizeof(MemoryChunk)));
    m = m->next;
  } else {
    gMemoryChunk = static_cast<MemoryChunk *>(malloc(sizeof(MemoryChunk)));
    m = gMemoryChunk;
  }

  m->address = address;
  m->data =
    static_cast<unsigned char *>(malloc(0x10010));  // the hex file sometimes goes over 0xffff addresses with a maximum of 0x10
  memset(m->data, 0xff, 0x10000);
  m->len = 0;
  m->order = 0;
  m->next = NULL;
  return m;
}

static void delete_memory_chunks() {
  MemoryChunk *m = gMemoryChunk, *next;
  while (m) {
    next = m->next;
    free(m->data);
    free(m);
    m = next;
  }

  gMemoryChunk = NULL;
}

// Sets the bootloader return address
static void entrypoint_startsequence(unsigned char *v) {
  char old_value[3];
  MemoryChunk *m = get_memory_chunk(0, 2);
  old_value[0] = m->data[0];
  old_value[1] = m->data[1];
  old_value[2] = m->data[4];

  m->data[0] = v[0];
  m->data[1] = v[1];
  m->data[2] = 0x04;
  m->data[3] = 0x00;
  m->data[4] = v[2];
  m->data[5] = 0x00;
  m->data[6] = 0x00;
  m->data[7] = 0x00;

  v[0] = old_value[0];
  v[1] = old_value[1];
  v[2] = old_value[2];
}

// Sets the bootloader return address
static void entrypoint_returnaddress(int bootloader, const unsigned char *address, int robot_id) {
  MemoryChunk *m = get_memory_chunk(bootloader, 1);
  unsigned char *data = m->data;
  data[108] = robot_id & 0xff;
  data[109] = (robot_id >> 8) & 0xff;
  data[110] = 0x00;
  data[111] = 0x00;
  data[112] = address[0];
  data[113] = address[1];
  data[114] = 0x04;
  data[115] = 0x00;
  data[116] = address[2];
  data[117] = 0x00;
  data[118] = 0x00;
  data[119] = 0x00;
  if (m->len < 120)
    m->len = 120;
}

static int readfileINHX32(const std::string &filename) {
  char hex[5];
  int line_counter;
  unsigned int value;
  MemoryChunk *m = NULL;

  std::ifstream file(filename.c_str(), fstream::in);
  if (!file)
    return -1;

  line_counter = 0;
  string line;
  while (getline(file, line)) {
    line_counter++;
    int i = 1;
    hex[0] = line[i++];
    hex[1] = line[i++];
    hex[2] = '\0';
    sscanf(hex, "%x", &value);
    const unsigned char len = (unsigned char)value;
    hex[0] = line[i++];
    hex[1] = line[i++];
    hex[2] = line[i++];
    hex[3] = line[i++];
    hex[4] = '\0';
    sscanf(hex, "%x", &value);
    const unsigned short int address = (unsigned short int)value;
    hex[0] = line[i++];
    hex[1] = line[i++];
    hex[2] = '\0';
    sscanf(hex, "%x", &value);
    const unsigned char type = (unsigned char)value;
    int j = i; /* start of data chunk */
    int k = (line.length() - j) / 2 - 1;
    if (k != len) {
      fprintf(stderr, "wrong data length on line %d, ignored\n", line_counter);
      continue;
    }
    i += 2 * len;
    hex[0] = line[i++];
    hex[1] = line[i--];  // return to previous position with i
    // 0 already set for hex[2]
    sscanf(hex, "%x", &value);
    // printf("%s (len=%d, address=%x, type=%d checksum=%x)\n",line,len,address,type,(unsigned char)value);
    if (type == 0 && m != NULL) {
      for (k = 0; k < len; k++) {
        hex[0] = line[j++];
        hex[1] = line[j++];
        // 0 already set for hex[2]
        sscanf(hex, "%x", &value);
        if (address + k >= 0x10010) {
          fprintf(
            stderr,
            "Error, a single line of the hex file runs past the 64 KB boundary of a chunk, which is not supported, aborting.");
          file.close();
          return 1;
        }
        m->data[address + k] = (unsigned char)value;
      }
      if (m->len < address + len)
        m->len = address + len;
    } else if (type == 4) {
      hex[0] = line[j++];
      hex[1] = line[j++];
      hex[2] = line[j++];
      hex[3] = line[j++];
      hex[4] = '\0';
      sscanf(hex, "%x", &value);
      if (value < 4) {
        value *= 0x10000;
        m = get_memory_chunk(value, 0);
      } else
        m = NULL;
    }
  }

  file.close();

  return 1;
}

// Prepares the packets to be transmitted
static void create_packets() {
  MemoryChunk *m, **m_array;
  Packet *p = NULL;
  int i, j, k, n = 0, writeaddress, sum;

  // sort the memory chunk by order in m_array
  for (m = gMemoryChunk; m; m = m->next)
    n++;

  m_array = static_cast<MemoryChunk **>(malloc(n * sizeof(MemoryChunk *)));
  i = 0;
  for (j = 0; j < 3; j++)
    for (m = gMemoryChunk; m; m = m->next)
      if (m->order == j)
        m_array[i++] = m;

  for (i = 0; i < n; i++) {
    int addr = 0;
    while (addr < m_array[i]->len) {
      if (p == NULL) {
        p = static_cast<Packet *>(malloc(sizeof(Packet)));
        gNumberOfPackets = 1;
        gPacket = p;
      } else {
        p->next = static_cast<Packet *>(malloc(sizeof(Packet)));
        gNumberOfPackets++;
        p = p->next;
      }

      writeaddress = (addr + m_array[i]->address) / 2;
      j = 0;
      sum = 0;
      p->data[j] = writeaddress & 0xff;
      sum += p->data[j++];
      p->data[j] = (writeaddress >> 8) & 0xff;
      sum += p->data[j++];
      p->data[j] = (writeaddress >> 16) & 0xff;
      sum += p->data[j++];
      p->data[j] = 0x60;
      sum += p->data[j++];
      for (k = 0; k < 32; k++) {
        if (addr < m_array[i]->len) {
          p->data[j] = m_array[i]->data[addr++];
          sum += p->data[j++];
          p->data[j] = m_array[i]->data[addr++];
          sum += p->data[j++];
          p->data[j] = m_array[i]->data[addr++];
          sum += p->data[j++];
        } else {
          p->data[j++] = 0xff;
          p->data[j++] = 0xff;
          p->data[j++] = 0xff;
          sum += 3 * 0xff;
        }
        addr++;
      }

      sum &= 0xff;
      p->data[j++] = ((sum == 0) ? 0 : 256 - sum);  // check sum
    }
    p->next = NULL;  // last packet
  }
  free(m_array);
}

static void delete_packets() {
  Packet *p = gPacket, *next;
  while (p) {
    next = p->next;
    free(p);
    p = next;
  }
  gPacket = NULL;
  gNumberOfPackets = 0;
}

static int send_packets(Serial *serial) {
  // send reset signal: R, ignore answered chars
  // send bootloader start char: 0xC1
  // receive 'q' and 'K' answer
  // send packet
  // receive 'K' answer
  // ...
  // send packet
  // receive 'K' answer
  // send termination: 0x95, 0x00, 0x00, 0xFF
  int j;
  char buffer[64];
  Packet *p;

  serial->talk("V\n");      // eat up first buggy answer if any
  serial->write("R\n", 2);  // state RESET_WAIT_q

  for (j = 0; j < 10; j++) {
    buffer[0] = 0xC1;
    int ok = 0;
    serial->write(buffer, 1);
    Time time;
    while (time.elapsedTime() < 1000) {  // give the e-puck 1s to answer
      int n = serial->read(buffer, 64, false);
      int i = 0;
      if (ok == 0)
        for (; i < n; i++)
          if (buffer[i] == 'q') {  // state RESET_WAIT_K
            ok = 1;
            i++;
            break;
          }
      if (ok == 1)
        for (; i < n; i++)
          if (buffer[i] == 'K') {
            ok = 2;
            break;
          }
    }
    if (ok == 2)
      break;

    if (gUploadCanceled)
      return -6;  // canceled by the user
    gUpdateProgressFunction(2, 100 - 10 * j);
  }

  if (j == 100)
    return -2;  // fail, we didn't receive the ack from bootloader on time

  p = gPacket;
  double globalProgress = 0.0;
  double progressStep = 100.0 / gNumberOfPackets;
  while (p) {
    if (gUploadCanceled)
      return -6;  // canceled by the user

    gUpdateProgressFunction(1, globalProgress);
    serial->write(reinterpret_cast<const char *>(p->data), 101);
    serial->read(buffer, 1, 1);

    switch (buffer[0]) {
      case 'K':
        break;  // all right
      case 'N':
        return -3;  // checksum error
      default:
        return -4;  // unknown error
    }

    fflush(stdout);
    p = p->next;
    globalProgress += progressStep;
  }

  buffer[0] = 0x95;
  buffer[1] = 0x00;
  buffer[2] = 0x00;
  buffer[3] = 0xff;
  serial->write(buffer, 4);  // send termination

  return 0;  // success
}

int Uploader::connect(const std::string &port) {
  try {
    cSerial = new Serial(port);
  } catch (const runtime_error &e) {
    cSerial = NULL;
    return -1;  // serial communication error
  }

  return 0;
}

void Uploader::disconnect() {
  delete cSerial;
  cSerial = NULL;
}

int Uploader::send(int robot_id, const std::string &hexfilename, void (*updateProgressFunction)(int, int)) {
  gUpdateProgressFunction = updateProgressFunction;
  gUploadCanceled = false;

  unsigned char program_address[] = {0x40, 0x7f, 0x01};

  if (!cSerial)
    return -5;  // serial communication error

  int success = -4;  // unknown error
  if (readfileINHX32(hexfilename) == -1)
    return -1;  // can't open file
  entrypoint_startsequence(program_address);
  entrypoint_returnaddress(0x17f00 * 2, program_address, robot_id);
  create_packets();
  success = send_packets(cSerial);

  delete_packets();
  delete_memory_chunks();
  disconnect();

  return success;
}

void Uploader::cancelUpload() {
  gUploadCanceled = true;
}
