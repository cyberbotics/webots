/*
 * Copyright 1996-2024 Cyberbotics Ltd.
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

#ifndef WEBOTS_RADIO_H
#define WEBOTS_RADIO_H

#ifdef __cplusplus
extern "C" {
#endif

/* event types for WebotsRadioEvent */
#define WEBOTS_RADIO_EVENT_RECEIVE          1
#define WEBOTS_RADIO_EVENT_NETWORK_DETECTED 2
#define WEBOTS_RADIO_EVENT_LOST_CONNECTION  3

struct WebotsRadioEvent {
  int         type;      /* see above the WEBOTS_RADIO_EVENT_* type list */
  const char *data;      /* data chunk received by the radio */
  int         data_size; /* size of the data chunk */
  const char *from;      /* address of the emitter of the data */
  double      rssi;      /* wireless networking parameter */
  void       *user_data; /* user pointer defined in radio_set_callback() */
};

void webots_radio_init(void);          /* initialization routine, to be called
                                          prior to any function call */

void webots_radio_cleanup(void);       /* cleanup rountine, no further
                                          webots_radio_xxx() function can be
                                          called after this one */

int webots_radio_new(void);            /* create a new radio node */

/* parameter setters */
void webots_radio_set_protocol(int radio,const char *protocol);
void webots_radio_set_address(int radio,const char *address);
void webots_radio_set_frequency(int radio,double frequency);
void webots_radio_set_channel(int radio,int channel);
void webots_radio_set_bitrate(int radio,double bitrate);
void webots_radio_set_rx_sensitivity(int radio,double rx_sensitivity);
void webots_radio_set_tx_power(int radio,double tx_power);
void webots_radio_set_callback(int radio,void *user_data,
			       void(*)(const int radio,const struct WebotsRadioEvent *));

/* move a radio node */
void webots_radio_move(int radio,double x,double y,double z);

void webots_radio_send(int radio,const void *dest,const char *data,int size,double delay);

void webots_radio_delete(int radio);    /* delete a radio node */

void webots_radio_run(double seconds);   /* run the network simulation for
                                           a given number of seconds */

/* radio connected mode: to be implemented later on */

int  webots_radio_connection_open(int radio,const char *destination);
int  webots_radio_connection_send(int connection,const char *data,int size);
void webots_radio_connection_set_callback(int radio,void *ptr,
					  void(*receive_func)(void *ptr,
							      char *data,
							      int size));
void webots_radio_connection_close(int connection);

#ifdef __cplusplus
}
#endif

#endif /* WEBOTS_RADIO_H */
