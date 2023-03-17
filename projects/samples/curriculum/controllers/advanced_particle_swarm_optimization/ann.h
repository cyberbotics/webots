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

/*************************************
 *
 * This header defines the properties
 * of the artificial neural network
 *
 **************************************/

#ifndef ANN_H
#define ANN_H

#define NB_SENSORS 8
#define IN_SIZE (NB_SENSORS + 4)  // network input layer size
#define HID_SIZE 24               // network hidden layer size
#define OUT_SIZE 2                // network output layer size

#include "pso.h"

/* GLOBAL DATA FOR NETWORK */

/*float i[IN_SIZE];
float W[OUT_SIZE][IN_SIZE];
float vW[OUT_SIZE][IN_SIZE];
float o[OUT_SIZE];
float g[OUT_SIZE];

layer_t layers[1] = {{IN_SIZE,OUT_SIZE,i,&W[0][0],&vW[0][0],o,g}};
network_t network = {1,layers};

float pi[IN_SIZE];
float pW[OUT_SIZE][IN_SIZE];
float pvW[OUT_SIZE][IN_SIZE];
float po[OUT_SIZE];
float pg[OUT_SIZE];

layer_t players[1] = {{IN_SIZE,OUT_SIZE,pi,&pW[0][0],&pvW[0][0],po,pg}};
network_t pnetwork = {1,players};

float gi[IN_SIZE];
float gW[OUT_SIZE][IN_SIZE];
float gvW[OUT_SIZE][IN_SIZE];
float go[OUT_SIZE];
float gg[OUT_SIZE];

layer_t glayers[1] = {{IN_SIZE,OUT_SIZE,gi,&gW[0][0],&gvW[0][0],go,gg}};
network_t gnetwork = {1,glayers};*/

float i1[IN_SIZE];
float W1[HID_SIZE][IN_SIZE];
float vW1[HID_SIZE][IN_SIZE];
float o1[HID_SIZE];
float g1[HID_SIZE];

float i2[HID_SIZE];
float W2[OUT_SIZE][HID_SIZE];
float vW2[OUT_SIZE][HID_SIZE];
float o2[OUT_SIZE];
float g2[OUT_SIZE];

// Create both layers.  Note that this intializes the layers
// with references to the memory allocated above.
layer_t layers[2] = {{IN_SIZE, HID_SIZE, i1, &W1[0][0], &vW1[0][0], o1, g1},
                     {HID_SIZE, OUT_SIZE, i2, &W2[0][0], &vW2[0][0], o2, g2}};

// Create the network and initialize with the layers previously
// allocated.
network_t network = {2, layers};

float pi1[IN_SIZE];
float pW1[HID_SIZE][IN_SIZE];
float pvW1[HID_SIZE][IN_SIZE];
float po1[HID_SIZE];
float pg1[HID_SIZE];

float pi2[HID_SIZE];
float pW2[OUT_SIZE][HID_SIZE];
float pvW2[OUT_SIZE][HID_SIZE];
float po2[OUT_SIZE];
float pg2[OUT_SIZE];

// Create both layers.  Note that this intializes the layers
// with references to the memory allocated above.
layer_t players[2] = {{IN_SIZE, HID_SIZE, pi1, &pW1[0][0], &pvW1[0][0], po1, pg1},
                      {HID_SIZE, OUT_SIZE, pi2, &pW2[0][0], &pvW2[0][0], po2, pg2}};

// Create the pbest network and initialize with the layers previously
// allocated.
network_t pnetwork = {2, players};

float gi1[IN_SIZE];
float gW1[HID_SIZE][IN_SIZE];
float gvW1[HID_SIZE][IN_SIZE];
float go1[HID_SIZE];
float gg1[HID_SIZE];

float gi2[HID_SIZE];
float gW2[OUT_SIZE][HID_SIZE];
float gvW2[OUT_SIZE][HID_SIZE];
float go2[OUT_SIZE];
float gg2[OUT_SIZE];

// Create both layers.  Note that this intializes the layers
// with references to the memory allocated above.
layer_t glayers[2] = {{IN_SIZE, HID_SIZE, gi1, &gW1[0][0], &gvW1[0][0], go1, gg1},
                      {HID_SIZE, OUT_SIZE, gi2, &gW2[0][0], &gvW2[0][0], go2, gg2}};

// Create the network and initialize with the layers previously
// allocated.
network_t gnetwork = {2, glayers};

#endif  // ANN_H
