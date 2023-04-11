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

/**
 * Defines interface for PSO unsupervised learning of the ANN
 */

#ifndef PSO_H
#define PSO_H

/**
 * layer structure
 */
typedef struct {
  int width;  // number of inputs to each neuron (M)
  int depth;  // number of neurons in the layer (N)

  float *x;   // pointer to layer input    [Mx1]
  float *W;   // pointer to weight matrix  [NxM]
  float *vW;  // pointer to weight speed matrix [NxM]
  float *y;   // pointer to layer output   [Nx1]
  float *g;   // pointer to layer gradient [Nx1]
} layer_t;

/**
 * network structure
 */
typedef struct {
  int size;         // number of layers
  layer_t *layers;  // pointer to layer array
} network_t;

/**********************************************************************
 * NETWORK MANIPULATION FUNCTIONS
 *
 * These are the core functions for using the backpropagation network
 *
 *********************************************************************/

/**
 * Input values to network
 */
extern void InputToNetwork(const network_t *n, const float *values);

/**
 * Activate the network.
 */
extern void ActivateNetwork(network_t *n);

/**
 * Output values from network
 */
extern void OutputFromNetwork(network_t *n, float *values);

/**
 * Randomize weights for a network
 */
extern void RandomizeNetwork(network_t *n);

/**
 * Evolve the network to the next PSO iteration.
 */
extern void EvolveNetwork(network_t *n, network_t *pbest, network_t *gbest);

/**********************************************************************
 * NETWORK VIEWING AND FILE I/O
 *
 * These functions are useful when writing a training program.
 *
 *********************************************************************/

/**
 * Print the entire network
 */
extern void PrintNetwork(network_t *n);

/**
 * Print the network output
 */
extern void PrintNetworkOutput(network_t *n);

/**
 * Save the network weights to a file
 */
extern void SaveNetworkWeights(network_t *n, const char *filename);

extern void SaveNetworkWeightsHDT(network_t *n, const char *filename);

/**
 * Load the network weights from a file
 */
extern void LoadNetworkWeights(network_t *n, const char *filename);

#endif  // PSO_H
