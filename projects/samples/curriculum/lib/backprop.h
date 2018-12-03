/*
 * Copyright 1996-2018 Cyberbotics Ltd.
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

/**
 * Defines interface for backpropagation network (backprop)
 * The network can be any size, but must be intialized before
 * using any of the functions. See xor_backprop.c for an example
 * of a backprop for the XOR problem.
 *
 * For most applications of the backprop, there will
 * be two programs: a training program and the application program.
 * The application program will only need to load the weight data,
 * input data to the network and retrieve the output.  For a training program,
 * the weights must be randomized, then trained with an appropriate
 * training set and the weights saved to an output file.
 *
 * When using creating a network with backprop.h, the first layer is implied.  Thus,
 * for a three-layer network, only two layers need to be defined.  In the case of
 * a three layer network, the hidden layer input serves also as the input layer.
 * See xor_backprop.h for an example of a simple three-layer network.
 *
 * The learning method used by the backprop is a gradient descent without
 * momentum.  For stable training used a learning rate close to 0 (e.g. 0.1 or 0.2),
 * for faster training use a learning rate close to 1 (e.g. 0.9 or 0.8).
 * Use caution when increasing the learning rate however because too large a rate
 * can cause the network never to converge on a set of weights.
 *
 * Note, there is no check performed on the data in the array, so if the pointers
 * are not referencing arrays of appropriate size, then memory errors will occur.
 * The sizes of the array which each pointer should reference are shown in the
 * layer_t definition.
 *
 *
 * written by:
 * Joshua Petitt
 * Center for Intelligent Information Processing (CIIPS)
 * University of Western Australia
 * 2003
 *
 * modified by:
 * Fabien Rohrer
 * Cyberbotics
 * 2008
 */

#ifndef BACKPROP_H
#define BACKPROP_H

/**
 * layer structure
 */
typedef struct {
  int width;  // number of inputs to each neuron (M)
  int depth;  // number of neurons in the layer (N)

  float *x;  // pointer to layer input    [Mx1]
  float *W;  // pointer to weight matrix  [NxM]
  float *y;  // pointer to layer output   [Nx1]
  float *g;  // pointer to layer gradient [Nx1]
} layer_t;

/**
 * network structure
 */
typedef struct {
  int size;         // number of layers
  float lr;         // learning rate;
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
extern void InputToNetwork(network_t *n, float *values);

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
 * Train a network for one step.  The average error between network
 * output and desired output (yd) is returned.
 */
extern float TrainNetwork(network_t *n, float *yd);

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

/**
 * Load the network weights from a file
 */
extern void LoadNetworkWeights(network_t *n, const char *filename);

#endif  // BACKPROP_H
