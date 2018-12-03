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
 * Implements a simple backpropagation neural network.
 * See backprop.h for more information on using the backprop network.
 *
 * written by
 * Joshua Petitt
 * Center for Intelligent Information Processing (CIIPS)
 * University of Western Australia
 * 2003
 *
 * modified by
 * Fabien Rohrer
 * Cyberbotics
 * 2008
 */

#include "backprop.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <webots/robot.h>

/***************************************************************
 * LOCAL FUNCTIONS
 **************************************************************/

double sigmoid(double x) {
  return 1.0 / (1.0 + exp(-x));  // the sigmoid function
}

/************************************
 * Layer Manipulation functions
 ************************************/

void RandomizeLayer(layer_t *l) {
  int i;
  float *W;

  W = l->W;
  for (i = 0; i < (l->depth) * (l->width); i++) {
    *W = (float)(2.0 * (((double)rand()) / RAND_MAX) - 1.0);
    W++;
  }
}

void InputToLayer(layer_t *l, float *values) {
  int i;
  float *x;

  x = l->x;
  for (i = 0; i < l->width; i++)
    x[i] = values[i];
}

void ActivateLayer(layer_t *l) {
  int i, j;
  float *W;

  // for each neuron in layer
  W = l->W;
  for (i = 0; i < l->depth; i++) {
    // calculate weighted input
    float sum = 0;
    for (j = 0; j < l->width; j++) {
      sum += (*W) * l->x[j];
      W++;
    }

    // compute activation function and save output of layer
    l->y[i] = (float)sigmoid(sum);
  }
}

void WeightedGradient(layer_t *l, float *Wg) {
  int i, j;

  for (i = 0; i < l->width; i++) {
    Wg[i] = 0;
    for (j = 0; j < l->depth; j++)
      Wg[i] += (*(l->W + j * l->width + i)) * (l->g[j]);
  }
}

void PrintLayerOutput(layer_t *l) {
  int i;
  char *main_buffer = (char *)malloc(sizeof(char) * (10 * l->depth + 1));
  main_buffer[0] = '\0';
  char buffer[10];
  printf("%f\n", l->y[0]);
  for (i = 0; i < l->depth; i++) {
    sprintf(buffer, "%4.3f ", l->y[i]);
    strcat(main_buffer, buffer);
  }
  strcat(main_buffer, "\n");
  printf("%s", main_buffer);
  free(main_buffer);
}

void SaveLayerWeights(layer_t *l, FILE *fp) {
  if (fwrite(l->W, (sizeof(float) * (l->depth) * (l->width)), 1, fp) != 1)
    printf("error writing to file\n");
}

void SaveLayerWeightsHDT(layer_t *l, FILE *fp) {
  int i, j;
  float *W;

  W = l->W;
  fprintf(fp, "{\n");
  for (i = 0; i < l->depth; i++) {
    fprintf(fp, "{");
    for (j = 0; j < l->width; j++) {
      fprintf(fp, "%f", *W);
      if (j < l->width - 1)
        fprintf(fp, ", ");
      W++;
    }

    if (i < l->depth - 1)
      fprintf(fp, "},\n");
    else
      fprintf(fp, "}\n");
  }
  fprintf(fp, "}");
}

void LoadLayerWeights(layer_t *l, FILE *fp) {
  if (fread(l->W, (sizeof(float) * (l->depth) * (l->width)), 1, fp) != 1)
    printf("error reading from file\n");
}

/************************************
 * Network Manipulation functions
 ************************************/

void InputToNetwork(network_t *n, float *values) {
  InputToLayer(&n->layers[0], values);
}

void OutputFromNetwork(network_t *n, float *values) {
  int i;
  layer_t *pl;

  pl = &n->layers[n->size - 1];
  for (i = 0; i < pl->depth; i++)
    values[i] = pl->y[i];
}

void ActivateNetwork(network_t *n) {
  int i;

  for (i = 0; i < n->size - 1; i++) {
    ActivateLayer(&n->layers[i]);
    InputToLayer(&n->layers[i + 1], n->layers[i].y);
  }
  ActivateLayer(&n->layers[n->size - 1]);
}

void RandomizeNetwork(network_t *n) {
  int i;

  for (i = 0; i < n->size; i++)
    RandomizeLayer(&n->layers[i]);
}

float TrainNetwork(network_t *n, float *yd) {
  int i, j, k;
  float error;
  float *W;

  layer_t *pl;

  // update the output layer
  error = 0;
  pl = &n->layers[n->size - 1];
  W = pl->W;
  for (i = 0; i < pl->depth; i++) {
    pl->g[i] = pl->y[i] * (1 - pl->y[i])  // local gradient
               * (yd[i] - pl->y[i]);      // output error

    error += (yd[i] - pl->y[i]) * (yd[i] - pl->y[i]);  // squared error

    // update the layer weights
    for (j = 0; j < pl->width; j++) {
      (*W) += (n->lr) *     // learning rate
              (pl->g[i]) *  // gradient
              (pl->x[j]);   // signal
      W++;
    }
  }
  error = sqrt(error) / (pl->depth);  // compute average error

  // compute error and update weights
  for (k = n->size - 2; k >= 0; k--)  // for each layer in the network
  {
    pl = &n->layers[k];
    layer_t *pl_next = &n->layers[k + 1];

    // calculate weighted gradient of next layer
    WeightedGradient(pl_next, pl->g);

    // calculate the error
    W = pl->W;
    for (i = 0; i < pl->depth; i++) {
      pl->g[i] *= pl->y[i] * (1 - pl->y[i]);  // local gradient

      for (j = 0; j < pl->width; j++) {
        (*W) += (n->lr) *     // learning rate
                (pl->g[i]) *  // gradient
                (pl->x[j]);   // signal
        W++;
      }
    }
  }

  return error;
}

void PrintNetwork(network_t *n) {
  int i;

  for (i = 0; i < n->size; i++)
    PrintLayerOutput(&n->layers[i]);
}

void PrintNetworkOutput(network_t *n) {
  PrintLayerOutput(&n->layers[n->size - 1]);
}

void SaveNetworkWeights(network_t *n, const char *filename) {
  FILE *fp;
  int i;

  if ((fp = fopen(filename, "w+b")) == NULL)
    printf("Cannot open %s\n", filename);

  for (i = 0; i < n->size; i++)
    SaveLayerWeights(&n->layers[i], fp);

  fclose(fp);
}

void LoadNetworkWeights(network_t *n, const char *filename) {
  FILE *fp;
  int i;

  if ((fp = fopen(filename, "rb")) == NULL)
    printf("Cannot open %s\n", filename);

  for (i = 0; i < n->size; i++)
    LoadLayerWeights(&n->layers[i], fp);

  fclose(fp);
}
