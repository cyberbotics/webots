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
 * Implements a simple PSO evolution for an ANN
 */

#include "pso.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <webots/robot.h>

#define PSO_W 1.0   // parameter w in PSO
#define PSO_NW 2.0  // parameter nw in PSO
#define PSO_PW 2.0  // parameter pw in PSO

/***************************************************************
 * LOCAL FUNCTIONS
 **************************************************************/

double sigmoid(double x) {
  return 1.0 / (1.0 + exp(-x));  // the sigmoid function
}

float rand01() {
  return (((float)rand()) / RAND_MAX);
}

/************************************
 * Layer Manipulation functions
 ************************************/

void RandomizeLayer(layer_t *l) {
  int i;
  float *W;
  float *vW;

  W = l->W;
  vW = l->vW;

  for (i = 0; i < (l->depth) * (l->width); i++) {
    *W = (float)(2.0 * (((double)rand()) / RAND_MAX) - 1.0);
    *vW = (float)(2.0 * (((double)rand()) / RAND_MAX) - 1.0);
    W++;
    vW++;
  }
}

static void InputToLayer(const layer_t *l, const float *values) {
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

/*void WeightedGradient(layer_t* l, float* Wg)
{
  int i,j;

  for(i=0;i<l->width;i++)
  {
    Wg[i]=0;
    for(j=0;j<l->depth;j++)
    {
      Wg[i] += (*(l->W+j*l->width+i))*(l->g[j]);
    }
  }
}*/

void EvolveLayer(layer_t *l, layer_t *pbest, layer_t *gbest) {
  int i, j;
  float *lW = l->W;
  float *lvW = l->vW;
  float *gbW = gbest->W;
  float *pbW = pbest->W;

  for (i = 0; i < l->width; i++) {
    for (j = 0; j < l->depth; j++) {
      float v = PSO_W * (*lvW + PSO_PW * rand01() * (*pbW - *lW) + PSO_NW * rand01() * (*gbW - *lW));
      *lvW = v;
      float x = *lW + v;
      x = x > 100.0 ? 100.0 : x;
      x = x < -100.0 ? -100.0 : x;
      *lW = x;
    }
  }
}

void PrintLayerOutput(layer_t *l) {
  int i;
  char *main_buffer = (char *)malloc(sizeof(char) * (10 * l->depth + 1));
  strcpy(main_buffer, "");
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

void InputToNetwork(const network_t *n, const float *values) {
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

void EvolveNetwork(network_t *n, network_t *pbest, network_t *gbest) {
  int i;
  // printf("START EVOLVING\n");
  // printf("===============\n");
  for (i = 0; i < n->size; i++)
    EvolveLayer(&n->layers[i], &pbest->layers[i], &gbest->layers[i]);
  // printf("===============\n");
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

  if ((fp = fopen(filename, "w+b")) == NULL) {
    fprintf(stderr, "Cannot open %s\n", filename);
    return;
  }

  for (i = 0; i < n->size; i++)
    SaveLayerWeights(&n->layers[i], fp);

  fclose(fp);
}

void SaveNetworkWeightsHDT(network_t *n, const char *filename) {
  FILE *fp;
  int i;

  if ((fp = fopen(filename, "w+b")) == NULL) {
    fprintf(stderr, "Cannot open %s\n", filename);
    return;
  }

  for (i = 0; i < n->size; i++)
    SaveLayerWeightsHDT(&n->layers[i], fp);

  fclose(fp);
}

void LoadNetworkWeights(network_t *n, const char *filename) {
  FILE *fp;
  int i;

  if ((fp = fopen(filename, "rb")) == NULL) {
    fprintf(stderr, "Cannot open %s\n", filename);
    return;
  }
  for (i = 0; i < n->size; i++)
    LoadLayerWeights(&n->layers[i], fp);

  fclose(fp);
}
