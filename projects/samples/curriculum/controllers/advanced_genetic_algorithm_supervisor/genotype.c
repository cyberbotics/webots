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

#include "genotype.h"
#include <assert.h>
#include <math.h>
#include <stdlib.h>
#include "random.h"

static const double MUTATION_PROBABLITY = 0.06;
static const double MUTATION_DEVIATION = 0.2;

struct _Genotype_ {
  double *genes;   // genome
  double fitness;  // fitness
};

static int genotype_size = -1;

int genotype_get_size() {
  return genotype_size;
}

void genotype_set_size(int size) {
  genotype_size = size;
}

Genotype genotype_create() {
  assert(genotype_size > 0);
  Genotype gen = malloc(sizeof(struct _Genotype_));
  gen->fitness = 0.0;
  gen->genes = malloc(genotype_size * sizeof(double));

  // initialize with random uniform numbers in the range [0,1]
  int i;
  for (i = 0; i < genotype_size; i++)
    gen->genes[i] = random_get_uniform();

  return gen;
}

void genotype_destroy(Genotype g) {
  free(g->genes);
  free(g);
}

Genotype genotype_clone(Genotype g) {
  Genotype clone = genotype_create();

  int i;
  for (i = 0; i < genotype_size; i++)
    clone->genes[i] = g->genes[i];

  clone->fitness = g->fitness;
  return clone;
}

// mutate genes with given probability and deviation
void genotype_mutate(Genotype g) {
  int i;
  for (i = 0; i < genotype_size; i++)
    if (random_get_uniform() < MUTATION_PROBABLITY)
      g->genes[i] += random_get_gaussian() * MUTATION_DEVIATION;
}

// crossover
Genotype genotype_crossover(Genotype parent1, Genotype parent2) {
  Genotype child = genotype_create();

  // choose random locus
  int locus1 = random_get_integer(genotype_size);
  int locus2 = random_get_integer(genotype_size);

  // if locus1 is after locus2, swap them
  if (locus1 > locus2) {
    int tmp = locus1;
    locus1 = locus2;
    locus2 = tmp;
  }

  int i;
  for (i = 0; i < genotype_size; i++)
    if (i > locus1 && i < locus2)
      child->genes[i] = parent1->genes[i];
    else
      child->genes[i] = parent2->genes[i];

  return child;
}

void genotype_set_fitness(Genotype g, double fitness) {
  g->fitness = fitness;
}

double genotype_get_fitness(Genotype g) {
  return g->fitness;
}

const double *genotype_get_genes(Genotype g) {
  return g->genes;
}

void genotype_fread(Genotype g, FILE *fd) {
  int i;
  for (i = 0; i < genotype_size; i++) {
    int ret = fscanf(fd, "%lf", &g->genes[i]);
    if (ret == EOF)
      fprintf(stderr, "Cannot decode the genotype file\n");
  }
}

void genotype_fwrite(Genotype g, FILE *fd) {
  int i;
  for (i = 0; i < genotype_size; i++)
    fprintf(fd, " %lf", g->genes[i]);

  fprintf(fd, "\n");
}
