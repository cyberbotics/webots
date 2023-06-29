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

#include "population.h"
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include "genotype.h"
#include "random.h"

// part of the population that is cloned from one generation to the next
static const double ELITE_PART = 0.1;

struct _Population_ {
  Genotype *genotypes;  // genotypes
  int size;             // population size
};

Population population_create(int pop_size, int gen_size) {
  Population p = malloc(sizeof(struct _Population_));
  p->size = pop_size;
  genotype_set_size(gen_size);
  p->genotypes = (Genotype *)malloc(p->size * sizeof(Genotype));
  int i;
  for (i = 0; i < p->size; i++)
    p->genotypes[i] = genotype_create();

  return p;
}

void population_destroy(Population p) {
  int i;
  for (i = 0; i < p->size; i++)
    genotype_destroy(p->genotypes[i]);

  free(p->genotypes);
}

// rank selection: the population need to be sorted by fitness
Genotype population_select_parent(Population p) {
  while (1) {
    int index = random_get_integer(p->size);
    if (index <= random_get_integer(p->size))
      return p->genotypes[index];
  }
}

// comparison function for qsort()
static int compare_genotype(const void *a, const void *b) {
  return genotype_get_fitness(*((Genotype *)a)) > genotype_get_fitness(*((Genotype *)b)) ? -1 : +1;
}

void population_reproduce(Population p) {
  // quick sort for rank selection
  qsort(p->genotypes, p->size, sizeof(Genotype), compare_genotype);

  // create new generation
  Genotype *next_generation = malloc(p->size * sizeof(Genotype));
  int i;
  for (i = 0; i < p->size; i++) {
    Genotype child;

    if (i < ELITE_PART * p->size) {
      // cloned elite
      child = genotype_clone(p->genotypes[i]);
    } else {
      // sexual reproduction
      // or asexual if both parents are the same individual
      Genotype mom = population_select_parent(p);
      Genotype dad = population_select_parent(p);
      child = genotype_crossover(mom, dad);
      genotype_mutate(child);
    }

    genotype_set_fitness(child, 0.0);
    next_generation[i] = child;
  }

  // destroy old generation
  for (i = 0; i < p->size; i++)
    genotype_destroy(p->genotypes[i]);

  free(p->genotypes);

  // switch generation pointers
  p->genotypes = next_generation;
}

Genotype population_get_fittest(Population p) {
  Genotype fittest = p->genotypes[0];
  int i;
  for (i = 1; i < p->size; i++) {
    Genotype candidate = p->genotypes[i];
    if (genotype_get_fitness(candidate) > genotype_get_fitness(fittest))
      fittest = candidate;
  }

  return fittest;
}

Genotype population_get_genotype(Population p, int index) {
  return p->genotypes[index];
}

double population_compute_average_fitness(Population p) {
  double sum_fitness = 0.0;
  int i;
  for (i = 0; i < p->size; i++)
    sum_fitness += genotype_get_fitness(p->genotypes[i]);

  return sum_fitness / p->size;
}
