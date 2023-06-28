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

#ifndef GENOTYPE_H
#define GENOTYPE_H

//   Description:   General-purpose genotype class with mutation and crossover operations

#include <stdio.h>

// abstract type definition
typedef struct _Genotype_ *Genotype;

// set/get global number of genes
void genotype_set_size(int size);
int genotype_get_size();

// create new genotypes
Genotype genotype_create();
Genotype genotype_clone(Genotype g);

// release memory associated with g
void genotype_destroy(Genotype g);

// mutation
void genotype_mutate(Genotype g);

// crossover: create a new genotype as a crossover between two parents
Genotype genotype_crossover(Genotype parent1, Genotype parent2);

// set/get fitness
void genotype_set_fitness(Genotype g, double fitness);
double genotype_get_fitness(Genotype g);

// for Emitter/Receiver transmission
const double *genotype_get_genes(Genotype g);

// read/write from stream
void genotype_fwrite(Genotype g, FILE *fd);
void genotype_fread(Genotype g, FILE *fd);

#endif
