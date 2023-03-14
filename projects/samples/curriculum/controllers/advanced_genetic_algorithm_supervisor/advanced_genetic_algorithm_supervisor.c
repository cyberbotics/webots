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

//   Description:   Supervisor code for genetic algorithm

#include <math.h>
#include <stdio.h>
#include <string.h>
#include <webots/display.h>
#include <webots/emitter.h>
#include <webots/keyboard.h>
#include <webots/robot.h>
#include <webots/supervisor.h>
#include "genotype.h"
#include "population.h"

static const int POPULATION_SIZE = 50;
static const int NUM_GENERATIONS = 25;
static const char *FILE_NAME = "fittest.txt";

// must match the values in the advanced_genetic_algorithm.c code
static const int NUM_SENSORS = 8;
static const int NUM_WHEELS = 2;
#define GENOTYPE_SIZE (NUM_SENSORS * NUM_WHEELS)

// index access
enum { X, Y, Z };

static int time_step;
static WbDeviceTag emitter;  // to send genes to robot
static WbDeviceTag display;  // to display the fitness evolution
static int display_width, display_height;

// the GA population
static Population population;

// for reading or setting the robot's position and orientation
static WbFieldRef robot_translation;
static WbFieldRef robot_rotation;
static double robot_trans0[3];  // a translation needs 3 doubles
static double robot_rot0[4];    // a rotation needs 4 doubles

// for reading or setting the load's position
static WbFieldRef load_translation;
static double load_trans0[3];

// start with a demo until the user presses the 'O' key
// (change this if you want)
static bool demo = true;

void draw_scaled_line(int generation, double y1, double y2) {
  const double XSCALE = (double)display_width / NUM_GENERATIONS;
  const double YSCALE = 10.0;
  wb_display_draw_line(display, (generation - 0.5) * XSCALE, display_height - y1 * YSCALE, (generation + 0.5) * XSCALE,
                       display_height - y2 * YSCALE);
}

// plot best and average fitness
void plot_fitness(int generation, double best_fitness, double average_fitness) {
  static double prev_best_fitness = 0.0;
  static double prev_average_fitness = 0.0;
  if (generation > 0) {
    wb_display_set_color(display, 0xff0000);  // red
    draw_scaled_line(generation, prev_best_fitness, best_fitness);

    wb_display_set_color(display, 0x00ff00);  // green
    draw_scaled_line(generation, prev_average_fitness, average_fitness);
  }

  prev_best_fitness = best_fitness;
  prev_average_fitness = average_fitness;
}

// run the robot simulation for the specified number of seconds
void run_seconds(double seconds) {
  int i, n = 1000.0 * seconds / time_step;
  for (i = 0; i < n; i++) {
    if (demo && wb_keyboard_get_key() == 'O') {
      demo = false;
      return;  // interrupt demo and start GA optimization
    }

    wb_robot_step(time_step);
  }
}

// compute fitness as the euclidian distance that the load was pushed
double measure_fitness() {
  const double *load_trans = wb_supervisor_field_get_sf_vec3f(load_translation);
  double dx = load_trans[X] - load_trans0[X];
  double dz = load_trans[Z] - load_trans0[Z];
  return sqrt(dx * dx + dz * dz);
}

// evaluate one genotype at a time
void evaluate_genotype(Genotype genotype) {
  // send genotype to robot for evaluation
  wb_emitter_send(emitter, genotype_get_genes(genotype), GENOTYPE_SIZE * sizeof(double));

  // reset robot and load position
  wb_supervisor_field_set_sf_vec3f(robot_translation, robot_trans0);
  wb_supervisor_field_set_sf_rotation(robot_rotation, robot_rot0);
  wb_supervisor_field_set_sf_vec3f(load_translation, load_trans0);

  // evaluation genotype during one minute
  run_seconds(60.0);

  // measure fitness
  double fitness = measure_fitness();
  genotype_set_fitness(genotype, fitness);

  printf("fitness: %g\n", fitness);
  WbNodeRef robot = wb_supervisor_node_get_from_def("ROBOT");
  wb_supervisor_node_reset_physics(robot);
}

void run_optimization() {
  wb_keyboard_disable();

  printf("---\n");
  printf("starting GA optimization ...\n");
  printf("population size is %d, genome size is %d\n", POPULATION_SIZE, GENOTYPE_SIZE);

  int i, j;
  for (i = 0; i < NUM_GENERATIONS; i++) {
    for (j = 0; j < POPULATION_SIZE; j++) {
      printf("generation: %d, genotype: %d\n", i, j);

      // evaluate genotype
      Genotype genotype = population_get_genotype(population, j);
      evaluate_genotype(genotype);
    }

    double best_fitness = genotype_get_fitness(population_get_fittest(population));
    double average_fitness = population_compute_average_fitness(population);

    // display results
    plot_fitness(i, best_fitness, average_fitness);
    printf("best fitness: %g\n", best_fitness);
    printf("average fitness: %g\n", average_fitness);

    // reproduce (but not after the last generation)
    if (i < NUM_GENERATIONS - 1)
      population_reproduce(population);
  }

  printf("GA optimization terminated.\n");

  // save fittest individual
  Genotype fittest = population_get_fittest(population);
  FILE *outfile = fopen(FILE_NAME, "w");
  if (outfile) {
    genotype_fwrite(fittest, outfile);
    fclose(outfile);
    printf("wrote best genotype into %s\n", FILE_NAME);
  } else
    printf("unable to write %s\n", FILE_NAME);

  population_destroy(population);
}

// show demo of the fittest individual
void run_demo() {
  wb_keyboard_enable(time_step);

  printf("---\n");
  printf("running demo of best individual ...\n");
  printf("select the 3D window and push the 'O' key\n");
  printf("to start genetic algorithm optimization\n");

  FILE *infile = fopen(FILE_NAME, "r");
  if (!infile) {
    printf("unable to read %s\n", FILE_NAME);
    return;
  }

  Genotype genotype = genotype_create();
  genotype_fread(genotype, infile);
  fclose(infile);

  while (demo)
    evaluate_genotype(genotype);
}

int main(int argc, const char *argv[]) {
  // initialize Webots
  wb_robot_init();

  // get simulation step in milliseconds
  time_step = wb_robot_get_basic_time_step();

  // the emitter to send genotype to robot
  emitter = wb_robot_get_device("emitter");

  // to display the fitness evolution
  display = wb_robot_get_device("display");
  display_width = wb_display_get_width(display);
  display_height = wb_display_get_height(display);
  wb_display_draw_text(display, "fitness", 2, 2);

  // initial population
  population = population_create(POPULATION_SIZE, GENOTYPE_SIZE);

  // find robot node and store initial position and orientation
  WbNodeRef robot = wb_supervisor_node_get_from_def("ROBOT");
  robot_translation = wb_supervisor_node_get_field(robot, "translation");
  robot_rotation = wb_supervisor_node_get_field(robot, "rotation");
  memcpy(robot_trans0, wb_supervisor_field_get_sf_vec3f(robot_translation), sizeof(robot_trans0));
  memcpy(robot_rot0, wb_supervisor_field_get_sf_rotation(robot_rotation), sizeof(robot_rot0));

  // find load node and store initial position
  WbNodeRef load = wb_supervisor_node_get_from_def("LOAD");
  load_translation = wb_supervisor_node_get_field(load, "translation");
  memcpy(load_trans0, wb_supervisor_field_get_sf_vec3f(load_translation), sizeof(load_trans0));

  if (demo)
    run_demo();

  // run GA optimization
  run_optimization();

  // cleanup Webots
  wb_robot_cleanup();
  return 0;  // ignored
}
