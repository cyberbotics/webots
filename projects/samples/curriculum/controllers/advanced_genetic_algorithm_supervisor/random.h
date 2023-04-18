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

#ifndef RANDOM_H
#define RANDOM_H

//   Description:   Random number functions

// returns a random integer number between [0;max-1] from a uniform distribution
int random_get_integer(int max);

// returns a random number between [0;1] from a uniform distribution
double random_get_uniform();

// returns a random number from a Gaussian distribution with mean 0 and standard deviation 1
double random_get_gaussian();

#endif
