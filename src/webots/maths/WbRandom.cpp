// Copyright 1996-2023 Cyberbotics Ltd.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     https://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "WbRandom.hpp"

#include <cmath>

static unsigned int gW = 0;
static unsigned int gZ = 362436069;

double WbRandom::nextGaussian() {
  // Use Marsaglia algorithm, this algorithm is faster than Box-Muller
  // This method uses 2 independent uniform variables in [-1;1]
  double v1;
  double w;

  do {
    v1 = 2 * nextUniform() - 1;
    double v2 = 2 * nextUniform() - 1;
    w = v1 * v1 + v2 * v2;
  } while (w > 1);  // Condition on w in order that the method works (condition not respected ~21.5% of the time)

  double r = sqrt(-2.0 * log(w) / w);
  return r * v1;
}

void WbRandom::setSeed(unsigned int s) {
  gW = s;
  gZ = 362436069;  // default value
}

unsigned int WbRandom::getSeed() {
  return gW;
}

unsigned int WbRandom::nextUInt() {
  gZ = 36969 * (gZ & 65535) + (gZ >> 16);
  gW = 18000 * (gW & 65535) + (gW >> 16);
  return (gZ << 16) + gW;
}

double WbRandom::nextUniform() {
  unsigned int u = nextUInt();
  return (u + 1) * 2.328306435454494e-10;
}
