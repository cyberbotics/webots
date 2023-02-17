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

#ifndef WB_RANDOM_HPP
#define WB_RANDOM_HPP

//
// Description: random number generator with uniform and gaussian distributions
//

namespace WbRandom {
  // reset the random seed
  void setSeed(unsigned int s);
  unsigned int getSeed();
  // use George Marsaglia's MWC algorithm to produce an unsigned integer
  unsigned int nextUInt();
  // returns a random number between 0..1 from a uniform distribution
  double nextUniform();
  // return random number from a normal (Gaussian) distribution with
  // mean 0.0 and standard deviation 1.0
  double nextGaussian();
};  // namespace WbRandom

#endif
