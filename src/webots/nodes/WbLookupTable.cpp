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

#include "WbLookupTable.hpp"

#include "WbLog.hpp"
#include "WbMFVector3.hpp"
#include "WbRandom.hpp"
#include "WbVector3.hpp"

WbLookupTable::WbLookupTable(const WbMFVector3 &lookupTable) {
  mSize = lookupTable.size();
  if (mSize) {
    mInputs = new double[mSize];
    mOutputs = new double[mSize];
    mNoises = new double[mSize];

    int j = 0;
    WbMFVector3::Iterator it(lookupTable);
    while (it.hasNext()) {
      const WbVector3 &v = it.next();
      mInputs[j] = v.x();
      if (j > 0 && mInputs[j - 1] >= mInputs[j])
        WbLog::warning(QObject::tr("The input values of a lookup table must be sorted in increasing order."), false,
                       WbLog::PARSING);
      mOutputs[j] = v.y();
      mNoises[j] = v.z();
      j++;
    }

    // compute min and max outputs
    mMinOutput = mOutputs[0];
    mMaxOutput = mOutputs[0];
    for (int i = 1; i < mSize; i++) {
      if (mOutputs[i] < mMinOutput)
        mMinOutput = mOutputs[i];
      if (mOutputs[i] > mMaxOutput)
        mMaxOutput = mOutputs[i];
    }
  } else {
    mInputs = NULL;
    mOutputs = NULL;
    mNoises = NULL;
    mMinOutput = 0.0;
    mMaxOutput = 0.0;
  }
}

WbLookupTable::~WbLookupTable() {
  delete[] mInputs;
  delete[] mOutputs;
  delete[] mNoises;
}

double WbLookupTable::lookup(double value) const {
  // empty lookupTable does not interpolate
  if (mSize == 0)
    return value;

  // compute activation and noise
  double v1;
  double noise;
  int last = mSize - 1;

  if (value <= mInputs[0]) {
    noise = mNoises[0];
    v1 = mOutputs[0];
  } else if (value >= mInputs[last]) {
    noise = mNoises[last];
    v1 = mOutputs[last];
  } else {
    // look for the closest values to v1 in the mOutputs lookupTable
    // using a dichotomic search
    v1 = value;
    int maxi = last;
    int mini = 0;
    int i = maxi / 2;
    while (mini + 1 < maxi) {
      if (v1 < mInputs[i])
        maxi = i;
      else
        mini = i;
      i = (mini + maxi) / 2;
    }

    // interpolate linearly v1 and noise
    double v2 = (v1 - mInputs[mini]) / (mInputs[maxi] - mInputs[mini]);
    noise = (1 - v2) * mNoises[mini] + v2 * mNoises[maxi];
    v1 = (1 - v2) * mOutputs[mini] + v2 * mOutputs[maxi];
  }

  // add noise if necessary
  if (noise != 0.0)
    return v1 * (1.0 + noise * WbRandom::nextGaussian());
  else
    return v1;
}
