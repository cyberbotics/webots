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

#ifndef WB_LOOKUP_TABLE_HH
#define WB_LOOKUP_TABLE_HH

class WbMFVector3;

class WbLookupTable {
public:
  explicit WbLookupTable(const WbMFVector3 &lookupTable);
  ~WbLookupTable();

  double lookup(double value) const;
  bool isEmpty() const { return mSize == 0; }
  double minMetricsRange() const { return mSize ? mInputs[0] : 0.0; }
  double maxMetricsRange() const { return mSize ? mInputs[mSize - 1] : 0.0; }
  double minValue() const { return mMinOutput; }
  double maxValue() const { return mMaxOutput; }

private:
  int mSize;
  double *mInputs;
  double *mOutputs;
  double *mNoises;
  double mMinOutput;
  double mMaxOutput;

  WbLookupTable(const WbLookupTable &);             // non constructor-copyable
  WbLookupTable &operator=(const WbLookupTable &);  // non copyable
};

#endif
