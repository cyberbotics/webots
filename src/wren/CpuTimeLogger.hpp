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

#ifndef CPU_TIME_LOGGER_HPP
#define CPU_TIME_LOGGER_HPP

#include <ctime>
#include <string>
#include <vector>

namespace wren {

  class CpuTimeLogger {
  public:
    CpuTimeLogger(int samplesPerMeasurement, int measurementsPerOutput, const std::string &name, const std::string &filename,
                  int maxPartialSamples = 0);

    void startSample();
    void startPartialSample();
    void endPartialSample();
    void endSample();

  private:
    struct Sample {
      explicit Sample(clock_t start) : mStart(start), mEnd(start) {}

      clock_t duration() const { return mEnd - mStart; }

      clock_t mStart;
      clock_t mEnd;
    };

    struct Measurement {
      Measurement(double mean, double variance) : mMean(mean), mVariance(variance) {}

      double mMean;
      double mVariance;
    };

    void computeMeasurement();
    void writeToFile();

    int mMaxPartialSamples;
    int mSamplesPerMeasurement;
    int mMeasurementsPerOutput;
    int mPartialSamplesTaken;
    int mSamplesTaken;
    int mMeasurementsTaken;

    bool mSamplingStarted;

    std::vector<Sample> mSamples;
    std::vector<Sample> mPartialSamples;
    std::vector<double> mSampleTimes;
    std::vector<Measurement> mMeasurements;

    std::string mName;
    std::string mFilename;
  };

  // Example usage
  // extern CpuTimeLogger cSceneRenderTimeLogger;
  // extern CpuTimeLogger cSceneTreeUpdateTimeLogger;
  // extern CpuTimeLogger cBoundingSphereComputationTimeLogger;
  // extern CpuTimeLogger cCullingTimeLogger;
  // extern CpuTimeLogger cParitioningTimeLogger;
  // extern CpuTimeLogger cStateSortingTimeLogger;
  // extern CpuTimeLogger cDistanceSortingTimeLogger;
  // extern CpuTimeLogger cOpaqueRenderingTimeLogger;
  // extern CpuTimeLogger cTranslucentRenderingTimeLogger;

}  // namespace wren

#endif  // CPU_TIME_LOGGER_HPP
