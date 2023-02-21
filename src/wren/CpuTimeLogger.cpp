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

#include "CpuTimeLogger.hpp"

#include "Constants.hpp"
#include "Debug.hpp"

#include <fstream>

namespace wren {

  // Example usage
  // CpuTimeLogger cSceneRenderTimeLogger(10, 1, "Scene render", "wren-scene-render.log");
  // CpuTimeLogger cSceneTreeUpdateTimeLogger(100, 10, "Scene tree update", "wren-scene-tree-update.log");
  // CpuTimeLogger cBoundingSphereComputationTimeLogger(100, 10, "BoundingSphereComputation",
  // "wren-bounding-sphere-computation.log"); CpuTimeLogger cCullingTimeLogger(100, 10, "Culling", "wren-culling.log");
  // CpuTimeLogger cParitioningTimeLogger(100, 10, "Partitioning", "wren-partitioning.log");
  // CpuTimeLogger cStateSortingTimeLogger(100, 10, "Sorting (state)", "wren-state-sorting.log");
  // CpuTimeLogger cDistanceSortingTimeLogger(100, 10, "Sorting (distance)", "wren-distance-sorting.log");
  // CpuTimeLogger cOpaqueRenderingTimeLogger(100, 10, "Rendering (opaques)", "wren-opaque-rendering.log");
  // CpuTimeLogger cTranslucentRenderingTimeLogger(100, 10, "Rendering (translucents)", "wren-translucent-rendering.log");

  static const double MILLISECONDS_PER_CLOCK = 1.0 / (1e-3 * static_cast<double>(CLOCKS_PER_SEC));

  CpuTimeLogger::CpuTimeLogger(int samplesPerMeasurement, int measurementsPerOutput, const std::string &name,
                               const std::string &filename, int maxPartialSamples) :
    mMaxPartialSamples(maxPartialSamples),
    mSamplesPerMeasurement(samplesPerMeasurement),
    mMeasurementsPerOutput(measurementsPerOutput),
    mPartialSamplesTaken(0),
    mSamplesTaken(0),
    mMeasurementsTaken(0),
    mSamplingStarted(false),
    mName(name),
    mFilename(filename) {
    mSamples.reserve(mSamplesPerMeasurement);
    mPartialSamples.reserve(mMaxPartialSamples);
    mSampleTimes.reserve(mSamplesPerMeasurement);
    mMeasurements.reserve(mMeasurementsPerOutput);
  }

  void CpuTimeLogger::startSample() {
    // Only actually sample if partial sampling is disabled.
    // Else, sampling starts with the first call to startPartialSample
    if (mMaxPartialSamples)
      mSamplingStarted = true;
    else
      mSamples.push_back(Sample(clock()));
  }

  void CpuTimeLogger::startPartialSample() {
    assert(mSamplingStarted);
    assert(static_cast<int>(mPartialSamples.size()) < mMaxPartialSamples);

    mPartialSamples.push_back(Sample(clock()));
  }

  void CpuTimeLogger::endPartialSample() {
    assert(mSamplingStarted);
    assert(mPartialSamples.back().mEnd == mPartialSamples.back().mStart);

    mPartialSamples.back().mEnd = clock();
    ++mPartialSamplesTaken;
  }

  void CpuTimeLogger::endSample() {
    if (!mMaxPartialSamples)
      mSamples.back().mEnd = clock();
    else {
      mSamples.push_back(Sample(mPartialSamples.front().mStart));

      for (int i = 0; i < mPartialSamplesTaken; ++i)
        mSamples.back().mEnd += mPartialSamples[i].duration();

      mPartialSamples.clear();
      mPartialSamplesTaken = 0;
    }

    mSamplingStarted = false;
    ++mSamplesTaken;

    if (mSamplesTaken == mSamplesPerMeasurement)
      computeMeasurement();
  }

  void CpuTimeLogger::computeMeasurement() {
    // compute mean
    double mean = 0.0;
    for (int i = 0; i < mSamplesPerMeasurement; ++i) {
      mSampleTimes.push_back(static_cast<double>(mSamples[i].duration()) * MILLISECONDS_PER_CLOCK);
      mean += mSampleTimes[i];
    }
    mean /= static_cast<double>(mSamplesPerMeasurement);

    // compute variance
    double variance = 0.0;
    for (int i = 0; i < mSamplesPerMeasurement; ++i) {
      variance += mSampleTimes[i] * mSampleTimes[i];
    }
    variance = (variance / static_cast<double>(mSamplesPerMeasurement)) - (mean * mean);

    mMeasurements.push_back(Measurement(mean, variance));

    ++mMeasurementsTaken;

    mSamples.clear();
    mSampleTimes.clear();
    mSamplesTaken = 0;

    if (mMeasurementsTaken == mMeasurementsPerOutput) {
      writeToFile();

      mMeasurements.clear();
      mMeasurementsTaken = 0;
    }
  }

  void CpuTimeLogger::writeToFile() {
    std::ofstream ofs;

    ofs.open(mFilename.c_str(), std::ofstream::out | std::ofstream::app);
    ofs << mName << ": writing " << mMeasurementsPerOutput << " measurements of " << mSamplesPerMeasurement << " samples"
        << std::endl;
    for (int i = 0; i < mMeasurementsPerOutput; ++i)
      ofs << "Measurement " << i << ": avg=" << mMeasurements[i].mMean << "ms, var=" << mMeasurements[i].mVariance << std::endl;

    ofs.close();
  }

}  // namespace wren
