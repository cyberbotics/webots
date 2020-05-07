#include "WbMultimediaStreamingLimiter.hpp"
#include "WbLog.hpp"
#include <QtCore/QString>
#include <QtCore/QObject>



WbMultimediaStreamingLimiter::WbMultimediaStreamingLimiter(const QSize &resolution, int frameRate) :
    mResolution(resolution),
    mResolutionFactor(1),
    mResolutionChanged(false),
    mFrameRate(frameRate),
    mLevel(-1),
    mIncreasingSteps(1),
    mDecreasingSteps(1),
    mIsStopped(false)
  {};

void WbMultimediaStreamingLimiter::resetResolution(const QSize &newSize) {
  mResolution = newSize;
  for (int i = mResolutionFactor; i > 1; i--)
    mResolution /= 2;
}

QSize WbMultimediaStreamingLimiter::fullResolution() const {
  QSize resolution(mResolution);
  for (int i = mResolutionFactor; i > 1; i--)
    resolution *= 2;
  return resolution;
}

void WbMultimediaStreamingLimiter::recomputeStreamingLimits(int skippedImages) {
  mIsStopped = false;
  mResolutionChanged = false;
  if (mIsStopped)
    return;

  if (skippedImages > 0)
    mIncreasingSteps++;
  else if (mIncreasingSteps > 0)
    mIncreasingSteps--;

  if (mIncreasingSteps == 0) {
    if (mLevel >= 0) {
      mLevel--;
      if (mLevel < 6 && mLevel % 2 == 0 && mResolutionFactor > 1) {
        mResolution *= 2;
        mResolutionFactor--;
        mResolutionChanged = true;
      } else if (mFrameRate > 50)
        mFrameRate -= 50;
    }
  } else {
    if (skippedImages > 3)
      mIsStopped = true;
    while (skippedImages > 0) {
      if (mResolutionFactor < 4 && mLevel % 2 == 0) {
        mLevel++;
        mResolution /= 2;
        mResolutionFactor++;
        mResolutionChanged = true;
      } else if (mFrameRate < 500) {
        mLevel++;
        mFrameRate += 50;
      } else
        break;
      skippedImages -= 3;
    }
  }

  WbLog::info(QString("Streaming limiter level: %2, resolution: %1, rate %3").arg(mResolution.width()).arg(mLevel).arg(mFrameRate));
}
