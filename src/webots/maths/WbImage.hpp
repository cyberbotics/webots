#ifndef WB_IMAGE_HPP
#define WB_IMAGE_HPP

class WbImage {
public:
  WbImage(const unsigned char *data, int width, int height, int channels) :
    mData(data),
    mWidth(width),
    mHeight(height),
    mChannels(channels)
  {}

  const unsigned char *data() const { return mData; }
  int width() const { return mWidth; }
  int height() const { return mHeight; }
  int channels() const { return mChannels; }

  WbImage *downscale(int width, int height);

private:
  const unsigned char *mData;
  int mWidth;
  int mHeight;
  int mChannels;
};

#endif
