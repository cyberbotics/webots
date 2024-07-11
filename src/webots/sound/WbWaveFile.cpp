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

#include "WbWaveFile.hpp"

#include "WbStandardPaths.hpp"

#include <QtCore/QDir>
#include <QtCore/QFileInfo>
#include <QtCore/QObject>
#include <QtCore/QProcess>

#include <cassert>

namespace {
  struct Chunk {
    char id[4];
    quint32 size;
  };

  struct RIFFChunkData {
    char type[4];
  };

  struct FormatChunkData {
    quint16 audioFormat;
    quint16 numChannels;
    quint32 sampleRate;
    quint32 byteRate;
    quint16 blockAlign;
    quint16 bitsPerSample;
  };
}  // namespace

WbWaveFile::WbWaveFile(const QString &filename, QIODevice *device) :
  mFilename(filename),
  mDevice(device),
  mBuffer(NULL),
  mBufferSize(0),
  mNChannels(0),
  mBitsPerSample(0),
  mRate(0),
  mOwnBuffer(false) {
}

WbWaveFile::WbWaveFile(qint16 *buffer, int bufferSize, int channelNumber, int bitsPerSample, int rate) :
  mDevice(NULL),
  mBuffer(buffer),
  mBufferSize(bufferSize),
  mNChannels(channelNumber),
  mBitsPerSample(bitsPerSample),
  mRate(rate),
  mOwnBuffer(true) {
}

WbWaveFile::~WbWaveFile() {
  if (mOwnBuffer)
    free(mBuffer);
}

void WbWaveFile::loadConvertedFile(int side) {
  if (mBuffer != NULL)
    free(mBuffer);

  mBuffer = NULL;
  mBufferSize = 0;
  mNChannels = 0;
  mBitsPerSample = 0;
  mRate = 0;

  try {
    bool riffChunkRead = false;
    bool formatChunkRead = false;
    bool dataChunkRead = false;

    while (1) {  // read chunks one by one
      Chunk chunk;
      qint64 readSize = mDevice->read((char *)&chunk, sizeof(Chunk));
      if (readSize <= 0)
        break;  // end of file
      if (readSize != sizeof(Chunk))
        throw QObject::tr("Cannot read chunk");

      if (strncmp(chunk.id, "RIFF", 4) == 0) {
        RIFFChunkData riffChunk;
        readSize = mDevice->read((char *)&riffChunk, sizeof(RIFFChunkData));
        if (readSize != sizeof(RIFFChunkData))
          throw QObject::tr("Cannot read RIFF chunk");

        if (strncmp(riffChunk.type, "WAVE", 4) != 0)
          throw QObject::tr("Wrong WAVE type for RIFF chunk");

        riffChunkRead = true;

      } else if (strncmp(chunk.id, "fmt ", 4) == 0) {
        FormatChunkData formatChunk;
        readSize = mDevice->read((char *)&formatChunk, sizeof(FormatChunkData));
        if (readSize != sizeof(FormatChunkData))
          throw QObject::tr("Cannot read format chunk");

        mNChannels = formatChunk.numChannels;
        mBitsPerSample = formatChunk.bitsPerSample;
        mRate = formatChunk.sampleRate;

        formatChunkRead = true;
      } else if (strncmp(chunk.id, "data", 4) == 0) {
        if (chunk.size % 2)
          mBufferSize = chunk.size / sizeof(qint16) + 1;
        else
          mBufferSize = chunk.size / sizeof(qint16);
        mBuffer = static_cast<qint16 *>(malloc(sizeof(qint16) * mBufferSize));
        if (!mBuffer)
          throw QObject::tr("Cannot allocate data memory");

        readSize = mDevice->read(reinterpret_cast<char *>(mBuffer), chunk.size);
        if (readSize != chunk.size)
          throw QObject::tr("Cannot read data chunk");

        dataChunkRead = true;
      } else {  // trash this chunk
        char *trash = new char[chunk.size];
        mDevice->read(trash, chunk.size);
        delete[] trash;
      }
      if (riffChunkRead && formatChunkRead && dataChunkRead)
        break;  // not necessary to go further
    }

    if (riffChunkRead == false)
      throw("Corrupted WAVE file: RIFF chunk not found");

    if (formatChunkRead == false)
      throw("Corrupted WAVE file: format chunk not found");

    if (dataChunkRead == false)
      throw("Corrupted WAVE file: data chunk not found");

  } catch (const QString &e) {
    // clean up
    if (mBuffer != NULL)
      free(mBuffer);
    // throw up
    throw;
  }

  mOwnBuffer = true;
  // if needed, remove one of the two channels
  if (mNChannels == 2 && side != 0) {
    int newSize = mBufferSize / 2;
    qint16 *newBuffer = static_cast<qint16 *>(malloc(sizeof(qint16) * newSize));
    if (mBitsPerSample == 8) {
      qint8 *currentBuffer = reinterpret_cast<qint8 *>(mBuffer);
      qint8 *outBuffer = reinterpret_cast<qint8 *>(newBuffer);
      for (int i = 0; i < newSize; i += 1) {
        if (side == 1)
          outBuffer[i] = currentBuffer[i * 2];
        else
          outBuffer[i] = currentBuffer[i * 2 + 1];
      }
    } else if (mBitsPerSample == 16) {
      for (int i = 0; i < newSize; i += 1) {
        if (side == 1)
          newBuffer[i] = mBuffer[i * 2];
        else
          newBuffer[i] = mBuffer[i * 2 + 1];
      }
    } else
      assert(false);
    free(mBuffer);
    mBuffer = newBuffer;
    mNChannels = 1;
    mBufferSize /= 2;
  }
}

void WbWaveFile::loadConvertedFile(int side, const QString &filename) {
  assert(mDevice == NULL);
  mDevice = new QFile(filename);
  mDevice->open(QIODevice::ReadOnly);
  loadConvertedFile(side);
  mDevice->close();
  delete mDevice;
  mDevice = NULL;
}

void WbWaveFile::loadFromFile(const QString &extension, int side) {
  if (extension == "wav") {
    if (mDevice)
      loadConvertedFile(side);
    else
      loadConvertedFile(side, mFilename);
    return;
  }

#ifdef __linux__
  static QString ffmpeg("avconv");
  static QString percentageChar = "%";
#elif defined(__APPLE__)
  static QString ffmpeg(QString("%1Contents/util/ffmpeg").arg(WbStandardPaths::webotsHomePath()));
  static QString percentageChar = "%";
#else  // _WIN32
  static QString ffmpeg = "ffmpeg.exe";
  static QString percentageChar = "%%";
#endif

  const QString outputFilename = WbStandardPaths::webotsTmpPath() + "output.wav";
  QString inputFilename;
  if (mDevice) {
    inputFilename = WbStandardPaths::webotsTmpPath() + "input." + extension;
    QFile input(inputFilename);
    input.open(QFile::WriteOnly);
    input.write(mDevice->readAll());
    input.close();
  } else
    inputFilename = mFilename;
  const QStringList arguments = QStringList() << "-y"
                                              << "-i" << inputFilename << outputFilename << "-sample_fmt"
                                              << "s16"
                                              << "-loglevel"
                                              << "quiet";

  QProcess *conversionProcess = new QProcess();
  conversionProcess->execute(ffmpeg, arguments);
  conversionProcess->waitForFinished(-1);
  delete conversionProcess;

  if (mDevice)
    QFile::remove(inputFilename);

  loadConvertedFile(side, outputFilename);
  QFile::remove(outputFilename);
}

void WbWaveFile::convertToMono(double balance) {
  if (mNChannels < 2)
    return;

  if (mNChannels > 2)
    assert(false);

  if (balance > 1)
    balance = 1;

  if (balance < -1)
    balance = -1;

  if (mBitsPerSample == 8) {
    // warning: 8 bit wav file are unsigned
    quint8 *currentBuffer = reinterpret_cast<quint8 *>(mBuffer);
    for (unsigned int i = 0; i < mBufferSize * sizeof(qint16); i += 2) {
      quint8 left = currentBuffer[i];
      quint8 right = currentBuffer[i + 1];
      quint8 monoSample = ((1.0 - balance) / 2.0) * left + ((1.0 + balance) / 2.0) * right;
      currentBuffer[i / 2] = monoSample & 0xff;
    }
  } else if (mBitsPerSample == 16) {
    // warning: 16 bit wav file are signed
    for (unsigned int i = 0; i < mBufferSize; i += 2) {
      qint16 left = mBuffer[i];
      qint16 right = mBuffer[i + 1];
      qint16 monoSample = ((1.0 - balance) / 2.0) * left + ((1.0 + balance) / 2.0) * right;
      mBuffer[i / 2] = monoSample;
    }
  } else
    assert(false);

  mNChannels = 1;
  mBufferSize /= 2;
}
