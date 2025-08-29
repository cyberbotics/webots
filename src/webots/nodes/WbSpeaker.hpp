// Copyright 1996-2024 Cyberbotics Ltd.
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

#ifndef WB_SPEAKER_HPP
#define WB_SPEAKER_HPP

#include "WbSolidDevice.hpp"

#include <QtCore/QMap>

class WbSoundSource;
class QDataStream;

class WbSpeaker : public WbSolidDevice {
  Q_OBJECT

public:
  // constructors and destructor
  explicit WbSpeaker(WbTokenizer *tokenizer = NULL);
  WbSpeaker(const WbSpeaker &other);
  explicit WbSpeaker(const WbNode &other);
  virtual ~WbSpeaker() override;

  // reimplemented public functions
  int nodeType() const override { return WB_NODE_SPEAKER; }
  void postFinalize() override;
  void handleMessage(QDataStream &stream) override;
  void writeAnswer(WbDataStream &) override;
  void postPhysicsStep() override;

private:
  // private data types

  // Data from wb_speaker_play_sound() API
  class SoundPlayData {
  public:
    explicit SoundPlayData(QDataStream &);

    const QString &file() const { return mFile; }
    double volume() const { return mVolume; }
    double pitch() const { return mPitch; }
    double balance() const { return mBalance; }
    bool loop() const { return mLoop; }
    short side() const { return mSide; }
    int rawLength() const { return mRawLength; }
    QByteArray &rawData() { return mRawData; }

  private:
    QString mFile;
    double mVolume;
    double mPitch;
    double mBalance;
    bool mLoop;
    short mSide;
    int mRawLength;
    QByteArray mRawData;
  };

  // private functions
  WbSpeaker &operator=(const WbSpeaker &);  // non copyable
  WbNode *clone() const override { return new WbSpeaker(*this); }
  void init();

  void playSound(SoundPlayData &playData);
  void playText(const char *text, double volume);
  void stop(const char *sound);
  void stopAll();

  void updateSoundSource(WbSoundSource *source);

  QMap<QString, WbSoundSource *> mSoundSourcesMap;
  QMap<QString, const WbSoundSource *> mPlayingSoundSourcesMap;
  QMap<QString, QByteArray *> mStreamedSoundDataMap;

  QString mControllerDir;
  QString mEngine;
  QString mLanguage;
};

#endif
