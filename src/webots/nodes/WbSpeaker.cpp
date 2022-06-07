// Copyright 1996-2022 Cyberbotics Ltd.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "WbSpeaker.hpp"

#include "WbNodeUtilities.hpp"
#include "WbProtoManager.hpp"
#include "WbProtoModel.hpp"
#include "WbRobot.hpp"
#include "WbSimulationState.hpp"
#include "WbSoundClip.hpp"
#include "WbSoundEngine.hpp"
#include "WbSoundSource.hpp"

#include "../../controller/c/messages.h"

#include <QtCore/QDataStream>
#include <QtCore/QDir>

#include <cassert>

#define TEXT_TO_SPEECH_KEY "WB_TEXT_TO_SPEECH"

void WbSpeaker::init() {
  mControllerDir = "";
  mSoundSourcesMap.clear();
  mPlayingSoundSourcesMap.clear();
  mEngine = QString("pico");
  mLanguage = QString("en-US");
}

WbSpeaker::WbSpeaker(WbTokenizer *tokenizer) : WbSolidDevice("Speaker", tokenizer) {
  init();
}

WbSpeaker::WbSpeaker(const WbSpeaker &other) : WbSolidDevice(other) {
  init();
}

WbSpeaker::WbSpeaker(const WbNode &other) : WbSolidDevice(other) {
  init();
}

WbSpeaker::~WbSpeaker() {
  foreach (WbSoundSource *source, mSoundSourcesMap) {
    if (source)
      WbSoundEngine::deleteSource(source);
  }
  mSoundSourcesMap.clear();
  mPlayingSoundSourcesMap.clear();
}

void WbSpeaker::postFinalize() {
  WbSolidDevice::postFinalize();
  WbRobot *robot = const_cast<WbRobot *>(static_cast<const WbRobot *>(WbNodeUtilities::findTopNode(this)));
  if (robot)
    mControllerDir = robot->controllerDir();
}

void WbSpeaker::handleMessage(QDataStream &stream) {
  unsigned char command;

  stream >> command;
  switch (command) {
    case C_SPEAKER_PLAY_SOUND: {
      int numberOfSound = 0;
      stream >> numberOfSound;
      for (int i = 0; i < numberOfSound; ++i) {
        short size;
        short side;
        double volume;
        double pitch;
        double balance;
        unsigned char loop;
        stream >> size;
        char soundFile[size];
        stream.readRawData(soundFile, size);
        stream >> volume;
        stream >> pitch;
        stream >> balance;
        stream >> side;
        stream >> loop;
        playSound(soundFile, volume, pitch, balance, (bool)loop, (int)side);
      }
      return;
    }
    case C_SPEAKER_STOP: {
      short numberOfSound = 0;
      stream >> numberOfSound;
      // cppcheck-suppress knownConditionTrueFalse
      if (numberOfSound == 0)
        stopAll();
      else {
        for (int i = 0; i < numberOfSound; ++i) {
          short size;
          stream >> size;
          char sound[size];
          stream.readRawData(sound, size);
          stop(sound);
        }
      }
      return;
    }
    case C_SPEAKER_SET_ENGINE: {
      short size;
      stream >> size;
      char engine[size];
      stream.readRawData(engine, size);
      mEngine = QString(engine);
      return;
    }
    case C_SPEAKER_SET_LANGUAGE: {
      short size;
      stream >> size;
      char language[size];
      stream.readRawData(language, size);
      mLanguage = QString(language);
      return;
    }
    case C_SPEAKER_SPEAK: {
      short size;
      double volume;
      stream >> size;
      char text[size];
      stream.readRawData(text, size);
      stream >> volume;
      playText(text, volume);
      return;
    }
    default:
      assert(0);
  }
}

void WbSpeaker::writeAnswer(QDataStream &stream) {
  foreach (const WbSoundSource *source, mPlayingSoundSourcesMap) {
    if (!source->isPlaying()) {
      if (mPlayingSoundSourcesMap.key(source) == TEXT_TO_SPEECH_KEY) {
        stream << (short unsigned int)tag();
        stream << (unsigned char)C_SPEAKER_SPEAK_OVER;
      } else {
        stream << (short unsigned int)tag();
        stream << (unsigned char)C_SPEAKER_SOUND_OVER;
        const QByteArray name = mPlayingSoundSourcesMap.key(source).toUtf8();
        stream.writeRawData(name.constData(), name.size() + 1);
      }
      mPlayingSoundSourcesMap.remove(mPlayingSoundSourcesMap.key(source));
    }
  }
}

void WbSpeaker::postPhysicsStep() {
  WbSolidDevice::postPhysicsStep();
  foreach (WbSoundSource *source, mSoundSourcesMap) {
    if (source)
      updateSoundSource(source);
  }
}

void WbSpeaker::playText(const char *text, double volume) {
  if (!mSoundSourcesMap.contains(TEXT_TO_SPEECH_KEY)) {  // text-to-speech was never used
    WbSoundSource *source = WbSoundEngine::createSource();
    updateSoundSource(source);
    mSoundSourcesMap[TEXT_TO_SPEECH_KEY] = source;
  }
  WbSoundSource *source = mSoundSourcesMap.value(TEXT_TO_SPEECH_KEY, NULL);
  if (source) {
    source->stop();
    // cd to the controller directory (because some tags in the text
    // may refeer to file relatively to the controller)
    QDir initialDir = QDir::current();
    if (!QDir::setCurrent(mControllerDir))
      this->warn(tr("Cannot change directory to: '%1'").arg(mControllerDir));
    WbSoundClip *soundClip = WbSoundEngine::soundFromText(text, mEngine, mLanguage);
    QDir::setCurrent(initialDir.path());
    if (soundClip) {
      source->setSoundClip(soundClip);
      source->play();
      source->setGain(volume);
      mPlayingSoundSourcesMap[TEXT_TO_SPEECH_KEY] = source;
    }
  }
}

void WbSpeaker::playSound(const char *file, double volume, double pitch, double balance, bool loop, int side) {
  QString filename = QString(file);
  QString key = filename;
  if (side == -1)
    key += "_left";
  else if (side == 1)
    key += "_right";

  if (!mSoundSourcesMap.contains(key)) {  // this sound was never played
    QString path = "";
    if (QFile::exists(mControllerDir + filename))  // check if path is relative to the controller
      path = mControllerDir;
    else {  // check if path is relative to the PROTO (or any ancestor PROTO)
      WbRobot *robot = const_cast<WbRobot *>(static_cast<const WbRobot *>(WbNodeUtilities::findTopNode(this)));
      if (robot && robot->isProtoInstance()) {
        WbProtoModel *protoModel = robot->proto();
        do {
          if (!protoModel->path().isEmpty()) {
            path = protoModel->path();
            if (QFile::exists(path + filename))
              break;
          }
          protoModel = WbProtoManager::instance()->findModel(protoModel->ancestorProtoName(), "");
        } while (protoModel);
      }
    }

    if (!QFile::exists(path + filename)) {  // sound file not found relatively to the PROTOs or controller
      if (!QFile::exists(filename)) {       // check if path is absolute
        this->warn(tr("Sound file '%1' not found. The sound file should be defined relatively to the controller, the PROTO or "
                      "absolutely.\n")
                     .arg(filename));
        return;
      }
      path = "";
    }
    WbSoundSource *source = WbSoundEngine::createSource();
    updateSoundSource(source);
    // cd to the path directory if required
    QDir initialDir = QDir::current();
    if (!path.isEmpty()) {
      if (!QDir::setCurrent(path))
        this->warn(tr("Cannot change directory to: '%1'").arg(path));
    }
    const QString extension = filename.mid(filename.lastIndexOf('.') + 1).toLower();
    WbSoundClip *soundClip = WbSoundEngine::sound(filename, extension, NULL, balance, side);
    if (!path.isEmpty())
      QDir::setCurrent(initialDir.path());
    if (!soundClip) {
      this->warn(tr("Impossible to play '%1'. Make sure the file format is supported (8 or 16 bits, mono or stereo wave).\n")
                   .arg(filename));
      return;
    }
    source->setSoundClip(soundClip);
    source->play();
    mSoundSourcesMap[key] = source;
  }

  WbSoundSource *source = mSoundSourcesMap.value(key, NULL);
  if (source) {
    mPlayingSoundSourcesMap[key] = source;
    if (!source->isPlaying())  // this sound was already played but is over
      source->play();
    source->setLooping(loop);
    source->setGain(volume);
    source->setPitch(pitch);
    if (WbSimulationState::instance()->isPaused() || WbSimulationState::instance()->isStep())
      source->pause();
  }
}

void WbSpeaker::stop(const char *sound) {
  QString key = QString(sound);
  WbSoundSource *source = mSoundSourcesMap.value(key, NULL);
  if (!source) {
    key = QString(sound) + "_left";
    source = mSoundSourcesMap.value(key, NULL);
  }
  if (!source) {
    key = QString(sound) + "_right";
    source = mSoundSourcesMap.value(key, NULL);
  }
  if (source) {
    source->stop();
    WbSoundEngine::deleteSource(source);
    mSoundSourcesMap.remove(key);
  }
  mPlayingSoundSourcesMap.remove(key);
}

void WbSpeaker::stopAll() {
  foreach (WbSoundSource *source, mSoundSourcesMap) {
    if (source) {
      source->stop();
      WbSoundEngine::deleteSource(source);
    }
  }
  mSoundSourcesMap.clear();
  mPlayingSoundSourcesMap.clear();
}

void WbSpeaker::updateSoundSource(WbSoundSource *source) {
  source->setPosition(position());
  source->setVelocity(linearVelocity());
  source->setDirection(rotationMatrix() * WbVector3(0, 1, 0));
}
