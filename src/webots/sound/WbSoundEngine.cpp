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

#include "WbSoundEngine.hpp"

#include "WbContactSoundManager.hpp"
#include "WbLog.hpp"
#include "WbPreferences.hpp"
#include "WbSFRotation.hpp"
#ifdef _WIN32
#include "WbMicrosoftTextToSpeech.hpp"
#endif
#include "WbMotorSoundManager.hpp"
#include "WbPicoTextToSpeech.hpp"
#include "WbSoundClip.hpp"
#include "WbSoundSource.hpp"
#include "WbStandardPaths.hpp"
#include "WbViewpoint.hpp"
#include "WbWaveFile.hpp"
#include "WbWorld.hpp"

#include <QtCore/QCoreApplication>
#include <QtCore/QFileInfo>
#include <QtCore/QObject>
#include <QtCore/QTemporaryFile>

#ifdef __APPLE__
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#include <OpenAL/al.h>
#include <OpenAL/alc.h>
#else
#include <AL/al.h>
#include <AL/alc.h>
#endif

#include <ode/ode.h>

#include <cassert>

static WbWorld *gWorld = NULL;
static bool gOpenAL = false;
static bool gMute = true;
static int gVolume = 80;
static QString gDevice;
static ALCdevice *gDefaultDevice = NULL;
static ALCcontext *gContext = NULL;
static QList<WbSoundClip *> gSounds;
static QList<WbSoundSource *> gSources;
static WbTextToSpeech *gTextToSpeech = NULL;

static void clearSounds() {
  qDeleteAll(gSounds);
  gSounds.clear();
}

static void clearSources() {
  qDeleteAll(gSources);
  gSources.clear();
}

static void cleanup() {
  clearSounds();
  clearSources();
  alcMakeContextCurrent(NULL);
  if (gContext)
    alcDestroyContext(gContext);
  if (gDefaultDevice)
    alcCloseDevice(gDefaultDevice);
  delete gTextToSpeech;
  gTextToSpeech = NULL;
}

static void init() {
  static bool initialized = false;
  if (initialized)  // init was already done
    return;
  initialized = true;
  gMute = WbPreferences::instance()->value("Sound/mute", true).toBool();
  gVolume = WbPreferences::instance()->value("Sound/volume", 80).toInt();
  WbLog::toggle(stderr);  // we want to disable stderr to avoid warnings in the console
  try {
    const ALCchar *defaultDeviceName = alcGetString(NULL, ALC_DEFAULT_DEVICE_SPECIFIER);
    if (defaultDeviceName == NULL)
      throw QObject::tr("Cannot find OpenAL default device");
    gDefaultDevice = alcOpenDevice(defaultDeviceName);
    if (gDefaultDevice == NULL)
      throw QObject::tr("Cannot initialize OpenAL default device '%1'").arg(defaultDeviceName);
    gContext = alcCreateContext(gDefaultDevice, NULL);
    if (gContext == NULL)
      throw QObject::tr("Cannot create OpenAL context");
    if (alcMakeContextCurrent(gContext) == ALC_FALSE)
      throw QObject::tr("Cannot make OpenAL current context");
    gDevice = QString(defaultDeviceName);
  } catch (const QString &e) {
    WbLog::toggle(stderr);
    WbLog::warning(QObject::tr("Cannot initialize the sound engine: %1").arg(e));
    return;
  }
  WbLog::toggle(stderr);
  gOpenAL = true;
  qAddPostRoutine(cleanup);
  WbSoundEngine::updateListener();
}

const QString &WbSoundEngine::device() {
  init();
  return (const QString &)gDevice;
}

bool WbSoundEngine::openAL() {
  init();
  return gOpenAL;
}

void WbSoundEngine::setWorld(WbWorld *world) {
  if (world) {
    gWorld = world;
    QObject::connect(gWorld, &WbWorld::viewpointChanged, &WbSoundEngine::updateViewpointConnection);

    updateListener();
  } else {
    gWorld = NULL;
    WbContactSoundManager::clearAllContactSoundSources();
    WbMotorSoundManager::clearAllMotorSoundSources();
    clearSources();
    clearSounds();
  }
}

void WbSoundEngine::updateViewpointConnection() {
  if (gWorld && gWorld->viewpoint())
    QObject::connect(gWorld->viewpoint(), &WbViewpoint::cameraParametersChanged, &WbSoundEngine::updateListener);
}

void WbSoundEngine::setMute(bool mute) {
  gMute = mute;
  updateListener();
}

void WbSoundEngine::setVolume(int volume) {
  gVolume = volume;
  updateListener();
}

void WbSoundEngine::setPause(bool pause) {
  if (gDefaultDevice) {
    foreach (WbSoundSource *source, gSources) {
      if (pause) {
        if (source->isPlaying())
          source->pause();
      } else {
        if (source->isPaused())
          source->play();
      }
    }
  }
}

void WbSoundEngine::updateListener() {
#ifdef __APPLE__  // macOS bug described at https://developer.apple.com/forums/thread/104309
  // It affects only Apple OpenAL, not OpenAL soft:
  // alListenerf(AL_GAIN, 0) doesn't work, it should be replaced with alListenerf(AL_GAIN, 0.0001f)
  alListenerf(AL_GAIN, (gMute || gVolume == 0) ? 0.0001f : 0.01f * gVolume);
#else
  alListenerf(AL_GAIN, gMute ? 0.0f : 0.01f * gVolume);
#endif
  if (gMute || gVolume == 0)
    return;

  // else update listener position and orientation
  ALfloat orientation[6] = {0.0f, 0.0f, 1.0f, 0.0f, 1.0f, 0.0f};
  ALfloat position[3] = {0.0f, 0.0f, 0.0f};

  if (gWorld && gWorld->viewpoint()) {
    const WbVector3 &translation = gWorld->viewpoint()->position()->value();
    const WbRotation &rotation = gWorld->viewpoint()->orientation()->value();
    WbVector3 rotationAt = rotation.direction();
    WbVector3 rotationUp = rotation.up();

    position[0] = translation.x();
    position[1] = translation.y();
    position[2] = translation.z();

    orientation[0] = rotationAt.x();
    orientation[1] = rotationAt.y();
    orientation[2] = rotationAt.z();
    orientation[3] = rotationUp.x();
    orientation[4] = rotationUp.y();
    orientation[5] = rotationUp.z();
  }

  // qDebug() << "OpenAL listener position:" << position[0] << position[1] << position[2];
  // qDebug() << "OpenAL listener orientation at:" << orientation[0] << orientation[1] << orientation[2];
  // qDebug() << "OpenAL listener orientation up:" << orientation[3] << orientation[4] << orientation[5];

  alListener3f(AL_POSITION, position[0], position[1], position[2]);
  alListenerfv(AL_ORIENTATION, orientation);

  // viewpoint velocity is not taken into account because not smooth
  // it may however be interesting to set it when following a robot.
  alListener3f(AL_VELOCITY, 0.0f, 0.0f, 0.0f);
}

void WbSoundEngine::updateAfterPhysicsStep() {
  if (gMute || gVolume == 0)
    return;
  assert(gWorld);

  WbContactSoundManager::update(gWorld->odeContacts());
  WbMotorSoundManager::update();
}

WbSoundClip *WbSoundEngine::sound(const QString &url, const QString &extension, QIODevice *device, double balance, int side) {
  if (url.isEmpty())
    return NULL;

  if (extension != "mp3" && extension != "wav") {
    WbLog::warning(QObject::tr("Invalid URL '%1'. Sounds must be in '.mp3' or '.wav' format.").arg(url));
    return NULL;
  }

  init();
  foreach (WbSoundClip *sound, gSounds) {
    if (sound->filename() == url && sound->side() == side && sound->balance() == balance)
      return sound;
  }
  WbSoundClip *sound = new WbSoundClip;
  try {
    sound->load(url, extension, device, balance, side);
    gSounds << sound;
    return sound;
  } catch (const QString &e) {
    WbLog::warning(QObject::tr("Could not open '%1' sound file: %2").arg(url).arg(e));
    delete sound;
    return NULL;
  }
}

WbSoundClip *WbSoundEngine::soundFromText(const QString &text, const QString &engine, const QString &language) {
  int size = 0;
#ifdef _WIN32
  if (gTextToSpeech) {
    if (engine == "pico" && dynamic_cast<WbPicoTextToSpeech *>(gTextToSpeech) == NULL) {
      delete gTextToSpeech;
      gTextToSpeech = NULL;
    } else if (engine == "microsoft" && dynamic_cast<WbMicrosoftTextToSpeech *>(gTextToSpeech) == NULL) {
      delete gTextToSpeech;
      gTextToSpeech = NULL;
    }
  }
#endif
  if (!gTextToSpeech) {
#ifdef _WIN32
    if (engine == "microsoft")
      gTextToSpeech = new WbMicrosoftTextToSpeech();
    else
#endif
      gTextToSpeech = new WbPicoTextToSpeech();
  }
  if (!gTextToSpeech->error().isEmpty()) {
    WbLog::warning(QObject::tr("TextToSpeech error: %1").arg(gTextToSpeech->error()));
    return NULL;
  }
  qint16 *buffer = gTextToSpeech->generateBufferFromText(text, &size, language);
  if (buffer == NULL) {
    WbLog::warning(QObject::tr("Could not open generated sound from '%1': %2").arg(text).arg(gTextToSpeech->error()));
    return NULL;
  }
  WbWaveFile wave(buffer, size, gTextToSpeech->generatedChannelNumber(), gTextToSpeech->generatedBitsPerSample(),
                  gTextToSpeech->generatedRate());
  WbSoundClip *sound = new WbSoundClip();
  try {
    sound->load(&wave);
    gSounds << sound;
    return sound;
  } catch (const QString &e) {
    WbLog::warning(QObject::tr("Could not open generated sound from '%1'.").arg(text));
    delete sound;
    return NULL;
  }
}

void WbSoundEngine::deleteSource(WbSoundSource *source) {
  if (gSources.contains(source)) {  // at reload/close the source might already have been deleted by cleanup()
    gSources.removeAll(source);
    delete source;
  }
}

void WbSoundEngine::stopAllSources() {
  foreach (WbSoundSource *source, gSources)
    source->stop();
}

WbSoundSource *WbSoundEngine::createSource() {
  init();
  WbSoundSource *source = new WbSoundSource;
  gSources << source;
  return source;
}

void WbSoundEngine::clearAllMotorSoundSources() {
  WbMotorSoundManager::clearAllMotorSoundSources();
}

void WbSoundEngine::clearAllContactSoundSources() {
  WbContactSoundManager::clearAllContactSoundSources();
}

#ifdef __APPLE__
#pragma GCC diagnostic pop
#endif
