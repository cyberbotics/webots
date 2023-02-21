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

#include <webots/Robot.hpp>
#include <webots/Speaker.hpp>

#include <LinuxDARwIn.h>
#include <string.h>

using namespace webots;
using namespace Robot;

#define N_SUPPORTED_LANGUAGES 53
static const char *supported_languages[N_SUPPORTED_LANGUAGES] = {
  "af", "bs", "ca", "cs", "cy", "da", "de", "el", "en",    "eo", "fi", "fr", "grc", "hi", "hr", "hu", "hy", "id", "is", "it",
  "ku", "la", "lv", "mk", "nl", "no", "pl", "pt", "pt-pt", "ro", "ru", "sk", "sq",  "sr", "sv", "sw", "ta", "tr", "zh"};

Speaker::Speaker(const std::string &name) : Device(name) {
  mSpeakPID = -1;
  mLanguage = "en";
}

Speaker::~Speaker() {
}

void Speaker::setLanguage(const std::string &language) {
  bool valid_language = false;
  const char *askedLanguage = language.c_str();
  for (int i = 0; i < N_SUPPORTED_LANGUAGES; ++i) {
    if (strcmp(askedLanguage, supported_languages[i]) == 0) {
      valid_language = true;
      break;
    }
  }
  if (!valid_language) {
    if (language == "en-US")
      mLanguage = "en";
    else if (language == "en-UK")
      mLanguage = "en";
    else if (language == "de-DE")
      mLanguage = "de";
    else if (language == "es-ES")
      mLanguage = "es";
    else if (language == "fr-FR")
      mLanguage = "fr";
    else if (language == "it-IT")
      mLanguage = "it";
  } else
    mLanguage = language;
}

void Speaker::playFile(const char *filename) {
  LinuxActionScript::PlayMP3(filename);
}

void Speaker::playFileWait(const char *filename) {
  LinuxActionScript::PlayMP3Wait(filename);
}

void Speaker::playSound(Speaker *left, Speaker *right, const std::string &sound, double volume, double pitch, double balance,
                        bool loop) {
  LinuxActionScript::PlayMP3(sound.c_str());
}

void Speaker::stop(const std::string &sound) {
  if (sound == "") {
    if (mSpeakPID != -1)
      kill(mSpeakPID, SIGKILL);
    mSpeakPID = -1;
  }
}

void Speaker::speak(const std::string &text, double volume) {
  if (mSpeakPID != -1)
    kill(mSpeakPID, SIGKILL);

  mSpeakPID = fork();

  switch (mSpeakPID) {
    case -1:
      fprintf(stderr, "Speaker: Fork failed\n");
      break;
    case 0: {
      fprintf(stderr, "Speaker: Saying \"%s\" ...\n", text.c_str());
      char *buffer = static_cast<char *>(malloc(strlen(text.c_str()) + 3));
      sprintf(buffer, "\"%s\"", text.c_str());
      execl("/usr/bin/espeak", "espeak", buffer, "-v", mLanguage.c_str(), (char *)NULL);
      free(buffer);
      fprintf(stderr, "Speaker: Exec failed\n");
    } break;
    default:
      break;
  }
}

void Speaker::speak(const char *text, const char *voice, int speed) {
  char speedBuffer[20];
  sprintf(speedBuffer, "%d", speed);

  if (mSpeakPID != -1)
    kill(mSpeakPID, SIGKILL);

  mSpeakPID = fork();

  switch (mSpeakPID) {
    case -1:
      fprintf(stderr, "Speaker: Fork failed\n");
      break;
    case 0: {
      fprintf(stderr, "Speaker: Saying \"%s\" ...\n", text);
      char *buffer = static_cast<char *>(malloc(strlen(text) + 3));
      sprintf(buffer, "\"%s\"", text);
      execl("/usr/bin/espeak", "espeak", buffer, "-v", voice, "-s", speedBuffer, (char *)NULL);
      free(buffer);
      fprintf(stderr, "Speaker: Exec failed\n");
    } break;
    default:
      break;
  }
}

void Speaker::speakFile(const char *filename, const char *voice, int speed) {
  char speedBuffer[20];
  sprintf(speedBuffer, "%d", speed);

  if (mSpeakPID != -1)
    kill(mSpeakPID, SIGKILL);

  mSpeakPID = fork();

  switch (mSpeakPID) {
    case -1:
      fprintf(stderr, "Speaker: Fork failed\n");
      break;
    case 0:
      fprintf(stderr, "Speaker: Saying text from file \"%s\" ...\n", filename);
      execl("/usr/bin/espeak", "espeak", "-f", filename, "-v", voice, "-s", speedBuffer, (char *)NULL);
      fprintf(stderr, "Speaker: Exec failed\n");
      break;
    default:
      break;
  }
}
