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

#ifndef WB_TEXT_TO_SPEECH_HPP
#define WB_TEXT_TO_SPEECH_HPP

// Interface to text-to-speech engine (abscract class)

#include <QtCore/QString>

class WbTextToSpeech {
public:
  virtual ~WbTextToSpeech() {}
  virtual qint16 *generateBufferFromText(const QString &text, int *size, const QString &language) = 0;
  virtual inline int generatedChannelNumber() { return 1; }
  virtual inline int generatedBitsPerSample() { return 16; }
  virtual inline int generatedRate() { return 16000; }
  virtual const QString error() { return ""; }
};

#endif
