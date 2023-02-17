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

#ifndef WB_PICO_TEXT_TO_SPEECH_HPP
#define WB_PICO_TEXT_TO_SPEECH_HPP

// Interface to Pico text-to-speech engine

#include "WbTextToSpeech.hpp"

class WbPicoTextToSpeech : public WbTextToSpeech {
public:
  WbPicoTextToSpeech();
  ~WbPicoTextToSpeech();
  qint16 *generateBufferFromText(const QString &text, int *size, const QString &language) override;
  const QString error() override;
};

#endif
