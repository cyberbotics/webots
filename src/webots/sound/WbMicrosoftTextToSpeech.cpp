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

#include "WbMicrosoftTextToSpeech.hpp"
#include "WbStandardPaths.hpp"

#include <QtCore/QCoreApplication>
#include <QtCore/QObject>
#include <QtCore/QVector>

#include <windows.h>

#include <sapi.h>
#include <cassert>

static QString gError = "";
static ISpVoice *gVoice = NULL;
static ISpStream *gStream = NULL;
static IStream *gBaseStream = NULL;
static WAVEFORMATEX *gWavFormatEx = NULL;

WbMicrosoftTextToSpeech::WbMicrosoftTextToSpeech() {
  assert(gStream == NULL);  // only one instance of this class may exist
  assert(gVoice == NULL);
  assert(gBaseStream == NULL);
  assert(gWavFormatEx == NULL);

  if (FAILED(::CoInitialize(NULL))) {
    gError = "Failed to initialize Microsoft COM.";
    return;
  }
  if (FAILED(CoCreateInstance(CLSID_SpStream, NULL, CLSCTX_ALL, __uuidof(ISpStream), (void **)&gStream))) {
    gError = "Failed to create COM instance for Stream.";
    return;
  }
  if (FAILED(CoCreateInstance(CLSID_SpVoice, NULL, CLSCTX_ALL, IID_ISpVoice, (void **)&gVoice))) {
    gError = "Failed to create COM instance for Voice.";
    return;
  }
  CreateStreamOnHGlobal(NULL, TRUE, &gBaseStream);
  if (!gBaseStream) {
    gError = "Failed to create memory stream.";
    return;
  }
  gWavFormatEx = (WAVEFORMATEX *)::CoTaskMemAlloc(sizeof(WAVEFORMATEX));
  assert(gWavFormatEx);
  gWavFormatEx->wFormatTag = WAVE_FORMAT_PCM;
  gWavFormatEx->nChannels = generatedChannelNumber();
  gWavFormatEx->nSamplesPerSec = generatedRate();
  gWavFormatEx->wBitsPerSample = generatedBitsPerSample();
  gWavFormatEx->nBlockAlign = (gWavFormatEx->nChannels * gWavFormatEx->wBitsPerSample) / 8;
  gWavFormatEx->nAvgBytesPerSec = gWavFormatEx->nSamplesPerSec * gWavFormatEx->nBlockAlign;
  gWavFormatEx->cbSize = 0;

  // Windows constant (not available in mingw, hence hardcoded here)
  const GUID SPDFID_WaveFormatEx = {0xC31ADBAE, 0x527F, 0x4FF5, {0xA2, 0x30, 0xF6, 0x2B, 0xB6, 0x1F, 0xF7, 0x0C}};
  if (FAILED(gStream->SetBaseStream(gBaseStream, SPDFID_WaveFormatEx, gWavFormatEx))) {
    gError = "Failed to set base stream.";
    return;
  }
  if (FAILED(gVoice->SetOutput(gStream, TRUE))) {
    gError = "Failed to set output stream for voice.";
    return;
  }
}

WbMicrosoftTextToSpeech::~WbMicrosoftTextToSpeech() {
  if (gVoice)
    gVoice->Release();
  gVoice = NULL;
  if (gStream)
    gStream->Release();
  gStream = NULL;
  if (gBaseStream)
    gBaseStream->Release();
  gBaseStream = NULL;
  if (gWavFormatEx)
    ::CoTaskMemFree(gWavFormatEx);
  gWavFormatEx = NULL;
  ::CoUninitialize();
}

qint16 *WbMicrosoftTextToSpeech::generateBufferFromText(const QString &text, int *size, const QString &language) {
  QString t = "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n"
              "<speak version=\"1.0\" xmlns=\"http://www.w3.org/2001/10/synthesis\" xml:lang=\"" +
              language + "\">" + text + "</speak>";
  HRESULT hr = gVoice->Speak(t.toStdWString().c_str(), SPF_IS_XML | SPF_ASYNC, NULL);
  if (!SUCCEEDED(hr)) {
    gError = "Failed to generate buffer from text.";
    return NULL;
  }
  gVoice->WaitUntilDone(-1);
  STATSTG stats;
  gBaseStream->Stat(&stats, STATFLAG_NONAME);
  ULONG sSize = stats.cbSize.QuadPart;
  ULONG bytesRead = 0;
  LARGE_INTEGER zero;
  zero.QuadPart = 0;
  char *pBuffer = static_cast<char *>(malloc(sSize));
  gBaseStream->Seek(zero, STREAM_SEEK_SET, NULL);
  gBaseStream->Read(pBuffer, sSize, &bytesRead);
  gBaseStream->Seek(zero, STREAM_SEEK_SET, NULL);
  ULARGE_INTEGER uZero;
  uZero.QuadPart = 0;
  gBaseStream->SetSize(uZero);
  gError.clear();
  *size = bytesRead / sizeof(qint16);
  return (qint16 *)pBuffer;
}

const QString WbMicrosoftTextToSpeech::error() {
  return gError;
}
