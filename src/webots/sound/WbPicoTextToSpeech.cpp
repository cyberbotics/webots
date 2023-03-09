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

#include "WbPicoTextToSpeech.hpp"
#include "WbStandardPaths.hpp"

#include <QtCore/QCoreApplication>
#include <QtCore/QObject>
#include <QtCore/QVector>
#include <QtXml/QDomDocument>

#include <picoapi.h>
#include <picoapid.h>
#include <picoos.h>
#include <cassert>

// adaptation layer defines
#define PICO_MEM_SIZE 25000000

// supported voices, Pico does not seperately specify the voice and locale
static const char *gLanguages[] = {"en-US", "en-UK", "de-DE", "es-ES", "fr-FR", "it-IT"};
static const char *gPicoTaLingwares[] = {"en-US_ta.bin", "en-GB_ta.bin", "de-DE_ta.bin",
                                         "es-ES_ta.bin", "fr-FR_ta.bin", "it-IT_ta.bin"};
static const char *gPicoSgLingwares[] = {"en-US_lh0_sg.bin", "en-GB_kh0_sg.bin", "de-DE_gl0_sg.bin",
                                         "es-ES_zl0_sg.bin", "fr-FR_nk0_sg.bin", "it-IT_cm0_sg.bin"};

static const int N_LANGUAGES = sizeof(gLanguages) / sizeof(char *);

// adapation layer global variables
static char *gPicoMemArea = NULL;
static pico_System gPicoSystem = NULL;
static pico_Engine gPicoEngine = NULL;
static pico_Resource gPicoTaResources[] = {NULL, NULL, NULL, NULL, NULL, NULL};
static pico_Resource gPicoSgResources[] = {NULL, NULL, NULL, NULL, NULL, NULL};
static pico_Char *gPicoTaFileNames[] = {NULL, NULL, NULL, NULL, NULL, NULL};
static pico_Char *gPicoSgFileNames[] = {NULL, NULL, NULL, NULL, NULL, NULL};
static pico_Char *gPicoTaResourceNames[] = {NULL, NULL, NULL, NULL, NULL, NULL};
static pico_Char *gPicoSgResourceNames[] = {NULL, NULL, NULL, NULL, NULL, NULL};

static bool gInitializationSuccess = false;
static int gCurrentLanguage = 0;
static QString gError = "";

static void terminate() {
  if (gPicoSystem) {
    pico_terminate(&gPicoSystem);
    gPicoSystem = NULL;
  }
  delete[] gPicoMemArea;
  gPicoMemArea = NULL;
  gInitializationSuccess = false;
}

static void releaseTaResource() {
  if (gPicoSystem) {
    for (int i = 0; i < N_LANGUAGES; ++i) {
      if (gPicoTaResources[i]) {
        pico_unloadResource(gPicoSystem, &gPicoTaResources[i]);
        gPicoTaResources[i] = NULL;
      }
      if (gPicoTaFileNames[i]) {
        delete[] gPicoTaFileNames[i];
        gPicoTaFileNames[i] = NULL;
      }
      if (gPicoTaResourceNames[i]) {
        delete[] gPicoTaResourceNames[i];
        gPicoTaResourceNames[i] = NULL;
      }
    }
  }
}

static void releaseSgResource() {
  if (gPicoSystem) {
    for (int i = 0; i < N_LANGUAGES; ++i) {
      if (gPicoSgResources[i]) {
        pico_unloadResource(gPicoSystem, &gPicoSgResources[i]);
        gPicoSgResources[i] = NULL;
      }
      if (gPicoSgFileNames[i]) {
        delete[] gPicoSgFileNames[i];
        gPicoSgFileNames[i] = NULL;
      }
      if (gPicoSgResourceNames[i]) {
        delete[] gPicoSgResourceNames[i];
        gPicoSgResourceNames[i] = NULL;
      }
    }
  }
}

static void releaseVoices() {
  if (gPicoSystem) {
    for (int i = 0; i < N_LANGUAGES; ++i)
      pico_releaseVoiceDefinition(gPicoSystem, reinterpret_cast<pico_Char *>(const_cast<char *>(gLanguages[i])));
  }
}

static void releaseEngine() {
  if (gPicoSystem) {
    if (gPicoEngine) {
      pico_disposeEngine(gPicoSystem, &gPicoEngine);
      gPicoEngine = NULL;
    }
  }
}

static void cleanup() {
  releaseEngine();
  releaseVoices();
  releaseSgResource();
  releaseTaResource();
  terminate();
}

static void createEngine(int languageIndex) {
  assert(languageIndex < N_LANGUAGES);
  int ret = 0;
  pico_Retstring outMessage;
  if ((ret = pico_newEngine(gPicoSystem, reinterpret_cast<const pico_Char *>(gLanguages[languageIndex]), &gPicoEngine))) {
    pico_getSystemStatusMessage(gPicoSystem, ret, outMessage);
    gError = QString("Cannot create a new pico engine (%1): %2").arg(ret).arg(outMessage);
    releaseEngine();
    return;
  }
}

static void init() {
  assert(!gInitializationSuccess);  // only one instance of this object can exist

  pico_Retstring outMessage;
  int ret = 0;

  gPicoMemArea = new char[PICO_MEM_SIZE];
  if ((ret = pico_initialize(static_cast<void *>(gPicoMemArea), PICO_MEM_SIZE, &gPicoSystem))) {
    pico_getSystemStatusMessage(gPicoSystem, ret, outMessage);
    gError = QString("Cannot initialize pico (%1): %2").arg(ret).arg(outMessage);
    terminate();
    return;
  }

  // Load the text analysis Lingware resource file.
  for (int i = 0; i < N_LANGUAGES; ++i) {
    gPicoTaFileNames[i] = new pico_Char[PICO_MAX_DATAPATH_NAME_SIZE + PICO_MAX_FILE_NAME_SIZE];
    strcpy(reinterpret_cast<char *>(gPicoTaFileNames[i]), WbStandardPaths::resourcesPicoPath().toStdString().c_str());
    strcat(reinterpret_cast<char *>(gPicoTaFileNames[i]), static_cast<const char *>(gPicoTaLingwares[i]));
    if ((ret = pico_loadResource(gPicoSystem, gPicoTaFileNames[i], &gPicoTaResources[i]))) {
      pico_getSystemStatusMessage(gPicoSystem, ret, outMessage);
      gError = QString("Cannot load text analysis resource file (%1): %2").arg(ret).arg(outMessage);
      cleanup();
      return;
    }

    // Load the signal generation Lingware resource file.
    gPicoSgFileNames[i] = new pico_Char[PICO_MAX_DATAPATH_NAME_SIZE + PICO_MAX_FILE_NAME_SIZE];
    strcpy(reinterpret_cast<char *>(gPicoSgFileNames[i]), WbStandardPaths::resourcesPicoPath().toStdString().c_str());
    strcat(reinterpret_cast<char *>(gPicoSgFileNames[i]), static_cast<const char *>(gPicoSgLingwares[i]));
    if ((ret = pico_loadResource(gPicoSystem, gPicoSgFileNames[i], &gPicoSgResources[i]))) {
      pico_getSystemStatusMessage(gPicoSystem, ret, outMessage);
      gError = QString("Cannot load signal generation Lingware resource file (%1): %2").arg(ret).arg(outMessage);
      cleanup();
      return;
    }

    // Get the text analysis resource name.
    gPicoTaResourceNames[i] = new pico_Char[PICO_MAX_RESOURCE_NAME_SIZE];
    if ((ret = pico_getResourceName(gPicoSystem, gPicoTaResources[i], reinterpret_cast<char *>(gPicoTaResourceNames[i])))) {
      pico_getSystemStatusMessage(gPicoSystem, ret, outMessage);
      gError = QString("Cannot get the text analysis resource name (%1): %2").arg(ret).arg(outMessage);
      cleanup();
      return;
    }

    // Get the signal generation resource name.
    gPicoSgResourceNames[i] = new pico_Char[PICO_MAX_RESOURCE_NAME_SIZE];
    if ((ret = pico_getResourceName(gPicoSystem, gPicoSgResources[i], reinterpret_cast<char *>(gPicoSgResourceNames[i])))) {
      pico_getSystemStatusMessage(gPicoSystem, ret, outMessage);
      gError = QString("Cannot get the signal generation resource name (%1): %2").arg(ret).arg(outMessage);
      cleanup();
      return;
    }

    // Create a voice definition.
    if ((ret = pico_createVoiceDefinition(gPicoSystem, reinterpret_cast<const pico_Char *>(gLanguages[i])))) {
      pico_getSystemStatusMessage(gPicoSystem, ret, outMessage);
      gError = QString("Cannot create voice definition (%1): %2").arg(ret).arg(outMessage);
      cleanup();
      return;
    }

    // Add the text analysis resource to the voice.
    if ((ret = pico_addResourceToVoiceDefinition(gPicoSystem, reinterpret_cast<const pico_Char *>(gLanguages[i]),
                                                 gPicoTaResourceNames[i]))) {
      pico_getSystemStatusMessage(gPicoSystem, ret, outMessage);
      gError = QString("Cannot add the text analysis resource to the voice (%1): %2").arg(ret).arg(outMessage);
      cleanup();
      return;
    }

    // Add the signal generation resource to the voice.
    if ((ret = pico_addResourceToVoiceDefinition(gPicoSystem, reinterpret_cast<const pico_Char *>(gLanguages[i]),
                                                 gPicoSgResourceNames[i]))) {
      pico_getSystemStatusMessage(gPicoSystem, ret, outMessage);
      gError = QString("Cannot add the signal generation resource to the voice (%1): %2").arg(ret).arg(outMessage);
      cleanup();
      return;
    }
  }

  // Create a new Pico engine.
  // Warning: only one engine is allowed at the same time, we therefore need to release it
  //          and create a new one when changing language.
  createEngine(gCurrentLanguage);

  gInitializationSuccess = true;
  gError.clear();
}

static void updateEngineIfRequired(const char *language) {
  int languageIndex = -1;
  for (int i = 0; i < N_LANGUAGES; ++i) {
    if (strcmp(language, gLanguages[i]) == 0) {
      languageIndex = i;
      break;
    }
  }
  if (languageIndex < 0) {
    languageIndex = 0;
    assert(false);
  }

  if (languageIndex != gCurrentLanguage) {
    releaseEngine();
    createEngine(languageIndex);
    gCurrentLanguage = languageIndex;
  }
}

static QString toSsml(const QDomElement e) {
  QString result;
  QDomNode n = e.firstChild();
  while (!n.isNull()) {
    if (n.isElement()) {
      QDomElement el = n.toElement();
      QString name = el.tagName().toLower();
      if (name == "prosody") {
        QString pitchString = el.attribute("pitch", "0st");
        pitchString.chop(2);
        double pitch = pitchString.toDouble();
        double rate = el.attribute("rate", "1").toDouble();
        double volume = el.attribute("volume", "-1").toDouble();
        if (pitch != 0) {
          if (pitch > 0)
            pitch *= 100.0 / 12.0;
          else
            pitch = -1200 / pitch;
          result += "<pitch level=\"" + QString::number(pitch) + "\">";
        }
        if (rate != 1)
          result += "<speed level=\"" + QString::number(rate * 100) + "\">";
        if (volume >= 0)
          result += "<volume level=\"" + QString::number(volume) + "\">";
        result += toSsml(el);
        if (volume >= 0)
          result += "</volume>";
        if (rate != 1)
          result += "</speed>";
        if (pitch != 0)
          result += "</pitch>";
      } else if (name == "audio")
        result += "<play file=\"" + el.attribute("src", "") + "\">" + toSsml(el) + "</play>";
    } else if (n.isText())
      result += n.toText().nodeValue();
    n = n.nextSibling();
  }
  return result;
}

static QString toSsml(const QString &text) {
  QString result;
  QDomDocument doc;
  doc.setContent("<?xml version=\"1.0\" encoding=\"UTF-8\"?><speak>" + text + "</speak>");
  QDomNode n = doc.firstChild();
  while (!n.isNull()) {
    if (n.isElement()) {
      QDomElement e = n.toElement();
      QString name = e.tagName().toLower();
      if (name == "speak")
        result += toSsml(e);
    }
    n = n.nextSibling();
  }
  return result;
}

WbPicoTextToSpeech::WbPicoTextToSpeech() {
  init();
}

WbPicoTextToSpeech::~WbPicoTextToSpeech() {
  cleanup();
}

qint16 *WbPicoTextToSpeech::generateBufferFromText(const QString &text, int *size, const QString &language) {
  if (!gInitializationSuccess)
    return NULL;
  updateEngineIfRequired(language.toStdString().c_str());
  // Flush engine-internal buffers (e.g. reset voice effects, but preserve language).
  pico_resetEngine(gPicoEngine, PICO_RESET_SOFT);

  const int PICO_BLOCK_SIZE = 1024;
  int bufferSize = 0;
  int bufferIndex = 0;
  qint16 *buffer = NULL;

  int status;
  QString t = toSsml(text);
  const QByteArray textUtf8 = t.toUtf8();  // seems vital: may be related to deep copy (see #4471)
  const pico_Char *remainingTextToSent = reinterpret_cast<const pico_Char *>((textUtf8.constData()));
  pico_Int16 bytesSent, bytesReceived, outDataType, remainingText = textUtf8.size() + 1;
  pico_Retstring outMessage;

  /* synthesis loop   */
  while (remainingText) {
    /* Feed the text into the engine.   */
    int ret;
    if ((ret = pico_putTextUtf8(gPicoEngine, remainingTextToSent, remainingText, &bytesSent))) {
      pico_getSystemStatusMessage(gPicoSystem, ret, outMessage);
      gError = QString("Cannot put text (%1): %2").arg(ret).arg(outMessage);
      releaseEngine();
      return NULL;
    }

    remainingText -= bytesSent;
    remainingTextToSent += bytesSent;

    do {
      /* Retrieve the samples and add them to the buffer. */
      while (bufferIndex + PICO_BLOCK_SIZE > bufferSize) {
        bufferSize += 1024 * 1024;  // 1 MB
        qint16 *previousBuffer = buffer;
        buffer = static_cast<qint16 *>(realloc(buffer, bufferSize));
        if (!buffer) {  // re-allocation failed, this is required otherwise CppCheck raises an error
          free(previousBuffer);
          gError = QString("Cannot re-allocate buffer.");
          return NULL;
        }
      }
      char *bufferPointer = (reinterpret_cast<char *>(buffer)) + bufferIndex;
      status = pico_getData(gPicoEngine, bufferPointer, PICO_BLOCK_SIZE, &bytesReceived, &outDataType);
      if (status != PICO_STEP_BUSY && status != PICO_STEP_IDLE) {
        free(buffer);
        pico_getSystemStatusMessage(gPicoSystem, status, outMessage);
        gError = QString("Cannot get data (%1): %2").arg(ret).arg(outMessage);
        releaseEngine();
        return NULL;
      }
      bufferIndex += bytesReceived;
    } while (PICO_STEP_BUSY == status);
  }
  gError.clear();
  *size = bufferIndex / sizeof(qint16);
  buffer = static_cast<qint16 *>(realloc(buffer, bufferIndex));  // reduce the size of the buffer to the minimum
  return buffer;
}

const QString WbPicoTextToSpeech::error() {
  return gError;
}
