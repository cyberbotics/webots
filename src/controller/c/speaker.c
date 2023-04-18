/*
 * Copyright 1996-2023 Cyberbotics Ltd.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     https://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <webots/nodes.h>
#include <webots/speaker.h>
#include "messages.h"
#include "robot_private.h"
#ifdef _WIN32
#include <windows.h>
#endif

static const char *pico_languages[] = {"en-US", "en-UK", "de-DE", "es-ES", "fr-FR", "it-IT"};

static const int N_PICO_LANGUAGES = sizeof(pico_languages) / sizeof(char *);

typedef struct Sound Sound;
struct Sound {
  char *sound_file;
  double volume;
  double pitch;
  double balance;
  bool loop;
  int side;  // 0: both, -1: left, 1: right
  bool need_update;
  bool need_stop;
  bool is_playing;
  struct Sound *next;
};

typedef struct {
  Sound *sound_list;
  char *text;
  char engine[10];   // either "pico" or "microsoft"
  char language[6];  // "en-US", "en-UK", "de-DE", "es-ES", "fr-FR", "it-IT", etc.
  double text_volume;
  bool need_stop;
  bool need_stop_all;
  bool need_engine_update;
  bool need_language_update;
  bool need_text_update;
} Speaker;

// Static functions

#ifdef _WIN32

static char **microsoft_languages = NULL;
static int N_MICROSOFT_LANGUAGES = 0;

#define MAX_KEY_LENGTH 255

static void init_microsoft_languages() {
  HKEY hKey;
  TCHAR ach_key[MAX_KEY_LENGTH], ach_class[MAX_PATH] = "";
  DWORD class_name = MAX_PATH, sub_keys = 0, max_sub_key, max_class, values, max_value, max_value_data, security_descriptor,
        name;
  FILETIME ftLastWriteTime;
  char language_code[6];
  if (RegOpenKeyEx(HKEY_LOCAL_MACHINE, "SOFTWARE\\Microsoft\\Speech\\Voices\\Tokens\\", 0, KEY_READ, &hKey) != ERROR_SUCCESS)
    return;
  // Get the class name and the value count.
  RegQueryInfoKey(hKey, ach_class, &class_name, NULL, &sub_keys, &max_sub_key, &max_class, &values, &max_value, &max_value_data,
                  &security_descriptor, &ftLastWriteTime);
  for (int i = 0; i < sub_keys; i++) {
    name = MAX_KEY_LENGTH;
    DWORD success = RegEnumKeyEx(hKey, i, ach_key, &name, NULL, NULL, NULL, &ftLastWriteTime);
    if (success == ERROR_SUCCESS) {
      if (strncmp(ach_key, "TTS_MS_", 7))
        continue;
      for (int j = 0; j < 5; ++j)
        language_code[j] = ach_key[j + 7];
      language_code[5] = '\0';
      // turn the first two characters to lowercase
      if (language_code[0] < 'a')
        language_code[0] += 32;
      if (language_code[1] < 'a')
        language_code[1] += 32;
      // turn the last two characters to uppercase
      if (language_code[3] > 'Z')
        language_code[3] -= 32;
      if (language_code[4] > 'Z')
        language_code[4] -= 32;
      bool super_continue = false;
      for (int j = 0; j < N_MICROSOFT_LANGUAGES; ++j) {
        if (strncmp(language_code, microsoft_languages[j], sizeof(language_code)) == 0) {
          super_continue = true;
          break;
        }
      }
      if (super_continue)
        continue;
      N_MICROSOFT_LANGUAGES++;
      microsoft_languages = (char **)realloc(microsoft_languages, N_MICROSOFT_LANGUAGES * sizeof(char *));
      if (microsoft_languages == NULL) {
        fprintf(stderr, "Error in Speaker initialization: not enough memory.\n");
        exit(EXIT_FAILURE);
      }
      microsoft_languages[N_MICROSOFT_LANGUAGES - 1] = malloc(sizeof(language_code));
      memcpy(microsoft_languages[N_MICROSOFT_LANGUAGES - 1], language_code, sizeof(language_code));
    }
  }
  RegCloseKey(hKey);
}

static void cleanup_microsoft_languages() {
  for (int i = 0; i < N_MICROSOFT_LANGUAGES; ++i)
    free(microsoft_languages[i]);
  free(microsoft_languages);
  microsoft_languages = NULL;
}

#endif

static Speaker *speaker_create() {
#ifdef _WIN32
  if (!microsoft_languages) {
    init_microsoft_languages();
    atexit(cleanup_microsoft_languages);
  }
#endif

  Speaker *speaker = malloc(sizeof(Speaker));
  speaker->sound_list = NULL;
  speaker->text = NULL;
  strncpy(speaker->engine, "pico", sizeof(speaker->engine));
  strncpy(speaker->language, "en-US", sizeof(speaker->language));
  speaker->text_volume = 0.0;
  speaker->need_stop = false;
  speaker->need_stop_all = false;
  speaker->need_engine_update = false;
  speaker->need_language_update = false;
  speaker->need_text_update = false;
  return speaker;
}

static Speaker *speaker_get_struct(WbDeviceTag t) {
  WbDevice *d = robot_get_device_with_node(t, WB_NODE_SPEAKER, true);
  return d ? d->pdata : NULL;
}

static int speaker_get_sound_to_update_number(Speaker *speaker) {
  int number = 0;
  Sound *sound = speaker->sound_list;
  while (sound) {
    if (sound->need_update)
      number++;
    sound = sound->next;
  }
  return number;
}

static int speaker_get_sound_to_stop_number(Speaker *speaker) {
  int number = 0;
  Sound *sound = speaker->sound_list;
  while (sound) {
    if (sound->need_stop)
      number++;
    sound = sound->next;
  }
  return number;
}

static Sound *speaker_get_sound_if_exist(WbDevice *d, const char *sound_name) {
  Speaker *speaker = (Speaker *)d->pdata;
  Sound *sound = speaker->sound_list;
  while (sound) {
    if (strcmp(sound->sound_file, sound_name) == 0)
      return sound;
    sound = sound->next;
  }
  return NULL;
}

static inline WbDevice *speaker_get_device(WbDeviceTag t) {
  return robot_get_device_with_node(t, WB_NODE_SPEAKER, true);
}

static void speaker_read_answer(WbDevice *d, WbRequest *r) {
  switch (request_read_uchar(r)) {
    case C_CONFIGURE:
      break;
    case C_SPEAKER_SOUND_OVER: {
      char *sound_name = request_read_string(r);
      Sound *sound = speaker_get_sound_if_exist(d, sound_name);
      free(sound_name);
      if (sound)
        sound->is_playing = false;
      break;
    }
    case C_SPEAKER_SPEAK_OVER: {
      Speaker *speaker = (Speaker *)d->pdata;
      ROBOT_ASSERT(speaker);
      // cppcheck-suppress nullPointerRedundantCheck
      free(speaker->text);
      // cppcheck-suppress nullPointerRedundantCheck
      speaker->text = NULL;
      break;
    }
    default:
      ROBOT_ASSERT(0);
  }
}

static void speaker_write_request(WbDevice *d, WbRequest *r) {
  Speaker *speaker = (Speaker *)d->pdata;
  if (speaker->need_stop) {  // it is important to send this request first
    request_write_uchar(r, C_SPEAKER_STOP);
    if (speaker->need_stop_all)
      request_write_uint16(r, 0);  // stop all sounds
    else {
      int number_of_sound_to_stop = speaker_get_sound_to_stop_number(speaker);
      request_write_int16(r, number_of_sound_to_stop);
      Sound *sound = speaker->sound_list;
      while (sound) {
        if (sound->need_stop) {
          request_write_uint16(r, strlen(sound->sound_file) + 1);
          request_write_string(r, sound->sound_file);
        }
        sound->need_stop = false;
        sound = sound->next;
      }
    }
    speaker->need_stop = false;
    speaker->need_stop_all = false;
  }
  int sound_number = speaker_get_sound_to_update_number(speaker);
  if (sound_number > 0) {
    request_write_uchar(r, C_SPEAKER_PLAY_SOUND);
    request_write_int32(r, sound_number);
    Sound *sound = speaker->sound_list;
    while (sound) {
      if (sound->need_update) {
        request_write_uint16(r, strlen(sound->sound_file) + 1);
        request_write_string(r, sound->sound_file);
        request_write_double(r, sound->volume);
        request_write_double(r, sound->pitch);
        request_write_double(r, sound->balance);
        request_write_uint16(r, sound->side);
        if (sound->loop)
          request_write_char(r, 1);
        else
          request_write_char(r, 0);
      }
      sound->need_update = false;
      sound = sound->next;
    }
  }
  if (speaker->need_engine_update) {
    request_write_uchar(r, C_SPEAKER_SET_ENGINE);
    request_write_uint16(r, strlen(speaker->engine) + 1);
    request_write_string(r, speaker->engine);
    speaker->need_engine_update = false;
  }
  if (speaker->need_language_update) {
    request_write_uchar(r, C_SPEAKER_SET_LANGUAGE);
    request_write_uint16(r, strlen(speaker->language) + 1);
    request_write_string(r, speaker->language);
    speaker->need_language_update = false;
  }
  if (speaker->need_text_update) {
    request_write_uchar(r, C_SPEAKER_SPEAK);
    request_write_uint16(r, strlen(speaker->text) + 1);
    request_write_string(r, speaker->text);
    request_write_double(r, speaker->text_volume);
    speaker->need_text_update = false;
  }
}

static void speaker_cleanup(WbDevice *d) {
  Speaker *speaker = (Speaker *)d->pdata;
  while (speaker->sound_list) {
    Sound *sound = speaker->sound_list->next;
    free(speaker->sound_list->sound_file);
    free(speaker->sound_list);
    speaker->sound_list = sound;
  }
  free(speaker->text);
  free(d->pdata);
}

static void speaker_toggle_remote(WbDevice *d, WbRequest *r) {
  // nothing to do.
}

static void speaker_play_sound(WbDevice *device, const char *sound_name, double volume, double pitch, double balance, bool loop,
                               int side) {
  Sound *sound = speaker_get_sound_if_exist(device, sound_name);
  if (!sound) {  // sound not in the Speaker sound list
    // insert first in the list
    sound = malloc(sizeof(Sound));
    int l = strlen(sound_name) + 1;
    sound->sound_file = malloc(l);
    memcpy(sound->sound_file, sound_name, l);
    sound->next = ((Speaker *)device->pdata)->sound_list;
    ((Speaker *)device->pdata)->sound_list = sound;
  }
  sound->volume = volume;
  sound->pitch = pitch;
  sound->balance = balance;
  sound->loop = loop;
  sound->side = side;
  sound->need_update = true;
  sound->need_stop = false;
  sound->is_playing = true;
}

// Exported functions

void wb_speaker_init(WbDevice *d) {
  d->pdata = speaker_create();
  d->read_answer = speaker_read_answer;
  d->write_request = speaker_write_request;
  d->cleanup = speaker_cleanup;
  d->toggle_remote = speaker_toggle_remote;
}

// Public functions (available from the user API)

void wb_speaker_play_sound(WbDeviceTag left, WbDeviceTag right, const char *sound, double volume, double pitch, double balance,
                           bool loop) {
  WbDevice *left_device = 0;
  WbDevice *right_device = 0;

  if (left) {
    left_device = speaker_get_device(left);
    if (!left_device) {
      fprintf(stderr, "Error: %s(): invalid 'left' device tag.\n", __FUNCTION__);
      return;
    }
  }
  if (right) {
    right_device = speaker_get_device(right);
    if (!right_device) {
      fprintf(stderr, "Error: %s(): invalid 'right' device tag.\n", __FUNCTION__);
      return;
    }
  }
  if (volume < 0) {
    fprintf(stderr, "Warning: %s() called with a negative volume value: %g. Setting volume to 0.\n", __FUNCTION__, volume);
    volume = 0;
  }
  if (volume > 1) {
    fprintf(stderr, "Warning: %s() called with a volume value greater than 1: %g. Setting volume to 1.\n", __FUNCTION__,
            volume);
    volume = 1;
  }
  if (pitch < 0) {
    fprintf(stderr, "Warning: %s() called with a negative pitch value: %g. Setting pitch to 0.\n", __FUNCTION__, pitch);
    pitch = 0;
  }
  if (balance < -1) {
    fprintf(stderr, "Warning: %s() called with a balance value less than -1: %g. Setting balance to -1.\n", __FUNCTION__,
            balance);
    balance = -1;
  }
  if (balance > 1) {
    fprintf(stderr, "Warning: %s() called with a balance value greater than 1: %g. Setting balance to 1.\n", __FUNCTION__,
            balance);
    balance = 1;
  }
  double left_volume, right_volume;
  if (balance <= 0) {
    left_volume = volume;
    right_volume = volume * (1 + balance);
  } else {
    left_volume = volume * (1 - balance);
    right_volume = volume;
  }
  if (left == right && volume > 0 && left_device)
    speaker_play_sound(left_device, sound, volume, pitch, balance, loop, 0);
  else {
    if (left_volume > 0 && left_device)
      speaker_play_sound(left_device, sound, left_volume, pitch, 0.0, loop, -1);
    if (right_volume > 0 && right_device)
      speaker_play_sound(right_device, sound, right_volume, pitch, 0.0, loop, 1);
  }
}

void wb_speaker_stop(WbDeviceTag tag, const char *sound) {
  Speaker *speaker = speaker_get_struct(tag);
  if (!speaker) {
    fprintf(stderr, "Error: %s(): invalid device tag.\n", __FUNCTION__);
    return;
  }
  speaker->need_stop = true;
  // reset all the sounds
  if (!sound || strlen(sound) == 0) {
    speaker->need_stop_all = true;
    Sound *s = speaker->sound_list;
    while (s) {
      s->need_update = false;
      s = s->next;
    }
    speaker->need_text_update = false;
    free(speaker->text);
    speaker->text = NULL;
  } else {
    WbDevice *device = robot_get_device_with_node(tag, WB_NODE_SPEAKER, true);
    Sound *s = speaker_get_sound_if_exist(device, sound);
    if (s) {
      s->need_update = false;
      s->need_stop = true;
      s->is_playing = false;
    }
  }
}

bool wb_speaker_is_sound_playing(WbDeviceTag tag, const char *sound) {
  const Speaker *speaker = speaker_get_struct(tag);
  if (!speaker) {
    fprintf(stderr, "Error: %s(): invalid device tag.\n", __FUNCTION__);
    return false;
  }

  WbDevice *device = robot_get_device_with_node(tag, WB_NODE_SPEAKER, true);

  if (sound == NULL || sound[0] == '\0') {
    // check for any sound (including TTS)
    if (speaker->text != NULL)
      return true;
    Sound *s = speaker->sound_list;
    while (s) {
      if (s->is_playing)
        return true;
      s = s->next;
    }
  } else {
    // check for one specific sound
    const Sound *s = speaker_get_sound_if_exist(device, sound);
    if (s)
      return s->is_playing;
  }

  return false;
}

bool wb_speaker_set_engine(WbDeviceTag tag, const char *engine) {
  Speaker *speaker = speaker_get_struct(tag);
  if (!speaker) {
    fprintf(stderr, "Error: %s(): invalid device tag.\n", __FUNCTION__);
    return false;
  }
  if (strncmp(engine, "pico", 5) == 0)
    strncpy(speaker->engine, "pico", sizeof(speaker->engine));
#ifdef _WIN32
  else if (strncmp(engine, "microsoft", 10) == 0)
    strncpy(speaker->engine, "microsoft", sizeof(speaker->engine));
#endif
  else
    return false;
  speaker->need_engine_update = true;
  return true;
}

bool wb_speaker_set_language(WbDeviceTag tag, const char *language) {
  int i;
  Speaker *speaker = speaker_get_struct(tag);
  if (!speaker) {
    fprintf(stderr, "Error: %s(): invalid device tag.\n", __FUNCTION__);
    return false;
  }
  int n_languages = 0;
  const char **languages;
  if (strncmp(speaker->engine, "pico", sizeof(speaker->engine)) == 0) {
    n_languages = N_PICO_LANGUAGES;
    languages = pico_languages;
  }
#ifdef _WIN32
  else if (strncmp(speaker->engine, "microsoft", sizeof(speaker->engine)) == 0) {
    n_languages = N_MICROSOFT_LANGUAGES;
    languages = (const char **)microsoft_languages;
  }
#endif
  if (strlen(language) > 5 || language[2] != '-' || language[0] < 'a' || language[0] > 'z' || language[1] < 'a' ||
      language[1] > 'z' || language[3] < 'A' || language[3] > 'Z' || language[4] < 'A' || language[4] > 'Z') {
    fprintf(stderr, "Error: %s() called with an invalid 'language' argument: \"%s\".\n", __FUNCTION__, language);
    fprintf(stderr, "'language' should follow the \"ll-CC\" format where ll is the ISO 639-1 language code and CC is the ISO "
                    "3166 country code, for example, \"en-US\" or \"fr-FR\".\n");
    fprintf(stderr, "Available languages for \"%s\" engine include:\n", speaker->engine);
    for (i = 0; i < n_languages; ++i)
      fprintf(stderr, " - \"%s\"\n", languages[i]);
    return false;
  }
  bool found = false;
  for (i = 0; i < n_languages; ++i) {
    if (strcmp(language, languages[i]) == 0) {
      found = true;
      break;
    }
  }
  if (!found)  // if the language is well formed, but not available, silently return false
    return false;
  if (strcmp(speaker->language, language) == 0)
    return true;  // already set
  strncpy(speaker->language, language, sizeof(speaker->language));
  speaker->need_language_update = true;
  return true;  // should be all right
}

const char *wb_speaker_get_engine(WbDeviceTag tag) {
  Speaker *speaker = speaker_get_struct(tag);
  if (!speaker) {
    fprintf(stderr, "Error: %s(): invalid device tag.\n", __FUNCTION__);
    return NULL;
  }
  return speaker->engine;
}

const char *wb_speaker_get_language(WbDeviceTag tag) {
  Speaker *speaker = speaker_get_struct(tag);
  if (!speaker) {
    fprintf(stderr, "Error: %s(): invalid device tag.\n", __FUNCTION__);
    return NULL;
  }
  return speaker->language;
}

void wb_speaker_speak(WbDeviceTag tag, const char *text, double volume) {
  Speaker *speaker = speaker_get_struct(tag);
  if (!speaker) {
    fprintf(stderr, "Error: %s(): invalid device tag.\n", __FUNCTION__);
    return;
  }
  if (volume < 0) {
    fprintf(stderr, "Warning: %s() called with negative volume value: %g. Setting volume to 0.\n", __FUNCTION__, volume);
    volume = 0;
  }
  if (volume > 1) {
    fprintf(stderr, "Warning: %s() called with volume value greater than 1: %g. Setting volume to 1.\n", __FUNCTION__, volume);
    volume = 1;
  }
  free(speaker->text);
  int l = strlen(text) + 1;
  speaker->text = malloc(l);
  memcpy(speaker->text, text, l);
  speaker->text_volume = volume;
  speaker->need_text_update = true;
}

bool wb_speaker_is_speaking(WbDeviceTag tag) {
  const Speaker *speaker = speaker_get_struct(tag);
  if (!speaker) {
    fprintf(stderr, "Error: %s(): invalid device tag.\n", __FUNCTION__);
    return false;
  }
  return speaker->text != NULL;
}
