/*
   File    : play_melody.c
   Author  : Eliot Ferragni, adapted by Olivier Michel
   Date    : 25 january 2019
   REV 1.1
   Functions and defines to play little melodies on the speaker
   Adapted from the code written by Dipto Pratyaksa
   Taken from https://www.princetronics.com/supermariothemesong/
*/

#include "play_melody.h"

#include <stddef.h>

// Mission Impossible
static const uint16_t mission_impossible_melody[] = {NOTE_G4,  0,       0,        NOTE_G4, 0, 0, NOTE_AS4, 0, NOTE_C5,  0,
                                                     NOTE_G4,  0,       0,        NOTE_G4, 0, 0, NOTE_F4,  0, NOTE_FS4, 0,

                                                     NOTE_G4,  0,       0,        NOTE_G4, 0, 0, NOTE_AS4, 0, NOTE_C5,  0,
                                                     NOTE_G4,  0,       0,        NOTE_G4, 0, 0, NOTE_F4,  0, NOTE_FS4, 0,

                                                     NOTE_AS5, NOTE_G5, NOTE_D5,  0,       0, 0, 0,        0, 0,        0,
                                                     NOTE_AS5, NOTE_G5, NOTE_CS5, 0,       0, 0, 0,        0, 0,        0,
                                                     NOTE_AS5, NOTE_G5, NOTE_C5,  0,       0, 0, 0,        0, 0,        0,
                                                     NOTE_AS4, NOTE_C5, 0,        0,       0, 0, 0,        0, 0,        0};
static const float mission_impossible_tempo[] = {
  12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12,
  12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12,
  12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12};

// We are the champions
static const uint16_t champions_melody[] = {
  NOTE_F5,  NOTE_E5,  NOTE_F5,  NOTE_E5,  NOTE_C5,  0,        NOTE_A4, NOTE_D5,  NOTE_A4, NOTE_A3,  0,       NOTE_G3,
  0,        NOTE_F3,  0,        0,        NOTE_G3,  NOTE_C5,  NOTE_F5, NOTE_G5,  NOTE_A5, NOTE_C6,  NOTE_A5, NOTE_D5,
  NOTE_E5,  NOTE_D5,  0,        NOTE_D4,  NOTE_C4,  NOTE_AS3, NOTE_A3, 0,        0,       NOTE_D5,  NOTE_C5, NOTE_D5,
  NOTE_C5,  NOTE_AS4, NOTE_AS5, NOTE_A5,  NOTE_AS5, NOTE_A5,  NOTE_G5, NOTE_A5,  NOTE_F5, NOTE_AS5, NOTE_A5, NOTE_F5,
  NOTE_AS5, NOTE_GS5, NOTE_F5,  NOTE_AS5, NOTE_GS5, NOTE_F5,  0,       NOTE_DS5, NOTE_C5, NOTE_F5};
static const float champions_tempo[] = {2.25, 9,   9,   4.5,  4.5, 9,   9,   4.5, 4.5, 18,   18,  18, 18,  4.5, 18,
                                        18,   4.5, 9,   2.25, 9,   9,   4.5, 4.5, 9,   9,    4.5, 9,  9,   9,   9,
                                        4.5,  4.5, 4.5, 3,    4.5, 9,   3,   3,   3,   4.5,  9,   3,  3,   3,   4.5,
                                        9,    3,   4.5, 9,    3,   4.5, 9,   3,   3,   2.25, 9,   9,  2.25};

static const uint16_t russia_melody[] = {0,       0,        NOTE_AS4, NOTE_DS5, NOTE_AS4, NOTE_C5,  NOTE_D5,  NOTE_G4,
                                         NOTE_C5, NOTE_AS4, NOTE_GS4, NOTE_AS4, NOTE_DS4, NOTE_DS4, NOTE_F4,  NOTE_F4,
                                         NOTE_G4, NOTE_GS4, NOTE_GS4, NOTE_AS4, NOTE_C5,  NOTE_D5,  NOTE_DS5, NOTE_F5};

static const float russia_tempo[] = {9, 9, 9, 3, 4.5, 9, 3, 3, 3, 4.5, 9, 3, 4.5, 9, 3, 4.5, 9, 3, 4.5, 9, 3, 4.5, 9, 1.5};

// Mario main theme
static const uint16_t mario_melody[] = {
  NOTE_E5, NOTE_E5, 0, NOTE_E5,  0,       NOTE_C5, NOTE_E5, 0,       NOTE_G5,  0,       0,       0,       NOTE_G4,
  0,       0,       0, NOTE_C5,  0,       0,       NOTE_G4, 0,       0,        NOTE_E4, 0,       0,       NOTE_A4,
  0,       NOTE_B4, 0, NOTE_AS4, NOTE_A4, 0,       NOTE_G4, NOTE_E5, NOTE_G5,  NOTE_A5, 0,       NOTE_F5, NOTE_G5,
  0,       NOTE_E5, 0, NOTE_C5,  NOTE_D5, NOTE_B4, 0,       0,       NOTE_C5,  0,       0,       NOTE_G4, 0,
  0,       NOTE_E4, 0, 0,        NOTE_A4, 0,       NOTE_B4, 0,       NOTE_AS4, NOTE_A4, 0,       NOTE_G4, NOTE_E5,
  NOTE_G5, NOTE_A5, 0, NOTE_F5,  NOTE_G5, 0,       NOTE_E5, 0,       NOTE_C5,  NOTE_D5, NOTE_B4, 0,       0};
static const float mario_tempo[] = {
  12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12,
  12, 12, 12, 12, 12, 12, 9,  9,  9,  12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12,
  12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 9,  9,  9,  12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12,
};

// Underworld
static const uint16_t underworld_melody[] = {
  NOTE_C4,  NOTE_C5,  NOTE_A3,  NOTE_A4, NOTE_AS3, NOTE_AS4, 0,        0,       NOTE_C4,  NOTE_C5,  NOTE_A3, NOTE_A4,
  NOTE_AS3, NOTE_AS4, 0,        0,       NOTE_F3,  NOTE_F4,  NOTE_D3,  NOTE_D4, NOTE_DS3, NOTE_DS4, 0,       0,
  NOTE_F3,  NOTE_F4,  NOTE_D3,  NOTE_D4, NOTE_DS3, NOTE_DS4, 0,        0,       NOTE_DS4, NOTE_CS4, NOTE_D4, NOTE_CS4,
  NOTE_DS4, NOTE_D4,  NOTE_GS3, NOTE_G3, NOTE_CS4, NOTE_C4,  NOTE_FS4, NOTE_F4, NOTE_E3,  NOTE_AS4, NOTE_A4, NOTE_GS4,
  NOTE_DS4, NOTE_B3,  NOTE_AS3, NOTE_A3, NOTE_GS3, 0,        0,        0};
static const float underworld_tempo[] = {12, 12, 12, 12, 12, 12, 6,  3,  12, 12, 12, 12, 12, 12, 6,  3,  12, 12, 12,
                                         12, 12, 12, 6,  3,  12, 12, 12, 12, 12, 12, 6,  6,  18, 18, 18, 6,  6,  6,
                                         6,  6,  6,  18, 18, 18, 18, 18, 18, 10, 10, 10, 10, 10, 10, 3,  3,  3};

// mario start
static const uint16_t mario_start_melody[] = {NOTE_C5,  NOTE_E5,  NOTE_G5,  NOTE_C5,  NOTE_E5,  NOTE_G5,  NOTE_E5,
                                              0,        NOTE_C5,  NOTE_DS5, NOTE_FS5, NOTE_C5,  NOTE_DS5, NOTE_FS5,
                                              NOTE_DS5, 0,        NOTE_D5,  NOTE_F5,  NOTE_AS5, NOTE_D5,  NOTE_F5,
                                              NOTE_AS5, NOTE_AS5, NOTE_AS5, NOTE_AS5, NOTE_C6,  0};
static const float mario_start_tempo[] = {16, 16, 16, 16, 16, 16, 12, 12, 16, 16, 16, 16, 16, 16,
                                          12, 12, 16, 16, 16, 16, 16, 16, 19, 19, 19, 10, 10};

// Mario death
static const uint16_t mario_death_melody[] = {NOTE_B3, NOTE_F4, 0,       NOTE_F4, NOTE_F4, NOTE_E4,
                                              NOTE_D4, NOTE_C4, NOTE_E3, 0,       NOTE_E3, NOTE_C3};
static const float mario_death_tempo[] = {12, 12, 12, 12, 9, 9, 9, 12, 12, 16, 12, 12};

// Mario flag pole
static const uint16_t mario_flagpole[] = {NOTE_G2,  NOTE_C3,  NOTE_E3,  NOTE_G3,  NOTE_C4,  NOTE_E4,  NOTE_G4,
                                          NOTE_E4,  NOTE_GS2, NOTE_C3,  NOTE_DS3, NOTE_GS3, NOTE_C4,  NOTE_DS4,
                                          NOTE_GS4, NOTE_DS4, NOTE_AS2, NOTE_D3,  NOTE_F3,  NOTE_AS3, NOTE_D4,
                                          NOTE_F4,  NOTE_AS4, NOTE_B4,  NOTE_B4,  NOTE_B4,  NOTE_C5};
static const float mario_flagpole_tempo[] = {15,  15,  15, 15, 15, 15, 4.5, 4.5, 15,  15, 15, 15, 15, 15,
                                             4.5, 4.5, 15, 15, 15, 15, 15,  15,  4.5, 15, 15, 15, 3};

// Walking
static const uint16_t walking_melody[] = {NOTE_F3, NOTE_A3, NOTE_C3, NOTE_E3, NOTE_F3, NOTE_A3, NOTE_C3, NOTE_D3, NOTE_E3};
static const float walking_melody_tempo[] = {6, 9, 6, 9, 6, 9, 12, 12, 12};

// Pirates of the Caribbean main theme
static const uint16_t pirate_melody[] = {
  NOTE_E4, NOTE_G4, NOTE_A4, NOTE_A4, 0, NOTE_A4, NOTE_B4, NOTE_C5, NOTE_C5, 0, NOTE_C5, NOTE_D5, NOTE_B4, NOTE_B4, 0, NOTE_A4,
  NOTE_G4, NOTE_A4, 0, NOTE_E4, NOTE_G4, NOTE_A4, NOTE_A4, 0, NOTE_A4, NOTE_B4, NOTE_C5, NOTE_C5, 0, NOTE_C5, NOTE_D5, NOTE_B4,
  NOTE_B4, 0, NOTE_A4, NOTE_G4, NOTE_A4, 0, NOTE_E4, NOTE_G4, NOTE_A4, NOTE_A4, 0, NOTE_A4, NOTE_C5, NOTE_D5, NOTE_D5, 0,
  NOTE_D5, NOTE_E5, NOTE_F5, NOTE_F5, 0, NOTE_E5, NOTE_D5, NOTE_E5, NOTE_A4, 0, NOTE_A4, NOTE_B4, NOTE_C5, NOTE_C5, 0, NOTE_D5,
  NOTE_E5, NOTE_A4, 0, NOTE_A4, NOTE_C5, NOTE_B4, NOTE_B4, 0, NOTE_C5, NOTE_A4, NOTE_B4, 0, NOTE_A4, NOTE_A4,
  // Repeat of first part
  NOTE_A4, NOTE_B4, NOTE_C5, NOTE_C5, 0, NOTE_C5, NOTE_D5, NOTE_B4, NOTE_B4, 0, NOTE_A4, NOTE_G4, NOTE_A4, 0, NOTE_E4, NOTE_G4,
  NOTE_A4, NOTE_A4, 0, NOTE_A4, NOTE_B4, NOTE_C5, NOTE_C5, 0, NOTE_C5, NOTE_D5, NOTE_B4, NOTE_B4, 0, NOTE_A4, NOTE_G4, NOTE_A4,
  0, NOTE_E4, NOTE_G4, NOTE_A4, NOTE_A4, 0, NOTE_A4, NOTE_C5, NOTE_D5, NOTE_D5, 0, NOTE_D5, NOTE_E5, NOTE_F5, NOTE_F5, 0,
  NOTE_E5, NOTE_D5, NOTE_E5, NOTE_A4, 0, NOTE_A4, NOTE_B4, NOTE_C5, NOTE_C5, 0, NOTE_D5, NOTE_E5, NOTE_A4, 0, NOTE_A4, NOTE_C5,
  NOTE_B4, NOTE_B4, 0, NOTE_C5, NOTE_A4, NOTE_B4, 0,
  // End of Repeat
  NOTE_E5, 0, 0, NOTE_F5, 0, 0, NOTE_E5, NOTE_E5, 0, NOTE_G5, 0, NOTE_E5, NOTE_D5, 0, 0, NOTE_D5, 0, 0, NOTE_C5, 0, 0, NOTE_B4,
  NOTE_C5, 0, NOTE_B4, 0, NOTE_A4, NOTE_E5, 0, 0, NOTE_F5, 0, 0, NOTE_E5, NOTE_E5, 0, NOTE_G5, 0, NOTE_E5, NOTE_D5, 0, 0,
  NOTE_D5, 0, 0, NOTE_C5, 0, 0, NOTE_B4, NOTE_C5, 0, NOTE_B4, 0, NOTE_A4};
static const float pirate_tempo[] = {
  10, 10, 5, 10, 10, 10, 10, 5, 10, 10, 10, 10, 5, 10, 10, 10, 10, 3, 10, 10, 10, 5, 10, 10, 10, 10, 5, 10, 10, 10, 10, 5, 10,
  10, 10, 10, 3, 10, 10, 10, 5, 10, 10, 10, 10, 5, 10, 10, 10, 10, 5, 10, 10, 10, 10, 3, 10, 10, 10, 10, 5, 10, 10, 5, 10, 5,
  10, 10, 10, 5, 10, 10, 10, 10, 3, 3, 5, 10,
  // Repeat of First Part
  10, 10, 5, 10, 10, 10, 10, 5, 10, 10, 10, 10, 3, 10, 10, 10, 5, 10, 10, 10, 10, 5, 10, 10, 10, 10, 5, 10, 10, 10, 10, 3, 10,
  10, 10, 5, 10, 10, 10, 10, 5, 10, 10, 10, 10, 5, 10, 10, 10, 10, 10, 10, 10, 10, 10, 5, 10, 10, 5, 10, 10, 10, 10, 10, 5, 10,
  10, 10, 10, 3, 3,
  // End of Repeat
  5, 10, 5, 5, 10, 5, 10, 10, 10, 10, 10, 10, 10, 10, 5, 5, 10, 5, 5, 10, 5, 10, 10, 10, 10, 10, 2, 5, 10, 5, 5, 10, 5, 10, 10,
  10, 10, 10, 10, 10, 10, 5, 5, 10, 5, 5, 10, 5, 10, 10, 10, 10, 10, 2};

// Simpson main theme
static const uint16_t simpson_melody[] = {NOTE_D3,  0,       0,       NOTE_E3,  0,        NOTE_FS3, 0,       NOTE_A3,  NOTE_G3,
                                          0,        0,       NOTE_E3, 0,        NOTE_C3,  0,        NOTE_A2, NOTE_FS2, NOTE_FS2,
                                          NOTE_FS2, NOTE_G2, 0,       NOTE_FS3, NOTE_FS3, NOTE_FS3, NOTE_G3, NOTE_AS3, 0,
                                          0,        NOTE_B3, 0,       0,        0,        0,        0};
static const float simpson_tempo[] = {12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12,
                                      12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12};

// Star Wars
static const uint16_t starwars_melody[] = {NOTE_A4, NOTE_A4, NOTE_A4, NOTE_F4, NOTE_C5, NOTE_A4,  NOTE_F4, NOTE_C5, NOTE_A4, 0,
                                           NOTE_E5, NOTE_E5, NOTE_E5, NOTE_F5, NOTE_C5, NOTE_GS4, NOTE_F4, NOTE_C5, NOTE_A4};
static const float starwars_tempo[] = {3.6, 3.6, 3.6, 4.8, 12, 3.6, 4.8, 12, 3.6, 2, 3.6, 3.6, 3.6, 4.8, 12, 3.6, 4.8, 12, 3.6};

// Sandstorms
static const uint16_t sandstorms_melody[] = {
  NOTE_B4, NOTE_B4, NOTE_B4, NOTE_B4, NOTE_B4, 0,       NOTE_B4, NOTE_B4, NOTE_B4, NOTE_B4, NOTE_B4, NOTE_B4, NOTE_B4,
  0,       NOTE_E5, NOTE_E5, NOTE_E5, NOTE_E5, NOTE_E5, NOTE_E5, NOTE_E5, 0,       NOTE_D5, NOTE_D5, NOTE_D5, NOTE_D5,
  NOTE_D5, NOTE_D5, NOTE_D5, 0,       NOTE_A4, NOTE_A4, NOTE_B4, NOTE_B4, NOTE_B4, NOTE_B4, NOTE_B4, 0,       NOTE_B4,
  NOTE_B4, NOTE_B4, NOTE_B4, NOTE_B4, NOTE_B4, NOTE_B4, 0,       NOTE_E5, NOTE_E5, NOTE_B4, NOTE_B4, NOTE_B4, NOTE_B4,
  NOTE_B4, 0,       NOTE_B4, NOTE_B4, NOTE_B4, NOTE_B4, NOTE_B4, NOTE_B4, NOTE_B4, 0};
static const float sandstorms_tempo[] = {24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24,
                                         24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24,
                                         24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24};

// seven nation army by White Stripes
static const uint16_t seven_nation_army_melody[] = {0, NOTE_D4, NOTE_D4, 0, NOTE_F4, 0, NOTE_D4, 0, NOTE_C4, NOTE_AS3, NOTE_A3,
                                                    0, NOTE_D4, NOTE_D4, 0, NOTE_F4, 0, NOTE_D4, 0, NOTE_C4, NOTE_AS3, NOTE_A3,
                                                    0, NOTE_D4, NOTE_D4, 0, NOTE_F4, 0, NOTE_D4, 0, NOTE_C4, NOTE_AS3, NOTE_A3,
                                                    0};
static const float seven_nation_army_tempo[] = {12, 3, 20, 20,  20,  9,  20, 9,  9,  2.2, 2.2, 70, 3, 20, 20,  20,  9,
                                                20, 9, 9,  2.2, 2.2, 70, 3,  20, 20, 20,  9,   20, 9, 9,  2.2, 2.2, 70};

static const melody_t melodies[NB_SONGS] = {
  {.notes = mission_impossible_melody,
   .tempo = mission_impossible_tempo,
   .length = sizeof(mission_impossible_melody) / sizeof(uint16_t)},
  {.notes = champions_melody, .tempo = champions_tempo, .length = sizeof(champions_melody) / sizeof(uint16_t)},
  {.notes = russia_melody, .tempo = russia_tempo, .length = sizeof(russia_melody) / sizeof(uint16_t)},
  {.notes = mario_melody, .tempo = mario_tempo, .length = sizeof(mario_melody) / sizeof(uint16_t)},
  {.notes = underworld_melody, .tempo = underworld_tempo, .length = sizeof(underworld_melody) / sizeof(uint16_t)},
  {.notes = mario_start_melody, .tempo = mario_start_tempo, .length = sizeof(mario_start_melody) / sizeof(uint16_t)},
  {.notes = mario_death_melody, .tempo = mario_death_tempo, .length = sizeof(mario_death_melody) / sizeof(uint16_t)},
  {.notes = mario_flagpole, .tempo = mario_flagpole_tempo, .length = sizeof(mario_flagpole) / sizeof(uint16_t)},
  {.notes = walking_melody, .tempo = walking_melody_tempo, .length = sizeof(walking_melody) / sizeof(uint16_t)},
  {.notes = pirate_melody, .tempo = pirate_tempo, .length = sizeof(pirate_melody) / sizeof(uint16_t)},
  {.notes = simpson_melody, .tempo = simpson_tempo, .length = sizeof(simpson_melody) / sizeof(uint16_t)},
  {.notes = starwars_melody, .tempo = starwars_tempo, .length = sizeof(starwars_melody) / sizeof(uint16_t)},
  {.notes = sandstorms_melody, .tempo = sandstorms_tempo, .length = sizeof(sandstorms_melody) / sizeof(uint16_t)},
  {.notes = seven_nation_army_melody,
   .tempo = seven_nation_army_tempo,
   .length = sizeof(seven_nation_army_melody) / sizeof(uint16_t)}};

static WbDeviceTag speaker = 0;
static double volume = 1.0;
static melody_t *melody = NULL;
static uint16_t melody_step = 0;
static double melody_time = 0;
static double next_tone = 0;
static double next_silence = 0;

static void reset() {
  melody = NULL;
  melody_step = 0;
  melody_time = 0;
  next_tone = 0;
  next_silence = 0;
}

void play_melody_set_song(song_selection_t choice) {
  reset();
  melody = &melodies[choice];
}

void play_melody_set_melody(melody_t *m) {
  reset();
  melody = m;
}

void play_melody_set_speaker(WbDeviceTag s) {
  speaker = s;
}

void play_melody_stop() {
  wb_speaker_stop(speaker, NULL);
  reset();
}

void play_melody_step(double tempo) {
  if (melody == NULL)
    return;
  if (melody_time >= next_tone) {
    const uint16_t note = melody->notes[melody_step];
    const double pitch = (double)note / (double)NOTE_A4;
    // to calculate the note duration, take one second
    // divided by the note type.
    // e.g. quarter note = 1000 / 4, eighth note = 1000/8, etc.
    const uint16_t duration = 1000 / melody->tempo[melody_step];
    next_silence = melody_time + duration;
    // to distinguish the notes, set a minimum time between them.
    // the note's duration + 30% seems to work well:
    next_tone = melody_time + 1.3 * duration;
    wb_speaker_play_sound(speaker, speaker, "sounds/440Hz.wav", volume, pitch, 0, true);
    melody_step++;
  } else if (melody_time >= next_silence) {
    wb_speaker_stop(speaker, NULL);
    next_silence = 0;
    if (melody_step == melody->length)
      reset();
  }
  melody_time += tempo;
}
