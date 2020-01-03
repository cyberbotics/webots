/*
 * Copyright 1996-2020 Cyberbotics Ltd.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef ROBOTBENCHMARK_H
#define ROBOTBENCHMARK_H

#include <webots/utils/default_robot_window.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static inline void robotbenchmark_record(const char *answer, const char *benchmark, double record) {
  const char *separator = strchr(&answer[7], ':');
  int n = strlen(answer) + 9;
  char *user = malloc(n);
  if (separator) {
    int i = separator - &answer[7];
    char *email = malloc(n);
    strncpy(email, &answer[7], i);
    email[i] = '\0';
    char *password = malloc(n);
    strcpy(password, &answer[7 + i + 1]);
    sprintf(user, "email=%s&password=%s", email, password);
    free(email);
    free(password);
  } else
    sprintf(user, "owner=%s", &answer[7]);
  char host[1024];
  FILE *file = fopen("../../host.txt", "r");
  if (!file) {
    fprintf(stderr, "Error: cannot open host file.\n");
    free(user);
    return;
  }
  n = fscanf(file, "%1023s", host);
  fclose(file);
  if (n < 1) {
    fprintf(stderr, "Error: cannot read host name.\n");
    free(user);
    return;
  }

  char fqdn[1024];
  int start;
  if (strncmp(host, "https://", 8) == 0)
    start = 8;
  else  // assuming "http://"
    start = 7;
  snprintf(fqdn, sizeof(fqdn), "%s", &host[start]);
  char *colon = strchr(fqdn, ':');
  if (colon)
    *colon = '\0';
  const char *WEBOTS_HOME = getenv("WEBOTS_HOME");
  n = strlen(WEBOTS_HOME) + 1024;
  char *filename = malloc(n);
  snprintf(filename, n, "%s/resources/web/server/key/%s", WEBOTS_HOME, fqdn);
  file = fopen(filename, "r");
  if (!file) {
    fprintf(stderr, "Error: cannot open key file '%s'\n", filename);
    free(filename);
    free(user);
    return;
  }
  free(filename);
  char key[1024];
  n = fread(key, 1, sizeof(key), file);
  key[n] = '\0';
  fclose(file);
  for (n = 0; n < sizeof(key); n++) {
    if (key[n] <= ' ') {
      key[n] = '\0';
      break;
    }
  }
  int l = 1024 + strlen(user) + strlen(benchmark);
  char *command = malloc(l);
  snprintf(command, l,
           "wget -qO- "
           "--post-data=\"%s&record=%f&benchmark=%s&key=%s\" "
           "%s/record.php",
           user, record, benchmark, key, host);
  free(user);
  file = popen(command, "r");
  if (!file) {
    fprintf(stderr, "Error: cannot run wget.\n");
    free(command);
    return;
  }
  snprintf(command, 8, "record:");
  n = fread(&command[7], 1, l - 7, file);
  command[7 + n] = '\0';
  pclose(file);
  wb_robot_wwi_send_text(command);
  free(command);
}

#endif  // ROBOTBENCHMARK_H
