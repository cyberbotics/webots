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

/*
 * Description:  Implementation of the linked_list.h file
 */

#include "linked_list.h"
#include <stdlib.h>

LLIST *list_add(LLIST **p, void *i) {
  if (p == NULL)
    return NULL;

  LLIST *n = malloc(sizeof(LLIST));
  if (n == NULL)
    return NULL;

  n->next = *p;
  *p = n;
  n->data = i;

  return *p;
}

LLIST *list_at(LLIST **p, int at) {
  if (p == NULL || at < 0)
    return NULL;
  else if (at == 0)
    return *p;
  else
    return list_at(&((*p)->next), at - 1);
}

void list_remove(LLIST **p) {
  if (p != NULL && *p != NULL) {
    LLIST *n = *p;
    *p = (*p)->next;
    free(n);
  }
}

LLIST **list_search(LLIST **n, void *i) {
  if (n == NULL)
    return NULL;

  while (*n != NULL) {
    if ((*n)->data == i) {
      return n;
    }
    n = &(*n)->next;
  }
  return NULL;
}

int list_size(LLIST *n) {
  int size = 0;
  while (n != NULL) {
    size++;
    n = n->next;
  }
  return size;
}
