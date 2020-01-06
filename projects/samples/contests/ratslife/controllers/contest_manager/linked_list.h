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
 * Description:  Interface of a linked list of void*
 * Example:
 *     LLIST *n = NULL;
 *     int zero=0, one=1, two=2, three=3, four=4;
 *     printf("A:%d\n", list_size(n));
 *     list_add(&n, &zero);                 // list: 0
 *     printf("B:%d\n", list_size(n));
 *     list_add(&n, &one);                  // list: 1 0
 *     list_add(&n, &two);                  // list: 2 1 0
 *     list_add(&n, &three);                // list: 3 2 1 0
 *     list_add(&n, &four);                 // list: 4 3 2 1 0
 *     printf("C:%d\n", list_size(n));
 *     list_remove(&n);                     // remove first (4)
 *     list_remove(&n->next);               // remove new second (2)
 *     list_remove(list_search(&n, &one));  // remove cell containing 1 (first)
 *     list_remove(&n->next);               // remove second to last node (0)
 *     list_remove(&n);                     // remove last (3)
 *     printf("D:%d\n", list_size(n));
 */

#ifndef LINKED_LIST_H
#define LINKED_LIST_H

typedef struct node {
  void *data;
  struct node *next; /* pointer to next element in list */
} LLIST;

LLIST *list_add(LLIST **p, void *i);     /* Function definition to add an element */
void list_remove(LLIST **p);             /* Function definition to remove element (head) */
LLIST **list_search(LLIST **n, void *i); /* Function definition to search the list - Look out: O(N)*/
int list_size(LLIST *n);                 /* Function definition to count the list elements - Look out: O(N)*/
LLIST *list_at(LLIST **p, int at);       /* Function definition to return the i-th element of the list - Look out: O(N)*/

#endif
