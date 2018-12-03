/*************************************************************************
 *                                                                       *
 * Open Dynamics Engine, Copyright (C) 2001,2002 Russell L. Smith.       *
 * All rights reserved.  Email: russ@q12.org   Web: www.q12.org          *
 *                                                                       *
 * This library is free software; you can redistribute it and/or         *
 * modify it under the terms of EITHER:                                  *
 *   (1) The GNU Lesser General Public License as published by the Free  *
 *       Software Foundation; either version 2.1 of the License, or (at  *
 *       your option) any later version. The text of the GNU Lesser      *
 *       General Public License is included with this library in the     *
 *       file LICENSE.TXT.                                               *
 *   (2) The BSD-style license that is included with this library in     *
 *       the file LICENSE-BSD.TXT.                                       *
 *                                                                       *
 * This library is distributed in the hope that it will be useful,       *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the files    *
 * LICENSE.TXT and LICENSE-BSD.TXT for more details.                     *
 *                                                                       *
 *************************************************************************/

#include <ode/common.h>
#include <ode/error.h>
#include <ode/memory.h>
#include "config.h"
#include "obstack.h"
#include "util.h"

//****************************************************************************
// macros and constants

#define ROUND_UP_OFFSET_TO_EFFICIENT_SIZE(arena,ofs) \
    ofs = (size_t) (dEFFICIENT_SIZE( ((size_t)(arena)) + ofs ) - ((size_t)(arena)) )

#define MAX_ALLOC_SIZE \
    ((size_t)(dOBSTACK_ARENA_SIZE - sizeof (Arena) - EFFICIENT_ALIGNMENT + 1))

//****************************************************************************
// dObStack

dObStack::dObStack():
    m_first(NULL), m_last(NULL),
    m_current_arena(NULL), m_current_ofs(0)
{
}

dObStack::~dObStack()
{
    // free all arenas
    Arena *a,*nexta;
    a = m_first;
    while (a) {
        nexta = a->m_next;
        dFree (a,dOBSTACK_ARENA_SIZE);
        a = nexta;
    }
}

void *dObStack::alloc (size_t num_bytes)
{
    if (num_bytes > MAX_ALLOC_SIZE) dDebug (0,"num_bytes too large");

    bool last_alloc_needed = false, last_init_needed = false;
    Arena **last_ptr = NULL;

    if (m_last != NULL) {
        if ((m_last->m_used + num_bytes) > dOBSTACK_ARENA_SIZE) {
            if (m_last->m_next != NULL) {
                m_last = m_last->m_next;
                last_init_needed = true;
            } else {
                last_ptr = &m_last->m_next;
                last_alloc_needed = true;
            }
        }
    } else {
        last_ptr = &m_last;
        last_alloc_needed = true;
    }

    if (last_alloc_needed) {
        Arena *new_last = (Arena *) dAlloc (dOBSTACK_ARENA_SIZE);
        new_last->m_next = 0;
        *last_ptr = new_last;
        if (m_first == NULL) {
            m_first = new_last;
        }
        m_last = new_last;
        last_init_needed = true;
    }

    if (last_init_needed) {
        m_last->m_used = sizeof (Arena);
        ROUND_UP_OFFSET_TO_EFFICIENT_SIZE (m_last,m_last->m_used);
    }

    // allocate an area in the arena
    char *c = ((char*) m_last) + m_last->m_used;
    m_last->m_used += num_bytes;
    ROUND_UP_OFFSET_TO_EFFICIENT_SIZE (m_last,m_last->m_used);
    return c;
}

void dObStack::freeAll()
{
    Arena *current = m_first;
    m_last = current;
    // It is necessary to reset used sizes in whole arena chain
    // otherwise enumeration may proceed to remains of old deleted joints in unused arenas
    while (current) {
        current->m_used = sizeof(Arena);
        ROUND_UP_OFFSET_TO_EFFICIENT_SIZE (current,current->m_used);
        current = current->m_next;
    }
}

void *dObStack::rewind()
{
    return switch_to_arena(m_first);
}

void *dObStack::next (size_t num_bytes)
{
    // this functions like alloc, except that no new storage is ever allocated
    if (!m_current_arena) {
        return 0;
    }

    m_current_ofs += num_bytes;
    ROUND_UP_OFFSET_TO_EFFICIENT_SIZE (m_current_arena,m_current_ofs);

    if (m_current_ofs < m_current_arena->m_used) {
        return ((char*) m_current_arena) + m_current_ofs;
    }

    return switch_to_arena(m_current_arena->m_next);
}

void *dObStack::switch_to_arena(Arena *next_arena)
{
    m_current_arena = next_arena;
    if (!next_arena) {
        return 0;
    }
    m_current_ofs = sizeof (Arena);
    ROUND_UP_OFFSET_TO_EFFICIENT_SIZE (next_arena, m_current_ofs);
    // Check if end of allocation has been reached
    if (m_current_ofs >= next_arena->m_used) {
        return 0;
    }
    return ((char*) next_arena) + m_current_ofs;
}
