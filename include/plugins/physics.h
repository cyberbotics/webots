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

#ifndef PHYSICS_H
#define PHYSICS_H
#include <ode/ode.h>
#include <pthread.h>
#ifdef _WIN32
#include <windows.h>
#endif

/* callback functions to be implemented in your physics plugin */
#ifdef __cplusplus
extern "C" {
#endif
#ifdef _MSC_VER
#define DLLEXPORT __declspec(dllexport)
#else
#define DLLEXPORT
#endif
DLLEXPORT void webots_physics_init();
DLLEXPORT int  webots_physics_collide(dGeomID,dGeomID);
DLLEXPORT void webots_physics_step();
DLLEXPORT void webots_physics_step_end();
DLLEXPORT void webots_physics_cleanup();
DLLEXPORT void webots_physics_draw(int pass, const char *view);

/* utility functions to be used in your callback functions */
#ifndef __linux__
extern dGeomID (*dWebotsGetGeomFromDEFProc)(const char *);
extern dBodyID (*dWebotsGetBodyFromDEFProc)(const char *);
extern dJointGroupID (*dWebotsGetContactJointGroupProc)();
extern void    (*dWebotsSendProc)(int,const void *,int);
extern void*   (*dWebotsReceiveProc)(int *);
extern void    (*dWebotsConsolePrintfProc)(const char *, ...);
extern double  (*dWebotsGetTimeProc)();
#define dWebotsGetGeomFromDEF(defName)     (*dWebotsGetGeomFromDEFProc)(defName)
#define dWebotsGetBodyFromDEF(defName)     (*dWebotsGetBodyFromDEFProc)(defName)
#define dWebotsGetContactJointGroup()      (*dWebotsGetContactJointGroupProc)()
#define dWebotsSend(channel,buff,size)     (*dWebotsSendProc)(channel,buff,size)
#define dWebotsReceive(size_ptr)           (*dWebotsReceiveProc)(size_ptr)

#if defined(__VISUALC__) || defined (_MSC_VER) || defined(__BORLANDC__)
#define dWebotsConsolePrintf(format, ...) (*dWebotsConsolePrintfProc)(format, __VA_ARGS__)
#else
#define dWebotsConsolePrintf(format, ...) (*dWebotsConsolePrintfProc)(format, ## __VA_ARGS__)
#endif

#define dWebotsGetTime()                  (*dWebotsGetTimeProc)()
#else
dGeomID dWebotsGetGeomFromDEF(const char *defName);
dBodyID dWebotsGetBodyFromDEF(const char *defName);
dJointGroupID dWebotsGetContactJointGroup();
void          dWebotsSend(int channel,const void *buffer,int size);
const void   *dWebotsReceive(int *size);
void          dWebotsConsolePrintf(const char *format, ...);
double        dWebotsGetTime();
#endif
#ifdef __cplusplus
}
#endif

#endif /* PHYSICS_H */
