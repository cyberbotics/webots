/***************************************************************/
/* This file is common to all physics plugin projects.         */
/* It is included automatically by the Makefile.include and    */
/* should be included in other non make-based projects         */
/***************************************************************/

#include <plugins/physics.h>
#include <stdarg.h>
#include <stdio.h>
#ifdef __cplusplus
extern "C" {
#endif

/* The rest of this file is necessary to make the plugin work under */
/* Windows and macOS */
#ifndef __linux__

dGeomID (*dWebotsGetGeomFromDEFProc)(const char *) = NULL;
dBodyID (*dWebotsGetBodyFromDEFProc)(const char *) = NULL;
dJointGroupID (*dWebotsGetContactJointGroupProc)() = NULL;
void (*dWebotsSendProc)(int, const void *, int) = NULL;
void *(*dWebotsReceiveProc)(int *) = NULL;
void (*dWebotsConsolePrintfProc)(const char *, ...) = NULL;
double (*dWebotsGetTimeProc)(void) = NULL;

DLLEXPORT void physics_special_init(dGeomID (*dWebotsGetGeomFromDEFProcA)(const char *),
                                    dBodyID (*dWebotsGetBodyFromDEFProcA)(const char *),
                                    dJointGroupID (*dWebotsGetContactJointGroupProcA)(),
                                    void (*dWebotsSendProcA)(int, const void *, int), void *(*dWebotsReceiveProcA)(int *),
                                    void (*dWebotsConsolePrintfProcA)(const char *, ...), double (*dWebotsGetTimeProcA)(void)) {
  dWebotsGetGeomFromDEFProc = dWebotsGetGeomFromDEFProcA;
  dWebotsGetBodyFromDEFProc = dWebotsGetBodyFromDEFProcA;
  dWebotsGetContactJointGroupProc = dWebotsGetContactJointGroupProcA;
  dWebotsSendProc = dWebotsSendProcA;
  dWebotsReceiveProc = dWebotsReceiveProcA;
  dWebotsConsolePrintfProc = dWebotsConsolePrintfProcA;
  dWebotsGetTimeProc = dWebotsGetTimeProcA;
}
#endif

#ifdef __cplusplus
}
#endif
