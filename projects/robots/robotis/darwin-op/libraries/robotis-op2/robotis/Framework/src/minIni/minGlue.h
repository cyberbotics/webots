/*  Glue functions for the minIni library, based on the C/C++ stdio library
 *
 *  Or better said: this file contains macros that maps the function interface
 *  used by minIni to the standard C/C++ file I/O functions.
 *
 *  Copyright (c) ITB CompuPhase, 2008-2010
 *
 *  Licensed under the Apache License, Version 2.0 (the "License"); you may not
 *  use this file except in compliance with the License. You may obtain a copy
 *  of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 *  WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 *  License for the specific language governing permissions and limitations
 *  under the License.
 *
 *  Version: $Id: minGlue.h 29 2010-07-06 13:38:05Z thiadmer.riemersma $
 */

/* map required file I/O to the standard C library */
#include <stdio.h>
#define ini_openread(filename,file)   ((*(file) = fopen((filename),"rt")) != NULL)
#define ini_openwrite(filename,file)  ((*(file) = fopen((filename),"wt")) != NULL)
#define ini_close(file)               fclose(*(file))
#define ini_read(buffer,size,file)    fgets((buffer),(size),*(file))
#define ini_write(buffer,file)        fputs((buffer),*(file))
#define ini_rename(source,dest)       rename((source),(dest))
#define ini_remove(filename)          remove(filename)
#define ini_rewind(file)              rewind(*(file))
