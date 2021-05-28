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

/* Library private error handling functions and macros */

#ifndef _ODE__PRIVATE_ERROR_H_
#define _ODE__PRIVATE_ERROR_H_

#include <ode/error.h>

/* debugging:
 *   IASSERT  is an internal assertion, i.e. a consistency check. if it fails
 *            we want to know where.
 *   UASSERT  is a user assertion, i.e. if it fails a nice error message
 *            should be printed for the user.
 *   AASSERT  is an arguments assertion, i.e. if it fails "bad argument(s)"
 *            is printed.
 *   DEBUGMSG just prints out a message
 */

#  if defined(__STDC__) && __STDC_VERSION__ >= 199901L
#    define __FUNCTION__ __func__
#  endif
#ifndef dNODEBUG
#  ifdef __GNUC__
#    define dIASSERT(a) { if (!(a)) { dDebug (d_ERR_IASSERT, \
      "assertion \"" #a "\" failed in %s() [%s:%u]",__FUNCTION__,__FILE__,__LINE__); } }
#    define dUASSERT(a,msg) { if (!(a)) { dDebug (d_ERR_UASSERT, \
      msg " in %s()", __FUNCTION__); } }
#    define dDEBUGMSG(msg) { dMessage (d_ERR_UASSERT,				\
  msg " in %s() [%s:%u]", __FUNCTION__,__FILE__,__LINE__); }
#  else // not __GNUC__
#    define dIASSERT(a) { if (!(a)) { dDebug (d_ERR_IASSERT, \
      "assertion \"" #a "\" failed in %s:%u",__FILE__,__LINE__); } }
#    define dUASSERT(a,msg) { if (!(a)) { dDebug (d_ERR_UASSERT, \
      msg " (%s:%u)", __FILE__,__LINE__); } }
#    define dDEBUGMSG(msg) { dMessage (d_ERR_UASSERT, \
      msg " (%s:%u)", __FILE__,__LINE__); }
#  endif
#  define dIVERIFY(a) dIASSERT(a)
#  define dUVERIFY(a, msg) dUASSERT(a, msg)
#else
#  define dIASSERT(a) ((void)0)
#  define dUASSERT(a,msg) ((void)0)
#  define dDEBUGMSG(msg) ((void)0)
#  define dIVERIFY(a) ((void)(a))
#  define dUVERIFY(a, msg) ((void)(a))
#endif

#  ifdef __GNUC__
#    define dICHECK(a) { if (!(a)) { dDebug (d_ERR_IASSERT, \
      "assertion \"" #a "\" failed in %s() [%s:%u]",__FUNCTION__,__FILE__,__LINE__); *(int *)0 = 0; } }
#  else // not __GNUC__
#    define dICHECK(a) { if (!(a)) { dDebug (d_ERR_IASSERT, \
      "assertion \"" #a "\" failed in %s:%u",__FILE__,__LINE__); *(int *)0 = 0; } }
#  endif

// Argument assert is a special case of user assert
#define dAASSERT(a) dUASSERT(a,"Bad argument(s)")
#define dSASSERT(e)  static_assert(e, #e)

#endif
