#                                               -*- Autoconf -*-
# Process this file with autoconf to produce a configure script.

#AC_PREREQ([2.65])
AC_INIT([libccd], [1.0], [danfis@danfis.cz])
AC_CONFIG_SRCDIR([src/ccd.c])
AC_CONFIG_HEADERS([src/config.h])
AM_INIT_AUTOMAKE

# Checks for programs.
AC_PROG_CXX
AC_PROG_CC
AC_PROG_INSTALL
AC_DISABLE_SHARED
LT_INIT

# Checks for libraries.
AC_CHECK_LIB([m], [main])
# FIXME: Replace `main' with a function in `-lrt':
AC_CHECK_LIB([rt], [main])

# Checks for header files.
AC_CHECK_HEADERS([float.h stdlib.h string.h unistd.h])

# Checks for typedefs, structures, and compiler characteristics.
AC_TYPE_SIZE_T

# Checks for library functions.
AC_FUNC_FORK
AC_CHECK_FUNCS([clock_gettime])

use_double=no
AC_ARG_ENABLE(double-precision,
              AS_HELP_STRING([--enable-double-precision],
                             [enable double precision computations instead of single precision]),
              [use_double=$enableval])
if test x$use_double = xno
then
        CCD_PRECISION=CCD_SINGLE
else
        CCD_PRECISION=CCD_DOUBLE
fi
AC_SUBST(CCD_PRECISION)

AC_CONFIG_FILES([Makefile
                 src/Makefile
                 src/ccd/precision.h
                 src/testsuites/Makefile
                 src/testsuites/cu/Makefile])
AC_OUTPUT
