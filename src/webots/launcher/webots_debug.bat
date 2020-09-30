:: script running gdb on webots in a DOS console
:: author: fabien.rohrer@cyberbotics.com

@ECHO off

ECHO Setup environment variables...
SET PWD=%~d0
SET WEBOTS_HOME=%~dp0\..\..\..
SET PATH=%PWD%\msys64\mingw64\bin;%PWD%\msys64\usr\bin;%PATH%

ECHO Generate the gdb input file...
SET INPUT=%WEBOTS_HOME%\webots_debug_input.txt
IF EXIST %INPUT% (
  DEL %INPUT%
)
ECHO run >> %INPUT%
ECHO where >> %INPUT%
ECHO quit >> %INPUT%

ECHO Delete the gdb output file if exists
SET OUTPUT=%WEBOTS_HOME%\webots_debug_output.txt
IF EXIST %OUTPUT% (
  DEL %OUTPUT%
)

ECHO Starting Webots in debug mode...
gdb.exe webots-bin.exe < %INPUT% >> %OUTPUT%

ECHO Delete the gdb input file...
DEL %INPUT%
