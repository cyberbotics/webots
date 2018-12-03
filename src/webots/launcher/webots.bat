@echo off
setlocal
set Path=%CD%\..\..\mingw64\bin;%CD%\..\..\usr\bin;%Path%
start webots.exe %*
