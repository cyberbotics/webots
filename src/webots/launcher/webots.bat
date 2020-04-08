@echo off
setlocal
set Path=%CD%;%CD%\..\..\usr\bin;%Path%
start webots-bin.exe %*
