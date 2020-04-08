@echo off
setlocal
set Path=%~dp0;%~dp0\..\..\usr\bin;%Path%
start /WAIT webots-bin.exe %*
