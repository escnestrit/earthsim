@echo off
rem Join a friend's game: play_join.bat 192.168.1.10
if "%1"=="" ( echo usage: play_join.bat HOST_IP & exit /b 1 )
start "" "%~dp0Build\Windows\TrueTorque.exe" %1 -windowed -ResX=1600 -ResY=900
