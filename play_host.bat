@echo off
rem Start a game others can join (listen server).
start "" "%~dp0Build\Windows\TrueTorque.exe" /Engine/Maps/Entry?listen -windowed -ResX=1600 -ResY=900
