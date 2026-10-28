@echo off
rem Streams real OpenStreetMap data around Minsk (needs internet). Cache is kept in the "cache" folder next to the exe.
earthsim.exe
if errorlevel 1 pause
