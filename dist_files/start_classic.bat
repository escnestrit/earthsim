@echo off
rem The original OpenGL version (fallback if the new one does not start on this PC).
earthsim-classic.exe
if errorlevel 1 pause
