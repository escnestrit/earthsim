@echo off
rem No internet needed: streams the bundled sample dataset instead (same code path as the real thing).
earthsim.exe --offline
if errorlevel 1 pause
