@echo off
rem Compiles the game code only (fast check). Output goes to compile.log next to this file.
setlocal
cd /d "%~dp0"
set UE=%UE_ROOT%
if "%UE%"=="" for /f "usebackq delims=" %%i in (`powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0find_ue.ps1"`) do set UE=%%i
echo Unreal Engine: %UE% > compile.log
if "%UE%"=="" ( echo Unreal Engine 5 not found >> compile.log & echo Unreal Engine 5 not found & pause & exit /b 1 )
call "%UE%\Engine\Build\BatchFiles\Build.bat" TrueTorqueEditor Win64 Development -Project="%~dp0unreal\TrueTorque.uproject" -WaitMutex -NoHotReload >> compile.log 2>&1
if errorlevel 1 ( echo COMPILE FAILED >> compile.log ) else ( echo COMPILE OK >> compile.log )
type compile.log | findstr /i /c:"error" /c:"COMPILE"
timeout /t 15
