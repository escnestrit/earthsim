@echo off
rem Compiles the game code only (fast check). Output goes to compile.log next to this file.
setlocal
cd /d "%~dp0"
set "PROJ=%~dp0unreal\TrueTorque.uproject"
if exist "%~dp0TrueTorque\TrueTorque.uproject" set "PROJ=%~dp0TrueTorque\TrueTorque.uproject"
set UE=%UE_ROOT%
if "%UE%"=="" for /f "usebackq delims=" %%i in (`powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0find_ue.ps1"`) do set UE=%%i
if "%UE%"=="" if exist "D:\egs\Epic Games\nereal\UE_5.5\Engine\Build\BatchFiles\Build.bat" set "UE=D:\egs\Epic Games\nereal\UE_5.5"
if "%UE%"=="" powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0find_ue.ps1" > find_ue.log 2>&1
if "%UE%"=="" dir "D:\egs\Epic Games\nereal\UE_5.5\Engine\Build\BatchFiles" > find_ue_dir.log 2>&1
echo Unreal Engine: %UE% > compile.log
if "%UE%"=="" ( echo Unreal Engine 5 not found >> compile.log & echo Unreal Engine 5 not found & timeout /t 5 & exit /b 1 )
call "%UE%\Engine\Build\BatchFiles\Build.bat" TrueTorqueEditor Win64 Development -Project="%PROJ%" -WaitMutex -NoHotReload >> compile.log 2>&1
if errorlevel 1 ( echo COMPILE FAILED >> compile.log ) else ( echo COMPILE OK >> compile.log )
type compile.log | findstr /i /c:"error" /c:"COMPILE"
timeout /t 15
