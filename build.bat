@echo off
rem Builds the playable game into .\Build\Windows. Output goes to build.log next to this file.
setlocal
cd /d "%~dp0"
set "PROJ=%~dp0unreal\TrueTorque.uproject"
if exist "%~dp0TrueTorque\TrueTorque.uproject" set "PROJ=%~dp0TrueTorque\TrueTorque.uproject"
set UE=%UE_ROOT%
if "%UE%"=="" for /f "usebackq delims=" %%i in (`powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0find_ue.ps1"`) do set UE=%%i
if "%UE%"=="" if exist "D:\egs\Epic Games\nereal\UE_5.5\Engine\Build\BatchFiles\Build.bat" set "UE=D:\egs\Epic Games\nereal\UE_5.5"
if "%UE%"=="" powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0find_ue.ps1" > find_ue.log 2>&1
if "%UE%"=="" dir "D:\egs\Epic Games\nereal\UE_5.5\Engine\Build\BatchFiles" > find_ue_dir.log 2>&1
echo Unreal Engine: %UE% > build.log
if "%UE%"=="" ( echo Unreal Engine 5 not found >> build.log & echo Unreal Engine 5 not found & timeout /t 5 & exit /b 1 )
call "%UE%\Engine\Build\BatchFiles\RunUAT.bat" BuildCookRun -project="%PROJ%" -noP4 -platform=Win64 -clientconfig=Development -build -cook -stage -pak -archive -archivedirectory="%~dp0Build" -utf8output >> build.log 2>&1
if errorlevel 1 ( echo BUILD FAILED >> build.log ) else ( echo BUILD OK >> build.log )
type build.log | findstr /i /c:"error" /c:"BUILD OK" /c:"BUILD FAILED"
timeout /t 15
