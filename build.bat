@echo off
rem Builds the playable game into .\Build\Windows. Output goes to build.log next to this file.
setlocal
cd /d "%~dp0"
set UE=%UE_ROOT%
if "%UE%"=="" for /f "usebackq delims=" %%i in (`powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0find_ue.ps1"`) do set UE=%%i
echo Unreal Engine: %UE% > build.log
if "%UE%"=="" ( echo Unreal Engine 5 not found >> build.log & echo Unreal Engine 5 not found & pause & exit /b 1 )
call "%UE%\Engine\Build\BatchFiles\RunUAT.bat" BuildCookRun -project="%~dp0unreal\TrueTorque.uproject" -noP4 -platform=Win64 -clientconfig=Development -build -cook -stage -pak -archive -archivedirectory="%~dp0Build" -utf8output >> build.log 2>&1
if errorlevel 1 ( echo BUILD FAILED >> build.log ) else ( echo BUILD OK >> build.log )
type build.log | findstr /i /c:"error" /c:"BUILD OK" /c:"BUILD FAILED"
timeout /t 15
