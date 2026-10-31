@echo off
rem Builds True Torque into .\Build\Windows (needs Unreal Engine 5.5 + Visual Studio 2022 with C++ game development).
setlocal
set UE=%UE_ROOT%
if "%UE%"=="" set UE=C:\Program Files\Epic Games\UE_5.5
if not exist "%UE%\Engine\Build\BatchFiles\RunUAT.bat" (
  echo Unreal Engine 5.5 not found. Set UE_ROOT to the engine folder, e.g. set UE_ROOT=D:\Epic\UE_5.5
  exit /b 1
)
call "%UE%\Engine\Build\BatchFiles\RunUAT.bat" BuildCookRun -project="%~dp0unreal\TrueTorque.uproject" -noP4 -platform=Win64 -clientconfig=Development -serverconfig=Development -build -cook -stage -pak -archive -archivedirectory="%~dp0Build" -utf8output
if errorlevel 1 (
  echo BUILD FAILED - send the last screen of messages to Claude
  exit /b 1
)
echo Done: %~dp0Build\Windows\TrueTorque.exe
