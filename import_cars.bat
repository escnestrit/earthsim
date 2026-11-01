@echo off
rem Imports car models from CarModels\<Car>\<Category>\<Variant>\*.fbx|obj into the game (editor closed).
setlocal
cd /d "%~dp0"
set "PROJ=%~dp0unreal\TrueTorque.uproject"
if exist "%~dp0TrueTorque\TrueTorque.uproject" set "PROJ=%~dp0TrueTorque\TrueTorque.uproject"
set UE=%UE_ROOT%
if "%UE%"=="" for /f "usebackq delims=" %%i in (`powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0find_ue.ps1"`) do set UE=%%i
if "%UE%"=="" if exist "D:\egs\Epic Games\nereal\UE_5.5\Engine\Build\BatchFiles\Build.bat" set "UE=D:\egs\Epic Games\nereal\UE_5.5"
if "%UE%"=="" powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0find_ue.ps1" > find_ue.log 2>&1
if "%UE%"=="" dir "D:\egs\Epic Games\nereal\UE_5.5\Engine\Build\BatchFiles" > find_ue_dir.log 2>&1
echo Unreal Engine: %UE% > import_cars.log
if "%TT_CARS%"=="" if exist "%~dp0CarModels" set "TT_CARS=%~dp0CarModels"
set "SCRIPT=%~dp0unreal\Tools\import_cars.py"
if exist "%~dp0TrueTorque\Tools\import_cars.py" set "SCRIPT=%~dp0TrueTorque\Tools\import_cars.py"
"%UE%\Engine\Binaries\Win64\UnrealEditor-Cmd.exe" "%PROJ%" -run=pythonscript -script="%SCRIPT%" -unattended -nop4 -nosplash -stdout >> import_cars.log 2>&1
findstr /c:"TT_CARS_DONE" /c:"TT import_cars" import_cars.log
timeout /t 15
