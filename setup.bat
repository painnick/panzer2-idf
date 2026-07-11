@echo off
setlocal EnableDelayedExpansion

echo === RC Tank ESP-IDF setup ===

REM components 디렉토리 생성
if not exist "components" mkdir "components"

REM Bluepad32 클론
if exist "components\bluepad32" (
    echo Bluepad32 already exists. git pull...
    cd /d "components\bluepad32"
    git pull
) else (
    echo Cloning bluepad32...
    git clone --recursive https://github.com/ricardoquesada/bluepad32.git "components\bluepad32"
    if errorlevel 1 (
        echo ERROR: clone failed
        exit /b 1
    )
)

REM BTstack 연동
echo Integrating BTstack...
cd /d "components\bluepad32\external\btstack\port\esp32"

if "%IDF_PATH%"=="" (
    echo WARNING: IDF_PATH not set. Run env.bat first.
    exit /b 1
)

python integrate_btstack.py
if errorlevel 1 (
    echo ERROR: btstack integration failed
    exit /b 1
)

cd /d "%~dp0"

echo.
echo === Done ===
echo Build:
echo   idf.py set-target esp32-c3
echo   idf.py build
echo   idf.py flash monitor

endlocal
