#!/bin/bash
# RC Tank ESP-IDF 프로젝트 설정 스크립트
# Bluepad32 + BTstack 소스를 클론하고 연동합니다.

set -e

PROJECT_DIR="$(cd "$(dirname "$0")" && pwd)"
COMPONENTS_DIR="${PROJECT_DIR}/components"

echo "=== RC Tank ESP-IDF 설정 ==="
echo "프로젝트 경로: ${PROJECT_DIR}"

# 1. components 디렉토리 생성
mkdir -p "${COMPONENTS_DIR}"

# 2. Bluepad32 클론
if [ -d "${COMPONENTS_DIR}/bluepad32" ]; then
    echo "Bluepad32 이미 존재함. git pull..."
    cd "${COMPONENTS_DIR}/bluepad32"
    git pull
else
    echo "Bluepad32 클론 중..."
    git clone --recursive https://github.com/ricardoquesada/bluepad32.git "${COMPONENTS_DIR}/bluepad32"
fi

# 3. BTstack 연동
echo "BTstack 연동 중..."
cd "${COMPONENTS_DIR}/bluepad32/external/btstack/port/esp32"

# IDF_PATH가 설정되어 있지 않으면 안내
if [ -z "$IDF_PATH" ]; then
    echo "WARNING: IDF_PATH가 설정되지 않았습니다."
    echo "다음 명령을 먼저 실행하세요:"
    echo "  . \$IDF_PATH/export.sh"
    echo "그 후 다시 이 스크립트를 실행하세요."
    exit 1
fi

# btstack을 bluepad32 소스 내에 설치 (권장)
python3 integrate_btstack.py

echo ""
echo "=== 설정 완료 ==="
echo "빌드 방법:"
echo "  cd ${PROJECT_DIR}"
echo "  idf.py set-target esp32-c3"
echo "  idf.py build"
echo "  idf.py flash monitor"
