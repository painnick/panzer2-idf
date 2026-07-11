# panzer2-idf

ESP32-C3 기반 RC 탱크 펌웨어입니다.  
BLE 게임패드로 **캐터필러 주행**, **터렛 회전**, **포/기관총 효과**, **DFPlayer 효과음**을 제어합니다.

Arduino/PlatformIO 프로젝트 **M3Stuart_ESP32C3** 를 ESP-IDF v5.5.2 로 포팅한 버전이며, 입력은 [Bluepad32](https://github.com/ricardoquesada/bluepad32) + BTstack 를 사용합니다.

## 기능

| 기능 | 설명 |
|------|------|
| 트랙 구동 | DRV8833, 좌/우 스틱 Y축 — panzer4식 가속/감속 램프(~1초 최대) |
| 터렛 | SG90 서보 — 패드 연결 시 attach, D-Pad 좌우 회전, 3초 무입력/해제 시 detach |
| 포신 | B 버튼 — LED·효과음 후 트랙 후진 리코일 |
| 게틀링(기관총) | A 버튼 — 게틀링 LED 깜빡임 + 효과음 |
| 볼륨 | L1 / R1, NVS에 저장 |
| 사운드 | DFPlayer Mini (대기 / 포 / 기관총 / 연결음) |

## 하드웨어

- **MCU**: ESP32-C3 (BLE only — Classic BT 미지원)
- **모터 드라이버**: DRV8833
- **서보**: SG90 (터렛)
- **오디오**: DFPlayer Mini + microSD
- **입력**: BLE HID 게임패드 (검증: GamePadPlus V3 / ShanWan BM-769)

### 핀맵

| 기능 | GPIO |
|------|------|
| 좌측 트랙 IN1 / IN2 | 4 / 3 |
| 우측 트랙 IN1 / IN2 | 0 / 5 |
| 포신 LED | 1 |
| 게틀링 LED | 6 |
| 터렛 서보 | 7 |
| DFPlayer TX / RX | 10 / 9 |

> GPIO20/21은 ESP32-C3 기본 UART0 콘솔용입니다. DFPlayer RX는 **GPIO9**를 사용합니다.

### PCB

회로 개략도는 `pcb/` 에 있습니다.

| 파일 | 설명 |
|------|------|
| `pcb/Schematic_M3-Stuart_2026-07-11.png` | M3 Stuart 스케매틱 |

### 3D 출력 (`3dp/`)

기구부 STEP 모델입니다. 슬라이서에서 열어 STL 등으로 변환해 출력하면 됩니다.

| 파일 | 설명 |
|------|------|
| `Idler.step` | 아이들러 |
| `Motor Guide 2mm.step` | 모터 가이드 2mm |
| `Motor Guide 3mm D.step` | 모터 가이드 3mm |
| `PCB 3.2.step` | PCB 브라켓 |
| `Sproket Guide.step` | 스프로킷 가이드 |
| `Turret.step` | 터렛 |

### DFPlayer 트랙 (SD 카드)

| 파일 | 용도 |
|------|------|
| `0001.mp3` | 대기 루프 |
| `0002.mp3` | 포신 발사 |
| `0003.mp3` | 기관총 |
| `0004.mp3` | 게임패드 연결 |

## 조작

| 입력 | 동작 |
|------|------|
| 좌 스틱 Y | 좌측 트랙 전/후진 (즉시 최대 아님, ~1초 램프 가속) |
| 우 스틱 Y | 우측 트랙 전/후진 (동일) |
| D-Pad ← / → | 터렛 좌/우 회전 (0°~180°, 약 120ms마다 1°) |
| Start | 터렛 서보 중앙(90°) |
| A | 게틀링(기관총) — LED 75ms 깜빡임, 약 0.5초 |
| B | 포신 — LED·효과음 후 리코일(후진) |
| L1 / R1 | 볼륨 감소 / 증가 |

### 터렛 서보

- **부팅**: 서보 미연결 (PWM 없음)
- **게임패드 연결 시**: 서보 attach 후 **중앙(90°)** 설정
- **게임패드 해제 시**: 서보 detach
- **회전 속도**: `TURRET_STEP_INTERVAL_MS`(기본 120ms)마다 1° — 전체 0°↔180°에 약 22초
- **무입력 해제**: D-Pad 터렛 입력이 `TURRET_IDLE_DISCONNECT_MS`(기본 **3초**) 없으면 detach (버즈·홀딩 전류 감소)
- **재연결**: D-Pad 좌/우로 각도를 바꿀 때 다시 attach

### 포 발사 시퀀스 (B)

1. 포신 LED ON + 포 효과음 재생
2. `RECOIL_DELAY_MS`(기본 250ms) 대기
3. 트랙 후진 리코일 (`RECOIL_BACK_DURATION` 40ms) → 정지 안정화 (`RECOIL_SETTLE_DURATION` 40ms)

스틱 전진이 음수 축이므로 리코일 후진은 양수 모터 속도(`RECOIL_BACK_SPEED`)를 사용합니다. 리코일 중에는 스틱 모터 입력을 잠시 무시합니다.

## 게임패드 연결 (중요)

ESP32-C3는 **BLE만** 지원합니다. DualShock 등 Classic BT 전용 패드는 연결되지 않습니다.

### GamePadPlus V3 / Terios T3 / ShanWan BM-769

1. 패드를 끈 뒤, **`Home + X`** 를 약 3초 눌러 페어링 모드로 진입 (표준 Android BLE HID)
2. 펌웨어 부팅 후 자동 스캔·연결
3. 성공 시 로그 예: `Gamepad ready (...), reports=2` → `게임패드 연결됨`

`Home + A` / `Home + Y` 모드는 BLE여도 HID 데이터 형식이 달라 동작하지 않을 수 있습니다.

### 연결 구현 메모

일부 저가 BLE 패드는 BTstack 기본 `hids_client` 경로에서 HID 설정이 멈추는 문제가 있습니다.  
이 프로젝트는 **Simple HOG** 경로를 사용합니다.

- GATT 서비스 탐색 → HID(0x1812) 특성 탐색
- Report Map 읽기
- CCC 직접 쓰기 + 알림 수신
- Report ID 프리펜드 후 Bluepad32 Android 파서로 입력 처리

부팅 시 본딩 키를 지우지 않습니다. 강제 재페어링이 필요하면 콘솔에서 키 삭제 명령을 사용하세요.

## 소프트웨어 구조

```
main/
  main.c           # 모터/서보/LED, 게임패드 처리, 제어 루프
  my_platform.c    # Bluepad32 커스텀 플랫폼, 공유 입력 상태
  dfplayer.c/.h    # DFPlayer Mini UART 제어
components/
  bluepad32/       # Bluepad32 (커스텀 Simple HOG 포함)
  btstack/         # BTstack ESP32 포트
```

- **bt_main**: BTstack 런 루프 (우선순위 높음)
- **tank_ctrl**: 10ms 주기로 게임패드 입력 → 액추에이터 반영

## 빌드 & 플래시

### 요구 사항

- [ESP-IDF v5.5.x](https://docs.espressif.com/projects/esp-idf/en/v5.5.2/esp32c3/get-started/index.html)
- 타깃: `esp32c3`

### 환경 설정 (Windows 예)

```bat
REM ESP-IDF export (경로는 설치에 맞게 수정)
call C:\Espressif\frameworks\esp-idf-v5.5.2\export.bat

cd panzer2-idf
idf.py set-target esp32c3
idf.py build
idf.py -p COMx flash monitor
```

`setup.bat` / `setup.sh` 로 Bluepad32·BTstack 의존성을 준비할 수 있습니다 (이미 `components/` 가 포함되어 있으면 생략 가능).

### 주요 sdkconfig

- BLE 컨트롤러 only (`CONFIG_BT_CONTROLLER_ONLY`)
- Wi-Fi 비활성
- Bluepad32 커스텀 플랫폼
- 기본 로그 레벨: INFO (상세 BLE 단계는 DEBUG)

## 라이선스 관련

- 애플리케이션 코드: 프로젝트 정책에 따름
- **Bluepad32**: Apache-2.0
- **BTstack**: 비상업 평가 가능, 상업 사용 시 [BlueKitchen](https://bluekitchen-gmbh.com/) 라이선스 필요

## 참고

- [Bluepad32 문서](https://bluepad32.readthedocs.io/)
- [ESP-IDF](https://docs.espressif.com/projects/esp-idf/)
- 원본: M3Stuart_ESP32C3 (Arduino/PlatformIO)
