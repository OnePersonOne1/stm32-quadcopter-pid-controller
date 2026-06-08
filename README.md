# STM32 Quadcopter Attitude Stabilization (PID)

> STM32F103C8T6(Blue Pill) 기반 쿼드콥터 자세 안정화 비행제어 펌웨어
> 인하대학교 항공우주공학실험1 · 6조 텀프로젝트

MPU6050 IMU로 측정한 자세를 **칼만 필터**로 추정하고, **PID 제어기**로 roll/pitch/yaw를 보정하여
4개 모터의 PWM duty를 실시간으로 믹싱하는 베어메탈 임베디드 비행제어 시스템입니다.

---

## 1. 개요

| 항목 | 내용 |
|---|---|
| MCU | STM32F103C8T6 (Cortex-M3, 72 MHz) "Blue Pill" |
| IMU | MPU6050 (3축 가속도 + 3축 자이로), I²C |
| 액추에이터 | BLDC 모터 4기 + ESC (TIM1 4채널 PWM) |
| 자세 추정 | 가속도/자이로 융합 → **Kalman Filter** (roll/pitch) |
| 제어 | roll·pitch·yaw **PID**, 목표 자세 = 수평(0°) |
| 모터 믹싱 | **+형(X) 쿼드콥터** 믹서 |
| 통신 | UART 명령어 인터페이스 (실시간 게인/추력 튜닝 + 텔레메트리) |
| 펌웨어 구조 | STM32 HAL + CMake/Ninja + ARM GCC, 베어메탈 (RTOS 미사용) |

## 2. 제어 시스템 구조

```
                 ┌─────────────┐   accel/gyro    ┌──────────────┐
   MPU6050 ──I²C─▶│  Read_All   │────────────────▶│ Kalman Filter │── roll, pitch
                 └─────────────┘                  └──────────────┘
                                                         │
   target = 0° (수평) ──────────────┐                    ▼
                                    ▼              ┌──────────────┐
                            ┌──────────────┐       │  PID (R/P/Y) │── uRoll, uPitch, uYaw
                            │   err = sp-mv│──────▶│ Kp·e+Ki∫e+Kd·ė│
                            └──────────────┘       └──────────────┘
                                                         │
                                                         ▼
                          ┌───────────────────────────────────────────┐
                          │  Motor Mixer  (+X quad)                     │
                          │  M = baseThrust ± uPitch ± uRoll ± uYaw     │
                          └───────────────────────────────────────────┘
                                                         │
                                          TIM1 CH1..CH4 PWM (duty %)
                                                         ▼
                                              ESC × 4  →  Motor × 4
```

- **자세 추정**: `MPU6050_Read_All()`이 가속도로부터 기하학적 각도를 구하고, 자이로 각속도와 칼만 필터로 융합해 roll/pitch를 추정합니다. yaw는 자이로 적분으로 처리합니다.
- **PID**: `pid_update()`가 오차의 비례·적분·미분 항을 합산해 보정 입력을 생성합니다.
- **믹싱**: `setMotorDuty()`가 보정 입력을 4개 모터 duty(0–100%)로 변환하여 `TIM1->CCRx`에 기록합니다 (ARR 기준 틱 변환).

## 3. 모터 배치 & 믹싱

`+`형 쿼드콥터 기준 모터 매핑 (TIM1 채널 → 핀):

| 채널 | 핀 | 위치 | 회전 |
|---|---|---|---|
| CH1 | PA8 | Rear-Left (M0) | CW |
| CH2 | PA9 | Front-Right (M1) | CW |
| CH3 | PA10 | Rear-Right (M2) | CCW |
| CH4 | PA11 | Front-Left (M3) | CCW |

믹싱 식 (PID 모드):
```
M0 = base + uPitch + uRoll - uYaw
M1 = base - uPitch - uRoll - uYaw
M2 = base + uPitch - uRoll + uYaw
M3 = base - uPitch + uRoll + uYaw
```

## 4. UART 명령어 인터페이스

USART2(PA2/PA3)로 실시간 제어/튜닝이 가능합니다. 줄바꿈(CR/LF)으로 명령을 구분하며, 모든 명령은 에코됩니다.
주기적 텔레메트리로 `R/P/Y` 각도와 4개 모터 CCR 값을 출력합니다.

| 명령 | 동작 |
|---|---|
| `0` `25` `50` `75` `100` | 기본 추력(base thrust) % 설정 |
| `mode1` | 모든 모터 동일 추력 (PID off) — 정지/스로틀 테스트 |
| `pid` | PID 자세 안정화 모드 |
| `m0`~`m3` | 개별 모터 단독 구동 (배선/회전방향 검증) |
| `reset` | 적분된 yaw(roll) 상태 초기화 |

> 단계적 검증 흐름: `m0~m3`로 개별 모터·배선 확인 → `mode1`으로 균형 추력 확인 → `pid`로 자세 안정화 활성화.

## 5. 빌드 & 개발 환경

### Toolchain
- STM32CubeMX v6.14.1 / STM32CubeCLT v1.18.0 / STMCUFinder v6.1.0
- Visual Studio Code + `STM32Cube for VS Code` 확장
- [CMake](https://cmake.org/download/) + [Ninja](https://github.com/ninja-build/ninja/releases) (PATH 등록 필요)

### CubeMX 핵심 설정
- `SYS` → Debug → Serial Wire
- `RCC` → HSE(Crystal/Ceramic Resonator), PLL→HSE, SYSCLK→PLLCLK, **HCLK 72 MHz**
- Project → Toolchain/IDE → **CMake**, peripheral 초기화는 `.c/.h` 페어로 생성

### 빌드
```bash
# VS Code: STM32Cube 확장에서 Import CMake project → 폴더 선택
# CMake: Configure → Debug 선택 후 빌드
cmake --preset Debug
cmake --build build/Debug
# 산출물: build/Debug/blank.elf  → ST-Link 등으로 플래시
```

## 6. 데모 & 자료

- 🎥 비행/제어 테스트 영상: [`docs/`](docs/) (`6조_termproject_test1.mp4`, `6조_termproject_test2.mp4`)
- 📄 프로젝트 보고서: [`docs/6조_termproject_보고서.docx`](docs/)
- 🔌 배선도: [`diagram/wiring.drawio.svg`](diagram/wiring.drawio.svg)

## 7. 프로젝트 구조

```
.
├── Core/
│   ├── Inc/            # 헤더 (main, mpu6050, tim, usart, i2c, gpio)
│   └── Src/
│       ├── main.c      # 메인 루프: 센서→칼만→PID→믹서, PID/모터 정의
│       ├── mpu6050.c   # MPU6050 드라이버 + 칼만 필터
│       ├── usart.c     # UART 명령 파서 / 텔레메트리
│       └── tim.c, i2c.c, gpio.c ...
├── Drivers/            # STM32F1xx HAL & CMSIS
├── cmake/              # ARM GCC toolchain & CubeMX CMake
├── diagram/            # 배선도(SVG)
├── docs/               # 보고서 + 데모 영상
├── blank.ioc           # STM32CubeMX 프로젝트 파일
└── CMakeLists.txt
```

## 8. 핵심 구현 포인트

- **센서 융합**: 가속도(저주파 정확)와 자이로(고주파 정확)를 칼만 필터로 결합해 드리프트/노이즈를 동시에 억제.
- **실시간 제어 루프**: HAL SysTick 기반 dt 계산으로 PID의 적분·미분 항을 시간 정규화.
- **하드웨어 PWM 믹싱**: 타이머 CCR 직접 제어로 4개 모터 duty를 ARR 기준 틱으로 변환.
- **인터랙티브 튜닝**: 펌웨어 재빌드 없이 UART로 추력/모드를 바꿔가며 게인을 검증할 수 있는 워크플로우 설계.

## 9. 팀 (인하대 항공우주공학실험1 6조)

한명일, 외 6조 팀원. 본 저장소는 팀 텀프로젝트의 펌웨어 파트를 정리한 것입니다.

> MPU6050 드라이버 및 Kalman 필터 구현은 [Bulanov Konstantin (leech001)](https://github.com/leech001/MPU6050) 의 오픈소스(GPLv3)를 기반으로 합니다.
