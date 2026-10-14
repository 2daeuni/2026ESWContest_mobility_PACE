# Haptic ECU

**PACE — 페달 오조작 예방을 위한 선제적 위험 인지 및 운전자 행동 교정 시스템**

제24회 임베디드SW경진대회 자동차·모빌리티 부문 PACE 프로젝트의 Haptic ECU 펌웨어입니다.

STM32F446RE 기반 Haptic ECU는 Raspberry Pi Safety ECU에서 전송하는 CAN 명령을 수신하고, USB HID 통신을 통해 SIMAGIC P2000 HCB 컨트롤러와 P-HPR 햅틱 모터를 제어합니다.

운전자에게 위험 단계에 따른 진동 피드백을 제공하여 페달 오조작 위험을 인지하고, 스스로 조작을 수정하도록 유도하는 역할을 담당합니다.

---

## 1. 시스템 구성

```text
Raspberry Pi 5 (Safety ECU)
        |
        | CAN 0x110 HAPTIC_COMMAND
        | 500 kbps
        v
STM32F446RE (Haptic ECU)
        |
        | USB HID Feature Report
        v
SIMAGIC P2000 HCB
        |
        v
SIMAGIC P-HPR Haptic Motor


STM32F446RE (Haptic ECU)
        |
        | CAN 0x200 HAPTIC_STATUS
        | 10 ms
        v
Raspberry Pi 5 (Safety ECU)
```

Haptic ECU는 CAN으로 수신한 위험 단계 및 햅틱 제어 명령을 USB HID 제어 데이터로 변환하고, 현재 상태와 오류 정보를 CAN으로 Safety ECU에 전달합니다.

## 2. 개발 환경

| 항목 | 내용 |
|---|---|
| MCU | STM32F446RE (NUCLEO-F446RE) |
| 개발 환경 | STM32CubeIDE |
| 개발 언어 | C |
| MCU Framework | STM32 HAL |
| CAN Interface | CAN1 (Classic CAN) |
| CAN Bitrate | 500 kbps |
| USB Interface | USB Host / HID |
| Haptic Controller | SIMAGIC P2000 HCB |
| Haptic Motor | SIMAGIC P-HPR |
| Debug UART | USART2, 115200 bps |

## 3. 주요 기능

### 3.1 CAN 명령 수신

- `HAPTIC_COMMAND (0x110)` 수신
- Risk Level 0~3 처리
- 진동 ON/OFF 명령 처리
- 진동 세기 및 주파수 설정
- CAN ID, Frame Type, DLC 검증
- 명령 데이터 및 Reserved Byte 유효성 검사
- Alive Counter 중복 검사
- CAN 명령 Timeout 감지

### 3.2 Haptic Motor 제어

- USB HID Feature Report 기반 P2000 HCB 제어
- 진동 세기: 1~100%
- 진동 주파수: 10~50Hz
- 진동 ON/OFF 제어
- USB 연결 상태 감시
- USB 전송 성공 시 내부 적용 상태 갱신

### 3.3 CAN 상태 송신

- `HAPTIC_STATUS (0x200)` 송신
- 10ms 주기 상태 메시지 생성
- 현재 Risk Level 및 진동 상태 전송
- 적용된 진동 세기·주파수 전송
- Fault Code 및 Command Timeout 상태 전송
- ECU Status 및 Alive Counter 전송

### 3.4 Fail-Safe

- 비정상 CAN 명령 수신 시 Haptic OFF 요청
- Alive Counter 중복 시 통신 오류 처리
- 100ms CAN 명령 Timeout 감지
- USB 연결 해제 및 드라이버 오류 감지
- 시작 시 Haptic OFF 요청
- USB 통신 중 BUSY 상태에서 재시도

---

## 4. CAN 통신

### 4.1 CAN 설정

| 항목 | 설정값 |
|---|---|
| CAN Controller | CAN1 |
| Bitrate | 500 kbps |
| Prescaler | 6 |
| BS1 | 11 TQ |
| BS2 | 2 TQ |
| SJW | 1 TQ |
| CAN Mode | Normal |
| Auto Bus-Off | Enable |
| Auto Retransmission | Enable |

CAN 수신 필터는 Standard CAN ID `0x110`만 수신하도록 구성되어 있습니다.

### 4.2 HAPTIC_COMMAND (0x110)

**Safety ECU → Haptic ECU**

| 항목 | 값 |
|---|---|
| CAN ID | `0x110` |
| Frame | Standard Data Frame |
| DLC | 8 Byte |
| 송신 ECU | Safety ECU |
| 수신 ECU | Haptic ECU |
| Timeout | 100 ms |

| Byte | Signal | 데이터 타입 | 범위 및 설명 |
|---|---|---|---|
| 0 | Risk Level | uint8_t | 0~3 |
| 1 | Vibration Command | uint8_t | 0: OFF / 1: ON |
| 2 | Intensity | uint8_t | 0~100 (%) |
| 3 | Frequency | uint8_t | 0 또는 10~50Hz |
| 4 | Reserved | uint8_t | 0x00 |
| 5 | Reserved | uint8_t | 0x00 |
| 6 | Reserved | uint8_t | 0x00 |
| 7 | Alive Counter | uint8_t | 0~255 |

**명령 유효성 검사**

- Risk Level은 0~3 범위여야 합니다.
- Vibration Command는 0 또는 1이어야 합니다.
- OFF 명령에서는 Intensity와 Frequency가 모두 0이어야 합니다.
- ON 명령에서는 Intensity가 1~100%, Frequency가 10~50Hz여야 합니다.
- Reserved Byte 4~6은 모두 0x00이어야 합니다.
- 동일한 Alive Counter가 연속 수신되면 통신 이상으로 처리합니다.

첫 번째 유효 명령의 Alive Counter는 초기값과 관계없이 수용하며, 이후 동일한 값이 반복되는지 검사합니다.

### 4.3 HAPTIC_STATUS (0x200)

**Haptic ECU → Safety ECU**

| 항목 | 값 |
|---|---|
| CAN ID | `0x200` |
| Frame | Standard Data Frame |
| DLC | 8 Byte |
| 송신 ECU | Haptic ECU |
| 수신 ECU | Safety ECU |
| 송신 주기 | 10 ms |

| Byte | Signal | 데이터 타입 | 설명 |
|---|---|---|---|
| 0 | Current Risk Level | uint8_t | 현재 Risk Level |
| 1 | Vibration Status | uint8_t | 0: OFF / 1: ON |
| 2 | Applied Intensity | uint8_t | 적용 상태로 기록된 진동 세기 |
| 3 | Applied Frequency | uint8_t | 적용 상태로 기록된 주파수 |
| 4 | Fault Code | uint8_t | 오류 코드 |
| 5 | Command Timeout | uint8_t | 0: 정상 / 1: Timeout |
| 6 | ECU Status | uint8_t | 0: ERROR / 1: NORMAL |
| 7 | Alive Counter | uint8_t | 0~255 |

Applied Intensity와 Applied Frequency는 수신한 목표값이 아니라 `USBH_HID_SetReport()`가 `USBH_OK`를 반환한 이후 갱신되는 값입니다.

HAPTIC_STATUS의 Alive Counter는 Haptic ECU에서 독립적으로 생성하며, CAN 송신 요청이 성공적으로 등록된 경우 증가합니다.

---

## 5. USB HID 제어

Haptic ECU는 STM32 USB Host 기능을 이용하여 SIMAGIC P2000 HCB에 Feature Report를 전송합니다.

### 5.1 USB Report 설정

| 항목 | 값 |
|---|---|
| USB Class | HID |
| Report Type | 0x03 (Feature Report) |
| Report ID | 0xF1 |
| Report Buffer | 49 Byte |
| Channel | 0 |

### 5.2 Feature Report 구조

| Byte | 데이터 | 설명 |
|---|---|---|
| 0 | 0xF1 | Report ID |
| 1 | 0xEC | 명령 식별 값 |
| 2 | Channel | 0 |
| 3 | Enable | 0: OFF / 1: ON |
| 4 | Frequency | 주파수(Hz) |
| 5 | Intensity | 진동 세기(%) |
| 6~48 | 0x00 | 미사용 영역 |

### 5.3 USB 전송 처리

1. CAN 명령을 수신하고 유효성을 검사합니다.
2. Main Loop에서 Haptic Command를 처리합니다.
3. P2000 Feature Report를 생성합니다.
4. USB Host가 준비된 상태에서 Report를 전송합니다.
5. `USBH_OK` 반환 시 내부 적용 상태를 갱신합니다.
6. `USBH_BUSY`인 경우 다음 Main Loop에서 재시도합니다.

CAN 수신 인터럽트 내부에서는 USB 통신을 직접 수행하지 않고, 명령을 Main Loop로 전달합니다.

---

## 6. Fault Code 및 Fail-Safe

### 6.1 Fault Code

| Code | 명칭 | 설명 |
|---|---|---|
| 0 | NO_FAULT | 오류 없음 |
| 1 | ACTUATOR | 액추에이터 오류용 코드 |
| 2 | DRIVER | USB 연결 또는 드라이버 오류 |
| 3 | INVALID_COMMAND | CAN 명령 데이터 유효성 오류 |
| 4 | COMMUNICATION | Alive Counter 중복 등 통신 오류 |

`FAULT_ACTUATOR`는 현재 상수로 정의되어 있지만, 제공된 `main.c`에는 해당 오류를 발생시키는 별도 검출 로직이 없습니다.

### 6.2 Fail-Safe 동작

| 발생 상황 | 처리 방식 |
|---|---|
| 비정상 CAN 명령 | Haptic OFF 요청, INVALID_COMMAND 설정 |
| Alive Counter 중복 | Haptic OFF 요청, COMMUNICATION 설정 |
| CAN 명령 100ms Timeout | Haptic OFF 요청, Timeout 상태 설정 |
| 최초 정상 명령 수신 전 | Timeout 상태 유지 및 초기 OFF 요청 |
| USB 연결 해제 | ECU ERROR, DRIVER Fault, 내부 적용 상태 OFF 보고 |
| USB Feature Report 미지원 | ECU ERROR 및 DRIVER Fault |
| USBH_BUSY | 전송 재시도 |

### 6.3 CAN Timeout 처리

마지막 정상 `HAPTIC_COMMAND` 수신 이후 100ms 이상 경과하면 Timeout으로 판단합니다.

Timeout 발생 시에는 다음 처리를 수행합니다.

- Command Timeout을 1로 설정
- Current Risk Level을 0으로 설정
- Haptic OFF 명령 생성
- 반복적인 OFF 명령 생성을 방지

정상적인 CAN 명령을 다시 수신하면 Command Timeout 상태를 해제하고 Alive Counter를 갱신합니다.

현재 구현은 Timeout 이후 정상 명령이 수신되면 자동으로 명령 처리를 재개하는 구조입니다.

---

## 7. 주요 소스 파일

| 파일 및 폴더 | 역할 |
|---|---|
| `Core/Src/main.c` | CAN 수신, 명령 처리, USB 햅틱 제어, 상태 송신 및 Fail-Safe |
| `Core/Inc/main.h` | Haptic ECU 메인 헤더 |
| `USB_HOST/` | USB Host 초기화 및 처리 |
| `Middlewares/` | STM32 USB Host Library |
| `Drivers/` | STM32 HAL Driver 및 CMSIS |
| `HapticECU.ioc` | STM32CubeMX 주변장치 설정 |
| `.project` / `.cproject` | STM32CubeIDE 프로젝트 설정 |

---

## 8. 개발 및 검증 현황

### 8.1 구현 및 동작 확인

- STM32F446RE 기반 Haptic ECU 개발 환경 구성
- USB HID 통신을 통한 P2000 HCB 제어
- 진동 세기 1~100% 설정
- 진동 주파수 10~50Hz 설정
- Haptic ON/OFF 제어
- 실제 페달에 SIMAGIC P-HPR 장착
- 실제 페달에서 진동 전달 확인

### 8.2 코드 구현 완료 — 최종 통합 검증 필요

- HAPTIC_COMMAND (0x110) 수신
- CAN 명령 데이터 유효성 검사
- Alive Counter 중복 검사
- CAN 명령 Timeout 감지
- 비정상 명령에 따른 Haptic OFF 처리
- USB 연결 상태 및 오류 관리
- HAPTIC_STATUS (0x200) 10ms 주기 송신
- Fault Code 및 ECU Status 관리

### 8.3 향후 검증 필요

- Safety ECU 실제 위험 단계와 Haptic ECU 연동
- 위험 단계별 진동 세기·주파수·패턴 검증
- CAN 송수신 주기 및 데이터 정합성 검증
- 실제 통신 Timeout 발생 시 Haptic OFF 동작 검증
- Alive Counter 오류 및 비정상 명령 검증
- USB 연결 해제 및 재연결 상황 검증
- CANoe 기반 CAN 통신 안정성 및 응답시간 검증
- 실제 운전자 대상 햅틱 단계 구분 및 행동교정 효과 평가

---

## 9. 향후 개발 방향

Safety ECU의 BNS/PMS 기반 위험도 판단 결과에 따라 Haptic ECU가 단계별 진동 피드백을 제공하도록 최종 통합할 예정입니다.

최종적으로 다음 동작을 검증하는 것을 목표로 합니다.

**위험 상황 발생 → Safety ECU 위험도 판단 → CAN Haptic Command 전송 → Haptic ECU 진동 제어 → 운전자 행동 변화 관찰**

이를 통해 페달 오조작 위험 상황에서 햅틱 피드백이 운전자의 자발적인 가속페달 해제 또는 브레이크 전환을 유도할 수 있는지 평가할 계획입니다.
