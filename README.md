# testController

Qt + Modbus 기반 **BinPicking 로봇/PLC 연동 테스트 컨트롤러**
로봇과 PLC 간의 handshake 시퀀스를 시뮬레이션하고, Pick pose 리스트를 Publish하는 테스트/디버깅용 툴입니다.

---

## 📂 Directory Structure

```bash
wannasleep3254-testcontroller/
├── README.md
├── AddressMap.json          # 기본 주소 매핑
├── CMakeLists.txt
├── LICENSE
├── config/
│   └── AddressMap.json
├── src/
│   ├── app/                 # GUI (Qt Widgets)
│   │   ├── main.cpp
│   │   ├── mainwindow.cpp
│   │   ├── mainwindow.h
│   │   └── mainwindow.ui
│   ├── core/
│   │   ├── modbus/          # Modbus TCP 통신
│   │   │   ├── ModbusClient.cpp
│   │   │   └── ModbusClient.h
│   │   ├── models/          # Pick pose 관리
│   │   │   ├── PickListModel.cpp
│   │   │   └── PickListModel.h
│   │   └── orchestrator/    # FSM 기반 오케스트레이터
│   │       ├── Orchestrator.cpp
│   │       └── Orchestrator.h
│   └── resources/
│       ├── AddressMap.json
│       └── resources.qrc
├── tools/
│   ├── validate_address_map_v2.py
│   └── validate_address_map_win.py
└── .githooks/
    └── pre-commit
```

---

## 🚀 Build Instructions

### Requirements

* CMake ≥ 3.16
* C++17
* Qt5.15 또는 Qt6.2 이상 (`Widgets`, `SerialBus`, `Core` 모듈 필요)

### Build (Linux / Windows)

```bash
git clone https://github.com/WannaSleep3254/testController.git
cd testController
cmake -B build -S .
cmake --build build
```

빌드 후 `build/testController` 실행 파일 생성.

---

## ▶️ Run

1. 실행 후 GUI 창에서:

   * **Host/Port** 입력
   * **Connect** → Modbus 서버 연결
   * **Start** → FSM 실행
   * **Stop** → FSM 중지
2. `PickListModel`에서 등록된 Pick Pose가 순차적으로 Publish됨.
3. `plainLog` 창에 FSM 상태와 Modbus 통신 로그 출력.

---

## 🔗 AddressMap.json

Modbus 주소 매핑 정의 파일. (schema\_version=3)

예시:

```json
"coils": {
  "PUBLISH_REQ": 100
},
"discrete_inputs": {
  "ROBOT_READY": 100,
  "PICK_DONE": 101,
  "ROBOT_BUSY": 102
},
"holding": {
  "TARGET_POSE_BASE": 132, "_comment": "132..143 (float×6)"
}
```

---

## 🤖 FSM Workflow

로봇/PLC 간 handshake는 다음 순서로 진행됩니다:

1. **Idle 대기**
   READY=1, BUSY=0, DONE=0, PUBLISH\_REQ=0
2. **Publish Pose**
   컨트롤러 → `TARGET_POSE_BASE`에 좌표 기록 + `PUBLISH_REQ=1`
3. **로봇 시작**
   BUSY=1 (READY는 유지)
4. **Pick 완료**
   BUSY=0 & DONE=1
5. **컨트롤러 ACK**
   PUBLISH\_REQ=0
6. **로봇 완료 신호 클리어**
   DONE=0 ↓
7. **다음 사이클 준비**
   READY=1 & BUSY=0 → 다시 Pose Publish

---

## 🛠 Tools

* `tools/validate_address_map_v2.py`
  AddressMap.json의 충돌/정합성 검사 스크립트
* Git pre-commit hook `.githooks/pre-commit`
  커밋 전 자동 검증 실행 가능

---

## 📌 Future Work

* Pose Queue (`TARGET_QUEUE_BASE`) 확장
* Safety I/O (`SAFE_DOOR`, `LIGHT_CURTAIN`, `E_STOP`) 연동 강화
* Reserved 구간 활용하여 사용자 정의 신호 확장
