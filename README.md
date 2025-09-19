# multiRobotController

Qt + Modbus 기반 **BinPicking 로봇/PLC 연동 테스트 컨트롤러**
로봇과 PLC 간의 handshake 시퀀스를 시뮬레이션하고, Pick pose 리스트를 Publish하는 테스트/디버깅용 툴입니다.

---

## 📂 Directory Structure

```bash
wannasleep3254-multiRobotController/
├─ CMakeLists.txt
├─ cmake/                         # (옵션) 툴체인/프리셋/Find*.cmake
├─ docs/                          # 설계/시퀀스/FSM/통신명세
│   ├─ handshake_fsm.md
│   └─ vision_protocol.md
├─ scripts/                       # 런처/배포/프리커밋 검증
│   ├─ run_dev.sh
│   └─ validate_address_map.py
├─ resources/
│   ├─ qml/                       # (옵션) QML 사용하는 경우
│   ├─ icons/
│   ├─ styles.qss
│   ├─ resources.qrc              # 리소스 묶음
│   └─ map/
│       ├─ AddressMap.json        # 공통 or 기본 맵
│       ├─ AddressMap_A.json      # 로봇 A용 (필요시)
│       ├─ AddressMap_B.json      # 로봇 B용 (필요시)
│       └─ AddressMap_C.json      # 로봇 C용 (필요시)
├─ config/
│   ├─ app.json                   # UI/로그/포트 등 앱 설정
│   └─ robots.json                # 각 로봇의 IP/Port/맵 경로/초기속도 등
├─ src/
│   ├─ app/                       # GUI/엔트리포인트
│   │   ├─ main.cpp
│   │   ├─ mainwindow.cpp
│   │   ├─ mainwindow.h
│   │   └─ mainwindow.ui
│   ├─ core/
│   │   ├─ models/
│   │   │   ├─ PickListModel.cpp
│   │   │   └─ PickListModel.h
│   │   ├─ modbus/
│   │   │   ├─ ModbusClient.cpp
│   │   │   └─ ModbusClient.h
│   │   ├─ orchestrator/
│   │   │   ├─ Orchestrator.cpp
│   │   │   └─ Orchestrator.h
│   │   ├─ vision/                # TCP 서버(수신측)
│   │   │   ├─ Server.cpp         # (기존 generic server) 또는 server.cpp
│   │   │   ├─ Server.h
│   │   │   ├─ VisionServer.cpp   # Server 상속/조합해 JSON 파싱/라우팅
│   │   │   └─ VisionServer.h
│   │   ├─ robots/                # 다중 로봇 컨텍스트/팩토리
│   │   │   ├─ RobotContext.h     # ModbusClient+Orch+Model 묶음
│   │   │   └─ RobotManager.(h|cpp) # ID→컨텍스트 라우팅, 시작/정지 일괄제어
│   │   └─ util/                  # 공용 유틸(인코딩/로깅/변환)
│   │       ├─ FloatPacking.h
│   │       └─ JsonHelpers.(h|cpp)
│   └─ plugins/                   # (옵션) 향후 확장: 로봇/비전 어댑터
├─ tests/                         # 유닛/통합 테스트
│   ├─ test_modbus.cpp
│   ├─ test_vision_server.cpp
│   └─ CMakeLists.txt
└─ third_party/                   # (옵션) 외부 라이브러리

```

---

## 🚀 Build Instructions

### Requirements

* CMake ≥ 3.16
* C++17
* Qt5.15 또는 Qt6.2 이상 (`Widgets`, `SerialBus`, `Core` 모듈 필요)

### Build (Linux / Windows)

```bash
git clone https://github.com/WannaSleep3254/multiRobotController.git
cd multiRobotController
cmake -B build -S .
cmake --build build
```

빌드 후 `build/multiRobotController` 실행 파일 생성.

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
