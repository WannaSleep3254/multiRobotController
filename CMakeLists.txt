cmake_minimum_required(VERSION 3.16)
project(multiRobotController VERSION 0.2 LANGUAGES CXX)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(QT NAMES Qt6 Qt5 REQUIRED COMPONENTS Widgets SerialBus Core Gui Network)
find_package(Qt${QT_VERSION_MAJOR} REQUIRED COMPONENTS Widgets SerialBus Core Gui Network)
find_package(Qt${QT_VERSION_MAJOR} REQUIRED COMPONENTS Core)
find_package(Qt${QT_VERSION_MAJOR} REQUIRED COMPONENTS Widgets)

# ---- core library
find_package(Qt6 REQUIRED COMPONENTS Core)

add_library(multiRobotController_core STATIC
    src/core/network/Server.cpp
    src/core/network/Server.h

    src/core/modbus/ModbusClient.cpp
    src/core/modbus/ModbusClient.h

    src/core/common/LogLevel.h
    src/core/common/RobotCommand.h
    src/core/common/RobotCommandParser.cpp
    src/core/common/RobotCommandParser.h
    src/core/common/RobotCommandSerializer.cpp
    src/core/common/RobotCommandSerializer.h

    src/core/orchestrator/Orchestrator.cpp
    src/core/orchestrator/Orchestrator.h

    src/core/models/Pose6D.h
    src/core/models/PickListModel.cpp
    src/core/models/PickListModel.h

    src/core/robots/RobotManager.h
    src/core/robots/RobotManager.cpp

    src/core/vision/VisionClient.h
    src/core/vision/VisionClient.cpp

    src/core/tf/EulerAngleConverter.cpp
    src/core/tf/EulerAngleConverter.h

    src/core/Motor/Com/modbus.cpp
    src/core/Motor/Com/modbus.h
    src/core/Motor/Leadshine/eld2.cpp
    src/core/Motor/Leadshine/eld2.h

    src/core/Motor/Leadshine/parameter.cpp
    src/core/Motor/Leadshine/parameter.h
    src/core/Motor/Port/serialport.cpp
    src/core/Motor/Port/serialport.h

    src/core/Widgets/motorconfig.cpp
    src/core/Widgets/motorconfig.h
    src/core/Widgets/motorconfig.ui

    src/core/Widgets/motorconnect.cpp
    src/core/Widgets/motorconnect.h
    src/core/Widgets/motorconnect.ui

    src/core/Widgets/motorposition.cpp
    src/core/Widgets/motorposition.h
    src/core/Widgets/motorposition.ui

    src/core/Widgets/motortest.cpp
    src/core/Widgets/motortest.h
    src/core/Widgets/motortest.ui
    src/core/robots/GentryManager.h src/core/robots/GentryManager.cpp
)
target_link_libraries(multiRobotController_core
    PRIVATE
      Qt${QT_VERSION_MAJOR}::Core
      Qt${QT_VERSION_MAJOR}::Widgets
      Qt${QT_VERSION_MAJOR}::Gui
      Qt${QT_VERSION_MAJOR}::Network
      Qt${QT_VERSION_MAJOR}::SerialBus
)
target_link_libraries(multiRobotController_core PRIVATE Qt6::Core)
target_link_libraries(multiRobotController_core PRIVATE Qt${QT_VERSION_MAJOR}::Core)
target_include_directories(multiRobotController_core
    PUBLIC                  # 공개 헤더 루트
      src/core
      src/core/modbus
      src/core/common
      src/core/orchestrator
      src/core/models
      src/core/network
      src/core/robots
      # Motor
      src/core/Motor
      src/core/Motor/Com
      src/core/Motor/Leadshine
      src/core/Motor/Port
      src/core/Widgets
      3rdparty/eigen-3.4.0
)

# ---- app executable
set(APP_SOURCES
    src/app/main.cpp

    src/app/mainwindow.cpp
    src/app/mainwindow.h
    src/app/mainwindow.ui

    src/app/widgets/RobotPanel.cpp
    src/app/widgets/RobotPanel.h

    src/app/widgets/MotorPanel.cpp
    src/app/widgets/MotorPanel.h

    src/resources/resources.qrc
)

if(${QT_VERSION_MAJOR} GREATER_EQUAL 6)
    qt_add_executable(multiRobotController
        MANUAL_FINALIZATION
        ${APP_SOURCES}
    )
else()
    add_executable(multiRobotController ${APP_SOURCES})
endif()

target_link_libraries(multiRobotController
    PRIVATE
      Qt${QT_VERSION_MAJOR}::Widgets
      Qt${QT_VERSION_MAJOR}::Network
      multiRobotController_core
)
target_link_libraries(multiRobotController PRIVATE Qt${QT_VERSION_MAJOR}::Widgets)

set_target_properties(multiRobotController PROPERTIES
    MACOSX_BUNDLE TRUE
    WIN32_EXECUTABLE TRUE
)

include(GNUInstallDirs)
install(TARGETS multiRobotController
    BUNDLE DESTINATION .
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

if(QT_VERSION_MAJOR EQUAL 6)
    qt_finalize_executable(multiRobotController)
endif()
