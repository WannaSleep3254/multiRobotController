cmake_minimum_required(VERSION 3.16)
project(multiRobotController VERSION 0.2 LANGUAGES CXX)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(QT NAMES Qt6 Qt5 REQUIRED COMPONENTS Widgets SerialBus Core Gui Network)
find_package(Qt${QT_VERSION_MAJOR} REQUIRED COMPONENTS Widgets SerialBus Core Gui Network)

# ---- core library
find_package(Qt6 REQUIRED COMPONENTS Core)

add_library(multiRobotController_core STATIC
    src/core/network/Server.cpp
    src/core/network/Server.h

    src/core/modbus/ModbusClient.cpp
    src/core/modbus/ModbusClient.h

    src/core/common/LogLevel.h

    src/core/orchestrator/Orchestrator.cpp
    src/core/orchestrator/Orchestrator.h

    src/core/models/Pose6D.h
    src/core/models/PickListModel.cpp
    src/core/models/PickListModel.h

    src/core/robots/RobotManager.h
    src/core/robots/RobotManager.cpp

    src/core/vision/VisionServer.h
    src/core/vision/VisionServer.cpp

    src/core/tf/EulerAngleConverter.cpp
    src/core/tf/EulerAngleConverter.h
)
target_link_libraries(multiRobotController_core
    PRIVATE
      Qt${QT_VERSION_MAJOR}::Core
      Qt${QT_VERSION_MAJOR}::Gui
      Qt${QT_VERSION_MAJOR}::Network
      Qt${QT_VERSION_MAJOR}::SerialBus
)
target_link_libraries(multiRobotController_core PRIVATE Qt6::Core)
target_include_directories(multiRobotController_core
    PUBLIC                  # 공개 헤더 루트
      src/core
      src/core/modbus
      src/core/common
      src/core/orchestrator
      src/core/models
      src/core/network
      src/core/robots

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
