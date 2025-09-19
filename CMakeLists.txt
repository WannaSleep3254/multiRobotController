cmake_minimum_required(VERSION 3.16)
project(multiRobotController VERSION 0.2 LANGUAGES CXX)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(QT NAMES Qt6 Qt5 REQUIRED COMPONENTS Widgets SerialBus Core Gui)
find_package(Qt${QT_VERSION_MAJOR} REQUIRED COMPONENTS Widgets SerialBus Core Gui)

# ---- core library
add_library(multiRobotController_core STATIC
    src/core/modbus/ModbusClient.cpp
    src/core/modbus/ModbusClient.h
    src/core/orchestrator/Orchestrator.cpp
    src/core/orchestrator/Orchestrator.h
    src/core/models/PickListModel.cpp
    src/core/models/PickListModel.h
)
target_link_libraries(multiRobotController_core
    PRIVATE
      Qt${QT_VERSION_MAJOR}::Core
      Qt${QT_VERSION_MAJOR}::Gui
      Qt${QT_VERSION_MAJOR}::SerialBus
)
target_include_directories(multiRobotController_core
    PUBLIC                  # 공개 헤더 루트
      src/core
      src/core/modbus
      src/core/orchestrator
      src/core/models
)

# ---- app executable
set(APP_SOURCES
    src/app/main.cpp
    src/app/mainwindow.cpp
    src/app/mainwindow.h
    src/app/mainwindow.ui
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
      #Qt${QT_VERSION_MAJOR}::SerialBus
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
