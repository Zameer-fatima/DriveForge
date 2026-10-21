cmake_minimum_required(VERSION 3.21)
project(DriveForge VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(Qt6 6.5 REQUIRED COMPONENTS Core Gui Widgets Qml Quick Quick3D QuickControls2 Charts Test)
qt_standard_project_setup(REQUIRES 6.5)

add_subdirectory(src/core)

qt_add_executable(driveforge src/app/main.cpp)
qt_add_qml_module(driveforge
    URI DriveForge
    VERSION 1.0
    QML_FILES
        qml/Main.qml
        qml/Scene3D.qml
        qml/Car.qml
        qml/VehicleBody.qml
        qml/Pedestrian3D.qml
        qml/Tree3D.qml
        qml/Dashboard.qml
        qml/TelemetryChart.qml
        qml/TelemetryPanel.qml
        qml/EventLog.qml
        qml/ScenarioPanel.qml
        qml/RecordedChart.qml
        qml/RecordedView.qml
    RESOURCES
        assets/textures/asphalt_color.jpg
        assets/textures/asphalt_normal.jpg
        assets/textures/asphalt_roughness.jpg
        assets/textures/grass_color.jpg
        assets/textures/grass_normal.jpg
        assets/textures/grass_roughness.jpg
        assets/sky/kloofendal_partly_cloudy_1k.hdr
    SOURCES
        src/app/CarGeometry.h
        src/app/CarGeometry.cpp
        src/app/SimulationController.h
        src/app/SimulationController.cpp
        src/app/EventLogModel.h
        src/app/EventLogModel.cpp
        src/app/WorldModel.h
        src/app/WorldModel.cpp
        src/app/BayModel.h
        src/app/BayModel.cpp
        src/app/ScenarioManager.h
        src/app/ScenarioManager.cpp
        src/app/RecordedData.h
        src/app/RecordedData.cpp
)
target_link_libraries(driveforge PRIVATE
    driveforge_core Qt6::Core Qt6::Gui Qt6::Widgets Qt6::Charts Qt6::Qml Qt6::Quick Qt6::Quick3D Qt6::QuickControls2)
target_include_directories(driveforge PRIVATE src/app)

enable_testing()
add_subdirectory(tests)
