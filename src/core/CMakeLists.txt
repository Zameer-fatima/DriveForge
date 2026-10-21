add_library(driveforge_core STATIC
    VehicleModel.cpp VehicleModel.h
    RingBuffer.h
    Telemetry.cpp Telemetry.h
    EventDetector.cpp EventDetector.h
    Road.h
    World.cpp World.h
    Sensors.cpp Sensors.h
    Aeb.cpp Aeb.h
    Acc.cpp Acc.h
    LaneKeeping.cpp LaneKeeping.h
    Longitudinal.cpp Longitudinal.h
    Parking.cpp Parking.h
    Adas.cpp Adas.h
    Simulation.cpp Simulation.h
    Recorded.cpp Recorded.h
    Scenario.cpp Scenario.h
    ScenarioRunner.cpp ScenarioRunner.h)
target_include_directories(driveforge_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_compile_features(driveforge_core PUBLIC cxx_std_17)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(driveforge_core PRIVATE -Wall -Wextra -Wpedantic)
endif()
