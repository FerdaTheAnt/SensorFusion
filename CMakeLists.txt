cmake_minimum_required(VERSION 3.16)
project(SensorFusionFramework LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS True)

include(CTest)
enable_testing()

find_package(Eigen3 REQUIRED)

if(POLICY CMP0167)
    cmake_policy(SET CMP0167 NEW)
endif()

find_package(Boost REQUIRED COMPONENTS json)

find_package(Catch2 REQUIRED)

include_directories(
    include
    ${Boost_INCLUDE_DIRECTORIES}
)

set(SRC_FILES
        src/main.cpp
        src/csv_loader.cpp
        src/SensorFusionEngine.cpp
        src/net/UDPReceiver.cpp
)
add_executable(sensor_fusion ${SRC_FILES})

target_link_libraries(sensor_fusion
    Eigen3::Eigen
)

set(TEST_FILES tests/test_csv.cpp tests/test_filter.cpp src/csv_loader.cpp)
add_executable(tests ${TEST_FILES})

target_link_libraries(tests
    Eigen3::Eigen
    Catch2::Catch2WithMain
)
add_test(NAME AllTests COMMAND tests WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
