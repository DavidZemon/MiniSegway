cmake_minimum_required(VERSION 3.3)
find_package(PropWare 3.0 REQUIRED)

project(MiniSegway)

set(32_BIT_DOUBLES OFF)
set(MODEL cmm)
set(BOARD dna)

create_executable(${PROJECT_NAME}
    MiniSegway
    AngleComputer
    SensorReader)
target_link_libraries(${PROJECT_NAME} m)
