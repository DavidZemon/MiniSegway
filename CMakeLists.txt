cmake_minimum_required(VERSION 3.3)
find_package(PropWare 3.0 REQUIRED)

project(MiniSegway)

#set(32_BIT_DOUBLES OFF)
set(MODEL cmm)
set(BOARD dna)

create_executable(${PROJECT_NAME}
    AngleComputer
    Logger
    MiniSegway
    MessageHandler
    MessageReceiver
    SdLogger
    SensorReader)
target_link_libraries(${PROJECT_NAME} m)
target_compile_definitions(${PROJECT_NAME} PUBLIC
    LOG_CONSOLE_SHORT=1
    LOG_CONSOLE_LONG=2)

# Set logging options
target_compile_definitions(${PROJECT_NAME} PUBLIC LOG_CONSOLE=LOG_CONSOLE_LONG)
#target_compile_definitions(${PROJECT_NAME} PUBLIC LOG_SD)
