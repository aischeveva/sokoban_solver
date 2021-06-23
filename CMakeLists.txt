cmake_minimum_required(VERSION 3.0.0)
project(sokoban VERSION 0.1.0)

include(CTest)
enable_testing()


include_directories(./src/)
link_directories(./src/)

set(LIBS src/Block.hpp src/Object.hpp src/Box.hpp src/Pusher.hpp src/FeatureSpace.hpp src/Board.hpp)
set(SRCS src/Object.cpp src/Board.cpp)
add_library(board ${SRCS} ${LIBS})
add_executable(sokoban src/main.cpp)
target_link_libraries(sokoban board)


set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)
