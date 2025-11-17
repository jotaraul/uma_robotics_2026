cmake_minimum_required(VERSION 3.8)
project(odom_simulator)

set(CMAKE_BUILD_TYPE "Debug") 
set(CMAKE_CXX_FLAGS_Release "-g -O3 ${CMAKE_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS_Debug "-g -O0 ${CMAKE_CXX_FLAGS}")


find_package(ament_cmake_auto REQUIRED)
find_package(ament_cmake REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_executable(odom_node src/odom_node.cpp)

install(TARGETS
    odom_node
    DESTINATION lib/${PROJECT_NAME}
)
ament_package()
