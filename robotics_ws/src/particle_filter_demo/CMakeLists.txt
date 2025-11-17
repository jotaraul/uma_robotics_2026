cmake_minimum_required(VERSION 3.8)
project(particle_filter_demo)

find_package(ament_cmake REQUIRED)


install(
	DIRECTORY resources
	DESTINATION share/${PROJECT_NAME}
)

install(
	DIRECTORY launch
	DESTINATION share/${PROJECT_NAME}
)

ament_package()
