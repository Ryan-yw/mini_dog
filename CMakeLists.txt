cmake_minimum_required(VERSION 3.12)
project(mini_dog)

set(CMAKE_CXX_STANDARD 17)


if(UNIX)
	add_definitions(-DUNIX)
	find_package(aris REQUIRED PATHS /usr/aris)
endif(UNIX)
IF(WIN32)
	find_package(aris REQUIRED PATHS C:/aris/aris-1.5.0)
endif(WIN32)
include_directories(${aris_INCLUDE_DIRS})

add_executable(mini_dog_motion  src/main.cpp  src/plan.h src/plan.cpp src/kinematics.h  src/kinematics.cpp  src/json.hpp)

target_link_libraries(mini_dog_motion ${aris_LIBRARIES} )
