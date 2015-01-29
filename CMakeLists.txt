cmake_minimum_required(VERSION 2.6)

set(projectname  look_around)

project(${projectname})

find_package(YARP)

#pick all header and code files from current directory
set(${projectname}_source headControllerTest.cpp)

include_directories(${YARP_INCLUDE_DIRS})
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

add_executable(${projectname}  ${${projectname}_source})

target_link_libraries(${projectname} ${YARP_LIBRARIES})

