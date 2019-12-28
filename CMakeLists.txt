cmake_minimum_required(VERSION 2.6)
project(plane)

find_package(PCL REQUIRED QUIET)

include_directories(${PCL_INCLUDE_DIRS})

link_directories(${PCL_LIBRARY_DIRS})

add_definitions(${PCL_DEFINITIONS})

add_executable(plane src/main.cpp src/plane.cpp )
#add_library(${PROJECT_NAME} 
#sensors.cpp
#)


target_link_libraries(plane ${PCL_LIBRARIES} )

find_package(cmake_modules REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIR})
add_definitions(${EIGEN3_DEFINITIONS})

install(TARGETS plane RUNTIME DESTINATION bin)

install(DIRECTORY src
  DESTINATION src
  FILES_MATCHING 
    PATTERN "*.h"
    PATTERN "*.hpp"
)
