cmake_minimum_required(VERSION 3.0.2)
project(dr_gins)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# optional: optional backward support (thirdparty/backward-cpp must exist)
add_subdirectory(${PROJECT_SOURCE_DIR}/thirdparty/backward-cpp EXCLUDE_FROM_ALL)

find_package(Boost REQUIRED COMPONENTS filesystem program_options system)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  ublox_msgs
)
find_package(Eigen3 REQUIRED)
find_package(TBB)

catkin_package(
  LIBRARIES dr_gins_core
  CATKIN_DEPENDS roscpp sensor_msgs nav_msgs geometry_msgs ublox_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}
)

set(THIRD_PARTY_LIBS
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)
if(TBB_FOUND)
  list(APPEND THIRD_PARTY_LIBS TBB::tbb)
endif()

# core static/shared library (like tracking in reference project)
add_library(dr_gins_core
  common/dr_gins_interface.cpp
  core/gins_eskf.cpp
  core/ins.cpp
  log/state_logger.cpp
)

target_link_libraries(dr_gins_core
  ${THIRD_PARTY_LIBS}
)

# tests / example nodes live in test/ (keeps CMake structure similar to reference)
add_subdirectory(test)

# install core library and any installed executables from test/
install(TARGETS dr_gins_core
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY
  launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)