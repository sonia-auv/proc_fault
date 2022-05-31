cmake_minimum_required(VERSION 3.0.2)
project(proc_fault)

if (NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Debug CACHE STRING
      "Choose the type of build, options are: None Debug Release RelWithDebInfo MinSizeRel."
      FORCE)
endif (NOT CMAKE_BUILD_TYPE)

set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -std=c++11 -Wall")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -std=c++11 -O3 -s")

set(proc_fault_SRC_DIR "src")

#===============================================================================
# C A T K I N   C O N F I G U R A T I O N

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    sensor_msgs
    sonia_common
    )

catkin_package(
    INCLUDE_DIRS ${proc_fault_SRC_DIR}
    LIBRARIES
    CATKIN_DEPENDS
    roscpp
    std_msgs
    sensor_msgs
    sonia_common
)

#===============================================================================
# I N C L U D E   F I L E S

file(GLOB_RECURSE proc_fault_FILES "${proc_fault_SRC_DIR}/*.cc" "${proc_fault_SRC_DIR}/*.h")

include_directories(
    ${catkin_INCLUDE_DIRS}
    ${proc_fault_SRC_DIR}
    ${sonia_common_INCLUDE_DIRS}
)

#==========================================================================
# C R E A T E   E X E C U T A B L E

add_executable(${PROJECT_NAME}_node ${proc_fault_FILES})
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES} ${sonia_common_LIBRARIES})
add_dependencies(${PROJECT_NAME}_node sonia_common_generate_messages_cpp)

#==========================================================================
# U N I T   T E S T S

if (CATKIN_ENABLE_TESTING AND ${CMAKE_CURRENT_SOURCE_DIR}/test)
  add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/test)
endif ()
