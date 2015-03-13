cmake_minimum_required(VERSION 2.8.3)
project(ed)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  class_loader
  geolib2
  roscpp
  roslib
  tue_config
  tue_filesystem
  tue_serialization
  tue_config
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ed_core
    CATKIN_DEPENDS tue_config tue_serialization
)

###########
## Build ##
###########

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

# ------------------------------------------------------------------------------------------------
#                                          ED CORE
# ------------------------------------------------------------------------------------------------

# Get all the headers
file(GLOB_RECURSE HEADER_FILES include/*.h)

# Declare a cpp library
add_library(ed_core
    src/entity.cpp
    src/measurement.cpp
    src/update_request.cpp
    src/world_model.cpp
    src/transform_cache.cpp

    src/error_context.cpp
    include/ed/error_context.h

    ${HEADER_FILES}
)
target_link_libraries(ed_core ${catkin_LIBRARIES})
add_dependencies(ed_core ${PROJECT_NAME}_gencpp)

# ------------------------------------------------------------------------------------------------
#                                               SERVER
# ------------------------------------------------------------------------------------------------

# Create executable
add_executable(ed
    src/ed.cpp
    src/server.cpp
    src/plugin_container.cpp
)
target_link_libraries(ed ed_core)
add_dependencies(ed ${PROJECT_NAME}_gencpp)
