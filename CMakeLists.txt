cmake_minimum_required(VERSION 3.8)
project(gazebo_logger)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)

if(WIN32)
  add_compile_definitions(
    _USE_MATH_DEFINES
    NOMINMAX
    WIN32_LEAN_AND_MEAN
  )
endif()

# Check if NAAD_WS_DIR is defined
if(NOT DEFINED ENV{NAAD_WS_DIR})
  message(FATAL_ERROR "Environment variable NAAD_WS_DIR is not set")
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(gazebo REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Core Widgets)

# Include directories and libraries
include_directories(
  ${Qt5Core_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

# Create shared library for the plugin
add_library(${PROJECT_NAME} SHARED src/gazebo_logger_plugin.cpp)

# Set include directories for the target
target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${gazebo_INCLUDE_DIRS}
)

# Set target properties
set_target_properties(${PROJECT_NAME} PROPERTIES CXX_STANDARD 17)

# Link the target to the required libraries
target_link_libraries(${PROJECT_NAME} ${GAZEBO_LIBRARIES} ${Qt5Widgets_LIBRARIES})

# Install the library
install(
  TARGETS ${PROJECT_NAME} EXPORT ${PROJECT_NAME}_export
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install include files
install(DIRECTORY include/
  DESTINATION include)

if(WIN32)
  target_compile_definitions(${PROJECT_NAME} PRIVATE "GAZEBO_LOGGER_BUILDING_DLL")
  install(
    TARGETS ${PROJECT_NAME}
    LIBRARY DESTINATION $ENV{GAZEBO_PLUGIN_PATH}
  )
endif()
# Export include directories and dependencies
ament_export_include_directories(include)

ament_export_dependencies(
  ament_cmake
  gazebo
)

ament_export_libraries(${PROJECT_NAME})

ament_package()
