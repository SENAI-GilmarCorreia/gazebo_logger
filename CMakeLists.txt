cmake_minimum_required(VERSION 3.8)
project(gazebo_logger)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(WIN32)
  add_compile_definitions(
    # For math constants
    _USE_MATH_DEFINES
    # Minimize Windows namespace collision
    NOMINMAX
    WIN32_LEAN_AND_MEAN
  )
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(gazebo REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Core Widgets)
include_directories(${Qt5Core_INCLUDE_DIRS})
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

add_library(${PROJECT_NAME} SHARED src/gazebo_logger_plugin.cpp)

# Link against Gazebo and Protobuf libraries
target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${gazebo_INCLUDE_DIRS}
)

set_target_properties(${PROJECT_NAME} PROPERTIES CXX_STANDARD 17)
target_link_libraries(${PROJECT_NAME} ${GAZEBO_LIBRARIES} ${Qt5Widgets_LIBRARIES})

ament_package()

install(DIRECTORY include/
        DESTINATION include)

if(WIN32)
  target_compile_definitions(${PROJECT_NAME} PRIVATE "GAZEBO_LOGGER_BUILDING_DLL")
endif()

set(EXPORT_DLL off)

if(EXPORT_DLL)
  add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy
        ${CMAKE_BINARY_DIR}/Release/${PROJECT_NAME}.dll
        $ENV{GAZEBO_PLUGIN_PATH}
    COMMENT "Copying gazebo_logger.dll to GAZEBO_PLUGIN_PATH"
  )
endif()
