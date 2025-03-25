cmake_minimum_required(VERSION 3.8)
project(motor_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rcutils REQUIRED)

# Find libgpiod package using pkg-config
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBGPIOD REQUIRED libgpiod)

# Print information about found libgpiod
message(STATUS "LIBGPIOD_INCLUDE_DIRS: ${LIBGPIOD_INCLUDE_DIRS}")
message(STATUS "LIBGPIOD_LIBRARIES: ${LIBGPIOD_LIBRARIES}")
message(STATUS "LIBGPIOD_LIBRARY_DIRS: ${LIBGPIOD_LIBRARY_DIRS}")

# Set RPATH
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
if(LIBGPIOD_LIBRARY_DIRS)
  link_directories(${LIBGPIOD_LIBRARY_DIRS})
endif()

pluginlib_export_plugin_description_file(hardware_interface hardware_interface_plugin.xml)

# Include directories
include_directories(include ${LIBGPIOD_INCLUDE_DIRS})

# Create the hardware interface library
add_library(motor_interface SHARED
  src/unibots_motor_hardware_interface.cpp
  src/motor_controller.cpp
)

# Add test program
add_executable(test_gpiod test/test_gpiod.cpp)
target_link_libraries(test_gpiod ${LIBGPIOD_LIBRARIES})

target_include_directories(motor_interface PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(motor_interface
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  rcutils)

# Link against libgpiod
target_link_libraries(motor_interface
  ${LIBGPIOD_LIBRARIES}
)

# Install the library
install(TARGETS motor_interface
  EXPORT motor_interface
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin)

# Install the headers
install(DIRECTORY include/
  DESTINATION include/)

# Install the plugin description
install(FILES motor_interface.xml
  DESTINATION share/${PROJECT_NAME})

# Export the library
ament_export_targets(motor_interface
  HAS_LIBRARY_TARGET)
ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  rcutils)

ament_package()
