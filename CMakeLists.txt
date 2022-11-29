cmake_minimum_required(VERSION 3.5)
project(teensyshot)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} cmake/)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(teensyshot_interfaces REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# let the compiler search for headers in the include folder
include_directories(include)

# define a library target called teensyshot_host
add_library(teensyshot STATIC src/teensyshot_host_lib.c)

# this line to exports the library
ament_export_targets(teensyshot HAS_LIBRARY_TARGET)

# install the include/teensyshot_host directory to the install/include/teensyshot_host
install(
  DIRECTORY include/teensyshot
  DESTINATION include
)

install(
  TARGETS teensyshot
  EXPORT teensyshot
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)


add_executable(teensyshot_host_ros src/teensyshot_host_ros.cpp)
ament_target_dependencies(teensyshot_host_ros rclcpp std_msgs teensyshot_interfaces)
target_link_libraries(teensyshot_host_ros teensyshot)
target_include_directories(teensyshot_host_ros PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

install(TARGETS teensyshot_host_ros
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
