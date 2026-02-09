cmake_minimum_required(VERSION 3.5)
project(omni_bot)

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

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  scripts/cmdvel_to_odom.py
  DESTINATION lib/${PROJECT_NAME}
)

# -------------------------
# Install package resources
# -------------------------
install(DIRECTORY
  launch
  config
  description
  DESTINATION share/${PROJECT_NAME}
)

# Optional: gazebo worlds folder
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/worlds")
  install(DIRECTORY
    worlds
    DESTINATION share/${PROJECT_NAME}
  )
endif()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
