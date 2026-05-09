cmake_minimum_required(VERSION 3.14)
project(kinect_audio_cpp)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# ── ROS 2 dependencies ────────────────────────────────────────────────────────
find_package(ament_cmake   REQUIRED)
find_package(rclcpp        REQUIRED)
find_package(std_msgs      REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs   REQUIRED)   # needed by kinect_tilt_accel_node

# ── libfreenect ───────────────────────────────────────────────────────────────
find_package(PkgConfig REQUIRED)
pkg_check_modules(FREENECT REQUIRED libfreenect)

if(NOT FREENECT_FOUND)
  message(FATAL_ERROR
    "libfreenect not found.  Install with:\n"
    "  sudo apt install libfreenect-dev\n"
    "If the package is missing libfreenect_audio.h, rebuild from source:\n"
    "  cmake .. -DBUILD_AUDIO=ON")
endif()

message(STATUS "libfreenect version : ${FREENECT_VERSION}")
message(STATUS "libfreenect include : ${FREENECT_INCLUDE_DIRS}")
message(STATUS "libfreenect libs    : ${FREENECT_LIBRARIES}")

# Check for audio header (needed by kinect_audio_node only)
find_file(FREENECT_AUDIO_HEADER
  NAMES libfreenect_audio.h
  PATHS ${FREENECT_INCLUDE_DIRS}
  PATH_SUFFIXES libfreenect
)
if(FREENECT_AUDIO_HEADER)
  message(STATUS "libfreenect_audio.h : ${FREENECT_AUDIO_HEADER}")
else()
  message(WARNING
    "libfreenect_audio.h not found – kinect_audio node may not build.\n"
    "Rebuild libfreenect from source with -DBUILD_AUDIO=ON if needed.")
endif()

# ── Helper: apply freenect flags to a target ──────────────────────────────────
function(target_link_freenect target)
  target_include_directories(${target} PRIVATE ${FREENECT_INCLUDE_DIRS})
  target_link_libraries(${target} ${FREENECT_LIBRARIES})
  if(FREENECT_LDFLAGS_OTHER)
    target_link_options(${target} PRIVATE ${FREENECT_LDFLAGS_OTHER})
  endif()
endfunction()

# ════════════════════════════════════════════════════════════════════════════
#  Executable 1: kinect_audio
#  Publishes raw 4-ch int32 mic + noise-cancelled int16 (via libfreenect audio)
# ════════════════════════════════════════════════════════════════════════════
add_executable(kinect_audio src/kinect_audio_node.cpp)
target_link_freenect(kinect_audio)
ament_target_dependencies(kinect_audio rclcpp std_msgs)

# ════════════════════════════════════════════════════════════════════════════
#  Executable 2: kinect_tilt_accel
#  Tilt motor control + accelerometer for Kinect 1473 (audio USB path)
#  Also compatible with 1414 (uses standard motor path automatically)
# ════════════════════════════════════════════════════════════════════════════
add_executable(kinect_tilt_accel src/kinect_tilt_accel_node.cpp)
target_link_freenect(kinect_tilt_accel)
ament_target_dependencies(kinect_tilt_accel
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
)

# ── Install both executables ───────────────────────────────────────────────────
install(TARGETS
  kinect_audio
  kinect_tilt_accel
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
