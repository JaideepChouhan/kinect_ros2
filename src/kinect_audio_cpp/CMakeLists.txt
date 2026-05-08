cmake_minimum_required(VERSION 3.14)
project(kinect_audio_cpp)

# ── C++17 ────────────────────────────────────────────────────────────────────
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# ── ROS 2 dependencies ───────────────────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(rclcpp      REQUIRED)
find_package(std_msgs    REQUIRED)

# ── libfreenect (via pkg-config) ─────────────────────────────────────────────
# Requires: sudo apt install libfreenect-dev
# Confirm audio header: ls /usr/include/libfreenect/libfreenect_audio.h
find_package(PkgConfig REQUIRED)
pkg_check_modules(FREENECT REQUIRED libfreenect)

if(NOT FREENECT_FOUND)
  message(FATAL_ERROR
    "libfreenect not found.  Install with:\n"
    "  sudo apt install libfreenect-dev\n"
    "Or build from source with -DBUILD_AUDIO=ON")
endif()

message(STATUS "libfreenect version : ${FREENECT_VERSION}")
message(STATUS "libfreenect include : ${FREENECT_INCLUDE_DIRS}")
message(STATUS "libfreenect libs    : ${FREENECT_LIBRARIES}")

# ── Check that the audio header exists ───────────────────────────────────────
find_file(FREENECT_AUDIO_HEADER
  NAMES libfreenect_audio.h
  PATHS ${FREENECT_INCLUDE_DIRS}
  PATH_SUFFIXES libfreenect
)

if(NOT FREENECT_AUDIO_HEADER)
  message(FATAL_ERROR
    "libfreenect_audio.h not found.\n"
    "The installed libfreenect may not have been compiled with audio support.\n"
    "Build from source:\n"
    "  git clone https://github.com/OpenKinect/libfreenect.git\n"
    "  mkdir libfreenect/build && cd libfreenect/build\n"
    "  cmake .. -DBUILD_AUDIO=ON -DCMAKE_INSTALL_PREFIX=/usr/local\n"
    "  make -j$(nproc) && sudo make install && sudo ldconfig")
else()
  message(STATUS "libfreenect_audio.h : ${FREENECT_AUDIO_HEADER}")
endif()

# ── Executable ───────────────────────────────────────────────────────────────
add_executable(kinect_audio src/kinect_audio_node.cpp)

target_include_directories(kinect_audio PRIVATE
  ${FREENECT_INCLUDE_DIRS}
)

target_link_libraries(kinect_audio
  ${FREENECT_LIBRARIES}
)

# Pass any extra linker flags from pkg-config (e.g. -lusb-1.0)
if(FREENECT_LDFLAGS_OTHER)
  target_link_options(kinect_audio PRIVATE ${FREENECT_LDFLAGS_OTHER})
endif()

ament_target_dependencies(kinect_audio
  rclcpp
  std_msgs
)

# ── Install ───────────────────────────────────────────────────────────────────
install(TARGETS kinect_audio
  DESTINATION lib/${PROJECT_NAME}
)

# ── Ament ─────────────────────────────────────────────────────────────────────
ament_package()
