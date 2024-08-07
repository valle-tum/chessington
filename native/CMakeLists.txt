cmake_minimum_required(VERSION 3.1)

PROJECT(bouncy)

INCLUDE(FetchContent)

FetchContent_Declare(godot_api
GIT_REPOSITORY https://github.com/godotengine/godot-headers.git
GIT_TAG        godot-3.5.1-stable
)

FetchContent_MakeAvailable(godot_api)

SET(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}")

SET(CMAKE_CXX_STANDARD 17)
# SET(OpenCV_DIR .../opencv-4.9.0/opencv/build/x64/vc16/lib) # added
# FIND_PACKAGE(OpenCV REQUIRED)

FIND_PACKAGE(OpenCV REQUIRED PATHS /usr/local/include(opencv4)) # added
INCLUDE_DIRECTORIES(AFTER "${OpenCV_INCLUDE_DIRS}" "include")
LINK_LIBRARIES(${OpenCV_LIBS})

ADD_LIBRARY(bouncy SHARED camera.cpp processing.cpp wrapper.c chessboard.cpp )

INCLUDE_DIRECTORIES(${godot_api_SOURCE_DIR})
