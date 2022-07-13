cmake_minimum_required(VERSION 3.22)
project(SemiGlobalMatchingLearning)

set(CMAKE_CXX_STANDARD 14)


set(OpenCV_DIR "D:\\DevTool\\OpenCV\\opencv-3.4.16\\mingw-build")
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(SemiGlobalMatchingLearning main.cpp SemiGlobalMatching.cpp SemiGlobalMatching.h SGMType.h SGMUtil.cpp SGMUtil.h)
target_link_libraries(SemiGlobalMatchingLearning ${OpenCV_LIBS})