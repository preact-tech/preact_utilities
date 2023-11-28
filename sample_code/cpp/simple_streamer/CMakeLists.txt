#
# examples/simple-streamer
#
cmake_minimum_required(VERSION 3.16 FATAL_ERROR)
project(simple_streamer )

find_package(Threads REQUIRED)
set(Boost_USE_STATIC_LIBS   ON)
set(THREADS_PREFER_PTHREAD_FLAG ON) # Without this flag CMake may resort to just '-lpthread'

find_package(Boost COMPONENTS program_options REQUIRED)
find_package(ToFCore REQUIRED)
find_package(usbp  REQUIRED)
find_package(OpenCV REQUIRED )
include_directories(
 ${OpenCV_INCLUDE_DIRS}
)

add_executable(simple-streamer simple-streamer.cpp)
target_link_libraries(simple-streamer tofcore pthread Boost::program_options tofcore usbp  ${OpenCV_LIBS})
