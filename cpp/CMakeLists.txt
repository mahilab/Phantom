cmake_minimum_required(VERSION 3.0.0)
project(phantom VERSION 0.1.0)

include(FetchContent) 
FetchContent_Declare(mahi-robo GIT_REPOSITORY https://github.com/mahilab/mahi-robo.git) 
FetchContent_MakeAvailable(mahi-robo)
FetchContent_Declare(mahi-com GIT_REPOSITORY https://github.com/mahilab/mahi-com.git) 
FetchContent_MakeAvailable(mahi-com)
# FetchContent_Declare(mahi-gui GIT_REPOSITORY https://github.com/mahilab/mahi-gui.git) 
# FetchContent_MakeAvailable(mahi-gui)

add_library(phantom SHARED dll.cpp)
target_link_libraries(phantom mahi::robo mahi::com)

add_executable(phantom_test main.cpp)
target_link_libraries(phantom_test mahi::robo mahi::com)