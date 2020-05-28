cmake_minimum_required(VERSION 3.0.0)
project(phantom VERSION 0.1.0)

include(FetchContent) 
FetchContent_Declare(mahi-robo GIT_REPOSITORY https://github.com/mahilab/mahi-robo.git) 
FetchContent_MakeAvailable(mahi-robo)
FetchContent_Declare(mahi-com GIT_REPOSITORY https://github.com/mahilab/mahi-com.git) 
FetchContent_MakeAvailable(mahi-com)
FetchContent_Declare(mahi-gui GIT_REPOSITORY https://github.com/mahilab/mahi-gui.git) 
FetchContent_MakeAvailable(mahi-gui)
FetchContent_Declare(mahi-daq GIT_REPOSITORY https://github.com/mahilab/mahi-daq.git) 
FetchContent_MakeAvailable(mahi-daq)

add_library(phantom SHARED src/dll.cpp src/PhantomModel.hpp src/PhantomModel.cpp)
target_link_libraries(phantom mahi::robo mahi::com mahi::gui)
add_custom_command(TARGET phantom POST_BUILD 
               COMMAND ${CMAKE_COMMAND} -E copy_if_different
               $<TARGET_FILE:phantom>              
               "${PROJECT_SOURCE_DIR}/../unity/Phantom/Assets/Phantom/Plugins/")

add_executable(phantom_control src/control.cpp src/PhantomModel.hpp src/PhantomModel.cpp)
target_link_libraries(phantom_control mahi::robo mahi::com mahi::gui mahi::daq)