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


add_subdirectory(3rdparty/iir)

add_executable(phantom_control src/phantom_control.cpp src/PhantomModel.hpp src/PhantomModel.cpp src/PhantomHardware.hpp 3rdparty/ImGuiColorTextEdit/TextEditor.cpp)
target_link_libraries(phantom_control mahi::robo mahi::com mahi::gui mahi::daq iir_static)
target_include_directories(phantom_control PRIVATE "3rdparty/iir" "3rdparty/ImGuiColorTextEdit")

add_library(phantom_plugin SHARED src/phantom_plugin.cpp src/PhantomModel.hpp src/PhantomModel.cpp src/PhantomHardware.hpp)
target_link_libraries(phantom_plugin mahi::robo mahi::com mahi::gui mahi::daq iir_static)
target_include_directories(phantom_plugin PRIVATE "3rdparty/iir/")
add_custom_command(TARGET phantom_plugin POST_BUILD 
               COMMAND ${CMAKE_COMMAND} -E copy_if_different
               $<TARGET_FILE:phantom_plugin>              
               "${PROJECT_SOURCE_DIR}/../unity/Phantom/Assets/Phantom/Plugins/")
add_custom_command(TARGET phantom_plugin POST_BUILD 
               COMMAND ${CMAKE_COMMAND} -E copy_if_different
               $<TARGET_FILE:phantom_plugin>              
               "${PROJECT_SOURCE_DIR}/../unity/Phantom/Build/Phantom_Data/Plugins/")