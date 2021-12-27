# This file was automatically generated for projects
# without default 'CMakeLists.txt' file.

cmake_minimum_required(VERSION 3.1)
set(CMAKE_CXX_FLAGS_DEBUG "-O0 -g -std=c++17")
set(COMPONENT_ADD_INCLUDEDIRS
    "."
    "include"
)
set(COMPONENT_SRCS 
    "main.cpp"
    "task/motion_planning.cpp"
    "task/planning_task.cpp"
    "task/ui.cpp"
    "task/buzzer_task.cpp"
    "task/integrated_entity.cpp"
    "task/main_task.cpp"
    "task/icm20689.cpp"
    "task/sensing_task.cpp"
    "search/adachi.cpp"
    "search/logic.cpp"
    )
      
register_component()