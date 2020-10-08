cmake_minimum_required(VERSION 2.8.3)

include_directories(
        src
)



add_executable("${PROJECT_NAME}_navigation"
  src/navigation.cpp
  src/main.cpp
)
target_link_libraries("${PROJECT_NAME}_navigation"
        despot
)
