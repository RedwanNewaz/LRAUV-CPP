cmake_minimum_required(VERSION 3.10)
project(LRAUV)

set(CMAKE_CXX_STANDARD 17)
find_package(Armadillo REQUIRED)
add_subdirectory(DespotBase/despot)

include_directories(
        include
        DespotBase/despot/include
        ${ARMADILLO_INCLUDE_DIRS}
        )

add_executable(LRAUV main.cpp include/FlowField.h include/EKF.h include/RobotModel.h lauv/LAVU.cpp lauv/LAVU.h)
add_subdirectory(navigation)
target_link_libraries("${PROJECT_NAME}"
        despot
        ${ARMADILLO_LIBRARIES}
        )



