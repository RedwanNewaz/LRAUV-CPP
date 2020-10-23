cmake_minimum_required(VERSION 3.15)
project(LRAUV_MCTS)

set(CMAKE_CXX_STANDARD 17)

set(Boost_USE_STATIC_LIBS   ON)
set(OpenCV_STATIC ON)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake") # CMAKE_CURRENT_LIST_DIR is the directory holding the CMakeLists.txt we are currently using
find_package(Armadillo REQUIRED)
include_directories(include
        ${ARMADILLO_INCLUDE_DIR}
        )
set(HEADER_FILES include/EKF.h include/RobotModel.h include/FlowField.h include/MyHelper.h src/TreeNode.h)
set(SOURCE_FILES src/MCTS.cpp src/MCTS.h)
add_library(MCTS ${HEADER_FILES} ${SOURCE_FILES})


find_package(OpenCV REQUIRED)
include_directories( ${OpenCV_INCLUDE_DIRS})

find_package(Boost COMPONENTS filesystem program_options REQUIRED)
include_directories( ${Boost_INCLUDE_DIRS})
add_executable(LRAUV_MCTS main.cpp include/SimulationView.h)
target_link_libraries(${PROJECT_NAME} PRIVATE
        ${ARMADILLO_LIBRARY}
        MCTS
        ${OpenCV_LIBS}
        ${Boost_LIBRARIES}
        )


