cmake_minimum_required(VERSION 3.17)
project(LRAUV_MCTS)

set(CMAKE_CXX_STANDARD 20)

include_directories(include)

find_package(Python3 COMPONENTS Development NumPy)
find_package(Armadillo REQUIRED)

set(HEADER_FILES
        include/EKF.h
        include/RobotModel.h
        include/FlowField.h include/MyHelper.h src/TreeNode.h)


set(SOURCE_FILES
        src/MCTS.cpp src/MCTS.h)

add_executable(LRAUV_MCTS main.cpp ${HEADER_FILES} ${SOURCE_FILES})

target_include_directories(${PROJECT_NAME}
        PRIVATE ${Python3_INCLUDE_DIRS} ${Python3_NumPy_INCLUDE_DIRS}
        PUBLIC ${ARMADILLO_INCLUDE_DIRS}
        )
target_link_libraries(${PROJECT_NAME}
        Python3::Python Python3::NumPy
        ${ARMADILLO_LIBRARIES}
        )