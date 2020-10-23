set(ARMADILLO_PATH
        ${PROJECT_SOURCE_DIR}/ThirdParty/armadillo
        )

find_path(ARMADILLO_INCLUDE_DIR armadillo   # The variable to store the path in and the name of the header files
        PATH_SUFFIXES include               # The folder name containing the header files
        PATHS ${ARMADILLO_PATH})       # Where to look (defined above)

find_library(ARMADILLO_LIBRARY               # The variable to store where it found the .a files
        NAMES armadillo                  # The name of the .a file (without the extension and without the 'lib')
        PATHS ${ARMADILLO_PATH}
        )
link_directories(
        ${PROJECT_SOURCE_DIR}/ThirdParty/armadillo
)
link_libraries(${ARMADILLO_LIBRARY} mkl_core mkl_sequential mkl_intel_lp64)


