cmake_minimum_required(VERSION 2.6)
project(sc2pcr)
find_package(PCL)
set (CMAKE_CXX_STANDARD 11)

include_directories(./include/)
include_directories(${PCL_INCLUDE_DIRS})
# include_directories(${LIBLAS_INCLUDE_DIR})

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})



list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")
add_executable(${PROJECT_NAME} src/main.cpp src/pcr.cpp src/FPFHPCR.cpp )
target_link_libraries (${PROJECT_NAME} ${PCL_LIBRARIES})
install(TARGETS ${PROJECT_NAME} RUNTIME DESTINATION bin)
