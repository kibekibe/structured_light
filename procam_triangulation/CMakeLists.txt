cmake_minimum_required (VERSION 2.8)
project (procam_triangulation)


add_executable(
    procam_triangulation
    main_procam_triangulation.cpp
)

find_package(OpenCV REQUIRED)


if(OpenCV_FOUND)
  include_directories(${OpenCV_INCLUDE_DIRS})
  target_link_libraries(procam_triangulation ${OpenCV_LIBS})
endif(OpenCV_FOUND)


