cmake_minimum_required (VERSION 2.8)
project (projcalib_from_decode_imgs)


add_executable(
    projcalib_from_decode_imgs
    main_projcalib.cpp
)

find_package(OpenCV REQUIRED)


if(OpenCV_FOUND)
  include_directories(${OpenCV_INCLUDE_DIRS})
  target_link_libraries(projcalib_from_decode_imgs ${OpenCV_LIBS})
endif(OpenCV_FOUND)


