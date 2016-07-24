cmake_minimum_required (VERSION 2.8)
project (gray_code)


add_executable(
    decode_gray_code
    main_decode.cpp
)

add_executable(
    encode_gray_code
    main_encode.cpp
)

find_package(OpenCV REQUIRED)


if(OpenCV_FOUND)
  include_directories(${OpenCV_INCLUDE_DIRS})
  target_link_libraries(encode_gray_code ${OpenCV_LIBS})
  target_link_libraries(decode_gray_code ${OpenCV_LIBS})
endif(OpenCV_FOUND)


