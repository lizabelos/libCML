# JPEG 8 bits project

include_directories(BEFORE
  ${CMAKE_CURRENT_BINARY_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR})
add_library(${JPEG_LIBRARY_NAME}8 ${JPEG_SOURCES})
