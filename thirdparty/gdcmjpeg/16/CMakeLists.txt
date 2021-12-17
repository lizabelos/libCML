# JPEG 16 bits project

include_directories(BEFORE
  ${CMAKE_CURRENT_BINARY_DIR}
  ${ITKJPEG_SOURCE_DIR})
add_library(${JPEG_LIBRARY_NAME}16 STATIC ${JPEG_SOURCES})

