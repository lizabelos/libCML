if(NOT DEFINED CMAKE_MINIMUM_REQUIRED_VERSION)
  cmake_minimum_required(VERSION 2.8.7)
endif()

set(JPEG_NAMESPACE "GDCMJPEG")
set(JPEG_STANDALONE 1)
string(TOLOWER ${JPEG_NAMESPACE} JPEG_LIBRARY_NAME)


project(${JPEG_NAMESPACE} CXX)

foreach(p
    CMP0042
    CMP0063
    )
  if(POLICY ${p})
    cmake_policy(SET ${p} NEW)
  endif()
endforeach()

# Do full dependency headers.
include_regular_expression("^.*$")

# let advanced user the option to define JPEG_API_VERSION:
if(NOT DEFINED JPEG_API_VERSION)
  set(JPEG_API_VERSION "${GDCM_MAJOR_VERSION}.${GDCM_MINOR_VERSION}")
endif()
set(JPEG_LIBRARY_PROPERTIES ${JPEG_LIBRARY_PROPERTIES}
  VERSION "${GDCM_VERSION}"
  SOVERSION "${JPEG_API_VERSION}"
)


# memmgr back ends: compile only one of these into a working library
# (For now, let's use the mode that requires the image fit into memory.
# This is the recommended mode for Win32 anyway.)
set(systemdependent_SRCS jmemnobs.cpp)

# library object files common to compression and decompression
set(common_SRCS
jcomapi.cpp jutils.cpp jerror.cpp jmemmgr.cpp
#jmemsrc.cpp
)

# compression library object files
set(compression_SRCS
jcapimin.cpp jcapistd.cpp jctrans.cpp jcparam.cpp  jcinit.cpp
jcmaster.cpp jcmarker.cpp jcmainct.cpp jcprepct.cpp jccoefct.cpp jccolor.cpp
jcsample.cpp jchuff.cpp jcphuff.cpp jcdctmgr.cpp jfdctfst.cpp jfdctflt.cpp
jdatadst.cpp
)

# Lossy (DCT) codec:
set(comp_lossy_SRCS
jfdctint.cpp
jclossy.cpp

# lossless
jclossls.cpp

#jdarith.cpp
jcodec.cpp
jcscale.cpp
jcshuff.cpp
jclhuff.cpp
jcpred.cpp
jcdiffct.cpp
)

# decompression library object files
set(decompression_SRCS
jdapimin.cpp jdapistd.cpp jdtrans.cpp  jdmaster.cpp
jdinput.cpp jdmarker.cpp jdhuff.cpp jdphuff.cpp jdmainct.cpp jdcoefct.cpp
jdpostct.cpp jddctmgr.cpp jidctfst.cpp jidctflt.cpp jidctint.cpp jidctred.cpp
jdsample.cpp jdcolor.cpp jquant1.cpp jquant2.cpp jdmerge.cpp
jdatasrc.cpp
)

set(decomp_lossy_SRCS
 jdlossls.cpp jdlossy.cpp
 #jcarith.cpp
jdlhuff.cpp
jddiffct.cpp
jdpred.cpp
jdscale.cpp
jdshuff.cpp
)

set(JPEG_HEADER_FILES
  jchuff.h
  jdct.h
  jdhuff.h
  jerror.h
  jinclude.h
  jmemsys.h
  jmorecfg.h
  jpegint.h
  jpeglib.h
  jversion.h
  jlossy.h
  jlossls.h
  )

set(JPEG_SOURCES)
foreach(src
  ${systemdependent_SRCS}
  ${common_SRCS}
  ${compression_SRCS}
  ${decompression_SRCS}
  ${comp_lossy_SRCS}
  ${decomp_lossy_SRCS}
  )
  set(JPEG_SOURCES ${JPEG_SOURCES}
    "${CMAKE_CURRENT_SOURCE_DIR}/${src}")
endforeach()

#if (BUILD_SHARED_LIBS)
#  set(JPEGDLL 1)
#else ()
  set(JPEGSTATIC 1)
#endif ()

# Generate three libraries: jpeg 8bits, 12bits and 16bits
# using only one source directory

#-----------------------------------------------------------------------------
include (${CMAKE_ROOT}/Modules/CheckIncludeFile.cmake)
CHECK_INCLUDE_FILE("stddef.h"       HAVE_STDDEF_H)
CHECK_INCLUDE_FILE("stdlib.h"       HAVE_STDLIB_H)

include_directories(${CMAKE_CURRENT_BINARY_DIR})
# Particular case jpegcmake.h needs to be specified either 8, 12 or 16 bits:
# also configure the name mangling
foreach(bits 8)
  set(CMAKE_BITS_IN_JSAMPLE ${bits})
  configure_file(${CMAKE_CURRENT_SOURCE_DIR}/jpegcmake.h.in
                 ${CMAKE_CURRENT_BINARY_DIR}/${bits}/jpegcmake.h
                 @ONLY)
  set(MANGLE_PREFIX ${JPEG_LIBRARY_NAME}${bits})
  configure_file(${CMAKE_CURRENT_SOURCE_DIR}/mangle_jpeg.h.in
                 ${CMAKE_CURRENT_BINARY_DIR}/${bits}/mangle_jpeg${bits}bits.h
                 @ONLY)


  # FIXME FIXME FIXME
  configure_file(${CMAKE_CURRENT_SOURCE_DIR}/jmorecfg.h
                 ${CMAKE_CURRENT_BINARY_DIR}/${bits}/jmorecfg.h
                 COPYONLY)
  configure_file(${CMAKE_CURRENT_SOURCE_DIR}/jconfig.h
                 ${CMAKE_CURRENT_BINARY_DIR}/${bits}/jconfig.h
                 COPYONLY)
  configure_file(${CMAKE_CURRENT_SOURCE_DIR}/jpeglib.h
                 ${CMAKE_CURRENT_BINARY_DIR}/${bits}/jpeglib.h
                 COPYONLY)
  configure_file(${CMAKE_CURRENT_SOURCE_DIR}/jerror.h
                 ${CMAKE_CURRENT_BINARY_DIR}/${bits}/jerror.h
                 COPYONLY)
  configure_file(${CMAKE_CURRENT_SOURCE_DIR}/jinclude.h
                 ${CMAKE_CURRENT_BINARY_DIR}/${bits}/jinclude.h
                 COPYONLY)
endforeach()

# I could not get the depencencies in CMake to work properly so instead
# have multiple directory for each target that will ensure proper recompilation
# of the objects files
add_subdirectory(8)
#add_subdirectory(12)
#add_subdirectory(16)
