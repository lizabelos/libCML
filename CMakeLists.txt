cmake_minimum_required(VERSION 3.9)
project(CML
        VERSION 1.0
        LANGUAGES C CXX
        )

#######################################################################################################################
# # # # # # CMAKE SETTINGS  # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #
#######################################################################################################################

set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake" ${CMAKE_MODULE_PATH})
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})

set_property(GLOBAL PROPERTY USE_FOLDERS ON)
set_property(GLOBAL PROPERTY ALLOW_DUPLICATE_CUSTOM_TARGETS 1)

cmake_policy(SET CMP0072 NEW)

#######################################################################################################################
# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #
#######################################################################################################################




#######################################################################################################################
# # # # # # OPTIONS # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #
#######################################################################################################################

option(ENABLE_GUI "Enable GUI" ON)
option(USE_OPENMP "Use OpenMP" ON)
option(ENABLE_ASSERT "Enable assert on Release mode" OFF)

#######################################################################################################################
# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #
#######################################################################################################################




#######################################################################################################################
# # # # # # FIND ALL THE PACKAGES # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #
#######################################################################################################################

if (ANDROID)
    set(ENABLE_DEFAULT_EXTERNAL_LIB OFF)
else()
    set(ENABLE_DEFAULT_EXTERNAL_LIB ON)
endif()

find_package(Threads REQUIRED)

if (USE_OPENMP)
    find_package(OpenMP REQUIRED)
endif()

option(USE_GOOGLE_HASH "Use Google Hash" OFF)

# set(BLA_VENDOR Intel10_64lp)
find_package(BLAS QUIET)
find_package(LAPACK QUIET)

option(USE_INTERNAL_EIGEN "Use Internal Eigen" ON)
if (USE_INTERNAL_EIGEN)
    set(EIGEN3_LIBRARY "")
    set(EIGEN3_INCLUDE_DIR "${CMAKE_SOURCE_DIR}/thirdparty/eigen")
    set(EIGEN3_FOUND 1)
else()
    find_package(Eigen3 REQUIRED)
    set(EIGEN3_LIBRARY Eigen3::Eigen)
endif()

option(USE_INTERNAL_SOPUHS "Use Internal Eigen" ON)
if (USE_INTERNAL_EIGEN)
    set(SOPHUS_INCLUDE_DIR "${CMAKE_SOURCE_DIR}/thirdparty/Sophus")
    set(SOPHUS_FOUND 1)
else()
    find_package(Sophus REQUIRED)
endif()

add_subdirectory(thirdparty/g2o)

if (ANDROID)
    set(AVFORMAT_FOUND 0)
    set(AVCODEC_FOUND 0)
    set(AVUTIL_FOUND 0)
    set(SWSCALE_FOUND 0)
else()
find_package(PkgConfig QUIET)
if (PKG_CONFIG_FOUND)
    option(ENABLE_FFMPEG "Enable FFMPEG" ${ENABLE_DEFAULT_EXTERNAL_LIB})
    if (ENABLE_FFMPEG)
        pkg_check_modules(AVFORMAT REQUIRED "libavformat")
        pkg_check_modules(AVCODEC REQUIRED "libavcodec")
        pkg_check_modules(AVUTIL REQUIRED "libavutil")
        pkg_check_modules(SWSCALE REQUIRED "libswscale")
    else()
        set(AVFORMAT_FOUND 0)
        set(AVCODEC_FOUND 0)
        set(AVUTIL_FOUND 0)
        set(SWSCALE_FOUND 0)
    endif()
else()
    set(AVFORMAT_FOUND 0)
    set(AVCODEC_FOUND 0)
    set(AVUTIL_FOUND 0)
    set(SWSCALE_FOUND 0)
endif()
endif()

option(USE_INTERNAL_CERES "Use Internal Ceres" ON)
if (USE_INTERNAL_CERES)
    add_subdirectory(thirdparty/ceres-solver)
    set(CERES_LIBRARIES ceres)
    set(CERES_INCLUDE_DIR "${CMAKE_SOURCE_DIR}/thirdparty/ceres-solver/include" "${CMAKE_SOURCE_DIR}/thirdparty/ceres-solver/internal/ceres")
    set(CERES_FOUND 1)
else()
    find_package(Ceres QUIET)
    if (CERES_FOUND)
        find_package(GLOG QUIET)
    endif()
endif()

find_package(TIFF COMPONENTS CXX REQUIRED)

option(ENABLE_LIBZIP "Enable LibZIP" ${ENABLE_DEFAULT_EXTERNAL_LIB})
if (ENABLE_LIBZIP)
    find_package(LibZip REQUIRED)
endif()

# if (NOT TORCH_FOUND)
#     find_package(Torch QUIET)
# endif()


option(ENABLE_OPENCV "Enable OpenCV (Version 4)" OFF)
if (ENABLE_OPENCV)
    find_package(OpenCV 4.0 REQUIRED)
endif()

option(USE_INTERNAL_YAML_CPP "Use Internal Yaml-CPP" ON)
if (USE_INTERNAL_YAML_CPP)
    add_subdirectory(thirdparty/yaml-cpp)
    set(Yaml-cpp_FOUND ON)
    set(YAML_CPP_INCLUDE_DIR thirdparty/yaml-cpp/include)
else()
    find_package(Yaml-cpp QUIET)
endif()

add_subdirectory(thirdparty/gdcmjpeg)

if (ENABLE_GUI)
    set(OpenGL_GL_PREFERENCE GLVND)
    # https://bugreports.qt.io/browse/QTCREATORBUG-25793
    #find_package(QT NAMES Qt6 COMPONENTS Core Widgets OpenGL OpenGLWidgets PrintSupport Multimedia REQUIRED)
    find_package(Qt6 CONFIG REQUIRED Core Widgets OpenGL OpenGLWidgets PrintSupport Multimedia Charts)
    file(COPY resources/style.css DESTINATION ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})
endif()

#if (ANDROID)
    #set(ANDROID_PLATFORM_LEVEL 30)
    #include(android/AddQtAndroidApk.cmake)
#endif()

#######################################################################################################################
# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #
#######################################################################################################################




#######################################################################################################################
# # # # # # SET C++ PREFERENCES # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #
#######################################################################################################################

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_EXTENSIONS ON)
set(OpenGL_GL_PREFERENCE LEGACY)

include(CheckCXXCompilerFlag)
check_cxx_compiler_flag(-fPIC COMPILER_SUPPORTS_FPIC)

if (COMPILER_SUPPORTS_FPIC)
    set(COMPILER_FLAG_FPIC "-fPIC")
else()
    set(COMPILER_FLAG_FPIC "")
endif()

set(CMAKE_EXE_LINKER_FLAGS "-export-dynamic")

include_directories(${CMAKE_BINARY_DIR})

#######################################################################################################################
# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #
#######################################################################################################################




#######################################################################################################################
# # # # # # CML CONFIGURATION  # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #
#######################################################################################################################

# Configuration file
set(CML_HAVE_AVFORMAT ${AVFORMAT_FOUND})
set(CML_HAVE_AVCODEC ${AVCODEC_FOUND})
set(CML_HAVE_AVUTIL ${AVUTIL_FOUND})
set(CML_HAVE_SWSCALE ${SWSCALE_FOUND})
set(CML_HAVE_LIBZIP ${LIBZIP_FOUND})
set(CML_HAVE_CERES ${CERES_FOUND})
set(CML_HAVE_G2O ${G2O_FOUND})
set(CML_HAVE_YAML_CPP ${Yaml-cpp_FOUND})
set(CML_HAVE_OPENCV ${OPENCV_FOUND})
set(CML_USE_OPENMP ${USE_OPENMP})
set(CML_ENABLE_GUI ${ENABLE_GUI})
set(CML_USE_GOOGLE_HASH ${USE_GOOGLE_HASH})
set(CML_IS_ANDROID ${ANDROID})
configure_file(include/cml/config.h.in include/cml/config.h)
include_directories(${CMAKE_CURRENT_BINARY_DIR}/include)

#######################################################################################################################
# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #
#######################################################################################################################




#######################################################################################################################
# # # # # # CML SOURCE CODE  # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #
#######################################################################################################################

include_directories(CML include)
include_directories(CML src)

add_subdirectory(src)

target_compile_options(CML PUBLIC ${COMPILER_FLAG_FPIC})

#if (NOT ANDROID)
#
#endif()
#target_compile_definitions(CML PUBLIC -DEIGEN_MAX_ALIGN_BYTES=64)
#target_compile_definitions(CML PUBLIC -DENABLE_SSE=1)
#target_compile_definitions(CML PUBLIC -DEIGEN_USE_MKL_ALL=1)

# target_compile_options(CML PUBLIC -Wall -Werror -Wuninitialized)

# target_link_options(CML PUBLIC -mllvm)

target_compile_options(CML PUBLIC ${OpenMP_CXX_FLAGS} -static)
target_link_options(CML PUBLIC ${OpenMP_CXX_FLAGS} -static)


if (CMAKE_BUILD_TYPE STREQUAL "Debug")
    target_compile_definitions(CML PUBLIC -DDEBUG=1)
    target_compile_options(CML PUBLIC -g)
elseif (CMAKE_BUILD_TYPE STREQUAL "Release")
    target_compile_options(CML PUBLIC -O2 -fno-math-errno)
elseif (CMAKE_BUILD_TYPE STREQUAL "RelWithDebInfo")
    target_compile_options(CML PUBLIC -Og -fno-math-errno -g)
    target_compile_definitions(CML PUBLIC -DNDEBUG=1)
endif()

if (USE_OPENMP)
    target_compile_options(CML PUBLIC ${OpenMP_CXX_FLAGS})
    target_link_options(CML PUBLIC ${OpenMP_CXX_FLAGS})
endif()

if (ENABLE_ASSERT)
    target_compile_definitions(CML PUBLIC ENABLE_ASSERTTHROW_ON_RELEASE=1)
endif()

file(COPY resources DESTINATION ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})

#######################################################################################################################
# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #
#######################################################################################################################




#######################################################################################################################
# # # # # # CML DOCUMENTATION  # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #
#######################################################################################################################

# Only do these if this is the main project, and not if it is included through add_subdirectory
if(CMAKE_PROJECT_NAME STREQUAL PROJECT_NAME)

    # Find Doxygen
    find_package(Doxygen)
    if(Doxygen_FOUND)
        message("Doxygen build started")
        # note the option ALL which allows to build the docs together with the application
        add_custom_target(doc_doxygen ALL COMMAND ${DOXYGEN_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/Doxyfile WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR} COMMENT "Generating API documentation with Doxygen" VERBATIM )
    else()
        message(STATUS "Doxygen not found, not building docs")
    endif()

endif()

#######################################################################################################################
# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #
#######################################################################################################################
