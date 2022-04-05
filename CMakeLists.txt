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
set(ANDROID_PACKAGE_SOURCE_DIR ${PROJECT_SOURCE_DIR}/android CACHE INTERNAL "")

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
option(USE_OPENMP "Use OpenMP" OFF)
option(ENABLE_ASSERT "Enable assert on Release mode" OFF)

if (CMAKE_BUILD_TYPE STREQUAL "Release")
    if (ANDROID)
        add_compile_options(-O3)
        add_link_options(-O3)
    else()
        add_compile_options(-march=native -O3)
        add_link_options(-march=native -O3)
    endif()
endif()

#######################################################################################################################
# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #
#######################################################################################################################




#######################################################################################################################
# # # # # # FIND ALL THE PACKAGES # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #
#######################################################################################################################

find_package(Threads REQUIRED)

if (USE_OPENMP)
    find_package(OpenMP REQUIRED)
endif()

option(USE_GOOGLE_HASH "Use Google Hash" OFF)

# set(BLA_VENDOR Intel10_64lp)
# find_package(BLAS QUIET)
# find_package(LAPACK QUIET)

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
    option(ENABLE_FFMPEG "Enable FFMPEG" OFF)
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

option(USE_INTERNAL_LIBTIFF "Use Internal libtiff" ON)
if (USE_INTERNAL_LIBTIFF)
    add_subdirectory(thirdparty/libtiff)
    set(LIBTIFF_LIBRARIES tiffxx)
    set(LIBTIFF_INCLUDE_DIR thirdparty/libtiff/libtiff)
    set(LIBTIFF_FOUND 1)
else()
    find_package(TIFF COMPONENTS CXX REQUIRED)
    set(LIBTIFF_LIBRARIES TIFF::TIFF tiffxx)
endif()

option(USE_INTERNAL_LIBZIP "Use Internal LibZIP" ON)
if (USE_INTERNAL_LIBZIP)
    add_subdirectory(thirdparty/libzip)
    set(LIBZIP_LIBRARY libzip::zip)
    set(LIBZIP_INCLUDE_DIR thirdparty/libzip/lib)
    set(LIBZIP_FOUND 1)
else()
    if (ENABLE_LIBZIP)
        find_package(LibZip REQUIRED)
    endif()
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

file(GLOB_RECURSE HEADER_LIST CONFIGURE_DEPENDS include/*.h)

add_library(CML STATIC

        ${HEADER_LIST}

        src/cml/types.cpp

        src/cml/base/AbstractSlam.cpp
        src/cml/base/Statistic.cpp

        src/cml/capture/CaptureImage.cpp
        src/cml/capture/KittyCapture.cpp
        src/cml/capture/TUMCapture.cpp
        src/cml/capture/EurocCapture.cpp
        src/cml/capture/TartanairCapture.cpp
        src/cml/capture/Eth3DCapture.cpp
        src/cml/capture/RobotCarCapture.cpp

        src/cml/features/corner/PixelSelector.cpp

        src/cml/image/Array2D.cpp
        src/cml/image/Filter.cpp

        src/cml/map/InternalCalibration.cpp
        src/cml/map/Camera.cpp
        src/cml/map/Frame.cpp
        src/cml/map/Hasher.cpp
        src/cml/map/Map.cpp
        src/cml/map/MapObject.cpp
        src/cml/map/Edge.cpp

        src/cml/maths/Rotation.cpp

        src/cml/optimization/Triangulation.cpp
        src/cml/optimization/dso/DSOTracer.cpp
        src/cml/optimization/EPnP.cpp
        src/cml/optimization/dso/DSOTracker.cpp
        src/cml/optimization/dso/DSOBundleAdjustment.cpp
        src/cml/optimization/dso/DSOInitializer.cpp
        src/cml/optimization/IndirectPointOptimizer.cpp

        src/cml/nn/Precomputed.cpp

        src/cml/robust/RobustFundamental8Points.cpp
        src/cml/robust/RobustHomography.cpp
        src/cml/robust/CameraChecker.cpp
        src/cml/robust/RobustPoseLoss.cpp
        src/cml/robust/RobustRaulmurInitializer.cpp

        src/cml/utils/Logger.cpp
        src/cml/utils/GarbageCollector.cpp

        src/cml/gui/viewer/ReprojectionViewer.cpp
        src/cml/gui/viewer/VarianceViewer.cpp

        src/cml/features/bow/BowVector.cpp
        src/cml/features/bow/FeatureVector.cpp
        src/cml/features/bow/QueryResults.cpp
        src/cml/features/bow/ScoringObject.cpp
        src/cml/features/bow/Relocalization.cpp

        src/cml/features/corner/ORB.cpp
        src/cml/features/corner/FAST.cpp
        src/cml/features/corner/BRIEF.cpp

        src/cml/features/cornerTracker/LSHTracker.cpp
        src/cml/features/cornerTracker/CornerMatcher.cpp
        src/cml/features/cornerTracker/BoWTracker.cpp
        src/cml/features/cornerTracker/VFC.cpp

        src/lodepng/lodepng.cpp
)

target_link_libraries(CML Threads::Threads ${EIGEN3_LIBRARY} ${SuiteSparse_LIBRARIES} ${LIBTIFF_LIBRARIES} gdcmjpeg8)
target_include_directories(CML PUBLIC ${EIGEN3_INCLUDE_DIR} ${SOPHUS_INCLUDE_DIR} ${LIBZIP_INCLUDE_DIR} ${LIBTIFF_INCLUDE_DIR} /usr/include/suitesparse)

if (USE_OPENMP)
    target_link_libraries(CML OpenMP::OpenMP_CXX)
endif()

if (Yaml-cpp_FOUND)
    target_link_libraries(CML yaml-cpp)
    target_include_directories(CML PUBLIC ${YAML_CPP_INCLUDE_DIR})
    else()
    message(WARNING yaml-cpp not found)
endif()

if (AVFORMAT_FOUND AND AVCODEC_FOUND AND AVUTIL_FOUND AND SWSCALE_FOUND)

    target_include_directories(CML PUBLIC ${AVFORMAT_INCLUDE_DIRS})
    target_link_directories(CML PUBLIC ${AVFORMAT_LIBRARY_DIRS})
    target_include_directories(CML PUBLIC ${AVCODEC_INCLUDE_DIRS})
    target_link_directories(CML PUBLIC ${AVCODEC_LIBRARY_DIRS})
    target_include_directories(CML PUBLIC ${AVUTIL_INCLUDE_DIRS})
    target_link_directories(CML PUBLIC ${AVUTIL_LIBRARY_DIRS})
    target_include_directories(CML PUBLIC ${SWSCALE_INCLUDE_DIRS})
    target_link_directories(CML PUBLIC ${SWSCALE_LIBRARY_DIRS})

    target_sources(CML PRIVATE cml/capture/VideoCapture.cpp)
    target_link_libraries(CML ${AVFORMAT_LIBRARIES} ${AVCODEC_LIBRARIES} ${AVUTIL_LIBRARIES} ${SWSCALE_LIBRARIES})

    set(FFMPEG_FOUND 1 PARENT_SCOPE)

else()
    set(FFMPEG_FOUND 0 PARENT_SCOPE)
endif()

if (LIBZIP_FOUND)
    target_link_libraries(CML ${LIBZIP_LIBRARY})
endif()

if (CHOLMOD_FOUND)
    target_link_libraries(CML ${CHOLMOD_LIBRARIES})
    target_include_directories(CML PUBLIC ${CHOLMOD_INCLUDE_DIR})
endif()

#if (g2o_FOUND)
    target_sources(CML PRIVATE src/cml/optimization/g2o/IndirectBundleAdjustment.cpp src/cml/optimization/g2o/IndirectCameraOptimizer.cpp)
target_link_libraries(CML core types_sba types_slam3d)
target_include_directories(CML PUBLIC thirdparty/g2o)
target_include_directories(CML PUBLIC ${G2O_INCLUDE_DIRECTORY})
#    target_link_libraries(CML g2o::stuff g2o::core g2o::g2o_hierarchical_library g2o::g2o_hierarchical_library g2o::types_slam2d g2o::types_slam2d_addons g2o::types_data g2o::types_sclam2d g2o::types_slam3d  g2o::types_slam3d_addons g2o::types_sba g2o::types_icp g2o::types_sim3 g2o::solver_pcg g2o::solver_dense g2o::solver_slam2d_linear g2o::solver_structure_only g2o::solver_csparse g2o::csparse_extension g2o::solver_cholmod g2o::solver_eigen)
#endif()

if (CERES_FOUND)
    target_sources(CML PRIVATE src/cml/optimization/ceres/IndirectBundleAdjustment.cpp src/cml/optimization/ceres/PlaneInitializer.cpp src/cml/optimization/ceres/IndirectCameraOptimizer.cpp src/cml/evaluation/Alignment.cpp)
    target_link_libraries(CML ${CERES_LIBRARIES})
    target_include_directories(CML PUBLIC ${CERES_INCLUDE_DIR})
endif()

if (SUITESPARSE_FOUND)
    target_link_libraries(CML ${SUITESPARSE_LIBRARIES})
endif()

if (TORCH_FOUND)
    target_sources(CML PRIVATE src/cml/nn/MonoDepth.cpp src/cml/nn/Midas.cpp)
    target_link_libraries(CML torch_cpu c10)
endif()

if (OpenCV_FOUND)
    target_sources(CML PRIVATE src/cml/features/corner/OpenCV.cpp)
    target_include_directories(CML PUBLIC ${OpenCV_INCLUDE_DIRS})
    target_link_libraries(CML ${OpenCV_LIBS})
endif()

if (GLOG_FOUND)
    target_link_libraries(CML ${GLOG_LIBRARIES})
endif()

if (ENABLE_GUI)
    qt6_wrap_cpp(ModelViewerWidgetSRC include/cml/gui/widgets/ModelWidget.h)
    qt6_wrap_cpp(LoggerWidgetSRC include/cml/gui/widgets/LoggerWidget.h)
    qt6_wrap_cpp(CameraViewerWidgetSRC include/cml/gui/widgets/CameraViewerWidget.h)
    qt6_wrap_cpp(WebcamCaptureSRC include/cml/capture/QtWebcamCapture.h)
    qt6_wrap_cpp(SpoilerSRC include/cml/gui/widgets/Spoiler.h)
    qt6_wrap_cpp(GroupsWidgetSRC include/cml/gui/widgets/GroupsWidget.h)
    qt6_wrap_cpp(DesktopMainWindowSRC include/cml/gui/MainSlamWidget.h)

    target_sources(CML PRIVATE src/cml/gui/widgets/ModelWidget.cpp ${ModelViewerWidgetSRC}
            src/cml/gui/drawboard/QtDrawBoard.cpp
            src/cml/gui/widgets/LoggerWidget.cpp ${LoggerWidgetSRC}
            src/cml/gui/widgets/CameraViewerWidget.cpp ${CameraViewerWidgetSRC}
            src/cml/gui/widgets/Spoiler.cpp ${SpoilerSRC}
            src/cml/gui/widgets/GroupsWidget.cpp ${GroupsWidgetSRC}
            src/cml/gui/drawboard/QtDrawBoardShaders.cpp
            src/cml/capture/QtWebcamCapture.cpp ${WebcamCaptureSRC}
            src/cml/gui/MainSlamWidget.cpp ${DesktopMainWindowSRC}
            )


    if (NOT ANDROID)
        qt6_wrap_cpp(FunctionWidgetSRC include/cml/gui/widgets/FunctionWidget.h)
        qt6_wrap_cpp(FunctionListWidgetSRC include/cml/gui/widgets/FunctionListWidget.h)
        qt6_wrap_cpp(ParameterIntegerSliderSRC include/cml/gui/widgets/ParameterIntegerSlider.h)
        qt6_wrap_cpp(ParameterFloatingSliderSRC include/cml/gui/widgets/ParameterFloatingSlider.h)
        qt6_wrap_cpp(ParameterCheckboxSRC include/cml/gui/widgets/ParameterCheckbox.h)
        qt6_wrap_cpp(StatisticWidgetSRC include/cml/gui/widgets/StatisticWidget.h)

        target_sources(CML PRIVATE src/cml/gui/widgets/ModelWidget.cpp ${ModelViewerWidgetSRC}
                src/cml/gui/widgets/FunctionWidget.cpp ${FunctionWidgetSRC}
                src/cml/gui/widgets/FunctionListWidget.cpp ${FunctionListWidgetSRC}
                src/cml/gui/widgets/ParameterIntegerSlider.cpp ${ParameterIntegerSliderSRC}
                src/cml/gui/widgets/ParameterFloatingSlider.cpp ${ParameterFloatingSliderSRC}
                src/cml/gui/widgets/ParameterCheckbox.cpp ${ParameterCheckboxSRC}
                src/cml/gui/widgets/StatisticWidget.cpp ${StatisticWidgetSRC}

       )
    endif()

    target_link_libraries(CML Qt6::Core Qt6::Widgets Qt6::OpenGL Qt6::OpenGLWidgets Qt6::PrintSupport Qt6::Multimedia Qt6::Charts ${OPENGL_LIBRARIES})

    if (MINGW)
        target_link_libraries(CML opengl32 glu32 mingw32)
    endif()

endif()


if (ANDROID)
    add_library(modslam MODULE
                src/cml/slam/modslam.cpp
                src/cml/slam/modslam/Hybrid.cpp
                src/cml/slam/modslam/Research.cpp
                src/cml/slam/modslam/Make.cpp
                src/cml/slam/modslam/indirect/Tracking.cpp
                src/cml/slam/modslam/indirect/Mapping.cpp
                src/cml/slam/modslam/direct/Mapping.cpp
                src/cml/slam/modslam/direct/Tracking.cpp
                )
    target_link_libraries(modslam CML)
    target_include_directories(modslam PUBLIC include/cml/slam/modslam)


    qt_android_generate_deployment_settings(modslam)
    qt_android_add_apk_target(modslam)


    file(MAKE_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/android-build/libs/${ANDROID_ABI}/")
    add_custom_command(
        TARGET modslam
        POST_BUILD
        COMMAND ${CMAKE_COMMAND}
        ARGS -E copy "${CMAKE_CURRENT_BINARY_DIR}/libmodslam.so" "${CMAKE_CURRENT_BINARY_DIR}/android-build/libs/${ANDROID_ABI}/libmodslam_${ANDROID_ABI}.so"
    )

else()
    add_executable(modslam
                src/cml/slam/modslam.cpp
                src/cml/slam/modslam/Hybrid.cpp
                src/cml/slam/modslam/Research.cpp
                src/cml/slam/modslam/Make.cpp
                src/cml/slam/modslam/indirect/Tracking.cpp
                src/cml/slam/modslam/indirect/Mapping.cpp
                src/cml/slam/modslam/direct/Mapping.cpp
                src/cml/slam/modslam/direct/Tracking.cpp
                )
    target_link_libraries(modslam CML)
    target_include_directories(modslam PUBLIC include/cml/slam/modslam)
endif()


target_compile_options(CML PUBLIC ${COMPILER_FLAG_FPIC})

#if (NOT ANDROID)
#
#endif()
#target_compile_definitions(CML PUBLIC -DEIGEN_MAX_ALIGN_BYTES=64)
#target_compile_definitions(CML PUBLIC -DENABLE_SSE=1)
#target_compile_definitions(CML PUBLIC -DEIGEN_USE_MKL_ALL=1)

# target_compile_options(CML PUBLIC -Wall -Werror -Wuninitialized)

# target_link_options(CML PUBLIC -mllvm)

if (WIN32)
target_compile_options(CML PUBLIC ${OpenMP_CXX_FLAGS} -static)
target_link_options(CML PUBLIC ${OpenMP_CXX_FLAGS} -static)
endif()

if (CMAKE_BUILD_TYPE STREQUAL "Debug")
    target_compile_definitions(CML PUBLIC -DDEBUG=1 -DNDEBUG=1)
    target_compile_options(CML PUBLIC -g)
elseif (CMAKE_BUILD_TYPE STREQUAL "Release")
    target_compile_options(CML PUBLIC -O3 -fno-math-errno)
elseif (CMAKE_BUILD_TYPE STREQUAL "RelWithDebInfo")
    target_compile_options(CML PUBLIC -Og -g)
    target_compile_definitions(CML PUBLIC -DDEBUG=1 -DNDEBUG=1)
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
