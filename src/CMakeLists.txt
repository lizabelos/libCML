file(GLOB_RECURSE HEADER_LIST CONFIGURE_DEPENDS ../include/*.h)

add_library(CML SHARED

        ${HEADER_LIST}

        cml/fastmath.cpp

        cml/base/AbstractSlam.cpp
        cml/base/Statistic.cpp

        cml/capture/CaptureImage.cpp
        cml/capture/KittyCapture.cpp
        cml/capture/TUMCapture.cpp
        cml/capture/EurocCapture.cpp
        cml/capture/TartanairCapture.cpp
        cml/capture/Eth3DCapture.cpp
        cml/capture/RobotCarCapture.cpp

        cml/features/corner/PixelSelector.cpp

        cml/image/Array2D.cpp
        cml/image/Filter.cpp

        cml/map/InternalCalibration.cpp
        cml/map/Camera.cpp
        cml/map/Frame.cpp
        cml/map/Hasher.cpp
        cml/map/Map.cpp
        cml/map/MapObject.cpp
        cml/map/Edge.cpp

        cml/maths/Rotation.cpp

        cml/optimization/Triangulation.cpp
        cml/optimization/dso/DSOTracer.cpp
        cml/optimization/EPnP.cpp
        cml/optimization/dso/DSOTracker.cpp
        cml/optimization/dso/DSOBundleAdjustment.cpp
        cml/optimization/dso/DSOInitializer.cpp
        cml/optimization/IndirectPointOptimizer.cpp

        cml/nn/Precomputed.cpp

        cml/robust/RobustFundamental8Points.cpp
        cml/robust/RobustHomography.cpp
        cml/robust/CameraChecker.cpp
        cml/robust/RobustPoseLoss.cpp
        cml/robust/RobustRaulmurInitializer.cpp

        cml/utils/Logger.cpp
        cml/utils/GarbageCollector.cpp
        cml/utils/Watchpoint.cpp

        cml/gui/viewer/ReprojectionViewer.cpp
        cml/gui/viewer/VarianceViewer.cpp

        cml/features/bow/BowVector.cpp
        cml/features/bow/FeatureVector.cpp
        cml/features/bow/QueryResults.cpp
        cml/features/bow/ScoringObject.cpp
        cml/features/bow/Relocalization.cpp

        cml/features/corner/ORB.cpp
        cml/features/corner/FAST.cpp
        cml/features/corner/BRIEF.cpp

        cml/features/cornerTracker/LSHTracker.cpp
        cml/features/cornerTracker/CornerMatcher.cpp
        cml/features/cornerTracker/BoWTracker.cpp
        cml/features/cornerTracker/VFC.cpp

        fast/fast.c
        fast/fast_9.c
        fast/fast_10.c
        fast/fast_11.c
        fast/fast_12.c
        fast/nonmax.c
)

target_link_libraries(CML Threads::Threads ${EIGEN3_LIBRARY} ${FLANN_LIBRARIES} ${SuiteSparse_LIBRARIES})
target_include_directories(CML PUBLIC ${EIGEN3_INCLUDE_DIR} ${SOPHUS_INCLUDE_DIR} ${FLANN_INCLUDE_DIR})

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
    target_sources(CML PRIVATE cml/optimization/g2o/IndirectBundleAdjustment.cpp cml/optimization/g2o/IndirectCameraOptimizer.cpp)
target_link_libraries(CML core types_sba types_slam3d)
target_include_directories(CML PUBLIC ../thirdparty/g2o)
target_include_directories(CML PUBLIC ${G2O_INCLUDE_DIRECTORY})
#    target_link_libraries(CML g2o::stuff g2o::core g2o::g2o_hierarchical_library g2o::g2o_hierarchical_library g2o::types_slam2d g2o::types_slam2d_addons g2o::types_data g2o::types_sclam2d g2o::types_slam3d  g2o::types_slam3d_addons g2o::types_sba g2o::types_icp g2o::types_sim3 g2o::solver_pcg g2o::solver_dense g2o::solver_slam2d_linear g2o::solver_structure_only g2o::solver_csparse g2o::csparse_extension g2o::solver_cholmod g2o::solver_eigen)
#endif()

if (CERES_FOUND)
    target_sources(CML PRIVATE cml/optimization/ceres/IndirectBundleAdjustment.cpp cml/optimization/ceres/PlaneInitializer.cpp cml/optimization/ceres/IndirectCameraOptimizer.cpp cml/evaluation/Alignment.cpp)
    target_link_libraries(CML ${CERES_LIBRARIES})
    target_include_directories(CML PUBLIC ${CERES_INCLUDE_DIR})
endif()

if (SUITESPARSE_FOUND)
    target_link_libraries(CML ${SUITESPARSE_LIBRARIES})
endif()

if (TORCH_FOUND)
    target_sources(CML PRIVATE cml/nn/MonoDepth.cpp cml/nn/Midas.cpp)
    target_link_libraries(CML torch_cpu c10)
endif()

if (OpenCV_FOUND)
    target_sources(CML PRIVATE cml/features/corner/OpenCV.cpp)
    target_include_directories(CML PUBLIC ${OpenCV_INCLUDE_DIRS})
    target_link_libraries(CML ${OpenCV_LIBS})
endif()

if (GLOG_FOUND)
    target_link_libraries(CML ${GLOG_LIBRARIES})
endif()

if (ENABLE_GUI)
    qt6_wrap_cpp(ModelViewerWidgetSRC ../include/cml/gui/widgets/ModelWidget.h)
    qt6_wrap_cpp(FunctionWidgetSRC ../include/cml/gui/widgets/FunctionWidget.h)
    qt6_wrap_cpp(FunctionListWidgetSRC ../include/cml/gui/widgets/FunctionListWidget.h)
    qt6_wrap_cpp(LoggerWidgetSRC ../include/cml/gui/widgets/LoggerWidget.h)
    qt6_wrap_cpp(CameraViewerWidgetSRC ../include/cml/gui/widgets/CameraViewerWidget.h)
    qt6_wrap_cpp(ParameterIntegerSliderSRC ../include/cml/gui/widgets/ParameterIntegerSlider.h)
    qt6_wrap_cpp(ParameterFloatingSliderSRC ../include/cml/gui/widgets/ParameterFloatingSlider.h)
    qt6_wrap_cpp(ParameterCheckboxSRC ../include/cml/gui/widgets/ParameterCheckbox.h)
    qt6_wrap_cpp(StatisticWidgetSRC ../include/cml/gui/widgets/StatisticWidget.h)
    qt6_wrap_cpp(QCustomPlotSRC ../include/qcustomplot/qcustomplot.h)
    qt6_wrap_cpp(WebcamCaptureSRC ../include/cml/capture/QtWebcamCapture.h)
    qt6_wrap_cpp(SpoilerSRC ../include/cml/gui/widgets/Spoiler.h)
    qt6_wrap_cpp(GroupsWidgetSRC ../include/cml/gui/widgets/GroupsWidget.h)
    qt6_wrap_cpp(CaptureSelectorSRC ../include/cml/gui/CaptureSelector.h)
    qt6_wrap_cpp(SlamSelectorSRC ../include/cml/gui/SlamSelector.h)
    qt6_wrap_cpp(MainWindowSRC ../include/cml/gui/MainWindow.h)
    qt6_wrap_cpp(DesktopMainWindowSRC ../include/cml/gui/MainSlamWidget.h)

    target_sources(CML PRIVATE cml/gui/widgets/ModelWidget.cpp ${ModelViewerWidgetSRC}
            cml/gui/drawboard/QtDrawBoard.cpp
            cml/gui/widgets/FunctionWidget.cpp ${FunctionWidgetSRC}
            cml/gui/widgets/FunctionListWidget.cpp ${FunctionListWidgetSRC}
            cml/gui/widgets/LoggerWidget.cpp ${LoggerWidgetSRC}
            cml/gui/widgets/CameraViewerWidget.cpp ${CameraViewerWidgetSRC}
            cml/gui/widgets/ParameterIntegerSlider.cpp ${ParameterIntegerSliderSRC}
            cml/gui/widgets/ParameterFloatingSlider.cpp ${ParameterFloatingSliderSRC}
            cml/gui/widgets/ParameterCheckbox.cpp ${ParameterCheckboxSRC}
            cml/gui/widgets/StatisticWidget.cpp ${StatisticWidgetSRC}
            qcustomplot/qcustomplot.cpp ${QCustomPlotSRC}
            cml/gui/widgets/Spoiler.cpp ${SpoilerSRC}
            cml/gui/widgets/GroupsWidget.cpp ${GroupsWidgetSRC}
            cml/gui/drawboard/QtDrawBoardShaders.cpp
            cml/gui/CaptureSelector.cpp ${CaptureSelectorSRC}
            cml/gui/SlamSelector.cpp ${SlamSelectorSRC}
            cml/gui/MainWindow.cpp ${MainWindowSRC}
            cml/capture/QtWebcamCapture.cpp ${WebcamCaptureSRC}
            cml/gui/MainSlamWidget.cpp ${DesktopMainWindowSRC}
            )
    target_link_libraries(CML Qt6::Core Qt6::Widgets Qt6::OpenGL Qt6::OpenGLWidgets Qt6::PrintSupport Qt6::Multimedia ${OPENGL_LIBRARIES})


endif()





add_executable(modslam
        cml/slam/modslam.cpp
        cml/slam/modslam/Hybrid.cpp
        cml/slam/modslam/Research.cpp
        cml/slam/modslam/Make.cpp
        cml/slam/modslam/indirect/Tracking.cpp
        cml/slam/modslam/indirect/Mapping.cpp
        cml/slam/modslam/direct/Mapping.cpp
        cml/slam/modslam/direct/Tracking.cpp
        )
target_link_libraries(modslam CML)
target_include_directories(modslam PUBLIC ../include/cml/slam/modslam)

file(COPY ../include/cml/slam/modslam.yaml DESTINATION ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})
