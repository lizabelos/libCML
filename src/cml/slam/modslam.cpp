#include <iostream>
#include <filesystem>

#if CML_ENABLE_GUI
#include <QApplication>
#include <QSurface>
#include <QSurfaceFormat>
#include <cml/gui/MainSlamWidget.h>
#include <cml/gui/MainWindow.h>
#include <cml/capture/QtWebcamCapture.h>
#endif

#include <cml/base/AbstractSlam.h>

#if CML_HAVE_AVCODEC
#include <cml/capture/VideoCapture.h>
#endif

#if CML_HAVE_LIBZIP
#include <cml/capture/TUMCapture.h>
#endif

#include <cml/capture/KittyCapture.h>
#include <cml/capture/EurocCapture.h>
#include <cml/capture/TartanairCapture.h>
#include <cml/capture/Eth3DCapture.h>
#include <cml/capture/RobotCarCapture.h>

#include <cml/image/Filter.h>

#include <argparse/argparse.hpp>

#if CML_HAVE_YAML_CPP
#include <yaml-cpp/yaml.h>
#endif

#include <cml/slam/modslam/Hybrid.h>

using namespace CML;

typedef enum ExecutionMode {
    GUI, CONSOLE
} ExecutionMode;

#if CML_ENABLE_GUI
Q_DECLARE_METATYPE(scalar_t)
#endif

#if !CML_IS_ANDROID
Ptr<AbstractCapture, Nullable> loadDataset(const std::string &path) {

#if CML_HAVE_AVFORMAT
#if CML_ENABLE_GUI
    if (path == "cam" || path == "webcam" || path == "camera") {
        return new QtWebcamCapture();
    }
#endif

    try {
        Ptr<AbstractCapture, Nullable> capture = new CML::VideoCapture(path);
        return capture;
    } catch (const std::exception &e) {

    }
#endif

#if CML_HAVE_LIBZIP
    try {
        Ptr<AbstractCapture, Nullable> capture = new CML::TUMCapture(path);
        return capture;
    } catch (const std::exception &e) {

    }
#endif

    try {
        Ptr<AbstractCapture, Nullable> capture = new CML::KittyCapture(path);
        return capture;
    } catch (const std::exception &e) {

    }

    try {
        Ptr<AbstractCapture, Nullable> capture = new CML::EurocCapture(path);
        return capture;
    } catch (const std::exception &e) {

    }

    try {
        Ptr<AbstractCapture, Nullable> capture = new CML::RobotCarCapture(path);
        return capture;
    } catch (const std::exception &e) {

    }

    try {
        Ptr<AbstractCapture, Nullable> capture = new CML::Eth3DCapture(path);
        return capture;
    } catch (const std::exception &e) {

    }

    try {
        Ptr<AbstractCapture, Nullable> capture = new CML::TartanairCapture(path);
        return capture;
    } catch (const std::exception &e) {

    }

    return nullptr;
}
#endif

void printTypeSize() {
    std::cout << "sizeof(List)=" << sizeof(List<void*>) << std::endl;
    std::cout << "sizeof(LinkedList)=" << sizeof(LinkedList<void*>) << std::endl;
    std::cout << "sizeof(Set)=" << sizeof(Set<void*>) << std::endl;
    std::cout << "sizeof(HashMap)=" << sizeof(HashMap<void*, void*>) << std::endl;
    std::cout << "sizeof(Mutex)=" << sizeof(Mutex) << std::endl;
    std::cout << "sizeof(Ptr)=" << sizeof(Ptr<int*, NonNullable>) << std::endl;
    std::cout << "sizeof(PrivateData)=" << sizeof(PrivateData) << std::endl;
    std::cout << "sizeof(MapPoint)=" << sizeof(MapPoint) << std::endl;
    std::cout << "sizeof(Frame)=" << sizeof(Frame) << std::endl;
}

template <typename T> void test(T f, std::string name) {
    FloatImage image(1280, 720);
    Timer timer;
    timer.start();
    for (int i = 0; i < 4; i++) {
        image = f(image, 7, 7, 2, 2);
    }
    timer.stopAndPrint(name);

}

template <typename T> void test2(T f, std::string name) {
    FloatImage image(1280, 720);
    Timer timer;
    timer.start();
    for (int i = 0; i < 4; i++) {
        image = f(image, 2);
    }
    timer.stopAndPrint(name);

}

void test3() {
    FloatImage image(1280, 720);
    Timer timer;
    timer.start();
    for (int i = 0; i < 4; i++) {
        image = image.transpose();
    }
    timer.stopAndPrint("Assignement result");
}

#if CML_IS_ANDROID
int main(int argc, char *argv[])
{
    QApplication a(argc, argv);

    QFile f("style.css");
    if (f.open(QFile::ReadOnly | QFile::Text)) {
        QString css = QTextStream(&f).readAll();
        a.setStyleSheet(css);
    }

    Ptr<AbstractSlam, Nullable> slam = new Hybrid();
    MainSlamWidget w(slam);

    w.show();
    slam->start(new QtWebcamCapture());
    a.exec();
}
#else
int main(int argc, char *argv[])
{

    // logger.info("OpenMP Max Threads : " + std::to_string(omp_get_max_threads()));

    /*for (int i = 0; i < 5; i++) {
        test2(Filter::applyGaussian, "applyGaussian");
        test(Filter::applyGaussianNaive, "applyGaussianNaive");
        test3();
    }
    return 0;
*/
    /*
    for (int i = 1; i < 51; i++) {
        std::string si = std::to_string(i);
        if (si.size() == 1) si = "0" + si;
        TUMCapture *capture = new TUMCapture("/home/thomas/Datasets/TUM/sequence_" + si);
        capture->play();
        while (capture->next().isNotNull()) {
            logger.important(std::to_string(i) + "->" + std::to_string(capture->remaining()));
        }
    }
    return 0;
     */
    std::string executionPath = weakly_canonical(std::filesystem::path(argv[0])).parent_path();

    logger.setLogLevel(IMPORTANT);

    printTypeSize();

    srand(29071996);

#if CML_ENABLE_GUI
    qRegisterMetaType<scalar_t>("scalar_t");
#endif

    argparse::ArgumentParser program("CML");

    Ptr<AbstractSlam, Nullable> slam = new Hybrid();
    Ptr<AbstractCapture, Nullable> capture;
    ExecutionMode executionMode = CONSOLE;
#if CML_HAVE_YAML_CPP
    YAML::Node configuration;
#endif
    std::string resultPath = "result";
    std::string resultFormat = "all";
    std::string saveImagePath = "";

    program.add_argument("-d", "--dataset").nargs(1).help("Path to the dataset").action([&capture](const std::string &value){
        capture = loadDataset(value);
        if (capture.isNull()) {
            logger.error("Can't load dataset '" + value + "'");
        }
    });
#if CML_ENABLE_GUI
    program.add_argument("-g", "--gui").nargs(0).help("Gui mode").default_value(true).implicit_value(false);
    program.add_argument("-t", "--terminal").nargs(0).help("Terminal mode").default_value(false).implicit_value(true);
#endif
#if CML_HAVE_YAML_CPP
    program.add_argument("-c", "--config").nargs(1).help("Configuration file for the slam").action([&configuration, &slam, &executionPath](const std::string &value){
        logger.info("Parsing : " + value);
        configuration = YAML::LoadFile(value);
        slam->setConfiguration(configuration);
    });
#endif
    program.add_argument("-r", "--result").nargs(1).help("Result path").action([&resultPath](const std::string &value){
        resultPath = value;
    });
    program.add_argument("-f", "--format").nargs(1).help("Result format").action([&resultFormat](const std::string &value){
        resultFormat = value;
    });
    program.add_argument("-l", "--log").nargs(1).help("Write the log to a file").action([](const std::string &value) {
        logger.redirect(value);
    });
    program.add_argument("-s", "--save").nargs(1).help("Save the images").action([&saveImagePath](const std::string &value) {
        saveImagePath = value;
    });
    program.parse_args(argc, argv);

#if CML_ENABLE_GUI
    if (program["--gui"] == true) {
        executionMode = GUI;
    }
    if (program["--terminal"] == true) {
        executionMode = CONSOLE;
    }
#else
    executionMode = CONSOLE;
#endif


#if CML_ENABLE_GUI
    if ((capture.isNull() || slam.isNull()) && executionMode == GUI) {
        QApplication a(argc, argv);
        QFile f("style.css");
        if (f.open(QFile::ReadOnly | QFile::Text)) {
            QString css = QTextStream(&f).readAll();
            a.setStyleSheet(css);
        }
        MainWindow w;
        w.show();
        return a.exec();
    }
#endif

    if (capture.isNull()) {
        logger.error("No dataset.");
        return EXIT_FAILURE;
    }

    if (slam.isNull()) {
        logger.error("No slam. Please specify a configuration file.");
        return EXIT_FAILURE;
    }

    if (executionMode == CONSOLE) {
        slam->startSingleThread(capture);
    }

#if CML_ENABLE_GUI
    else if (executionMode == GUI) {
        QApplication a(argc, argv);

        QFile f("style.css");
        if (f.open(QFile::ReadOnly | QFile::Text)) {
            QString css = QTextStream(&f).readAll();
            a.setStyleSheet(css);
        }

        MainSlamWidget w(slam, !saveImagePath.empty());

        if (!saveImagePath.empty()) {
            logger.important("Saving the images to " + saveImagePath);
            w.saveImagesTo(saveImagePath);
        }

        w.show();
        slam->start(capture);
        a.exec();


    }
#endif

    if (slam->getCapture()->remaining() == 0) {

        logger.info("Exporting the results");
        if (resultFormat == "tum") {
            slam->getMap().exportResults(resultPath, MAP_RESULT_FORMAT_TUM, false);
        }
        if (resultFormat == "kitti") {
            slam->getMap().exportResults(resultPath, MAP_RESULT_FORMAT_KITTI, false);
        }
        if (resultFormat == "all") {
            slam->getMap().exportResults(resultPath + ".tum.txt", MAP_RESULT_FORMAT_TUM, false);
            slam->getMap().exportResults(resultPath + ".kitti.txt", MAP_RESULT_FORMAT_KITTI, false);
        }

    }

    return EXIT_SUCCESS;

}
#endif
