#include <iostream>
#include <filesystem>
#include <limits>

#include <cml/slam/modslam/Hybrid.h>
#include <cml/utils/Complexity.h>

#if CML_ENABLE_GUI
#include <QApplication>
#include <QSurface>
#include <QSurfaceFormat>
#include <cml/gui/MainSlamWidget.h>
#include <cml/capture/QtWebcamCapture.h>
#endif

#include <cml/base/AbstractSlam.h>

#if CML_HAVE_AVCODEC
#include <cml/capture/VideoCapture.h>
#endif

#if CML_HAVE_LIBZIP
#include <cml/capture/TUMCapture.h>
#include <cml/capture/ZipStereopolisCapture.h>
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

using namespace CML;

typedef enum ExecutionMode {
    GUI, CONSOLE
} ExecutionMode;

#if CML_ENABLE_GUI
Q_DECLARE_METATYPE(scalar_t)
#endif

#if !CML_IS_ANDROID
Ptr<AbstractCapture, Nullable> loadDataset(const std::string &path) {

#if CML_ENABLE_GUI
    if (path == "cam" || path == "webcam" || path == "camera") {
        return new QtWebcamCapture();
    }
#endif

#if CML_HAVE_AVFORMAT
    try {
        Ptr<AbstractCapture, Nullable> capture = new CML::VideoCapture(path);
        return capture;
    } catch (const std::exception &e) {
        CML_LOG_ERROR(e.what());
    }
#endif

#if CML_HAVE_LIBZIP
    CML_LOG_INFO("Trying to load TUM dataset");
    try {
        CML::TUMCapture *capture = new CML::TUMCapture(path);
        capture->decompressAll();
        return capture;
    } catch (const std::exception &e) {
        CML_LOG_ERROR(e.what());
    }

    CML_LOG_INFO("Trying to load dataset as a ZipStereopolisCapture");
    try {
        CML::ZipStereopolisCapture *capture = new CML::ZipStereopolisCapture(path);
        capture->decompressAll();
        return capture;
    } catch (const std::exception &e) {
        CML_LOG_ERROR(e.what());
    }
#endif

    CML_LOG_INFO("Trying to load dataset as a KittyCapture");
    try {
        Ptr<AbstractCapture, Nullable> capture = new CML::KittyCapture(path);
        return capture;
    } catch (const std::exception &e) {
        CML_LOG_ERROR(e.what());
    }
/*
    try {
        Ptr<AbstractCapture, Nullable> capture = new CML::EurocCapture(path);
        return capture;
    } catch (const std::exception &e) {
        CML_LOG_ERROR(e.what());
    }

    try {
        Ptr<AbstractCapture, Nullable> capture = new CML::RobotCarCapture(path);
        return capture;
    } catch (const std::exception &e) {
        CML_LOG_ERROR(e.what());
    }

   try {
        Ptr<AbstractCapture, Nullable> capture = new CML::Eth3DCapture(path);
        return capture;
    } catch (const std::exception &e) {
        CML_LOG_ERROR(e.what());
    }

    try {
        Ptr<AbstractCapture, Nullable> capture = new CML::TartanairCapture(path);
        return capture;
    } catch (const std::exception &e) {
        CML_LOG_ERROR(e.what());
    }
*/
    return nullptr;
}
#endif

void printTypeSize() {
 /*   std::cout << "sizeof(List)=" << sizeof(List<void*>) << std::endl;
    std::cout << "sizeof(LinkedList)=" << sizeof(LinkedList<void*>) << std::endl;
    std::cout << "sizeof(Set)=" << sizeof(Set<void*>) << std::endl;
    std::cout << "sizeof(HashMap)=" << sizeof(HashMap<void*, void*>) << std::endl;
    std::cout << "sizeof(Mutex)=" << sizeof(Mutex) << std::endl;
    std::cout << "sizeof(Ptr)=" << sizeof(Ptr<int*, NonNullable>) << std::endl;
    std::cout << "sizeof(PrivateData)=" << sizeof(PrivateData) << std::endl;
    std::cout << "sizeof(MapPoint)=" << sizeof(MapPoint) << std::endl;
    std::cout << "sizeof(Frame)=" << sizeof(Frame) << std::endl;*/
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

class StatisticPrinter : public CML::Statistic::Observer {

public:
    StatisticPrinter(std::string name) {
        std::replace( name.begin(), name.end(), ' ', '_');
        mName = name;
    }

    virtual void onNewValue(Statistic *statistic, scalar_t x, scalar_t y) {
        logger.raw("STAT " + mName + " " + std::to_string(x) + " " + std::to_string(y) + "\n");
    }

private:
    std::string mName;

};

#if CML_IS_ANDROID
int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    Q_INIT_RESOURCE(resources);

    CML::initCML();
    logger.setLogLevel(CML::IMPORTANT);

    Ptr<AbstractSlam, Nullable> slam = new Hybrid();

    MainSlamWidget w(slam);
    w.show();
    a.exec();
}
#else
int main(int argc, char *argv[])
{
    CML::initCML();

    //CML::Test::testNaiveMap();

    typedef std::numeric_limits< double > dbl;
    std::cout.precision(dbl::max_digits10);

    std::string executionPath = weakly_canonical(std::filesystem::path(argv[0])).parent_path().string();

    logger.setLogLevel(CML::INFO);

    printTypeSize();

    // srand(29071996);
    srand(time(nullptr));

#if CML_ENABLE_GUI
    qRegisterMetaType<scalar_t>("scalar_t");
#endif

    argparse::ArgumentParser program("CML");

    Ptr<AbstractSlam, Nullable> slam = new Hybrid();
    Ptr<AbstractCapture, Nullable> capture;
#if TEST_DETERMINISITY
    Ptr<AbstractSlam, Nullable> slamDeter = new Hybrid();
    Ptr<AbstractCapture, Nullable> captureDeter;
#else
    Ptr<AbstractSlam, Nullable> slamDeter;
    Ptr<AbstractCapture, Nullable> captureDeter;
#endif
    ExecutionMode executionMode = CONSOLE;
#if CML_HAVE_YAML_CPP
    YAML::Node configuration;
#endif
    std::string resultPath = "result";
    std::string resultFormat = "all";
    std::string saveImagePath = "";

    program.add_argument("-d", "--dataset").nargs(1).help("Path to the dataset").action([&capture, &captureDeter](const std::string &value){
        capture = loadDataset(value);
#if TEST_DETERMINISITY
        captureDeter = loadDataset(value);
#endif
        if (capture.isNull()) {
            CML_LOG_ERROR("Can't load dataset '" + value + "'");
        }
    });
    program.add_argument("-b", "--reverse").nargs(0).help("Reverse mode").default_value(false).implicit_value(true);

//#if CML_ENABLE_GUI
    program.add_argument("-g", "--gui").nargs(0).help("Gui mode").default_value(true).implicit_value(true);
    program.add_argument("-t", "--terminal").nargs(0).help("Terminal mode").default_value(false).implicit_value(true);
//#endif
#if CML_HAVE_YAML_CPP
    program.add_argument("-c", "--config").nargs(1).help("Configuration file for the slam").action([&configuration, &slam, &slamDeter, &executionPath](const std::string &value){
        CML_LOG_INFO("Parsing : " + value);
        configuration = YAML::LoadFile(value);
        slam->setConfiguration(configuration);
#if TEST_DETERMINISITY
        slamDeter->setConfiguration(configuration);
#endif
    });
#endif
    program.add_argument("-r", "--result").nargs(1).help("Result path").action([&resultPath](const std::string &value){
        resultPath = value;
    });
    program.add_argument("-f", "--format").nargs(1).help("Result format").action([&resultFormat](const std::string &value){
        resultFormat = value;
    });
    program.add_argument("-z", "--stats").nargs(0).help("Print the stats to cout").default_value(false).implicit_value(true);
    program.add_argument("-s", "--save").nargs(1).help("Save the images").action([&saveImagePath](const std::string &value) {
        saveImagePath = value;
    });
    program.add_argument("-v", "--verbose").nargs(0).help("Verbose").default_value(false).implicit_value(true);

    program.parse_args(argc, argv);

    if (program["--verbose"] == true) {
        logger.setLogLevel(CML::MORE);
    }

    if (program["--reverse"] == true) {
        if (capture.isNotNull()) {
            capture->revere();
        }
    }

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

    if (program["--stats"] == true) {

        for (auto stat: slam->getStatistics()) {
            StatisticPrinter *sp = new StatisticPrinter("main." + stat->getName());
            stat->subscribeObserver(sp);
        }
        for (auto fn: slam->getChildFunctions()) {
            if (fn->getAlias() == "") {
                continue;
            }
            for (auto stat: fn->getStatistics()) {
                StatisticPrinter *sp = new StatisticPrinter(fn->getAlias() + "." + stat->getName());
                stat->subscribeObserver(sp);
            }
        }
    }


#if CML_ENABLE_GUI
    if ((capture.isNull() || slam.isNull()) && executionMode == GUI) {
        return 0;
    }
#endif

    if (capture.isNull()) {
        CML_LOG_ERROR("No dataset.");
        return EXIT_FAILURE;
    }

    if (slam.isNull()) {
        CML_LOG_ERROR("No slam. Please specify a configuration file.");
        return EXIT_FAILURE;
    }

    if (executionMode == CONSOLE) {
#if TEST_DETERMINISITY
        slamDeter->start(captureDeter);
#endif
        Timer timer;
        timer.start();
        slam->startSingleThread(capture);
        timer.stop();
        std::cout << "Time : " << timer.getValue() << std::endl;
        dumpComplexityReport();
    }

#if CML_ENABLE_GUI
    else if (executionMode == GUI) {
        QApplication a(argc, argv);
        Q_INIT_RESOURCE(resources);

        QFile f("style.css");
        if (f.open(QFile::ReadOnly | QFile::Text)) {
            QString css = QTextStream(&f).readAll();
            a.setStyleSheet(css);
        }

        MainSlamWidget w(slam, !saveImagePath.empty());

        if (!saveImagePath.empty()) {
            CML_LOG_IMPORTANT("Saving the images to " + saveImagePath);
            w.saveImagesTo(saveImagePath);
        }

        w.show();
        slam->start(capture, true);
#if TEST_DETERMINISITY
        slamDeter->start(captureDeter);
#endif
        a.exec();


    }
#endif


    if (slam->getCapture()->remaining() == 0) {

        CML_LOG_INFO("Exporting the results");
        if (resultFormat == "tum") {
            slam->getMap().exportResults(resultPath, MAP_RESULT_FORMAT_TUM, false, capture->isReverse());
        }
        if (resultFormat == "kitti") {
            slam->getMap().exportResults(resultPath, MAP_RESULT_FORMAT_KITTI, false, capture->isReverse());
        }
        if (resultFormat == "all") {
            slam->getMap().exportResults(resultPath + ".tum.txt", MAP_RESULT_FORMAT_TUM, false, capture->isReverse());
            slam->getMap().exportResults(resultPath + ".kitti.txt", MAP_RESULT_FORMAT_KITTI, false, capture->isReverse());
            slam->getMap().exportResults(resultPath + ".gt.tum.txt", MAP_RESULT_FORMAT_TUM, true, capture->isReverse());
            slam->getMap().exportResults(resultPath + ".gt.kitti.txt", MAP_RESULT_FORMAT_KITTI, true, capture->isReverse());
            slam->getMap().exportResults(resultPath + ".all", CML::MAP_RESULT_FORMAT_CSV, false, capture->isReverse());
        }

    }

    return EXIT_SUCCESS;

}
#endif
