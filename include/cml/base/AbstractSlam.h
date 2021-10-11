//
// Created by tbelos on 11/07/19.
//

#ifndef CML_ABSTRACTSLAM_H
#define CML_ABSTRACTSLAM_H

#include "cml/config.h"
#include "AbstractCaptureProcessor.h"
#include "AbstractFunction.h"
#include "cml/capture/AbstractCapture.h"
#include "cml/map/Map.h"
#include "cml/features/cornerTracker/CornerMatcher.h"

#include <vector>

#ifdef WIN32
#include <windows.h>
#else
#include <dlfcn.h>
#endif

#include <yaml-cpp/yaml.h>
#include <argparse/argparse.hpp>

namespace CML {

    class AbstractSlam : public AbstractCaptureProcessor, public AbstractFunction {

    public:
        AbstractSlam();

        virtual ~AbstractSlam() = default;

        void start(Ptr<AbstractCapture, NonNullable> capture);

        void startSingleThread(Ptr<AbstractCapture, NonNullable> capture);

        void interrupt();

        void wait();

        void setPaused(bool state);

        bool isPaused();

        virtual void stop(std::string reason);

        void restart();

        void restartOrStop(std::string reason);

        void next();

        bool isStopped();

        Map &getMap() final;

        Ptr<AbstractCapture, Nullable> getCapture();

        OptPFrame getNextFrame();

        Ptr<CaptureImage, Nullable> getLastCaptureFrame();

        virtual inline std::string getName() {
            return "Abstract Slam";
        }

        virtual inline InitializationState getInitializationState() {
            return OK;
        }

        void setConfiguration(const YAML::Node &node) {
            mConfiguration = node;
            for (auto p : mConfiguration) {
                mUnusedParameters.insert(p.first.as<std::string>());
            }
            mUnusedParameters.erase("slam");
            refreshParameters();
            if (mUnusedParameters.size() > 0) {
                for (auto p : mUnusedParameters) {
                    logger.fatal("Unused parameter : " + p);
                }
                throw std::runtime_error("Unused parameters");
            }
        }

        const YAML::Node &getConfiguration() {
            return mConfiguration;
        }

        void setArgument(const argparse::ArgumentParser &args) {
            mArguments = args;
        }

        const argparse::ArgumentParser &getArguments() {
            return mArguments;
        }

        inline void onNewParameter(std::string name, Parameter &parameter) {
            if (!getConfiguration()[name].IsDefined()) {
                return;
            }
            try {
                switch (parameter.type()) {
                    case INTEGER:
                        parameter.set(getConfiguration()[name].as<int>());
                        mUnusedParameters.erase(name);
                        break;
                    case FLOATING:
                        parameter.set(getConfiguration()[name].as<float>());
                        mUnusedParameters.erase(name);
                        break;
                    case BOOLEAN:
                        parameter.set(getConfiguration()[name].as<bool>());
                        mUnusedParameters.erase(name);
                        break;
                    default:
                        throw std::runtime_error("Invalid parameter type : " + name);
                }
            } catch (...) {
                throw std::runtime_error("Invalid parameter value : " + name + "=" + getConfiguration()[name].as<std::string>());
            }
        }

    protected:
        virtual void onReset() = 0;

        virtual void run() = 0;

        void pauseHere();

        void addFrame(PFrame frame);

    private:
        YAML::Node mConfiguration;
        argparse::ArgumentParser mArguments;

        Map mMap;

        Atomic<bool> mIsStopped, mIsPaused, mNeedToRestart;
        Atomic<int> mPausedNextFrame;
        std::thread mThread;
        Ptr<AbstractCapture, Nullable> mCapture;
        Ptr<CaptureImage, Nullable> mLastCaptureImage;

        GarbageCollectorInstance mGarbageCollectorInstance;

        int mMemoryLimit = 0; // MB

        Set<std::string> mUnusedParameters;


    };

    typedef AbstractSlam* (*MakeFunction)();

#ifdef WIN32
#define DL_EXT "dll"
    inline MakeFunction loadExt(std::string path) {

        HMODULE hdll = LoadLibrary(path.c_str());
        if (hdll == NULL) {
             throw std::runtime_error("Can't load " + path);
        }
        MakeFunction make = (MakeFunction)GetProcAddress(hdll, "make");
        if (hdll == NULL) {
            throw std::runtime_error("Can't find make in " + path);
        }
        return make;

    }
#else
#define DL_EXT "so"
    inline MakeFunction loadExt(std::string path) {

        void *handle;
        MakeFunction make;
        handle = dlopen (path.c_str(), RTLD_LAZY);
        if (!handle) {
            throw std::runtime_error(dlerror());
        }
        dlerror();    /* Clear any existing error */
        make = (CML::MakeFunction)dlsym(handle, "make");
        char *error = dlerror();
        if (error != NULL)  {
            throw std::runtime_error(error);
        }
        return make;
    }
#endif



}


#endif //CML_ABSTRACTSLAM_H
