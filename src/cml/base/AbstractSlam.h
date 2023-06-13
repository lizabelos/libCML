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

#if CML_HAVE_YAML_CPP
#include <yaml-cpp/yaml.h>
#endif
#include <argparse/argparse.hpp>

namespace CML {

    class AbstractSlam : public AbstractCaptureProcessor, public AbstractFunction {

    public:
        AbstractSlam();

        virtual ~AbstractSlam() = default;

        void start(Ptr<AbstractCapture, NonNullable> capture, bool useAsMainThread = false);

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

#if CML_HAVE_YAML_CPP
        void setConfiguration(const YAML::Node &node) {
            mConfiguration = node;
            for (auto p : mConfiguration) {
                mUnusedParameters.insert(p.first.as<std::string>());
            }
            mUnusedParameters.erase("slam");
            refreshParameters();
            if (mUnusedParameters.size() > 0) {
                for (auto p : mUnusedParameters) {
                    CML_LOG_FATAL("Unused parameter : " + p);
                }
                throw std::runtime_error("Unused parameters");
            }
        }

        const YAML::Node &getConfiguration() {
            return mConfiguration;
        }
#endif

        void setArgument(const argparse::ArgumentParser &args) {
            mArguments = args;
        }

        const argparse::ArgumentParser &getArguments() {
            return mArguments;
        }

        inline void onNewParameter(std::string name, Parameter &parameter) {
#if CML_HAVE_YAML_CPP
            if (!getConfiguration()[name].IsDefined()) {
                return;
            }
            //try {
                switch (parameter.type()) {
                    case PARAM_INTEGER:
                        CML_LOG_IMPORTANT("Int parameter : " + name);
                        parameter.set(getConfiguration()[name].as<int>());
                        mUnusedParameters.erase(name);
                        break;
                    case PARAM_FLOATING:
                        CML_LOG_IMPORTANT("Float parameter : " + name);
                        parameter.set(getConfiguration()[name].as<float>());
                        mUnusedParameters.erase(name);
                        break;
                    case PARAM_BOOLEAN:
                        CML_LOG_IMPORTANT("Bool parameter : " + name);
                        parameter.set(getConfiguration()[name].as<bool>());
                        mUnusedParameters.erase(name);
                        break;
                    case PARAM_STRING:
                        CML_LOG_IMPORTANT("String parameter : " + name);
                        parameter.set(getConfiguration()[name].Scalar());
                        mUnusedParameters.erase(name);
                        break;
                    default:
                        throw std::runtime_error("Invalid parameter type : " + name);
                }
            /*} catch (...) {
                throw std::runtime_error("Invalid parameter value : " + name + "=" + getConfiguration()[name].as<std::string>());
            }*/
#endif
        }
    protected:
        virtual void onReset() = 0;

        virtual void run() = 0;

        void pauseHere();

        void addFrame(PFrame frame);

    private:
#if CML_HAVE_YAML_CPP
        YAML::Node mConfiguration;
#endif
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

}


#endif //CML_ABSTRACTSLAM_H
