#ifndef COMPLEXITY_H
#define COMPLEXITY_H

#include "cml/config.h"
#include "cml/utils/Timer.h"
#include <string>

#define CML_ENABLE_COMPLEXITY_REPORT 1

#if CML_ENABLE_COMPLEXITY_REPORT

void _signalMethodStart(const std::string &methodName);
void _signalMethodEnd(const std::string &methodName, float time);

void dumpComplexityReport();

namespace CML {
    class ComplexitySignaler {
    public:
        ComplexitySignaler(const std::string &methodName) : mMethodName(methodName) {
            _signalMethodStart(methodName);
            mTimer.start();
        }
        ~ComplexitySignaler() {
            mTimer.stop();
            _signalMethodEnd(mMethodName, mTimer.getValue());
        }
    private:
        std::string mMethodName;
        Timer mTimer;
    };
}

#define signalMethodStart(methodName) CML::ComplexitySignaler complexitySignaler(methodName)


#else

#define signalMethodStart(methodName)
#define dumpComplexityReport()

#endif


#endif