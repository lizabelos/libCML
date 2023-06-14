//
// Created by tbelos on 03/05/19.
//

#ifndef CML_LOGGER_H
#define CML_LOGGER_H

#include <string>
#include <vector>
#include <iostream>
#include <mutex>
#include <fstream>
#include <unordered_set>
#include <optional>
#include <sstream>

#include "spdlog/spdlog.h"

#define CML_DISABLE_LOGGER 0

#if CML_DISABLE_LOGGER
#define CML_LOG_DEBUG(msg)
#define CML_LOG_INFO(msg)
#define CML_LOG_WARN(msg)
#define CML_LOG_ERROR(msg)
#define CML_LOG_FATAL(msg)
#define CML_LOG_IMPORTANT(msg)
#else
#define CML_LOG_DEBUG(msg) spdlog::debug(msg)
#define CML_LOG_INFO(msg) spdlog::info(msg)
#define CML_LOG_WARN(msg) spdlog::warn(msg)
#define CML_LOG_ERROR(msg) spdlog::error(msg)
#define CML_LOG_FATAL(msg) spdlog::critical(msg)
#define CML_LOG_IMPORTANT(msg) spdlog::info(msg)
#endif

namespace CML {

    template <typename A, typename B> void dumpSystem(A a, B b) {
        std::ofstream outfile;

        outfile.open("systemError.txt", std::ios_base::app); // append instead of overwrite
        outfile << "H : ";
        outfile << a;
        outfile << "J : ";
        outfile << b;
        outfile << "====================";
    }

}

#endif //CML_LOGGER_H
