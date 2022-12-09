#ifndef CML_OS_H
#define CML_OS_H

#include <thread>
#include <cmath>
#ifndef M_PI
#define M_PI 3.14159265359
#endif

#if CML_HAVE_LIBZIP
#include <zip.h>
#endif

#ifdef WIN32
#include <windows.h>
#include <psapi.h>
#endif

namespace CML {

    class OS {
    public:
        static int getNumberOfCores() {
            return std::thread::hardware_concurrency();
        }

        static int getNumberOfThreads() {
            return std::thread::hardware_concurrency();
        }

        static void usleep(int microseconds) {
            std::this_thread::sleep_for(std::chrono::microseconds(microseconds));
        }

        static float memoryUsage() {
#ifdef linux
            FILE *file = fopen("/proc/self/status", "r");
        int result = -1;
        char line[128];

        while (fgets(line, 128, file) != NULL) {
            if (strncmp(line, "VmRSS:", 6) == 0) {

                {
                    int i = strlen(line);
                    const char* p = line;
                    while (*p <'0' || *p > '9') p++;
                    line[i-3] = '\0';
                    i = atoi(p);
                    result = i;
                }

                break;

            }
        }
        fclose(file);

        float memoryUsage = (float)result * 0.001f;
        return memoryUsage;
/*#elifdef WIN32
        //get the handle to this process
        auto myHandle = GetCurrentProcess();
        //to fill in the process' memory usage details
        PROCESS_MEMORY_COUNTERS pmc;
        //return the usage (bytes), if I may
        if (GetProcessMemoryInfo(myHandle, &pmc, sizeof(pmc))) {
            return(pmc.WorkingSetSize);
        }
        else {
            return 0;
        }*/
#else
            return 0;
#endif
        }

    };

#ifndef ANDROID
    inline std::string readWholeBinaryFile(const std::string &filename, bool isResource) {
        std::ifstream file(filename, std::ios::in | std::ios::binary);
        if (!file.is_open()) {
            throw std::runtime_error("Could not open file: " + filename);
        }
        file.seekg(0, std::ios::end);
        size_t size = file.tellg();
        file.seekg(0, std::ios::beg);
        std::vector<char> buffer(size);
        file.read(buffer.data(), size);
        file.close();
        return std::string(buffer.begin(), buffer.end());
    }

#if CML_HAVE_LIBZIP
    inline std::string readWholeZipFile(const std::string &filename, const std::string &entry, bool isResource) {
        int ziperror;
        zip_t *zipArchive = zip_open(filename.c_str(), ZIP_RDONLY, &ziperror);
        zip_file_t *zipFile = zip_fopen(zipArchive, entry.c_str(), 0);
        if (!zipFile) {
            throw std::runtime_error("Could not open file: " + entry);
        }
        std::stringstream stream;
        char *data = new char[40000];
        while (true) {
            size_t n = zip_fread(zipFile, data, 40000);
            if (n == 0) break;
            stream << std::string(data, n);
        }
        zip_fclose(zipFile);
        delete []data;
        return stream.str();
    }
#endif
#else
    inline std::string readWholeBinaryFile(std::string filename, bool isResource) {
        // Read the file using QFile
        if (isResource) {
            filename = ":/" + filename;
        }
        QFile file(filename.c_str());
        if (!file.open(QIODevice::ReadOnly)) {
            throw std::runtime_error("Could not open file: " + filename);
        }
        QByteArray data = file.readAll();
        return std::string(data.constData(), data.size());
    }

#if CML_HAVE_LIBZIP
    inline std::string readWholeZipFile(const std::string &filename, const std::string &entry, bool isResource) {
        std::string zipContent = readWholeBinaryFile(filename, isResource);
        zip_error_t ziperror;
        zip_source_t *src = zip_source_buffer_create(zipContent.c_str(), zipContent.size(), 1, &ziperror);
        zip_t *zipArchive = zip_open_from_source(src, 0, &ziperror);
        zip_file_t *zipFile = zip_fopen(zipArchive, entry.c_str(), 0);
        if (!zipFile) {
            throw std::runtime_error("Could not open file: " + entry);
        }
        std::stringstream stream;
        char *data = new char[40000];
        while (true) {
            size_t n = zip_fread(zipFile, data, 40000);
            if (n == 0) break;
            stream << std::string(data, n);
        }
        zip_fclose(zipFile);
        delete []data;
        return stream.str();
    }
#endif
#endif

    inline std::vector<std::string> listDirectory(std::string path, std::string ext = "") {

        std::vector<std::string> result;

        for (const auto & entry : std::filesystem::directory_iterator(path)) {
            result.emplace_back(entry.path().filename().string());
        }

        return result;

    }

}

#endif