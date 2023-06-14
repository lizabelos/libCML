#ifndef CML_OS_H
#define CML_OS_H

#include <thread>
#include <cmath>
#include <vector>
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

    std::string readWholeBinaryFile(std::string filename, bool isResource);

#if CML_HAVE_LIBZIP
    std::string readWholeZipFile(const std::string &filename, const std::string &entry, bool isResource);
#endif

    std::vector<std::string> listDirectory(std::string path, std::string ext = "");

}

#endif