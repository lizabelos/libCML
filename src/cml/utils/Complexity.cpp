#include "cml/utils/Complexity.h"
#include "cml/utils/Timer.h"


#if CML_ENABLE_COMPLEXITY_REPORT

    CML::HashMap<std::string, CML::Pair<int, float>> *methodComplexity = nullptr;

    void initializeComplexityReport() {
        if (methodComplexity == nullptr) {
            methodComplexity = new CML::HashMap<std::string, CML::Pair<int, float>>();
        }
    }

    void _signalMethodStart(const std::string &methodName) {
    }

    void _signalMethodEnd(const std::string &methodName, float time) {
        initializeComplexityReport();
        (*methodComplexity)[methodName].first++;
        (*methodComplexity)[methodName].second += time;
    }

    void dumpComplexityReport() {
        // dump the complexity to the existing csv output.csv (create if not exist)
        std::ofstream file;
        file.open("output.csv", std::ios::app);

#if CML_FRAME_MAP_IMPLEMENTATION == CML_FRAME_MAP_IMPLEMENTATION_NAIVE
        file << "frame:naive;";
#elif CML_FRAME_MAP_IMPLEMENTATION == CML_FRAME_MAP_IMPLEMENTATION_STD
        file << "frame:std;";
#elif CML_FRAME_MAP_IMPLEMENTATION == CML_FRAME_MAP_IMPLEMENTATION_PHMAP
        file << "frame:phmap;";
#elif CML_FRAME_MAP_IMPLEMENTATION == CML_FRAME_MAP_IMPLEMENTATION_PHMAP_PARALLEL
        file << "frame:phmap_parallel;";
#endif

#if CML_POINT_MAP_IMPLEMENTATION == CML_POINT_MAP_IMPLEMENTATION_NAIVE
        file << "point:naive;";
#elif CML_POINT_MAP_IMPLEMENTATION == CML_POINT_MAP_IMPLEMENTATION_STD
        file << "point:std;";
#elif CML_POINT_MAP_IMPLEMENTATION == CML_POINT_MAP_IMPLEMENTATION_PHMAP
        file << "point:phmap;";
#elif CML_POINT_MAP_IMPLEMENTATION == CML_POINT_MAP_IMPLEMENTATION_PHMAP_PARALLEL
        file << "point:phmap_parallel;";
#endif

        file << std::endl;

        for (auto &entry : *methodComplexity) {
            file << entry.first << ";" << entry.second.first << ";" << entry.second.second << std::endl;
        }
        file.close();
    }

#endif