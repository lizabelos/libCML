#include "cml/config.h"
#include "cml/types/OS.h"

#include <filesystem>

#include <QFile>

std::string CML::readWholeBinaryFile(std::string filename, bool isResource) {
    // Read the file using QFile
    if (isResource) {
        filename = ":/modslam/" + filename;
    }
    QFile file(filename.c_str());
    if (!file.open(QIODevice::ReadOnly)) {
        throw std::runtime_error("Could not open file: " + filename);
    }
    QByteArray data = file.readAll();
    return std::string(data.constData(), data.size());
}

#if CML_HAVE_LIBZIP
std::string CML::readWholeZipFile(const std::string &filename, const std::string &entry, bool isResource) {
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

std::vector<std::string> CML::listDirectory(std::string path, std::string ext) {

    std::vector<std::string> result;

    for (const auto & entry : std::filesystem::directory_iterator(path)) {
        result.emplace_back(entry.path().filename().string());
    }

    return result;

}