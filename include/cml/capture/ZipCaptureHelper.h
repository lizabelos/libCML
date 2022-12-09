#ifndef CML_ZIPCAPTURE_H
#define CML_ZIPCAPTURE_H

#include "cml/config.h"
#include "AbstractCapture.h"

#if CML_HAVE_LIBZIP

#include <zip.h>

namespace CML {

    class ZipCaptureHelper {

    protected:
        inline void loadZip(const std::string &filename, const std::string &suffix, std::string extractPath = "") {

            mZipPath = filename;
            mExtractPath = extractPath;

            int ziperror = 0;
            mZipArchive = zip_open(filename.c_str(),  ZIP_RDONLY, &ziperror);

            if (ziperror != 0) {
                throw std::runtime_error("Can't open " + filename);
            }

            int numEntries = zip_get_num_entries(mZipArchive, 0);
            CML_LOG_INFO("Found " + std::to_string(numEntries) + " files inside the zip");
            mZipFilePath.reserve(numEntries);

            for(int k = 0; k < numEntries; k++)
            {
                std::string name = zip_get_name(mZipArchive, k,  ZIP_FL_ENC_STRICT);
                if (hasEnding(name, suffix)) {
                    mZipFilePath.emplace_back(name);
                }
            }

            std::sort(mZipFilePath.begin(), mZipFilePath.end());
        }

        inline Pair<FloatImage, Image> decompressImage(int id) {
            uint8_t *data;
            size_t size;
            decompressFile(id, &data, &size);
            return loadJpegImage(data, size);
        }

        inline std::string decompressFile(const std::string &filename, uint8_t **data, size_t *size) {
            if (mZipBuffer.size() == 0) {
                mZipBuffer.resize(1e+8);
            }

            FILE *f = nullptr;
            std::string extractedFilePath = "";
            if (mExtractPath != "") {

                std::string codedFilename = filename;
                std::replace( codedFilename.begin(), codedFilename.end(), '/', '_');
                std::replace( codedFilename.begin(), codedFilename.end(), '\\', '_');


                extractedFilePath = mExtractPath + "/" + codedFilename;

                f = fopen(extractedFilePath.c_str(), "rb");
                if (f != nullptr) {

                    CML_LOG_DEBUG("Using extracted image");

                    *size = 0;
                    while (true) {
                        size_t n = fread(mZipBuffer.data() + *size, 1, mZipBuffer.size(), f);
                        if (n == 0) break;
                        *size += n;
                    }

                    fclose(f);
                    *data = (uint8_t *) mZipBuffer.data();
                    return extractedFilePath;
                }
                f = fopen((mExtractPath + "/" + codedFilename).c_str(), "wb");
                if (f == nullptr) {
                    CML_LOG_ERROR("Can't create file " + (mExtractPath + "/" + filename));
                    extractedFilePath = "";
                }
            }

            *size = 0;

            zip_file_t *zipFile = zip_fopen(mZipArchive, filename.c_str(), 0);
            while (true) {
                size_t n = zip_fread(zipFile, mZipBuffer.data() + *size, mZipBuffer.size());
                if (n == 0) break;
                if (f != nullptr) {
                    fwrite(mZipBuffer.data() + *size, 1, n, f);
                }
                *size += n;
            }
            zip_fclose(zipFile);
            if (f != nullptr) {
                fclose(f);
            }
            *data = (uint8_t*)mZipBuffer.data();
            return extractedFilePath;
        }

        inline std::string decompressFile(int id, uint8_t **data, size_t *size) {
            return decompressFile(mZipFilePath[id], data, size);
        }

        inline const std::string &getFilename(int id) {
            return mZipFilePath[id];
        }

        inline int getImageNumber() {
            return mZipFilePath.size();
        }

    public:
        inline void decompressAll() {
            uint8_t *data;
            size_t size;
            CML_LOG_IMPORTANT("Decompressing all files");
            for (int k = 0; k < getImageNumber(); k++) {
                CML_LOG_IMPORTANT("Decompressing " + std::to_string(k) + "/" + std::to_string(getImageNumber()));
                decompressFile(k, &data, &size);
            }
        }

    private:
        zip_t *mZipArchive;
        std::string mZipPath, mExtractPath;
        List<std::string> mZipFilePath;
        std::vector<char> mZipBuffer;


    };

}

#endif

#endif