#ifndef CML_ZIPCAPTURE_H
#define CML_ZIPCAPTURE_H

#include "cml/config.h"
#include "AbstractCapture.h"

#if CML_HAVE_LIBZIP

#include <zip.h>

namespace CML {

    class ZipCaptureHelper {

    protected:
        inline void loadZip(const std::string &filename, const std::string &suffix) {
            int ziperror = 0;
            mZipArchive = zip_open(filename.c_str(),  ZIP_RDONLY, &ziperror);

            if (ziperror != 0) {
                throw std::runtime_error("Can't open " + filename);
            }

            int numEntries = zip_get_num_entries(mZipArchive, 0);
            logger.info("Found " + std::to_string(numEntries) + " files inside the zip");
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

        inline void decompressFile(const std::string &filename, uint8_t **data, size_t *size) {
            if (mZipBuffer.size() == 0) {
                mZipBuffer.resize(1e+8);
            }

            *size = 0;

            zip_file_t *zipFile = zip_fopen(mZipArchive, filename.c_str(), 0);
            while (true) {
                size_t n = zip_fread(zipFile, mZipBuffer.data() + *size, mZipBuffer.size());
                if (n == 0) break;
                *size += n;
            }
            zip_fclose(zipFile);
            *data = (uint8_t*)mZipBuffer.data();
        }

        inline void decompressFile(int id, uint8_t **data, size_t *size) {
            return decompressFile(mZipFilePath[id], data, size);
        }

        inline const std::string &getFilename(int id) {
            return mZipFilePath[id];
        }

        inline int getImageNumber() {
            return mZipFilePath.size();
        }

    private:
        zip_t *mZipArchive;
        List<std::string> mZipFilePath;
        std::vector<char> mZipBuffer;


    };

}

#endif

#endif