//
// Created by thomas on 06/12/2020.
//

#ifndef CML_CAPTURESELECTOR_H
#define CML_CAPTURESELECTOR_H

#include <cml/config.h>
#include <cml/capture/AbstractCapture.h>

#include <QWidget>
#include <QGridLayout>
#include <QPushButton>
#include <QLabel>

namespace CML {

    class CaptureSelector : public QWidget {

        Q_OBJECT

    public:
        CaptureSelector();

        inline Ptr<AbstractCapture, Nullable> getCapture() {
            return mCapture;
        }

    private slots:
        void openKitty();
        void openTum();
        void openWebcam();
        void openVideo();
        void openIp();

    private:
        Ptr<AbstractCapture, Nullable> mCapture;

        QGridLayout mLayout;

        QLabel mLabelInstruction, mLabelError;
        QPushButton mButtonKitty, mButtonTUM, mButtonWebcam, mButtonVideo, mButtonIP;





    };

}


#endif //CML_CAPTURESELECTOR_H
