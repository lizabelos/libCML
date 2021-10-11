//
// Created by thomas on 06/12/2020.
//

#include "cml/gui/CaptureSelector.h"

#include <QFileDialog>
#include <QInputDialog>

#if CML_HAVE_AVFORMAT
#include "cml/capture/VideoCapture.h"
#endif

#if CML_HAVE_LIBZIP
#include "cml/capture/TUMCapture.h"
#endif

#include "cml/capture/KittyCapture.h"
#include "cml/utils/Logger.h"
#include "cml/base/AbstractSlam.h"
// #include "cml/gui/capture/QtWebcamCapture.h"

CML::CaptureSelector::CaptureSelector() {

    setObjectName("CaptureSelector");

    // QLabel mLabelInstruction, mLabelError;
   // QPushButton mButtonKitty, mButtonTUM, mButtonWebcam, mButtonVideo, mButtonIP;

    mLabelInstruction.setText("Please select a capture device : ");
    mLayout.addWidget(&mLabelInstruction, 0, 0);

    mLabelError.setText("");
    mLayout.addWidget(&mLabelError, 1, 0);

    mButtonKitty.setText("KITTY");
    connect(&mButtonKitty, SIGNAL(pressed()), this, SLOT(openKitty()));
    mLayout.addWidget(&mButtonKitty, 3, 0);

    mButtonTUM.setText("TUM");
    connect(&mButtonTUM, SIGNAL(pressed()), this, SLOT(openTum()));
    mLayout.addWidget(&mButtonTUM, 4, 0);

    mButtonWebcam.setText("Camera");
    connect(&mButtonWebcam, SIGNAL(pressed()), this, SLOT(openWebcam()));
    mLayout.addWidget(&mButtonWebcam, 5, 0);

    mButtonVideo.setText("Video");
    connect(&mButtonVideo, SIGNAL(pressed()), this, SLOT(openVideo()));
    mLayout.addWidget(&mButtonVideo, 6, 0);

    mButtonIP.setText("IP Camera");
    connect(&mButtonIP, SIGNAL(pressed()), this, SLOT(openIp()));
    mLayout.addWidget(&mButtonIP, 7, 0);
    
    setLayout(&mLayout);

}

void CML::CaptureSelector::openTum() {
#if CML_HAVE_LIBZIP

    QString tumPath = QFileDialog::getExistingDirectory(this, "Open Kitty Dataset");

    try {
        mCapture = new CML::TUMCapture(tumPath.toStdString());
    } catch (const std::exception &e) {
        mLabelError.setText(QString::fromStdString(e.what()));
        CML::logger.fatal(e.what());
    }

#else
    mLabelError.setText("CML need to be compiled with libzip for TUM to work");
#endif

}

void CML::CaptureSelector::openKitty() {

    QString kittyPath = QFileDialog::getExistingDirectory(this, "Open TUM Dataset");

    try {
        mCapture = new CML::KittyCapture(kittyPath.toStdString());
    } catch (const std::exception &e) {
        mLabelError.setText(QString::fromStdString(e.what()));
        CML::logger.fatal(e.what());
    }

}

void CML::CaptureSelector::openWebcam() {

    try {
//        mCapture = new QtWebcamCapture();
    } catch (const std::exception &e) {
        mLabelError.setText(QString::fromStdString(e.what()));
        CML::logger.fatal(e.what());
    }

}

void CML::CaptureSelector::openIp() {
#if CML_HAVE_AVFORMAT

    bool ok;
    QString text = QInputDialog::getText(this, tr("QInputDialog::getText()"),tr("IP :"), QLineEdit::Normal, "localhost", &ok);
    if (ok && !text.isEmpty()) {

        try {
            mCapture = new CML::VideoCapture(text.toStdString());
        } catch (const std::exception &e) {
            mLabelError.setText(QString::fromStdString(e.what()));
            CML::logger.fatal(e.what());
        }

    }
#else
    mLabelError.setText("CML need to be compiled with ffmpeg for ip video to work");
#endif
}

void CML::CaptureSelector::openVideo() {
#if CML_HAVE_AVFORMAT

    QString videoPath = QFileDialog::getOpenFileName(this, "Open Video File");

    try {
        mCapture = new CML::VideoCapture(videoPath.toStdString());
    } catch (const std::exception &e) {
        mLabelError.setText(QString::fromStdString(e.what()));
        CML::logger.fatal(e.what());
    }
#else
    mLabelError.setText("CML need to be compiled with ffmpeg for video to work");
#endif
}
