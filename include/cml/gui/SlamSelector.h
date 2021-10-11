//
// Created by thomas on 06/12/2020.
//

#ifndef CML_SLAMSELECTOR_H
#define CML_SLAMSELECTOR_H

#include <cml/config.h>
#include <cml/base/AbstractSlam.h>

#include <QWidget>
#include <QLabel>
#include <QPushButton>
#include <QGridLayout>

namespace CML {

    class SlamSelector : public QWidget {

    Q_OBJECT

    public:
        SlamSelector();

        ~SlamSelector();

        Ptr<AbstractSlam, Nullable> getSLAM() {
            return mSLAM;
        }

    public slots:
        void changeSlam(std::string soPath);
        void changeSlamDialog();

    private:
        QGridLayout mLayout;
        QLabel mLabelInstructions;
        QLabel mLabelError;

        List<QPushButton*> mButtonsSLAM;
        QPushButton mButtonCustomSLAM;

        Ptr<AbstractSlam, Nullable> mSLAM;

    };

}


#endif //CML_SLAMSELECTOR_H
