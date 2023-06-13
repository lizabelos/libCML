//
// Created by tbelos on 22/08/19.
//

#ifndef CML_FUNCTIONLISTWIDGET_H
#define CML_FUNCTIONLISTWIDGET_H

#ifndef ANDROID

#include <vector>

#include <QScrollArea>
#include <QScrollBar>
#include <QWidget>
#include <QVBoxLayout>
#include <QFrame>
#include "FunctionWidget.h"
#include "cml/base/AbstractSlam.h"

namespace CML {

    class FunctionListWidget : public QScrollArea {

        Q_OBJECT

    public:
        explicit FunctionListWidget(AbstractSlam *slam, QWidget *parent = nullptr);

    protected:
        void paintEvent(QPaintEvent *);

    private:
        AbstractSlam *mSLAM;
        QFrame *mFrame;
        QVBoxLayout *mLayout;
        FunctionWidget* mFunctionWidget;

    };

}

#endif

#endif //CML_FUNCTIONLISTWIDGET_H
