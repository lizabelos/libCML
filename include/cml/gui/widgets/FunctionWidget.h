//
// Created by tbelos on 22/08/19.
//

#ifndef CML_FUNCTIONWIDGET_H
#define CML_FUNCTIONWIDGET_H

#ifndef ANDROID

#include <QWidget>
#include <QGridLayout>
#include <QLabel>
#include <QSlider>
#include <QCheckBox>
#include <QBasicTimer>
#include <QTimerEvent>
#include <QMouseEvent>
#include <QCheckBox>
#include <QPushButton>
#include <QStyle>
#include <QPainter>
#include <QStyleOption>

#include "cml/base/AbstractFunction.h"
#include "ParameterCheckbox.h"
#include "ParameterFloatingSlider.h"
#include "ParameterIntegerSlider.h"
#include "StatisticWidget.h"
#include "Spoiler.h"

namespace CML {

    class FunctionWidget : public QWidget, public Timer::Observer {

        Q_OBJECT

    public:
        explicit FunctionWidget(Ptr<AbstractFunction, NonNullable> function, int level, QWidget *parent = nullptr);

        ~FunctionWidget() {
            for (auto widget : mChildWidgets) delete widget;
        }

        void onNewTimerValue(const Timer &timer, scalar_t value) final;

    protected:
        void paintEvent(QPaintEvent *);

    private slots:
        bool viewStateChange();

        void requestRefresh(scalar_t value);

    private:
        Ptr<AbstractFunction, NonNullable> mFunction;

        QVBoxLayout mMainLayout;
        QVBoxLayout mChildsLayout;
        QGridLayout mParametersLayout;
        QVBoxLayout mStatisticsLayout;

        List<FunctionWidget*> mChildWidgets;


        QLabel mTitle;
        QPushButton mViewCheck;

    };

}

#endif

#endif //CML_FUNCTIONWIDGET_H
