//
// Created by tbelos on 23/08/19.
//

#ifndef CML_STATISTICWIDGET_H
#define CML_STATISTICWIDGET_H

#include <QLabel>
#include <QBasicTimer>
#include <QTimerEvent>
#include <QGridLayout>
#include <QMouseEvent>
#include <QFileDialog>
#include <QLineSeries>
#include <QChart>
#include <QChartView>

#include "cml/base/AbstractFunction.h"

namespace CML {

    class StatisticWidget : public QWidget, public Statistic::Observer {

        Q_OBJECT

    public:
        explicit StatisticWidget(PStatistic statistic, QWidget *parent = nullptr);

        ~StatisticWidget();

        void onNewValue(Statistic *statistic, scalar_t x, scalar_t y) final;

    protected slots:
        void plotClick(QMouseEvent *mouseEvent);

        void requestRefresh(scalar_t x, scalar_t y);

    private:
        PStatistic mStatistic;
        QGridLayout *mLayout;
        QChart *mChart;
        QChartView *mChartView;
        QLineSeries *mLineSeries;

        int mMaxSize = 100;
        int mI = 0;
        int mSize = 0;
        QVector<double> mX, mY;

    };

}


#endif //CML_STATISTICWIDGET_H
