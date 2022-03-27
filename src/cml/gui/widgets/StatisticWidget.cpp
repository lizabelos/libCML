//
// Created by tbelos on 23/08/19.
//

#include "cml/gui/widgets/StatisticWidget.h"

CML::StatisticWidget::StatisticWidget(PStatistic statistic, QWidget *parent) : QWidget(parent), mStatistic(statistic) {

    setMinimumHeight(200.0f / 96.0f * logicalDpiX());

    mLayout = new QGridLayout();
    setLayout(mLayout);

    mChart = new QChart();
    mChartView = new QChartView(mChart);
    mLineSeries = new QLineSeries();

    mChart->addSeries(mLineSeries);
    mLayout->addWidget(mChartView);

    mChart->legend()->setVisible(true);


    //  connect(mCustomPlot, SIGNAL(mousePress(QMouseEvent*)), this, SLOT(plotClick(QMouseEvent*)));

    //mStatistic->subscribeObserver(this);
}

CML::StatisticWidget::~StatisticWidget() {
    mStatistic->removeObserver(this);
}

void CML::StatisticWidget::onNewValue(Statistic *statistic, scalar_t x, scalar_t y) {
    QMetaObject::invokeMethod(this, "requestRefresh", Qt::QueuedConnection, Q_ARG(scalar_t, x),  Q_ARG(scalar_t, y));
}

void CML::StatisticWidget::requestRefresh(scalar_t x, scalar_t y) {

    mLineSeries->append(x, y);
    mChart->removeSeries(mLineSeries); // WHYYYYY I NEED THIS ?
    mChart->addSeries(mLineSeries);

}

void CML::StatisticWidget::plotClick(QMouseEvent *mouseEvent) {

  /*  QString filename = QFileDialog::getSaveFileName(this,tr("Save Image"), QString(), tr("PNG File (*.png)"));
    if (filename.isEmpty()) {
        return;
    }

    mCustomPlot->savePng(filename, 640, 480);
*/
}
