//
// Created by tbelos on 23/08/19.
//

#include "cml/gui/widgets/StatisticWidget.h"

CML::StatisticWidget::StatisticWidget(PStatistic statistic, QWidget *parent) : QWidget(parent), mStatistic(statistic) {

    setMinimumHeight(200.0f / 96.0f * logicalDpiX());

    mLayout = new QGridLayout();
    setLayout(mLayout);

    mCustomPlot = new QCustomPlot();

    mCustomPlot->plotLayout()->insertRow(0);
    QCPTextElement *title = new QCPTextElement(mCustomPlot, QString::fromStdString(statistic->getName()), QFont("sans", 12, QFont::Bold));
    title->setTextColor(QColor(255,255,255));
    mCustomPlot->plotLayout()->addElement(0, 0, title);

    QCPGraph *graph = mCustomPlot->addGraph();
   // graph->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssCircle, QPen(Qt::black, 1.5), QBrush(Qt::white), 9));
    graph->setPen(QPen(QColor(50, 50, 255), 2));


    mCustomPlot->xAxis->setBasePen(QPen(Qt::white, 1));
    mCustomPlot->yAxis->setBasePen(QPen(Qt::white, 1));
    mCustomPlot->xAxis->setTickPen(QPen(Qt::white, 1));
    mCustomPlot->yAxis->setTickPen(QPen(Qt::white, 1));
    mCustomPlot->xAxis->setSubTickPen(QPen(Qt::white, 1));
    mCustomPlot->yAxis->setSubTickPen(QPen(Qt::white, 1));
    mCustomPlot->xAxis->setTickLabelColor(Qt::white);
    mCustomPlot->yAxis->setTickLabelColor(Qt::white);
    mCustomPlot->xAxis->grid()->setPen(QPen(QColor(140, 140, 140), 1, Qt::DotLine));
    mCustomPlot->yAxis->grid()->setPen(QPen(QColor(140, 140, 140), 1, Qt::DotLine));
    mCustomPlot->xAxis->grid()->setSubGridPen(QPen(QColor(80, 80, 80), 1, Qt::DotLine));
    mCustomPlot->yAxis->grid()->setSubGridPen(QPen(QColor(80, 80, 80), 1, Qt::DotLine));
    mCustomPlot->xAxis->grid()->setSubGridVisible(true);
    mCustomPlot->yAxis->grid()->setSubGridVisible(true);
    mCustomPlot->xAxis->grid()->setZeroLinePen(Qt::NoPen);
    mCustomPlot->yAxis->grid()->setZeroLinePen(Qt::NoPen);
    mCustomPlot->xAxis->setUpperEnding(QCPLineEnding::esSpikeArrow);
    mCustomPlot->yAxis->setUpperEnding(QCPLineEnding::esSpikeArrow);
    QLinearGradient plotGradient;
    plotGradient.setStart(0, 0);
    plotGradient.setFinalStop(0, 350);
    plotGradient.setColorAt(0, QColor(80, 80, 80));
    plotGradient.setColorAt(1, QColor(50, 50, 50));
    mCustomPlot->setBackground(plotGradient);
    QLinearGradient axisRectGradient;
    axisRectGradient.setStart(0, 0);
    axisRectGradient.setFinalStop(0, 350);
    axisRectGradient.setColorAt(0, QColor(80, 80, 80));
    axisRectGradient.setColorAt(1, QColor(30, 30, 30));
    mCustomPlot->axisRect()->setBackground(axisRectGradient);


    mLayout->addWidget(mCustomPlot);

  //  connect(mCustomPlot, SIGNAL(mousePress(QMouseEvent*)), this, SLOT(plotClick(QMouseEvent*)));

    mStatistic->subscribeObserver(this);
}

CML::StatisticWidget::~StatisticWidget() {
    mStatistic->removeObserver(this);
}

void CML::StatisticWidget::onNewValue(Statistic *statistic, scalar_t x, scalar_t y) {
    //QMetaObject::invokeMethod(this, "requestRefresh", Qt::QueuedConnection, Q_ARG(scalar_t, x),  Q_ARG(scalar_t, y));
}

void CML::StatisticWidget::requestRefresh(scalar_t x, scalar_t y) {

    if (mX.size() == mSize) {
        mX.remove(0);
        mY.remove(0);
    }

    mX.append(x);
    mY.append(y);

    double maxValue = 0;
    for (auto v : mY) maxValue = std::max(maxValue, v);

    mCustomPlot->graph(0)->setData(mX, mY, true);
    mCustomPlot->graph(0)->rescaleAxes();
    mCustomPlot->yAxis->setRange(0, maxValue);
    mCustomPlot->replot();

}

void CML::StatisticWidget::plotClick(QMouseEvent *mouseEvent) {

  /*  QString filename = QFileDialog::getSaveFileName(this,tr("Save Image"), QString(), tr("PNG File (*.png)"));
    if (filename.isEmpty()) {
        return;
    }

    mCustomPlot->savePng(filename, 640, 480);
*/
}
