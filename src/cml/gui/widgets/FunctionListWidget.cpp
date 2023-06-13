//
// Created by tbelos on 22/08/19.
//

#ifndef ANDROID

#include "cml/gui/widgets/FunctionListWidget.h"

CML::FunctionListWidget::FunctionListWidget(CML::AbstractSlam *slam, QWidget *parent) : QScrollArea(parent), mSLAM(slam) {

    setObjectName("FunctionListWidget");

    setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    horizontalScrollBar()->setEnabled(false);

    mFrame = new QFrame();

    mLayout = new QVBoxLayout();

    mFunctionWidget = new FunctionWidget(slam, 1);
    mLayout->addWidget(mFunctionWidget);

    mFrame->setLayout(mLayout);
    setWidget(mFrame);
    setWidgetResizable(true);

}

void CML::FunctionListWidget::paintEvent(QPaintEvent *) {
    QStyleOption opt;
    opt.initFrom(this);
    QPainter p(this);
    style()->drawPrimitive(QStyle::PE_Widget, &opt, &p, this);
}

#endif
