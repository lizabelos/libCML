//
// Created by tbelos on 22/08/19.
//

#ifndef ANDROID

#include "cml/gui/widgets/FunctionWidget.h"

CML::FunctionWidget::FunctionWidget(Ptr<AbstractFunction, NonNullable> function, int level, QWidget *parent) : mFunction(function), mMainLayout(this) {

    setObjectName("FunctionWidget");

    /// THE TITLE
    mTitle.setText(QString::fromStdString("<h" + std::to_string(level) + ">" + function->getName() + "</h" + std::to_string(level) + ">"));
    mMainLayout.addWidget(&mTitle);

    /// CHILDS
    List<Ptr<AbstractFunction, NonNullable>> childs = function->getChildFunctions();
    for (auto child : childs) {
        FunctionWidget *functionWidget = new FunctionWidget(child, level + 1);
        mChildsLayout.addWidget(functionWidget);
        mChildWidgets.emplace_back(functionWidget);
    }
    mMainLayout.addLayout(&mChildsLayout);

    /// VIEW BUTTON
    mViewCheck.setCheckable(true);
    mViewCheck.setText("Show");
    connect(&mViewCheck, SIGNAL(clicked()), this, SLOT(viewStateChange()));
    mMainLayout.addWidget(&mViewCheck);

    /// PARAMETERS
    mParametersLayout.setColumnStretch(0, 5);
    mParametersLayout.setColumnStretch(1, 1);

    unsigned int line = 0;
    for (auto parameter : function->getParameters()) {

        switch (parameter.type()) {

            case PARAM_INTEGER: {
                QLabel *label = new QLabel(QString::fromStdString(parameter.name()));
                mParametersLayout.addWidget(label, line, 0);

                ParameterIntegerSlider *integerSlider = new ParameterIntegerSlider(parameter);
                mParametersLayout.addWidget(integerSlider, line, 1);
                break;
            }
            case PARAM_FLOATING: {
                QLabel *label = new QLabel(QString::fromStdString(parameter.name()));
                mParametersLayout.addWidget(label, line, 0);

                ParameterFloatingSlider *floatingSlider = new ParameterFloatingSlider(parameter);
                mParametersLayout.addWidget(floatingSlider, line, 1);
                break;
            }
            case PARAM_BOOLEAN: {
                QLabel *label = new QLabel(QString::fromStdString(parameter.name()));
                mParametersLayout.addWidget(label, line, 0);

                ParameterCheckbox *checkbox = new ParameterCheckbox(parameter);
                mParametersLayout.addWidget(checkbox, line, 1);
                break;
            }
        }

        line++;

    }

    mMainLayout.addLayout(&mParametersLayout);


    /// STATISTICS

    for (PStatistic statistic : function->getStatistics()) {

        QLabel *label = new QLabel(QString::fromStdString(statistic->getName()));
        mStatisticsLayout.addWidget(label);

        line++;

        StatisticWidget *statisticWidget = new StatisticWidget(statistic);
        mStatisticsLayout.addWidget(statisticWidget);

        line++;
    }

    mMainLayout.addLayout(&mStatisticsLayout);

    /// APPLY THE MAIN LAYOUT TO THIS WIDGET
   // setContentLayout(mMainLayout);



}

bool CML::FunctionWidget::viewStateChange() {
    mFunction->setViewable(mViewCheck.isChecked());
    return true;
}

void CML::FunctionWidget::onNewTimerValue(const Timer &timer, scalar_t value) {
    QMetaObject::invokeMethod(this, "requestRefresh", Qt::QueuedConnection, Q_ARG(scalar_t, value));
}

void CML::FunctionWidget::requestRefresh(scalar_t value) {
    // mTime->setText(QString::fromStdString(std::to_string(mFunction->getTimer().getValue()) + "s"));
    mViewCheck.setChecked(mFunction->isViewableOnModel() || mFunction->isViewableOnCapture());
}

void CML::FunctionWidget::paintEvent(QPaintEvent *) {
    QStyleOption opt;
    opt.initFrom(this);
    QPainter p(this);
    style()->drawPrimitive(QStyle::PE_Widget, &opt, &p, this);
}

#endif
