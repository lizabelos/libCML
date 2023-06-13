//
// Created by tbelos on 22/08/19.
//

#include "cml/gui/widgets/LoggerWidget.h"
#include "cml/utils/Logger.h"

CML::LoggerWidget::LoggerWidget(QWidget *parent) : QWidget(parent) {

    mLayout = new QGridLayout(this);

    mSearchEdit = new QLineEdit(this);
    mSearchEdit->setPlaceholderText("Search bar");
    mLayout->addWidget(mSearchEdit, 0, 0);

    mTextBrowser = new QListView(this);
    mModel = new QStringListModel(this);

    mTextBrowser->setModel(mModel);
    mTextBrowser->setEditTriggers(QAbstractItemView::NoEditTriggers);

    mLayout->addWidget(mTextBrowser, 1, 0);

    logger.subscribeObserver(this);
}

CML::LoggerWidget::~LoggerWidget() {
    logger.removeObserver(this);
}

void CML::LoggerWidget::onNewMessage(LoggerMessage message) {

    QString color = "black";

    switch (message.level) {

        case MORE:
            color = "black";
            break;
        case INFO:
            color = "blue";
            break;
        case WARN:
            color = "orange";
            break;
        case ERR:
            color = "red";
            break;
        case DEADLY:
            color = "red";
            break;
        case IMPORTANT:
            color = "red";
            break;
    }

    QString html = "<span style='color:" + color + "'>" + QString::fromStdString(message.formatted) + "</span>";

    //QMetaObject::invokeMethod(this, "addNewMessage", Qt::QueuedConnection, Q_ARG(QString, html));

}

void CML::LoggerWidget::addNewMessage(QString message) {
    int row = mModel->rowCount();
    mModel->insertRow(row);

    QModelIndex index = mModel->index(row);
    mModel->setData(index, message);
}
