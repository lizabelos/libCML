//
// Created by tbelos on 22/08/19.
//

#ifndef CML_LOGGERWIDGET_H
#define CML_LOGGERWIDGET_H

#include <cml/utils/Logger.h>

#include <QListView>
#include <QStringListModel>
#include <QGridLayout>
#include <QBasicTimer>
#include <QScrollBar>
#include <QLineEdit>

namespace CML {

    class LoggerWidget : public QWidget, public Logger::Observer {

        Q_OBJECT

    public:
        explicit LoggerWidget(QWidget *parent = nullptr);

        ~LoggerWidget();

        void onNewMessage(LoggerMessage message) final;

    protected slots:
        void addNewMessage(QString message);

    private:
        QLineEdit *mSearchEdit;
        QGridLayout *mLayout;
        QListView *mTextBrowser;
        QStringListModel *mModel;

    };

}


#endif //CML_LOGGERWIDGET_H
