//
// Created by tbelos on 23/08/19.
//

#ifndef CML_PARAMETERFLOATINGSLIDER_H
#define CML_PARAMETERFLOATINGSLIDER_H

#ifndef ANDROID

#include <QLineEdit>
#include <QMouseEvent>
#include "cml/base/AbstractFunction.h"

namespace CML {

    class ParameterFloatingSlider : public QLineEdit {

    Q_OBJECT

    public:
        explicit ParameterFloatingSlider(Parameter parameter, QWidget *parent = nullptr);

    private slots:
        void valueHasChanged();

    private:
        Parameter mParameter;

    };

}


#endif

#endif //CML_PARAMETERFLOATINGSLIDER_H
