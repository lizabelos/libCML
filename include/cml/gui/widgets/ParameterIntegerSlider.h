//
// Created by tbelos on 23/08/19.
//

#ifndef CML_PARAMETERINTEGERSLIDER_H
#define CML_PARAMETERINTEGERSLIDER_H

#include <QSpinBox>
#include <QMouseEvent>
#include "cml/base/AbstractFunction.h"

namespace CML {

    class ParameterIntegerSlider : public QSpinBox {

    Q_OBJECT

    public:
        explicit ParameterIntegerSlider(Parameter parameter, QWidget *parent = nullptr);

    private slots:
        void valueHasChanged();

    private:
        Parameter mParameter;

    };

}

#endif //CML_PARAMETERINTEGERSLIDER_H
