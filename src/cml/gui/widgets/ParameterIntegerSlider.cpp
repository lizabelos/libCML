//
// Created by tbelos on 23/08/19.
//

#include "cml/gui/widgets/ParameterIntegerSlider.h"

CML::ParameterIntegerSlider::ParameterIntegerSlider(Parameter parameter, QWidget *parent) : QSpinBox(parent), mParameter(parameter) {
    setValue(parameter.i());

    connect(this, SIGNAL(valueChanged(int)), this, SLOT(valueHasChanged()));
}

void CML::ParameterIntegerSlider::valueHasChanged() {
    mParameter.set((int)value());
}
