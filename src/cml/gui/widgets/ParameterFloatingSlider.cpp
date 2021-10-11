//
// Created by tbelos on 23/08/19.
//

#include "cml/gui/widgets/ParameterFloatingSlider.h"

#include <QRegExpValidator>

CML::ParameterFloatingSlider::ParameterFloatingSlider(Parameter parameter, QWidget *parent) : QLineEdit(parent), mParameter(parameter) {
    //setValue(parameter.f());
    setText(QString::fromStdString(std::to_string(parameter.f())));
    connect((QLineEdit*)this, SIGNAL(textEdited(QString)), this, SLOT(valueHasChanged()));

    QRegExp re("^[0-9]*[.,]?[0-9]+$");
    QRegExpValidator *validator = new QRegExpValidator(re, this);
    setValidator(validator);
}

void CML::ParameterFloatingSlider::valueHasChanged() {
    mParameter.set(text().toFloat());
    //mParameter.set((float)value());
}
