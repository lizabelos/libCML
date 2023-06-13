//
// Created by tbelos on 23/08/19.
//

#ifndef CML_PARAMETERCHECKBOX_H
#define CML_PARAMETERCHECKBOX_H

#ifndef ANDROID

#include <QCheckBox>
#include "cml/base/AbstractFunction.h"

namespace CML {

    class ParameterCheckbox : public QCheckBox {

        Q_OBJECT

    public:
        explicit ParameterCheckbox(Parameter parameter, QWidget *parent = nullptr);

    private:
        Parameter mParameter;

    };

}

#endif

#endif //CML_PARAMETERCHECKBOX_H
