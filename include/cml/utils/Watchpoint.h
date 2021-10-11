//
// Created by thomas on 16/02/2021.
//

#ifndef CML_WATCHPOINT_H
#define CML_WATCHPOINT_H

#include <cml/config.h>

namespace CML {

    class Watchpoint {

    public:
        explicit Watchpoint(void *addr);

        ~Watchpoint();

    private:
        void *mAddr;

    };


}

#endif //CML_WATCHPOINT_H
