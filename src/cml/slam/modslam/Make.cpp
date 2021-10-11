#include "Hybrid.h"

extern "C" {
AbstractSlam *make() {
    return new Hybrid();
}
}