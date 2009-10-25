#ifndef TKDEPS_0_H
#define TKDEPS_0_H

#include <tkdeps_lib.h>

namespace tkdeps {
    struct OnlyLib {
        tkdeps_lib::Test a;
    };

    struct FixedSizeHandling
    {
        double test[40];
    };
}

#endif

