#pragma once

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

