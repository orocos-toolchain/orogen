#pragma once

#include <tkdeps_lib.h>
#include <tkdeps_parent/subdir/tkdeps_parent.h>

namespace tkdeps {
    struct Combined {
        tkdeps_lib::Test a;
        tkdeps_parent::Test b;
    };
}

