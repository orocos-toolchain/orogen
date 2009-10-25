#ifndef TKDEPS_PARENT_H
#define TKDEPS_PARENT_H
namespace tkdeps_parent {
    struct Test
    {
        double a;
        double b;
    };

    // This type is for testing the specific handling of fixed size arrays. Only
    // one convertion function should be generated for any array size.
    struct FixedSizeArray
    {
        double a[10];
        double b[20];
    };
}
#endif
