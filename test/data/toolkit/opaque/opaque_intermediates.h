#ifndef TEST_OPAQUE_INTERMEDIATES_HH
#define TEST_OPAQUE_INTERMEDIATES_HH

namespace TestOpaque
{
    struct Point2D
    {
        short padding; // to make sure TestOpaque::Point2D and NotOrogenCompatible::Point2D are never aligned in the same way
        double x, y;
    };

    struct Position
    {
        double timestamp;
        NotOrogenCompatible::Point2D p;

#ifndef __orogen
        bool operator == (Position const& other) const
        { return other.timestamp == timestamp && other.p == p; }
#endif
    };
}

#endif

