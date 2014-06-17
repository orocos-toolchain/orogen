#ifndef TEST_OPAQUE_INTERMEDIATES_HH
#define TEST_OPAQUE_INTERMEDIATES_HH

namespace NotOrogenCompatible
{
    class Point2D
    {
        float _x, _y;

    public:
        explicit Point2D(float x = 0, float y = 0)
            : _x(x), _y(y) {}

        float x() const { return _x; }
        float y() const { return _y; }
        float& x() { return _x; }
        float& y() { return _y; }

        bool operator == (Point2D const& other) const
        { return other.x() == x() && other.y() == y(); }
    };
}

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

        bool operator == (Position const& other) const
        { return other.timestamp == timestamp && other.p == p; }
    };
}

#endif

