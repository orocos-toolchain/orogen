
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
    };
}

