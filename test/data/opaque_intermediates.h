
namespace TestOpaque
{
    struct Point2D
    {
        short padding; // to make sure TestOpaque::Point2D and NotOrogenCompatible::Point2D are never aligned in the same way
        double x, y;
    };
}

