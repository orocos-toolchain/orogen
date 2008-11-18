#include "TestOpaqueToolkitUser.hpp"


void TestOpaque::to_intermediate(TestOpaque::Point2D& intermediate, NotOrogenCompatible::Point2D const& real_type)
{
    intermediate.padding = 100;
    intermediate.x = real_type.x();
    intermediate.y = real_type.y();
}
void TestOpaque::from_intermediate(NotOrogenCompatible::Point2D& real_type, TestOpaque::Point2D const& intermediate)
{
    real_type.x() = intermediate.x;
    real_type.y() = intermediate.y;
}


