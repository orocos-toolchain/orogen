#include "opaqueToolkitUser.hpp"


void opaque::to_intermediate(TestOpaque::Point2D& intermediate, NotOrogenCompatible::Point2D const& real_type)
{
    intermediate.padding = 100;
    intermediate.x = real_type.x();
    intermediate.y = real_type.y();
}
void opaque::from_intermediate(NotOrogenCompatible::Point2D& real_type, TestOpaque::Point2D& intermediate)
{
    real_type.x() = intermediate.x;
    real_type.y() = intermediate.y;
}


std::vector<float> const& opaque::to_intermediate(boost::shared_ptr<std::vector<float> > const& real_type)
{
    return *real_type;
}
bool opaque::from_intermediate(boost::shared_ptr<std::vector<float> >& real_type, std::vector<float>* intermediate)
{
    real_type.reset( intermediate );
    return true; // we get the ownership on +intermediate+
}
