#include <opaque/typekit/OpaqueTypes.hpp>
#include <opaque/typekit/Opaques.hpp>

void orogen_typekits::toIntermediate(TestOpaque::Point2D& intermediate, NotOrogenCompatible::Point2D const& real_type)
{
    intermediate.padding = 100;
    intermediate.x = real_type.x();
    intermediate.y = real_type.y();
}
void orogen_typekits::fromIntermediate(NotOrogenCompatible::Point2D& real_type, TestOpaque::Point2D const& intermediate)
{
    real_type.x() = intermediate.x;
    real_type.y() = intermediate.y;
}


std::vector<float> const& orogen_typekits::toIntermediate(boost::shared_ptr<std::vector<float> > const& real_type)
{
    return *real_type;
}
bool orogen_typekits::fromIntermediate(boost::shared_ptr<std::vector<float> >& real_type, std::vector<float>* intermediate)
{
    if (intermediate != real_type.get())
        real_type.reset( intermediate );
    return true; // we get the ownership on +intermediate+
}
