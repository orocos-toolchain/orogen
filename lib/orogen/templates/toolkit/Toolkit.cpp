#include "<%= component.name %>ToolkitImpl.hpp"
#include "<%= component.name %>Toolkit.hpp"
#include <iostream>
#include <rtt/Property.hpp>
#include <boost/cstdint.hpp>
#include <boost/lexical_cast.hpp>
#include <rtt/Toolkit.hpp>
<% if has_opaques_with_templates? %>
#include "<%= component.name %>ToolkitIntermediates.hpp"
<% end %>

using RTT::Property;

<% converted_types.each do |type| %>
bool orogen_toolkits::toPropertyBag(std::string const& basename, <%= type.arg_type %> value, RTT::PropertyBag& target_bag)
{
<% if type.contains_int64? %>
    RTT::log(RTT::Error) << "<%= type.cxx_name %> contains 64 bit integers and therefore cannot be marshalled as XML" << RTT::endlog();
    return false;
<% else %>
<%= result = ""
        type.to_property_bag(toolkit, result, " " * 4) 
        result
        %> 
    return true;
<% end %>
}

bool orogen_toolkits::fromPropertyBag(std::string const& basename, <%= type.ref_type %> value, RTT::PropertyBag const& bag)
{
<% if type.contains_int64? %>
    RTT::log(RTT::Error) << "<%= type.cxx_name %> contains 64 bit integers and therefore cannot be marshalled as XML" << RTT::endlog();
    return false;
<% else %>
<%= result = ""
        type.from_property_bag(toolkit, result, " " * 4) 
        result
        %> 
    return true;
<% end %>
}
bool orogen_toolkits::toStream(std::string const& basename, <%= type.arg_type %> value, std::ostream& io)
{
<%= result = ""
        type.to_stream(toolkit, result, " " * 4)
        result %>
    return true;
}
bool orogen_toolkits::fromStream(std::string const& basename, <%= type.ref_type %> value, std::istream& io)
{
    return false;
}
<% end %>

<% opaques.each do |opdef|
    type = opdef.type
    intermediate_type = component.find_type(opdef.intermediate)
    %>
<%= if opdef.code_generator
      opdef.code_generator.call(type, intermediate_type)
    end %>
bool orogen_toolkits::toPropertyBag(std::string const& basename, <%= type.arg_type %> value, RTT::PropertyBag& target_bag)
{
    <%= toolkit.code_toIntermediate(intermediate_type, opdef.needs_copy?, "    ") %>
    return toPropertyBag(basename, intermediate, target_bag);
}

bool orogen_toolkits::fromPropertyBag(std::string const& basename, <%= type.ref_type %> value, RTT::PropertyBag const& bag)
{
    <% if opdef.needs_copy? %>
    <%= intermediate_type.cxx_name %> intermediate;
    if (!fromPropertyBag(basename, intermediate, bag))
        return false;
    <%= toolkit.code_fromIntermediate(intermediate_type, true, "    ") %>
    <% else %>
    std::auto_ptr< <%= intermediate_type.cxx_name %> > intermediate(new <%= intermediate_type.cxx_name %>);
    if (!fromPropertyBag(basename, *intermediate, bag))
        return false;
    <%= toolkit.code_fromIntermediate(intermediate_type, false, "    ") %>
    <% end %>
    return true;
}
bool orogen_toolkits::toStream(std::string const& basename, <%= type.arg_type %> value, std::ostream& io)
{
    <%= toolkit.code_toIntermediate(intermediate_type, opdef.needs_copy?, "    ") %>
    return toStream(basename, intermediate, io);
}
bool orogen_toolkits::fromStream(std::string const& basename, <%= type.ref_type %> value, std::istream& io)
{
    return false;
}
<% end %>



<% namespace = '/' %>
<% registered_types.each do |type| %>
<%= code = Generation.adapt_namespace(namespace, type.namespace)
   namespace = type.namespace 
   code %>
std::ostream& operator << (std::ostream& io, <%= type.cxx_name %> const& value)
{
    orogen_toolkits::toStream("", value, io);
    return io;
}
std::istream& operator >> (std::istream& io, <%= type.cxx_name %>& value)
{
    orogen_toolkits::fromStream("", value, io);
    return io;
}
<%= code = Generation.adapt_namespace(namespace, '/')
   namespace = '/'
   code %>
namespace orogen_toolkits {
    struct <%= type.method_name(true) %>TypeInfo :
	public RTT::TemplateTypeInfo<<%= type.cxx_name %>, true>
    {
        <%= type.method_name(true) %>TypeInfo();

    protected:
	bool decomposeTypeImpl(const <%= type.cxx_name %>& value, RTT::PropertyBag& target_bag) const;
	bool composeTypeImpl(const RTT::PropertyBag& bag, <%= type.cxx_name %>& out) const;
    };
}

orogen_toolkits::<%= type.method_name(true) %>TypeInfo::<%= type.method_name(true) %>TypeInfo()
    : RTT::TemplateTypeInfo<<%= type.cxx_name %>, true>("<%= type.full_name %>") {}

bool orogen_toolkits::<%= type.method_name(true) %>TypeInfo::decomposeTypeImpl(const <%= type.cxx_name %>& value, RTT::PropertyBag& target_bag) const
{
    target_bag.setType("<%= type.full_name %>");
    return orogen_toolkits::toPropertyBag("", value, target_bag);
}
bool orogen_toolkits::<%= type.method_name(true) %>TypeInfo::composeTypeImpl(const RTT::PropertyBag& bag, <%= type.cxx_name %>& out) const
{
    return orogen_toolkits::fromPropertyBag("", out, bag);
}

<% end %>

bool orogen_toolkits::<%= component.name %>ToolkitPlugin::loadTypes()
{
    RTT::TypeInfoRepository::shared_ptr ti_repository = RTT::TypeInfoRepository::Instance();
    RTT::TypeInfo* ti = 0;

    <% registered_types.each do |type| %>
    ti = new <%= type.method_name(true) %>TypeInfo();
    ti_repository->addType( ti );
    <% end %>

    return true;
}

bool orogen_toolkits::<%= component.name %>ToolkitPlugin::loadOperators()
{ return true; }
bool orogen_toolkits::<%= component.name %>ToolkitPlugin::loadConstructors()
{ return true; }
std::string orogen_toolkits::<%= component.name %>ToolkitPlugin::getName()
{ return "/orogen/<%= component.name %>"; }

orogen_toolkits::<%= component.name %>ToolkitPlugin orogen_toolkits::<%= component.name %>Toolkit;
ORO_TOOLKIT_PLUGIN(orogen_toolkits::<%= component.name %>Toolkit);

