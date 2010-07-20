// First load all RTT interfaces so that we get all "extern template"
// declarations in the TypekitImpl files
#include <rtt/Ports.hpp>
#include <rtt/Attribute.hpp>
#include <rtt/Properties.hpp>
#include <rtt/DataSource.hpp>

#include "typekit/<%= component.name %>TypekitC.h"
#include "<%= component.name %>TypekitCorba.hpp"
#include "<%= component.name %>TypekitCorbaImpl.hpp"
#include <rtt/corba/CorbaTemplateProtocol.hpp>
#include <rtt/Typekit.hpp>
<% if has_opaques? %>
#include "<%= component.name %>TypekitIntermediates.hpp"
<% end %>
<% typekit.used_typekits.each do |tk| %>
#include <typekit/<%= tk.name %>TypekitCorbaImpl.hpp>
<% end %>
using namespace RTT;

<% converted_types.each do |type| %>
bool orogen_typekits::toCORBA( <%= type.corba_ref_type %> corba, <%= type.arg_type %> value )
{
<%= result = ""
	type.to_corba(typekit, result, " " * 4)
	result 
	%>
    return true;
}
bool orogen_typekits::fromCORBA( <%= type.ref_type %> value, <%= type.corba_arg_type %> corba )
{
<%= result = ""
	type.from_corba(typekit, result, " " * 4)
	result 
	%>
    return true;
}
<% end %>
<% array_types.each do |type| %>
bool orogen_typekits::toCORBA( <%= type.corba_ref_type %> corba, <%= type.arg_type %> value, int length )
{
<%= result = ""
	type.to_corba(typekit, result, " " * 4)
	result 
	%>
    return true;
}
bool orogen_typekits::fromCORBA( <%= type.ref_type %> value, int length, <%= type.corba_arg_type %> corba )
{
<%= result = ""
	type.from_corba(typekit, result, " " * 4)
	result 
	%>
    return true;
}
<% end %>

<% opaques.each do |opdef|
    type = opdef.type
    intermediate_type = component.find_type(opdef.intermediate)
    %>
bool orogen_typekits::toCORBA( <%= intermediate_type.corba_ref_type %> corba, <%= type.arg_type %> value )
{
<%= typekit.code_toIntermediate(intermediate_type, opdef.needs_copy?, "    ") %>
    return toCORBA(corba, intermediate);
}
bool orogen_typekits::fromCORBA( <%= type.ref_type %> value, <%= intermediate_type.corba_arg_type %> corba )
{
    <% if opdef.needs_copy? %>
    <%= intermediate_type.cxx_name %> intermediate;
    if (!fromCORBA(intermediate, corba))
        return false;
<%= typekit.code_fromIntermediate(intermediate_type, true, "    ") %>
    <% else %>
    std::auto_ptr< <%= intermediate_type.cxx_name %> > intermediate(new <%= intermediate_type.cxx_name %>);
    if (!fromCORBA(*intermediate, corba))
        return false;
<%= typekit.code_fromIntermediate(intermediate_type, false, "    ") %>
    <% end %>
    return true;
}
<% end %>

<% registered_types.each do |type| %>
namespace orogen_typekits {
    struct <%= type.method_name %>CorbaMarshaller
        : public RTT::detail::CorbaTemplateProtocol< <%= type.cxx_name %> >
    {
        <% if type.opaque?
             opdef = typekit.opaque_specification(type)
             intermediate_type = typekit.find_type(opdef.intermediate)
        %>
        typedef <%= intermediate_type.corba_name(true) %> CorbaType;
        <% else %>
        typedef <%= type.corba_name(true) %> CorbaType;
        <% end %>
        typedef <%= type.cxx_name %>   BaseType;

        virtual void* createBlob( RTT::DataSourceBase::shared_ptr source) const;
        virtual bool updateBlob(const void* blob, RTT::DataSourceBase::shared_ptr target) const;
    };
}

void* orogen_typekits::<%= type.method_name %>CorbaMarshaller::createBlob( DataSourceBase::shared_ptr source) const
{
    DataSource<BaseType>* d = AdaptDataSource<BaseType>()( source );
    if ( d )
    {
        <% if type.inlines_code? || type <= Typelib::EnumType %>
        CorbaType corba;
        if (!toCORBA(corba, d->value()))
            return false;
        CORBA::Any_ptr ret = new CORBA::Any();
        *ret <<= corba;
        <% else %>
        std::auto_ptr<CorbaType> corba( new CorbaType );
        if (!toCORBA(*corba, d->value()))
            return false;
        CORBA::Any_ptr ret = new CORBA::Any();
        *ret <<= corba.release();
        <% end %>
        return ret;
    }
    return 0;
}

bool orogen_typekits::<%= type.method_name %>CorbaMarshaller::updateBlob(const void* blob, DataSourceBase::shared_ptr target) const
{
    AssignableDataSource<BaseType>::shared_ptr ad = AssignableDataSource<BaseType>::narrow( target.get() );
    const CORBA::Any* any = static_cast<const CORBA::Any*>(blob);
    if ( ad )
    {
        <% if type.inlines_code? || type <= Typelib::EnumType %>
        CorbaType  corba;
        if (!(*any >>= corba))
            return false;
        return fromCORBA(ad->set(), corba);
        <% else %>
        CorbaType*  corba;
        if (!(*any >>= corba))
            return false;
        bool ret = fromCORBA(ad->set(), *corba);
        return ret;
        <% end %>
    }
    return false;
}
<% end %>

bool orogen_typekits::<%= component.name %>CorbaTransportPlugin::registerTransport(std::string type_name, TypeInfo* ti)
{
    <% first_type = true;
       registered_types.each do |type| %>
    <%= 'else ' unless first_type %>if ("<%= type.name %>" == type_name)
    {
        ti->addProtocol(ORO_CORBA_PROTOCOL_ID, new orogen_typekits::<%= type.method_name %>CorbaMarshaller());
        return true;
    }
    <% first_type = false
    end %>
    return false;
}
std::string orogen_typekits::<%= component.name %>CorbaTransportPlugin::getTransportName() const
{ return "CORBA"; }
std::string orogen_typekits::<%= component.name %>CorbaTransportPlugin::getName() const
{ return "/orogen/<%= component.name %>/CORBA"; }

orogen_typekits::<%= component.name %>CorbaTransportPlugin orogen_typekits::<%= component.name %>CorbaTransport;
ORO_TOOLKIT_PLUGIN(orogen_typekits::<%= component.name %>CorbaTransport);

