#include "<%= component.name %>ToolkitCorba.hpp"
#include "<%= component.name %>ToolkitCorbaImpl.hpp"
#include <rtt/Toolkit.hpp>
using namespace RTT;

<% converted_types.each do |type| %>
bool orogen_toolkits::toCORBA( <%= type.corba_ref_type %> corba, <%= type.arg_type %> value )
{
<%= result = ""
	type.to_corba(toolkit, result, " " * 4)
	result 
	%>
    return true;
}
bool orogen_toolkits::fromCORBA( <%= type.corba_arg_type %> corba, <%= type.ref_type %> value )
{
<%= result = ""
	type.from_corba(toolkit, result, " " * 4)
	result 
	%>
    return true;
}
<% end %>

<% registered_types.each do |type| %>
namespace orogen_toolkits {
    struct <%= type.method_name %>CorbaMarshaller
        : public RTT::detail::CorbaTemplateProtocol< <%= type.cxx_name %> >
    {
        typedef <%= type.corba_name %> CorbaType;
        typedef <%= type.cxx_name %>   BaseType;

        virtual void* createBlob( RTT::DataSourceBase::shared_ptr source) const;
        virtual bool updateBlob(const void* blob, RTT::DataSourceBase::shared_ptr target) const;
    };
}

void* orogen_toolkits::<%= type.method_name %>CorbaMarshaller::createBlob( DataSourceBase::shared_ptr source) const
{
    DataSource<BaseType>* d = AdaptDataSource<BaseType>()( source );
    if ( d )
    {
        std::auto_ptr<CorbaType> corba( new CorbaType );
        if (!toCORBA(*corba, d->value()))
            return false;

        CORBA::Any_ptr ret = new CORBA::Any();
        *ret <<= corba.release();
        return ret;
    }
    return 0;
}

bool orogen_toolkits::<%= type.method_name %>CorbaMarshaller::updateBlob(const void* blob, DataSourceBase::shared_ptr target) const
{
    AssignableDataSource<BaseType>::shared_ptr ad = AssignableDataSource<BaseType>::narrow( target.get() );
    const CORBA::Any* any = static_cast<const CORBA::Any*>(blob);
    if ( ad )
    {
        <% if type.inlines_code? || type <= Typelib::EnumType %>
        CorbaType  corba;
        if (!(*any >>= corba))
            return false;
        return fromCORBA(corba, ad->set());
        <% else %>
        CorbaType*  corba;
        if (!(*any >>= corba))
            return false;
        bool ret = fromCORBA(*corba, ad->set());
        delete corba;
        return ret;
        <% end %>
    }
    return false;
}
<% end %>

bool orogen_toolkits::<%= component.name %>CorbaTransportPlugin::registerTransport(std::string type_name, TypeInfo* ti)
{
    <% first_type = true;
       registered_types.each do |type| %>
    <%= 'else ' unless first_type %>if ("<%= type.name %>" == type_name)
    {
        ti->addProtocol(ORO_CORBA_PROTOCOL_ID, new orogen_toolkits::<%= type.method_name %>CorbaMarshaller());
        return true;
    }
    <% first_type = false
    end %>
    return false;
}
std::string orogen_toolkits::<%= component.name %>CorbaTransportPlugin::getTransportName() const
{ return "CORBA"; }
std::string orogen_toolkits::<%= component.name %>CorbaTransportPlugin::getName() const
{ return "/orogen/<%= component.name %>/CORBA"; }

orogen_toolkits::<%= component.name %>CorbaTransportPlugin orogen_toolkits::<%= component.name %>CorbaTransport;
ORO_TOOLKIT_PLUGIN(orogen_toolkits::<%= component.name %>CorbaTransport);

