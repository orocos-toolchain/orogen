// First load all RTT interfaces so that we get all "extern template"
// declarations in the ToolkitImpl files
#include <rtt/Ports.hpp>
#include <rtt/Attribute.hpp>
#include <rtt/Properties.hpp>
#include <rtt/DataSource.hpp>

#include "<%= component.name %>ToolkitImpl.hpp"
#include "<%= component.name %>Toolkit.hpp"
#include <iostream>
#include <rtt/Property.hpp>
#include <boost/cstdint.hpp>
#include <boost/lexical_cast.hpp>
#include <rtt/Toolkit.hpp>
<% if has_opaques? %>
#include "<%= component.name %>ToolkitIntermediates.hpp"
<% end %>
<% toolkit.used_toolkits.each do |tk| %>
#include <toolkit/<%= tk.name %>ToolkitImpl.hpp>
<% end %>

#include <typelib/value_ops.hh>
#include <typelib/pluginmanager.hh>
#include <utilmm/configfile/pkgconfig.hh>
using RTT::DataSourceBase;

using RTT::Property;

// Sanity check on Typelib handling: check that typelib's computed sizes match
// the actual type sizes
<% toolkit.self_types.find_all { |t| !t.contains_opaques? }.each do |type| %>
BOOST_STATIC_ASSERT(sizeof(<%= type.cxx_name %>) == <%= type.size %>);
<% end %>

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

<% array_types.each do |type| %>
bool orogen_toolkits::toPropertyBag(std::string const& basename, <%= type.arg_type %> value, int length, RTT::PropertyBag& target_bag)
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

bool orogen_toolkits::fromPropertyBag(std::string const& basename, <%= type.ref_type %> value, int length, RTT::PropertyBag const& bag)
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
bool orogen_toolkits::toStream(std::string const& basename, <%= type.arg_type %> value, int length, std::ostream& io)
{
<%= result = ""
        type.to_stream(toolkit, result, " " * 4)
        result %>
    return true;
}
bool orogen_toolkits::fromStream(std::string const& basename, <%= type.ref_type %> value, int length, std::istream& io)
{
    return false;
}
<% end %>

<% opaques.each do |opdef|
    type = opdef.type
    intermediate_type = component.find_type(opdef.intermediate)
    %>
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

template<typename T>
struct BufferGetter : public RTT::detail::TypeTransporter
{
    Typelib::MemoryLayout layout;
    BufferGetter(std::string const& name, Typelib::Registry const& registry)
    {
        layout = Typelib::layout_of(*registry.get(name), false, false);
    }

    void* createBlob(RTT::DataSourceBase::shared_ptr data) const
    {
        std::vector<uint8_t>* buffer = new std::vector<uint8_t>;
        return reuseBlob(buffer, data);
    }

    void* reuseBlob(void* blob, RTT::DataSourceBase::shared_ptr data) const
    {
        typename RTT::DataSource<T>::shared_ptr obj = boost::dynamic_pointer_cast< RTT::DataSource<T> >(data);
        T sample = obj->get();

        std::vector<uint8_t>* buffer = reinterpret_cast< std::vector<uint8_t> *>(blob);
        buffer->clear();
        Typelib::dump(reinterpret_cast<uint8_t*>(&sample), *buffer, layout);
        return buffer;
    }

    bool updateBlob(const void* blob, DataSourceBase::shared_ptr target) const { return false; }
    DataSourceBase* proxy(void* data ) const { return 0; }
    void* server(DataSourceBase::shared_ptr source, bool assignable, void* arg) const { return 0; }
    void* method(DataSourceBase::shared_ptr source, RTT::MethodC* orig, void* arg) const { return 0; }
    DataSourceBase* dataProxy( RTT::PortInterface* data ) const { return 0; }
    DataSourceBase* dataProxy( void* data ) const { return 0; }
    void* dataServer( DataSourceBase::shared_ptr source, void* arg) const { return 0; }
    RTT::BufferBase* bufferProxy( RTT::PortInterface* data ) const { return 0; }
    RTT::BufferBase* bufferProxy( void* data ) const { return 0; }
    void* bufferServer( RTT::BufferBase::shared_ptr source, void* arg) const { return 0; }
    DataSourceBase* narrowDataSource(DataSourceBase* dsb) { return 0; }
    DataSourceBase* narrowAssignableDataSource(DataSourceBase* dsb) { return 0; }
};

<% registered_types.find_all { |t| t.contains_opaques? }.each do |type| %>

template<>
struct BufferGetter< <%= type.cxx_name %> > : public RTT::detail::TypeTransporter
{

<% if type.opaque?
    spec = toolkit.opaque_specification(type)
    intermediate = component.find_type(spec.intermediate) %>

    Typelib::MemoryLayout layout_<%= intermediate.method_name %>;
    BufferGetter(std::string const& name, Typelib::Registry const& registry)
    {
        layout_<%= intermediate.method_name %> = Typelib::layout_of(*registry.get("<%= intermediate.name %>"), false, false);
    }

    void* createBlob(RTT::DataSourceBase::shared_ptr data) const
    {
        std::vector<uint8_t>* buffer = new std::vector<uint8_t>;
        return reuseBlob(buffer, data);
    }

    void* reuseBlob(void* blob, RTT::DataSourceBase::shared_ptr data) const
    {
        RTT::DataSource< <%= type.cxx_name %> >::shared_ptr obj = boost::dynamic_pointer_cast< RTT::DataSource< <%= type.cxx_name %> > >(data);
        <%= type.cxx_name %> sample = obj->get();

        std::vector<uint8_t>* buffer = reinterpret_cast< std::vector<uint8_t> *>(blob);
        buffer->clear();
        <% if opaque_specification(type).needs_copy? %>
        <%= intermediate.cxx_name %> temp;
        <%= component.name %>::to_intermediate(temp, sample);
        <% else %>
        <%= intermediate.cxx_name %> const& temp = <%= component.name %>::to_intermediate(sample);
        <% end %>
        Typelib::dump(reinterpret_cast<uint8_t const*>(&temp), *buffer, layout_<%= intermediate.method_name %>);

        return buffer;
    }

<% else %>

    Typelib::MemoryLayout layout;

    BufferGetter(std::string const& name, Typelib::Registry const& registry)
    {
        layout = Typelib::layout_of(*registry.get("<%= type.name %>_m"), false, true);
    }

    void* createBlob(RTT::DataSourceBase::shared_ptr data) const
    {
        std::vector<uint8_t>* buffer = new std::vector<uint8_t>;
        return reuseBlob(buffer, data);
    }

    void* reuseBlob(void* blob, RTT::DataSourceBase::shared_ptr data) const
    {
        RTT::DataSource< <%= type.cxx_name %> >::shared_ptr obj = boost::dynamic_pointer_cast< RTT::DataSource< <%= type.cxx_name %> > >(data);
        <%= type.cxx_name %> sample = obj->get();

        <%= type.cxx_name %>_m temp;
        <% type.each_field do |field_name, field_type| %>
            <% if field_type.opaque? %>
                <% if opaque_specification(field_type).needs_copy? %>
                <%= component.name %>::to_intermediate(temp.<%= field_name %>, sample.<%= field_name %>);
                <% else %>
                temp.<%= field_name %> = <%= component.name %>::to_intermediate(sample.<%= field_name %>);
                <% end %>
            <% else %>
                temp.<%= field_name %> = sample.<%= field_name %>;
            <% end %>
        <% end %>

        std::vector<uint8_t>* buffer = reinterpret_cast< std::vector<uint8_t> *>(blob);
        buffer->clear();
        Typelib::dump(reinterpret_cast<uint8_t*>(&temp), *buffer, layout);

        return buffer;
    }

<% end %>

    bool updateBlob(const void* blob, DataSourceBase::shared_ptr target) const { return false; }
    DataSourceBase* proxy(void* data ) const { return 0; }
    void* server(DataSourceBase::shared_ptr source, bool assignable, void* arg) const { return 0; }
    void* method(DataSourceBase::shared_ptr source, RTT::MethodC* orig, void* arg) const { return 0; }
    DataSourceBase* dataProxy( RTT::PortInterface* data ) const { return 0; }
    DataSourceBase* dataProxy( void* data ) const { return 0; }
    void* dataServer( DataSourceBase::shared_ptr source, void* arg) const { return 0; }
    RTT::BufferBase* bufferProxy( RTT::PortInterface* data ) const { return 0; }
    RTT::BufferBase* bufferProxy( void* data ) const { return 0; }
    void* bufferServer( RTT::BufferBase::shared_ptr source, void* arg) const { return 0; }
    DataSourceBase* narrowDataSource(DataSourceBase* dsb) { return 0; }
    DataSourceBase* narrowAssignableDataSource(DataSourceBase* dsb) { return 0; }
};

<% end %>

static const int ORO_TYPELIB_MARSHALLER_ID = 42;

orogen_toolkits::<%= component.name %>ToolkitPlugin::<%= component.name %>ToolkitPlugin()
    : m_registry() {}

#define TOOLKIT_PACKAGE_NAME_aux0(target) #target
#define TOOLKIT_PACKAGE_NAME_aux(target) "<%= component.name %>-toolkit-" TOOLKIT_PACKAGE_NAME_aux0(target)
#define TOOLKIT_PACKAGE_NAME TOOLKIT_PACKAGE_NAME_aux(OROCOS_TARGET)
bool orogen_toolkits::<%= component.name %>ToolkitPlugin::loadTypes()
{
    RTT::TypeInfoRepository::shared_ptr ti_repository = RTT::TypeInfoRepository::Instance();

    try {
        utilmm::pkgconfig pkg(TOOLKIT_PACKAGE_NAME);
        std::string tlb = pkg.get("type_registry");
        m_registry = Typelib::PluginManager::load("tlb", tlb);
    }
    catch(utilmm::not_found)
    {
        std::cerr << "cannot find the pkg-config specification associated with this toolkit:" << std::endl;
        std::cerr << "  " << TOOLKIT_PACKAGE_NAME << std::endl;
        std::cerr << "this is required to use the toolkit. Aborting" << std::endl;
        return false;
    }

    RTT::TypeInfo* ti = 0;
    <% registered_types.each do |type| %>
    ti = new <%= type.method_name(true) %>TypeInfo();
    try
    { ti->addProtocol(ORO_TYPELIB_MARSHALLER_ID, new BufferGetter< <%= type.cxx_name %> >("<%= type.name %>", *m_registry)); }
    catch(Typelib::NoLayout)
    { std::cerr << "could not register a typelib marshaller for <%= type.name %>" << std::endl; }
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

// Finally, explicitely instanciate the main interface classes
<% registered_types.each do |type| %>
template class RTT::OutputPort< <%= type.cxx_name %> >;
template class RTT::InputPort< <%= type.cxx_name %> >;
template class RTT::Property< <%= type.cxx_name %> >;
template class RTT::Attribute< <%= type.cxx_name %> >;
template class RTT::DataSource< <%= type.cxx_name %> >;
template class RTT::ValueDataSource< <%= type.cxx_name %> >;
template class RTT::ConstantDataSource< <%= type.cxx_name %> >;
template class RTT::AssignableDataSource< <%= type.cxx_name %> >;
template class RTT::ReferenceDataSource< <%= type.cxx_name %> >;
template class RTT::detail::DataSourceAdaptor< <%= type.cxx_name %> const, <%= type.cxx_name %> >;
template class RTT::detail::DataSourceAdaptor< <%= type.cxx_name %> const &, <%= type.cxx_name %> >;
template class RTT::detail::DataSourceAdaptor< <%= type.cxx_name %>&, <%= type.cxx_name %> >;
<% end %>
