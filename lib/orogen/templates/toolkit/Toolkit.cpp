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
#include "TypelibMarshaller.hpp"

using namespace RTT;

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

struct orogen_transports::TypelibMarshallerBase::Handle
{
    /** The TypelibMarshallerBase that created us
     */
    TypelibMarshallerBase* marshaller;
    /** Type-pruned pointer of the sample that Typelib can understand. It may be
     * the same as orocos_sample
     */
    uint8_t* typelib_sample;
    /** If true, the handle owns the memory addressed by \c typelib_sample, and
     * will therefore destroy it when it gets deleted.
     */
    bool owns_typelib;
    /** Type-pruned pointer of the sample that Orocos can understand. It may be
     * the same as typelib_sample
     */
    uint8_t* orocos_sample;
    /** If true, the handle owns the memory addressed by \c orocos_sample, and
     * will therefore destroy it when it gets deleted.
     */
    bool owns_orocos;

    void reset()
    {
        typelib_sample = 0;
        orocos_sample  = 0;
    }

    Handle(TypelibMarshallerBase* marshaller)
        : marshaller(marshaller)
        , typelib_sample(0), owns_typelib(true)
        , orocos_sample(0), owns_orocos(true) {}

    template<typename Type>
    Handle(TypelibMarshallerBase* marshaller, Type* data)
        : marshaller(marshaller)
        , typelib_sample(reinterpret_cast<uint8_t*>(data))
        , owns_typelib(true)
        , orocos_sample(typelib_sample)
        , owns_orocos(true) {}

    template<typename TypelibType, typename OrocosType>
    Handle(TypelibMarshallerBase* marshaller, TypelibType* typelib_data, OrocosType* orocos_data)
        : marshaller(marshaller)
        , typelib_sample(reinterpret_cast<uint8_t*>(typelib_data))
        , owns_typelib(true)
        , orocos_sample(reinterpret_cast<uint8_t*>(orocos_data))
        , owns_orocos(true) {}

    ~Handle()
    {
        if (owns_typelib && typelib_sample)
            marshaller->deleteTypelibSample(this);
        if (owns_orocos && orocos_sample)
            marshaller->deleteOrocosSample(this);
    }
};

typedef orogen_transports::TypelibMarshallerBase::Handle MarshallingHandle;

namespace orogen_transports
{
    TypelibMarshallerBase::TypelibMarshallerBase(std::string const& typelib_typename,
            std::string const& orocos_typename,
            Typelib::Registry const& registry)
        : m_typename_typelib(typelib_typename)
        , m_typename_orocos(orocos_typename)
    {
        Typelib::Type const* type_def = registry.get(typelib_typename);
        if (!type_def)
            throw std::runtime_error(typelib_typename + " is not present in the toolkit registry");

        layout    = Typelib::layout_of(*type_def, false, false);
    }

    MarshallingHandle* TypelibMarshallerBase::createHandle() { return new MarshallingHandle(this); }
    void TypelibMarshallerBase::deleteHandle(MarshallingHandle* handle) { delete handle; }
    uint8_t* TypelibMarshallerBase::getTypelibSample(MarshallingHandle* handle)
    {
        return handle->typelib_sample;
    }

    char const* TypelibMarshallerBase::getMarshallingType() const
    { return m_typename_typelib.c_str(); }
    size_t TypelibMarshallerBase::getMarshallingSize(MarshallingHandle const* handle) const
    { return Typelib::getDumpSize(handle->typelib_sample, layout); }
    void TypelibMarshallerBase::marshal(int fd, MarshallingHandle* handle) const
    { Typelib::dump(handle->typelib_sample, fd, layout); }
    void TypelibMarshallerBase::marshal(std::ostream& stream, MarshallingHandle* handle) const
    { Typelib::dump(handle->typelib_sample, stream, layout); }
    void TypelibMarshallerBase::marshal(std::vector<uint8_t>& buffer, MarshallingHandle* handle) const
    { Typelib::dump(handle->typelib_sample, buffer, layout); }
}

template<typename T>
struct TypelibMarshaller : public orogen_transports::TypelibMarshallerBase
{
    TypelibMarshaller(std::string const& orocos_name, Typelib::Registry const& registry)
        : orogen_transports::TypelibMarshallerBase(orocos_name, orocos_name, registry) { }

    MarshallingHandle* createSample() { return new MarshallingHandle(this, new T); }
    void deleteOrocosSample(MarshallingHandle* data)  { deleteSamples(data); }
    void deleteTypelibSample(MarshallingHandle* data) { deleteSamples(data); }
    void deleteSamples(MarshallingHandle* data)
    {
        delete reinterpret_cast<T*>(data->orocos_sample);
        data->reset();
    }

    void setTypelibSample(MarshallingHandle* handle, uint8_t* data)
    {
        handle->orocos_sample = handle->typelib_sample = reinterpret_cast<uint8_t*>(data);
        handle->owns_orocos = handle->owns_typelib = false;
    }

    bool readDataSource(RTT::DataSourceBase& source_base, MarshallingHandle* handle)
    {
        DataSource<T>& source = dynamic_cast<DataSource<T>&>(source_base);
        if (source.evaluate())
        {
            T& data = *reinterpret_cast<T*>(handle->orocos_sample);
            data = dynamic_cast<DataSource<T>&>(source_base).get();
            return true;
        }
        return false;
    }
    void writeDataSource(RTT::DataSourceBase& source, MarshallingHandle const* handle)
    {
        T const& data = *reinterpret_cast<T const*>(handle->orocos_sample);
        dynamic_cast<AssignableDataSource<T>&>(source).set(data);
    }

    bool readPort(InputPortInterface& port, MarshallingHandle* handle)
    {
        T& data = *reinterpret_cast<T*>(handle->orocos_sample);
        return dynamic_cast< InputPort<T>& >(port).read(data);
    }
    void writePort(OutputPortInterface& port, MarshallingHandle const* handle)
    {
        T const& data = *reinterpret_cast<T const*>(handle->orocos_sample);
        return dynamic_cast< OutputPort<T>& >(port).write(data);
    }
};

// Now handle opaque types
<% registered_types.find_all { |t| t.contains_opaques? }.each do |type|
    if type.opaque?
        spec = toolkit.opaque_specification(type)
        needs_copy = spec.needs_copy?
        intermediate = component.find_type(spec.intermediate)
    else
        # This is not an opaque, but has fields that are opaque themselves
        needs_copy = true
        intermediate = toolkit.find_type("#{type.name}_m")
    end
%>

template<>
struct TypelibMarshaller< <%= type.cxx_name %> > : public orogen_transports::TypelibMarshallerBase
{

    typedef <%= type.cxx_name %> opaque_t;
    typedef <%= intermediate.cxx_name %> intermediate_t;

    TypelibMarshaller(std::string const& name, Typelib::Registry const& registry)
        : orogen_transports::TypelibMarshallerBase("<%= intermediate.name %>", name, registry) { }

    MarshallingHandle* createSample()
    {
        <% if needs_copy %>
        return new MarshallingHandle(this, new intermediate_t, new opaque_t);
        <% else %>
        return new MarshallingHandle(this, (intermediate_t*)0, new opaque_t);
        <% end %>
    }

    void deleteOrocosSample(MarshallingHandle* data)
    { delete reinterpret_cast<opaque_t*>(data->orocos_sample); }
    void deleteTypelibSample(MarshallingHandle* data)
    { delete reinterpret_cast<intermediate_t*>(data->typelib_sample); }

    void setTypelibSample(MarshallingHandle* handle, uint8_t* typelib_data)
    {
        opaque_t& opaque_sample =
            *reinterpret_cast<opaque_t*>(handle->orocos_sample);
        intermediate_t const& new_value =
            *reinterpret_cast<intermediate_t const*>(typelib_data);

        <% if needs_copy %>
        <%= component.name %>::from_intermediate(opaque_sample, new_value);
        <% else %>
        if (!handle->typelib_sample)
        {
            handle->typelib_sample = reinterpret_cast<uint8_t*>(new intermediate_t);
            handle->owns_typelib = true;
        }
        intermediate_t& intermediate_sample =
            *reinterpret_cast<intermediate_t*>(handle->typelib_sample);
        intermediate_sample = new_value;
        handle->owns_typelib =
            !<%= component.name %>::from_intermediate(opaque_sample, &intermediate_sample);
        <% end %>
    }

    void updateIntermediate(MarshallingHandle* handle)
    {
        opaque_t& opaque_sample =
            *reinterpret_cast<opaque_t*>(handle->orocos_sample);

        <% if needs_copy %>
        intermediate_t& intermediate_sample =
            *reinterpret_cast<intermediate_t*>(handle->typelib_sample);
        <%= component.name %>::to_intermediate(intermediate_sample, opaque_sample);
        <% else %>
        if (handle->typelib_sample && handle->owns_typelib)
            deleteTypelibSample(handle);

        handle->typelib_sample = const_cast<uint8_t*>(
                reinterpret_cast<uint8_t const*>(
                    &<%= component.name %>::to_intermediate(opaque_sample) ));
        handle->owns_typelib = false;
        <% end %>
    }

    bool readDataSource(RTT::DataSourceBase& source_base, MarshallingHandle* handle)
    {
        DataSource<opaque_t>& source = dynamic_cast<DataSource<opaque_t>&>(source_base);
        if (source.evaluate())
        {
            opaque_t& opaque_sample =
                *reinterpret_cast<opaque_t*>(handle->orocos_sample);
            opaque_sample = source.value();
            updateIntermediate(handle);
            return true;
        }
        return false;
    }
    void writeDataSource(RTT::DataSourceBase& source, MarshallingHandle const* handle)
    {
        opaque_t const& data = *reinterpret_cast<opaque_t const*>(handle->orocos_sample);
        dynamic_cast<AssignableDataSource<opaque_t>&>(source).set(data);
    }

    bool readPort(InputPortInterface& port, MarshallingHandle* handle)
    {
        InputPort<opaque_t>& typed_port = dynamic_cast< InputPort<opaque_t>& >(port);

        opaque_t& opaque_sample =
            *reinterpret_cast<opaque_t*>(handle->orocos_sample);
        bool did_read = typed_port.read(opaque_sample);
        if (! did_read)
            return NULL;

        updateIntermediate(handle);
        return true;
    }

    void writePort(OutputPortInterface& port, MarshallingHandle const* handle)
    {
        opaque_t const& opaque_sample =
            *reinterpret_cast<opaque_t const*>(handle->orocos_sample);
        dynamic_cast< OutputPort<opaque_t>& >(port).write(opaque_sample);
    }
};
<% end %>

orogen_toolkits::<%= component.name %>ToolkitPlugin::<%= component.name %>ToolkitPlugin()
    : m_registry() {}

orogen_toolkits::<%= component.name %>ToolkitPlugin::~<%= component.name %>ToolkitPlugin()
{ delete m_registry; }


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
    { ti->addProtocol(orogen_transports::TYPELIB_MARSHALLER_ID, new TypelibMarshaller< <%= type.cxx_name %> >("<%= type.name %>", *m_registry)); }
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

