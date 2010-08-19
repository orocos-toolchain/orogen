#include "OpaqueConvertions.hpp"
#include "Types.hpp"
#include "transports/typelib/TypelibMarshaller.hpp"
#include "transports/typelib/Registration.hpp"

typedef orogen_transports::TypelibMarshallerBase::Handle MarshallingHandle;

namespace orogen_transports {

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

    void refreshOrocosSample(MarshallingHandle* handle) const
    {
        opaque_t& opaque_sample =
            *reinterpret_cast<opaque_t*>(handle->orocos_sample);
        intermediate_t& typelib_sample =
            *reinterpret_cast<intermediate_t*>(handle->typelib_sample);
        <% if needs_copy %>
        <%= typekit.name %>::from_intermediate(opaque_sample, typelib_sample);
        <% else %>
        handle->owns_typelib =
            !<%= typekit.name %>::from_intermediate(opaque_sample, &typelib_sample);
        <% end %>
    }

    void setTypelibSample(MarshallingHandle* handle, uint8_t* typelib_data)
    {
        opaque_t& opaque_sample =
            *reinterpret_cast<opaque_t*>(handle->orocos_sample);
        intermediate_t& new_value =
            *reinterpret_cast<intermediate_t*>(typelib_data);

        <% if needs_copy %>
        <%= typekit.name %>::from_intermediate(opaque_sample, new_value);
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
            !<%= typekit.name %>::from_intermediate(opaque_sample, &intermediate_sample);
        <% end %>
    }

    void updateIntermediate(MarshallingHandle* handle)
    {
        opaque_t& opaque_sample =
            *reinterpret_cast<opaque_t*>(handle->orocos_sample);

        <% if needs_copy %>
        intermediate_t& intermediate_sample =
            *reinterpret_cast<intermediate_t*>(handle->typelib_sample);
        <%= typekit.name %>::to_intermediate(intermediate_sample, opaque_sample);
        <% else %>
        if (handle->typelib_sample && handle->owns_typelib)
            deleteTypelibSample(handle);

        handle->typelib_sample = const_cast<uint8_t*>(
                reinterpret_cast<uint8_t const*>(
                    &<%= typekit.name %>::to_intermediate(opaque_sample) ));
        handle->owns_typelib = false;
        <% end %>
    }

    bool readDataSource(RTT::base::DataSourceBase& source_base, MarshallingHandle* handle)
    {
        RTT::internal::DataSource<opaque_t>& source = dynamic_cast<RTT::internal::DataSource<opaque_t>&>(source_base);
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
    void writeDataSource(RTT::base::DataSourceBase& source, MarshallingHandle const* handle)
    {
        opaque_t const& data = *reinterpret_cast<opaque_t const*>(handle->orocos_sample);
        dynamic_cast<RTT::internal::AssignableDataSource<opaque_t>&>(source).set(data);
    }

    RTT::FlowStatus readPort(RTT::base::InputPortInterface& port, MarshallingHandle* handle)
    {
        RTT::InputPort<opaque_t>& typed_port = dynamic_cast< RTT::InputPort<opaque_t>& >(port);

        opaque_t& opaque_sample =
            *reinterpret_cast<opaque_t*>(handle->orocos_sample);
        bool did_read = typed_port.read(opaque_sample);
        if (! did_read)
            return NULL;

        updateIntermediate(handle);
        return true;
    }

    void writePort(RTT::base::OutputPortInterface& port, MarshallingHandle const* handle)
    {
        opaque_t const& opaque_sample =
            *reinterpret_cast<opaque_t const*>(handle->orocos_sample);
        dynamic_cast< RTT::OutputPort<opaque_t>& >(port).write(opaque_sample);
    }

    void unmarshal(std::vector<uint8_t>& buffer, Handle* handle) const
    {
        if (!handle->typelib_sample)
        {
            handle->typelib_sample = reinterpret_cast<uint8_t*>(new intermediate_t);
            handle->owns_typelib = true;
        }
        TypelibMarshallerBase::unmarshal(buffer, handle);
    }
};
}

orogen_transports::TypelibMarshallerBase* orogen_typekits::<%= type.method_name %>_TypelibMarshaller(Typelib::Registry const& registry)
{
    return new orogen_transports::TypelibMarshaller< <%= type.cxx_name %> >("<%= type.name %>", registry);
}

