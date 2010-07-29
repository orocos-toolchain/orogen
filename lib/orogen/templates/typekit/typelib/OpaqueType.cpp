

namespace orogen_transports {
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

    void refreshOrocosSample(MarshallingHandle* handle) const
    {
        opaque_t& opaque_sample =
            *reinterpret_cast<opaque_t*>(handle->orocos_sample);
        intermediate_t const& typelib_sample =
            *reinterpret_cast<intermediate_t const*>(handle->typelib_sample);
        <% if needs_copy %>
        <%= component.name %>::from_intermediate(opaque_sample, typelib_sample);
        <% else %>
        handle->owns_typelib =
            !<%= component.name %>::from_intermediate(opaque_sample, &typelib_sample);
        <% end %>
    }

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
}

