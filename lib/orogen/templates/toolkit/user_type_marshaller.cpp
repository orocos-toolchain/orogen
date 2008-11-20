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
        RTT::DataSource<<%= type.cxx_name %>>::shared_ptr obj = boost::dynamic_pointer_cast< RTT::DataSource<<%= type.cxx_name %>> >(data);
        <%= type.cxx_name %> sample = obj->get();

        std::vector<uint8_t>* buffer = new std::vector<uint8_t>;
        <%= intermediate.cxx_name %> temp;
        to_intermediate(temp, sample);
        Typelib::dump(reinterpret_cast<uint8_t*>(&temp), *buffer, layout_<%= intermediate.method_name %>);

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
        RTT::DataSource<<%= type.cxx_name %>>::shared_ptr obj = boost::dynamic_pointer_cast< RTT::DataSource<<%= type.cxx_name %>> >(data);
        <%= type.cxx_name %> sample = obj->get();

        <%= type.cxx_name %>_m temp;
        <% type.each_field do |field_name, field_type| %>
            <% if field_type.opaque? %>
                to_intermediate(temp.<%= field_name %>, sample.<%= field_name %>);
            <% else %>
                temp.<%= field_name %> = sample.<%= field_name %>;
            <% end %>
        <% end %>

        std::vector<uint8_t>* buffer = new std::vector<uint8_t>;
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

