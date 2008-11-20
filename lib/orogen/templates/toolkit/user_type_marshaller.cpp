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
    <% opaque_map.each do |_, intermediate, _| %>
        Typelib::MemoryLayout layout_<%= intermediate.method_name %>;
    <% end %>

    BufferGetter(std::string const& name, Typelib::Registry const& registry)
    {
        layout = Typelib::layout_of(*registry.get("<%= type.name %>"), false, true);
        <% opaque_map.each do |_, intermediate, _| %>
            layout_<%= intermediate.method_name %> = Typelib::layout_of(*registry.get("<%= intermediate.name %>"), false, false);
        <% end %>
    }

    void* createBlob(RTT::DataSourceBase::shared_ptr data) const
    {
        RTT::DataSource<<%= type.cxx_name %>>::shared_ptr obj = boost::dynamic_pointer_cast< RTT::DataSource<<%= type.cxx_name %>> >(data);
        <%= type.cxx_name %> sample = obj->get();

        std::vector<uint8_t>* buffer = new std::vector<uint8_t>;
        Typelib::dump(reinterpret_cast<uint8_t*>(&sample), *buffer, layout);

        <% opaque_map.each do |field_type, intermediate, field_path| %>
        {
            <%= intermediate.cxx_name %> temp;
            to_intermediate(temp, sample.<%= field_path %>);
            Typelib::dump(reinterpret_cast<uint8_t*>(&temp), *buffer, layout_<%= intermediate.method_name %>);
        }
        <% end %>

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

