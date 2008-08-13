#include <rtt/Types.hpp>
#include <rtt/TemplateTypeInfo.hpp>
#include <rtt/PropertyBag.hpp>
#include <rtt/Toolkit.hpp>
#include <rtt/TemplateTypeInfo.hpp>
#include "<%= name %>Toolkit.hpp"
#include "<%= name %>ToolkitTypes.hpp"
<% if corba_enabled? %>
#include <rtt/corba/CorbaTemplateProtocol.hpp>
#include "<%= name %>ToolkitCorba.hpp"
<% end %>

using RTT::Property;
using RTT::PropertyBag;
using RTT::TypeInfoRepository;
using RTT::DataSourceBase;

<% generated_types.each do |type| 
      code = Generation.adapt_namespace(namespace, type.namespace)
      namespace = type.namespace %>
<%= code %>
<%= Orocos::Generation.render_template 'toolkit/type_info.cpp', binding %>
<% end %>
<%= Generation.adapt_namespace(namespace, '/') %>

namespace <%= name %> {
    template<typename T>
    class BufferGetter : public RTT::detail::TypeTransporter
    {
        void* createBlob(RTT::DataSourceBase::shared_ptr data) const
        {
            typename RTT::DataSource<T>::shared_ptr obj = boost::dynamic_pointer_cast< RTT::DataSource<T> >(data);
            T sample = obj->get();
            uint8_t* buffer = new uint8_t[sizeof(T)];
            memcpy(buffer, &sample, sizeof(T));
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

    std::string ToolkitPlugin::getName() { return "<%= name %>"; }
    bool ToolkitPlugin::loadTypes()
    {
	TypeInfoRepository::shared_ptr ti_repository = TypeInfoRepository::Instance();
	RTT::TypeInfo* ti;

	<% generated_types.each do |type| %>ti = new <%= type.cxx_name %>TypeInfo();
	<% if corba_enabled? %>ti->addProtocol(ORO_CORBA_PROTOCOL_ID, new RTT::detail::CorbaTemplateProtocol< <%= type.cxx_name %> >());<% end %>
        ti->addProtocol(ORO_UNTYPED_PROTOCOL_ID, new BufferGetter< <%= type.cxx_name %> >);
	ti_repository->addType( ti );<% end %>
	return true;
    }

    bool ToolkitPlugin::loadOperators() { return true; }
    bool ToolkitPlugin::loadConstructors() { return true; }

    ToolkitPlugin Toolkit;
}
