#include <rtt/Types.hpp>
#include <rtt/TemplateTypeInfo.hpp>
#include <rtt/PropertyBag.hpp>
#include <rtt/Toolkit.hpp>
#include <rtt/TemplateTypeInfo.hpp>
#include "<%= component.name %>Toolkit.hpp"
#include "<%= component.name %>ToolkitTypes.hpp"
#include <typelib/value_ops.hh>
#include <typelib/pluginmanager.hh>
#include <utilmm/configfile/pkgconfig.hh>
<% if corba_enabled? %>
#include <rtt/corba/CorbaTemplateProtocol.hpp>
#include "<%= component.name %>ToolkitCorba.hpp"
<% end %>
<% if !opaques.empty? %>
#include "<%= component.name %>ToolkitUser.hpp"
<% end %>

#include <rtt/Ports.hpp>
#include <rtt/Attribute.hpp>
#include <rtt/Property.hpp>

using RTT::Property;
using RTT::PropertyBag;
using RTT::TypeInfoRepository;
using RTT::DataSourceBase;

<% opaques.each do |opaque_def|
    type = component.find_type(opaque_def.intermediate)
    code = Generation.adapt_namespace(namespace, type.namespace)
    namespace = type.namespace %>
<%= code %>
<%= Orocos::Generation.render_template 'toolkit/type_info.cpp', binding %>
<% end %>

<% opaques.each do |opaque_def|
    type = opaque_def.type
    intermediate_type = component.find_type(opaque_def.intermediate)
    code = Generation.adapt_namespace(namespace, type.namespace)
    namespace = type.namespace %>
<%= code %>
<%= Orocos::Generation.render_template 'toolkit/user_type_info.cpp', binding %>
<% end %>

<% generated_types.each do |type| 
      next if toolkit.intermediate_type?(type)
      code = Generation.adapt_namespace(namespace, type.namespace)
      namespace = type.namespace %>
<%= code %>
<%= Orocos::Generation.render_template 'toolkit/type_info.cpp', binding %>
<% end %>

<%= Generation.adapt_namespace(namespace, '/') %>

<% generated_types.each do |type|
    next if toolkit.intermediate_type?(type) %>
    template class RTT::OutputPort<<%= type.cxx_name %>>;
    template class RTT::InputPort<<%= type.cxx_name %>>;
    template class RTT::Property<<%= type.cxx_name %>>;
    template class RTT::Attribute<<%= type.cxx_name %>>;
<% end %>

namespace <%= component.name %> {
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
            typename RTT::DataSource<T>::shared_ptr obj = boost::dynamic_pointer_cast< RTT::DataSource<T> >(data);
            T sample = obj->get();

            std::vector<uint8_t>* buffer = new std::vector<uint8_t>;
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

<% (generated_types + opaques.map { |opdef| opdef.type }).each do |type|
    if type < Typelib::CompoundType && !type.opaque?
        opaque_map = build_opaque_map(type)
        next if opaque_map.empty?
    end %>
    <%= Orocos::Generation.render_template 'toolkit/user_type_marshaller.cpp', binding %>
<% end %>
}


namespace <%= component.name %> {
#define TOOLKIT_PACKAGE_NAME_aux0(target) #target
#define TOOLKIT_PACKAGE_NAME_aux(target) "<%= component.name %>-toolkit-" TOOLKIT_PACKAGE_NAME_aux0(target)
#define TOOLKIT_PACKAGE_NAME TOOLKIT_PACKAGE_NAME_aux(OROCOS_TARGET)

    ToolkitPlugin::ToolkitPlugin()
        : m_registry(0) {}
    ToolkitPlugin::~ToolkitPlugin()
    {
        delete m_registry;
    }

    std::string ToolkitPlugin::getName() { return "<%= component.name %>"; }
    bool ToolkitPlugin::loadTypes()
    {
	TypeInfoRepository::shared_ptr ti_repository = TypeInfoRepository::Instance();

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
	<% (generated_types | opaques.map { |d| d.type }).each do |type| %>
            ti = new <%= type.cxx_namespace %><%= type.method_name(false) %>TypeInfo();
            <% if corba_enabled? %>
                ti->addProtocol(ORO_CORBA_PROTOCOL_ID, new RTT::detail::CorbaTemplateProtocol< <%= type.cxx_name %> >());
            <% end %>

            try
            {
                ti->addProtocol(ORO_UNTYPED_PROTOCOL_ID, new BufferGetter< <%= type.cxx_name %> >("<%= type.name %>", *m_registry));
            }
            catch(Typelib::NoLayout) {}

            ti_repository->addType( ti );
        <% end %>
	return true;
    }

    bool ToolkitPlugin::loadOperators() { return true; }
    bool ToolkitPlugin::loadConstructors() { return true; }

    ToolkitPlugin Toolkit;
}

// Export the toolkit as a deployer-compatible plugin
ORO_TOOLKIT_PLUGIN(<%= component.name %>::Toolkit);

