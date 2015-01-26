/* Generated from orogen/lib/orogen/templates/typekit/type_info/OpaqueInfo.hpp */

<%= typekit.cxx_gen_includes(*typekit.include_for_type(type)) %>
<%= typekit.cxx_gen_includes(*typekit.include_for_type(intermediate_type)) %>
<%= typekit.cxx_gen_includes(*typekit.type_info_includes_for_type(type)) %>
#include <<%= typekit.name %>/typekit/OpaqueConvertions.hpp>

<% base_class =
    if !TypekitMarshallers::TypeInfo::Plugin.rtt_scripting?
        ["RTT::types::PrimitiveTypeInfo< #{type.cxx_name} >", "RTT::types::TemplateConnFactory< #{type.cxx_name} >"]
    else
	["RTT::types::TemplateTypeInfo< #{type.cxx_name} >"]
    end
%>

namespace orogen_typekits {
    struct <%= type.method_name(true) %>TypeInfo :
        public <%= base_class.join(", public ") %>
    {
        mutable RTT::types::TypeInfo* intermediate_type_info;

        void getIntermediateTypeInfo() const
        {
            if (intermediate_type_info)
                return;
            
            RTT::types::TypeInfoRepository::shared_ptr ti = RTT::types::TypeInfoRepository::Instance();
            intermediate_type_info = ti->type("<%= intermediate_type.name %>");
            if (!intermediate_type_info)
                throw std::runtime_error("cannot initialize TypeInfo for opaque <%= type.full_name %> as I cannot find the TypeInfo for the intermediate type <%= intermediate_type.name %>");
        }

        typedef <%= type.cxx_name %> T;

        <%= type.method_name(true) %>TypeInfo()
            : <%= base_class.first %>("<%= type.full_name %>")
        {
        }

<% if TypekitMarshallers::TypeInfo::Plugin.rtt_scripting? %>
        virtual bool composeType(RTT::base::DataSourceBase::shared_ptr source, RTT::base::DataSourceBase::shared_ptr target) const
        {
            getIntermediateTypeInfo();

            <% if needs_copy %>
            <%= intermediate_type.cxx_name %> intermediate;
            typedef RTT::internal::ReferenceDataSource< <%= intermediate_type.cxx_name %> > IntermediateSource;
            IntermediateSource::shared_ptr intermediate_ptr =
               new IntermediateSource(intermediate);
            if (!intermediate_type_info->getCompositionFactory()->composeType(source, intermediate_ptr))
                return false;
            <% else %>
            std::auto_ptr< <%= intermediate_type.cxx_name %> > intermediate(new <%= intermediate_type.cxx_name %>);
            typedef RTT::internal::ReferenceDataSource< <%= intermediate_type.cxx_name %> > IntermediateSource;
            IntermediateSource::shared_ptr intermediate_ptr =
               new IntermediateSource(*intermediate);
            if (!intermediate_type_info->getCompositionFactory()->composeType(source, intermediate_ptr))
                return false;
            <% end %>

            RTT::internal::AssignableDataSource<T>::shared_ptr value_source =
                boost::dynamic_pointer_cast< RTT::internal::AssignableDataSource<T> >( target );
            <%= type.cxx_name %>& value = value_source->set();
            <%= typekit.code_fromIntermediate(intermediate_type, needs_copy, " " * 8) %>
            return true;
        }

        virtual RTT::base::DataSourceBase::shared_ptr getMember(RTT::base::DataSourceBase::shared_ptr item, const std::string& name) const {
            getIntermediateTypeInfo();

            RTT::internal::AssignableDataSource<T>::shared_ptr value_source =
                boost::dynamic_pointer_cast< RTT::internal::AssignableDataSource<T> >( item );
            if ( !value_source ) {
                log(RTT::Error) << "TypeInfo of type "<< this->getTypeName() <<" can't handle (non-assignable) types of type "<< item->getTypeInfo() <<RTT::endlog();
                return RTT::base::DataSourceBase::shared_ptr();
            }

            <%= type.cxx_name %> const& value = (*value_source).get();
            <%= typekit.code_toIntermediate(intermediate_type, needs_copy, " " * 8) %>

            typedef RTT::internal::ConstReferenceDataSource< <%= intermediate_type.cxx_name %> > IntermediateSource;
            IntermediateSource::shared_ptr intermediate_ptr =
               new IntermediateSource(intermediate);
            return intermediate_type_info->getMemberFactory()->getMember(intermediate_ptr, name);
        }

        virtual RTT::base::DataSourceBase::shared_ptr getMember(RTT::base::DataSourceBase::shared_ptr item,
                RTT::base::DataSourceBase::shared_ptr id) const {
            getIntermediateTypeInfo();

            RTT::internal::AssignableDataSource<T>::shared_ptr value_source =
                boost::dynamic_pointer_cast< RTT::internal::AssignableDataSource<T> >( item );
            if ( !value_source ) {
                log(RTT::Error) << "TypeInfo of type "<< this->getTypeName() <<" can't handle (non-assignable) types of type "<< item->getTypeInfo() <<RTT::endlog();
                return RTT::base::DataSourceBase::shared_ptr();
            }

            <%= type.cxx_name %> const& value = (*value_source).get();
            <%= typekit.code_toIntermediate(intermediate_type, needs_copy, " " * 8) %>
            typedef RTT::internal::ConstReferenceDataSource< <%= intermediate_type.cxx_name %> > IntermediateSource;
            IntermediateSource::shared_ptr intermediate_ptr =
               new IntermediateSource(intermediate);
            return intermediate_type_info->getMemberFactory()->getMember(intermediate_ptr, id);
        }
<% end %>

<%  if !TypekitMarshallers::TypeInfo::Plugin.rtt_scripting? %>
        bool installTypeInfoObject(RTT::types::TypeInfo* ti) {
            // This shared pointer MUST be taken HERE, and MUST be pointing to
            // the most derived class. Otherwise, you'll get double-free at
            // deinitialization time
            boost::shared_ptr< <%= type.method_name(true) %>TypeInfo > mthis =
                boost::dynamic_pointer_cast< <%= type.method_name(true) %>TypeInfo >( this->getSharedPtr() );

            // Allow base to install first
            RTT::types::PrimitiveTypeInfo< <%= type.cxx_name %> >::installTypeInfoObject(ti);
            // Install the factories for primitive types
            ti->setPortFactory(mthis);
            // don't delete us, we're memory managed
            return false;
        }
<% end %>
    };

    RTT::types::TypeInfoGenerator* <%= type.method_name(true) %>_TypeInfo()
    { return new <%= type.method_name(true) %>TypeInfo(); }
}

<%= Generation.render_template('typekit', 'TemplateInstanciation.cpp', binding) %>


