#include <<%= typekit.name %>TypekitTypes.hpp>
#include <rtt/types/StructTypeInfo.hpp>
#include <rtt/types/TypeInfoRepository.hpp>
#include <<%= typekit.name %>TypekitIntermediates.hpp>

<% if intermediate_type.respond_to?(:to_boost_serialization) %>
namespace boost
{
    namespace serialization
    {
        template<typename Archive>
        void serialize(Archive& a, <%= intermediate_type.cxx_name %>& value, unsigned int version);

        template<typename Archive>
        void serialize(Archive& a, <%= type.cxx_name %>& value, unsigned int version)
        {
            using boost::serialization::make_nvp;
            <%= typekit.code_toIntermediate(intermediate_type, opdef.needs_copy?, "    ") %>
            serialize(a, intermediate, version);
        }
    }
}

namespace orogen_typekits {
    struct <%= type.method_name(true) %>TypeInfo :
	public RTT::types::StructTypeInfo< <%= type.cxx_name %> >
    {
        <%= type.method_name(true) %>TypeInfo()
            : RTT::types::StructTypeInfo< <%= type.cxx_name %> >("<%= type.full_name %>") {}
    };

    RTT::types::TypeInfo* <%= type.method_name(true) %>_TypeInfo()
    { return new <%= type.method_name(true) %>TypeInfo(); }
}
<% else %>
namespace orogen_typekits {
    struct <%= type.method_name(true) %>TypeInfo :
	public RTT::types::TemplateTypeInfo< <%= type.cxx_name %> >
    {
        typedef RTT::types::TemplateTypeInfo< <%= intermediate_type.cxx_name %> > IntermediateTypeInfo;
        IntermediateTypeInfo* intermediate_type_info;

        typedef <%= type.cxx_name %> T;

        <%= type.method_name(true) %>TypeInfo()
            : RTT::types::TemplateTypeInfo< <%= type.cxx_name %> >("<%= type.full_name %>")
        {
            RTT::types::TypeInfoRepository::shared_ptr ti = RTT::types::TypeInfoRepository::Instance();
            intermediate_type_info = dynamic_cast< IntermediateTypeInfo* >(ti->type("<%= intermediate_type.name %>"));
        }

        virtual bool composeTypeImpl(const RTT::PropertyBag& source, RTT::internal::AssignableDataSource<T>::reference_t value) const
        {
            std::auto_ptr< <%= intermediate_type.cxx_name %> > intermediate(new <%= intermediate_type.cxx_name %>);
            if (!intermediate_type_info->composeTypeImpl(source, *intermediate))
                return false;

            <%= typekit.code_fromIntermediate(intermediate_type, opdef.needs_copy?, " " * 8) %>
            return true;
        }

        virtual RTT::base::DataSourceBase::shared_ptr getMember(RTT::base::DataSourceBase::shared_ptr item, const std::string& name) const {
            RTT::internal::AssignableDataSource<T>::shared_ptr value_source =
                boost::dynamic_pointer_cast< RTT::internal::AssignableDataSource<T> >( item );
            if ( !value_source ) {
                log(RTT::Error) << "TypeInfo of type "<< this->getTypeName() <<" can't handle (non-assignable) types of type "<< item->getTypeInfo() <<RTT::endlog();
                return RTT::base::DataSourceBase::shared_ptr();
            }

            <%= type.cxx_name %> const& value = (*value_source).get();
            <%= typekit.code_toIntermediate(intermediate_type, opdef.needs_copy?, " " * 8) %>

            typedef RTT::internal::ValueDataSource< <%= intermediate_type.cxx_name %> > IntermediateSource;
            IntermediateSource::shared_ptr intermediate_ptr =
               new IntermediateSource(intermediate);
            return intermediate_type_info->getMember(intermediate_ptr, name);
        }

        virtual RTT::base::DataSourceBase::shared_ptr getMember(RTT::base::DataSourceBase::shared_ptr item,
                RTT::base::DataSourceBase::shared_ptr id) const {

            RTT::internal::AssignableDataSource<T>::shared_ptr value_source =
                boost::dynamic_pointer_cast< RTT::internal::AssignableDataSource<T> >( item );
            if ( !value_source ) {
                log(RTT::Error) << "TypeInfo of type "<< this->getTypeName() <<" can't handle (non-assignable) types of type "<< item->getTypeInfo() <<RTT::endlog();
                return RTT::base::DataSourceBase::shared_ptr();
            }

            <%= type.cxx_name %> const& value = (*value_source).get();
            <%= typekit.code_toIntermediate(intermediate_type, opdef.needs_copy?, " " * 8) %>
            typedef RTT::internal::ValueDataSource< <%= intermediate_type.cxx_name %> > IntermediateSource;
            IntermediateSource::shared_ptr intermediate_ptr =
               new IntermediateSource(intermediate);
            return intermediate_type_info->getMember(intermediate_ptr, id);
        }
    };

    RTT::types::TypeInfo* <%= type.method_name(true) %>_TypeInfo()
    { return new <%= type.method_name(true) %>TypeInfo(); }
}
<% end %>

