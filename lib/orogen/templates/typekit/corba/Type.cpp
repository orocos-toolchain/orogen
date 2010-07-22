#include "Types.hpp"
#include "transports/corba/Registration.hpp"
#include "transports/corba/Convertions.hpp"
#include <rtt/transports/corba/CorbaTemplateProtocol.hpp>

namespace orogen_typekits {
    struct <%= type.method_name %>CorbaMarshaller
        : public RTT::corba::CorbaTemplateProtocol< <%= type.cxx_name %> >
    {
        <% if type.opaque?  %>
        typedef <%= intermediate_type.corba_name(true) %> CorbaType;
        <% else %>
        typedef <%= type.corba_name(true) %> CorbaType;
        <% end %>
        typedef <%= type.cxx_name %>   BaseType;

        virtual CORBA::Any_ptr createAny( RTT::base::DataSourceBase::shared_ptr source) const;
        virtual bool updateAny(RTT::base::DataSourceBase::shared_ptr target, CORBA::Any& blob) const;
    };
}

CORBA::Any* orogen_typekits::<%= type.method_name %>CorbaMarshaller::createAny( RTT::base::DataSourceBase::shared_ptr source) const
{
    RTT::internal::DataSource<BaseType>* d = RTT::internal::DataSource<BaseType>::narrow( source.get() );
    if ( d )
    {
        <% if type.inlines_code? || type <= Typelib::EnumType %>
        CorbaType corba;
        if (!toCORBA(corba, d->value()))
            return false;
        CORBA::Any_ptr ret = new CORBA::Any();
        *ret <<= corba;
        <% else %>
        std::auto_ptr<CorbaType> corba( new CorbaType );
        if (!toCORBA(*corba, d->value()))
            return false;
        CORBA::Any_ptr ret = new CORBA::Any();
        *ret <<= corba.release();
        <% end %>
        return ret;
    }
    return 0;
}

bool orogen_typekits::<%= type.method_name %>CorbaMarshaller::updateAny(RTT::base::DataSourceBase::shared_ptr target, CORBA::Any& any) const
{
    RTT::internal::AssignableDataSource<BaseType>::shared_ptr ad = RTT::internal::AssignableDataSource<BaseType>::narrow( target.get() );
    if ( ad )
    {
        <% if type.inlines_code? || type <= Typelib::EnumType %>
        CorbaType  corba;
        if (!(any >>= corba))
            return false;
        return fromCORBA(ad->set(), corba);
        <% else %>
        CorbaType*  corba;
        if (!(any >>= corba))
            return false;
        bool ret = fromCORBA(ad->set(), *corba);
        return ret;
        <% end %>
    }
    return false;
}

namespace orogen_typekits {
    RTT::corba::CorbaTypeTransporter*  <%= type.method_name %>_CorbaTransport()
    {
        return new <%= type.method_name %>CorbaMarshaller();
    }
}

