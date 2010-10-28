/* Generated from orogen/lib/orogen/templates/typekit/corba/Type.cpp */

#include "Types.hpp"
#include "transports/corba/Registration.hpp"
#include "transports/corba/Convertions.hpp"
#include <rtt/transports/corba/CorbaTemplateProtocol.hpp>

namespace RTT
{
    namespace corba
    {
        template<>
        struct AnyConversion< <%= type.cxx_name %> >
        {
            <% if type.opaque?  %>
            typedef <%= intermediate_type.corba_name %> CorbaType;
            <% else %>
            typedef <%= type.corba_name %> CorbaType;
            <% end %>
            typedef <%= type.cxx_name %>   BaseType;

            static bool update(const CORBA::Any& any, BaseType& tp)
            {
                <% if type.inlines_code? || type <= Typelib::EnumType %>
                CorbaType  corba;
                if (!(any >>= corba))
                    return false;
                return orogen_typekits::fromCORBA(tp, corba);
                <% else %>
                CorbaType*  corba;
                if (!(any >>= corba))
                    return false;
                bool ret = orogen_typekits::fromCORBA(tp, *corba);
                return ret;
                <% end %>
            }

            static CORBA::Any_ptr createAny( BaseType const& tp )
            {
                std::auto_ptr< CORBA::Any > ret( new CORBA::Any() );
                if (!updateAny(tp, *ret))
                    return 0;
                return ret.release();
            }

            static bool updateAny( BaseType const& value, CORBA::Any& any )
            {
                <% if type.inlines_code? || type <= Typelib::EnumType %>
                CorbaType corba;
                if (!orogen_typekits::toCORBA(corba, value))
                    return false;
                any <<= corba;
                <% else %>
                std::auto_ptr<CorbaType> corba( new CorbaType );
                if (!orogen_typekits::toCORBA(*corba, value))
                    return false;
                any <<= corba.release();
                <% end %>
                return true;
            }
        };
    }
}

namespace orogen_typekits {
    RTT::corba::CorbaTypeTransporter*  <%= type.method_name %>_CorbaTransport()
    {
        return new RTT::corba::CorbaTemplateProtocol< <%= type.cxx_name %> >();
    }
}

