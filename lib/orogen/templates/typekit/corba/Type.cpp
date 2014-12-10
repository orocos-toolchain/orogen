/* Generated from orogen/lib/orogen/templates/typekit/corba/Type.cpp */

#include "<%= typekit.name %>/transports/corba/<%= typekit.name %>TypesC.h"
<%= typekit.cxx_gen_includes(*typekit.include_for_type(type)) %>
#include "transports/corba/Registration.hpp"
#include <rtt/transports/corba/CorbaTemplateProtocol.hpp>

namespace orogen_typekits
{
    <%= type.to_corba_signature(typekit) %>;
    <%= type.from_corba_signature(typekit) %>;
}

namespace RTT
{
    namespace corba
    {
        template<>
        struct AnyConversion< <%= type.cxx_name %> >
        {
            typedef <%= target_type.corba_name %> CorbaType;
            typedef <%= type.cxx_name %>   BaseType;

            static bool update(const CORBA::Any& any, BaseType& tp)
            {
                <% if type.inlines_code? %>
                CorbaType  corba;
                <%= type.inline_fromAny("any", "corba", " " * 16) %>;
                <%= type.inline_fromCorba("tp", "corba", " " * 16) %>;
                return true;
                <% elsif type <= Typelib::EnumType %>
                CorbaType  corba;
                if (!(any >>= corba))
                    return false;
                return orogen_typekits::fromCORBA(tp, corba);
                <% else %>
                CorbaType*  corba;
                if (!(any >>= corba))
                    return false;
                return orogen_typekits::fromCORBA(tp, *corba);
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
                <% if type.inlines_code? %>
                CorbaType corba;
                <%= type.inline_toCorba("corba", "value", " " * 16) %>;
                <%= type.inline_toAny("any", "corba", " " * 16) %>;
                <% elsif type <= Typelib::EnumType %>
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

