#ifndef OROCOS_<%= component.name.upcase %>_CORBA_HPP
#define OROCOS_<%= component.name.upcase %>_CORBA_HPP

#include "<%= component.name %>ToolkitC.h"
#include <rtt/corba/CorbaConversion.hpp>
<% if !toolkit.opaques.empty? %>
#include "<%= component.name %>ToolkitUser.hpp"
<% end %>

namespace RTT {
    namespace details {
        template<class Type>
        struct conversion_traits
        {
            typedef typename boost::call_traits<Type>::param_type  in_t;
            typedef typename boost::call_traits<Type>::reference   out_t;
        };

        /** Common implementation of some conversion methods */
        template<class Derived, class Type, class CorbaType>
        struct conversion_base
        {
            typedef Derived impl_t;
            typedef Type    value_t;
            typedef CorbaType corba_t;

            static bool updateFromAny(const CORBA::Any& any, typename conversion_traits<Type>::out_t _value) {
                CorbaType result;
                if ( any >>= result ) {
                    impl_t::get(result, _value);
                    return true;
                }
                return false;
            }
            static bool update(const CORBA::Any& any, typename conversion_traits<Type>::out_t _value) { return updateFromAny(any, _value); }

            static CORBA::Any_ptr createAny( typename conversion_traits<Type>::in_t t ) {
                CORBA::Any_ptr ret = new CORBA::Any();
                *ret <<= impl_t::toAny( t );
                return ret;
            }
        };

        template<class Type, class CorbaType = Type>
        struct PODHelper
        {
            typedef Type      value_t;
            typedef CorbaType corba_t;

            // static bool updateFromBlob(const Corba::UntypedBuffer& buffer, typename conversion_traits<Type>::out_t tp) {
            //     Logger::log(Logger::Warning) << "Initializing type " << detail::DataSourceTypeInfo<Type>::getType() << " from blob" << Logger::endl;
            //     tp = *reinterpret_cast<Type const*>(buffer.get_buffer());
            //     return true;
            // }

            // static Corba::UntypedBuffer* createBlob( typename conversion_traits<Type>::in_t tp ) {
            //     Logger::log(Logger::Warning) << "Creating blob for transmitting " << detail::DataSourceTypeInfo<Type>::getType() << Logger::endl;
            //     return new Corba::UntypedBuffer(sizeof(tp), sizeof(tp), reinterpret_cast<CORBA::Octet*>(&const_cast<Type&>(tp)));
            // }
        };

        /** Conversion template for types which have a direct equivalent in
         * CORBA.
         */
        template<class Type, class CorbaType = Type>
        struct StandardHelper : public conversion_base< StandardHelper<Type, CorbaType>, Type, CorbaType> 
        {
            static void get(Type t, CorbaType& ret) { ret = t; }
            static CorbaType toAny( Type t ) { return t; }
        };
    }

<% generated_intermediate_types = ValueSet.new
opaques.each do |opaque_def|
    type = component.find_type(opaque_def.intermediate)
    if generated_intermediate_types.include?(type)
        next
    end
    generated_intermediate_types << type
%>
<%= Orocos::Generation.render_template 'toolkit/type_corba.hpp', binding %>
<% end %>

<% opaques.each do |opaque_def|
    type = opaque_def.type
    intermediate_type = component.find_type(opaque_def.intermediate) %>
<%= Orocos::Generation.render_template 'toolkit/user_type_corba.hpp', binding %>
<% end %>

<% generated_types.each do |type|
    next if toolkit.intermediate_type?(type) %>
<%= Orocos::Generation.render_template 'toolkit/type_corba.hpp', binding %>
<% end %>
}

#endif

