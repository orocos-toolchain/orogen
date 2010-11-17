module Orocos
    module TypekitMarshallers
    module MQueue
    class Plugin
        def initialize
            Typelib::Type          .extend(TypekitMarshallers::MQueue::Type)
            Typelib::NumericType   .extend(TypekitMarshallers::MQueue::NumericType)
            Typelib::ContainerType .extend(TypekitMarshallers::MQueue::ContainerType)
            Typelib::EnumType      .extend(TypekitMarshallers::MQueue::EnumType)
            Typelib::CompoundType  .extend(TypekitMarshallers::MQueue::CompoundType)
            Typelib::ArrayType     .extend(TypekitMarshallers::MQueue::ArrayType)
        end

        def self.name; "mqueue" end
        def name; "mqueue" end

        def dependencies(typekit)
            result = []
            typekit.used_typekits.each do |tk|
                build_dep = Orocos::Generation::BuildDependency.new(
                    tk.name.upcase + "_TRANSPORT_MQUEUE",
                    tk.pkg_transport_name('mqueue'))
                build_dep.in_context('mqueue', 'include')
                build_dep.in_context('mqueue', 'link')
                result << build_dep
            end
            result
        end

        def separate_cmake?; true end

        def plugin_name(typekit_name)
            "orogen_typekits::#{typekit_name}MQueueTransportPlugin"
        end

        def generate(typekit, typesets)
            headers, impl = [], []
            
            code  = Generation.render_template "typekit", "mqueue", "TransportPlugin.hpp", binding
            headers << typekit.save_automatic("transports", "mqueue",
                    "TransportPlugin.hpp", code)
            code  = Generation.render_template "typekit", "mqueue", "TransportPlugin.cpp", binding
            impl << typekit.save_automatic("transports", "mqueue",
                    "TransportPlugin.cpp", code)

            typesets.converted_types.each do |type|
                code  = Generation.render_template "typekit", "mqueue", "Type.cpp", binding
                impl << typekit.save_automatic("transports", "mqueue",
                        "#{type.name_as_word}.cpp", code)
            end
            typesets.opaque_types.each do |opdef|
                type = opdef.type
                intermediate_type = typekit.find_type(opdef.intermediate)
                code  = Generation.render_template "typekit", "mqueue", "Type.cpp", binding
                impl << typekit.save_automatic("transports", "mqueue",
                        "#{type.name_as_word}.cpp", code)
            end

            code  = Generation.render_template "typekit", "mqueue", "Registration.hpp", binding
            typekit.save_automatic("transports", "mqueue", "Registration.hpp", code)

            pkg_config = Generation.render_template 'typekit/mqueue/transport-mqueue.pc', binding
            typekit.save_automatic("transports", "mqueue", "#{typekit.name}-transport-mqueue.pc.in", pkg_config)
            code = Generation.render_template "typekit/mqueue/CMakeLists.txt", binding
            typekit.save_automatic("transports", "mqueue", "CMakeLists.txt", code)

            #code  = Generation.render_template "typekit/mqueue/Transport.hpp", binding
            #typekit.save_automatic("mqueue",
            #        "#{typekit.name}MQueueTransport.hpp", code)

            #mqueue_hpp = Generation.render_template "typekit/mqueue/TypekitMQueue.hpp", binding
            #impl << typekit.save_automatic("mqueue", "#{component.name}TypekitMQueue.hpp", mqueue_hpp)
            #mqueue_impl_hpp = Generation.render_template "typekit/mqueue/TypekitMQueueImpl.hpp", binding
            #impl << typekit.save_automatic("mqueue", "#{component.name}TypekitMQueueImpl.hpp", mqueue_impl_hpp)
            #mqueue_cpp = Generation.render_template "typekit/mqueue/TypekitMQueue.cpp", binding
            #impl << typekit.save_automatic("mqueue", "#{component.name}TypekitMQueue.cpp", mqueue_cpp)
            #pkg_config = Generation.render_template 'typekit/mqueue/transport-mqueue.pc', binding
            #impl << typekit.save_automatic("mqueue", "#{component.name}-transport-mqueue.pc.in", pkg_config)

            return [], []
        end
    end

    module Type
    end

    module NumericType
    end

    module ContainerType
    end

    module EnumType
    end

    module CompoundType
    end

    module ArrayType
    end
    Orocos::Generation::Typekit.register_plugin(Plugin)

    end
    end
end

