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
            Typelib::OpaqueType    .extend(TypekitMarshallers::MQueue::OpaqueType)
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

            mqueue_registered_types =
                typesets.registered_types.find_all do |type|
                    if !type.mqueue_compatible?
                        STDERR.puts "WARN: #{type.name} cannot be marshalled in the MQueue transport"
                        false
                    else true
                    end
                end
            
            code  = Generation.render_template "typekit", "mqueue", "TransportPlugin.hpp", binding
            headers << typekit.save_automatic("transports", "mqueue",
                    "TransportPlugin.hpp", code)
            code  = Generation.render_template "typekit", "mqueue", "TransportPlugin.cpp", binding
            impl << typekit.save_automatic("transports", "mqueue",
                    "TransportPlugin.cpp", code)

            code_snippets = mqueue_registered_types.map do |type|
                code  = Generation.render_template "typekit", "mqueue", "Type.cpp", binding
                [type, code]
            end
            impl += typekit.render_typeinfo_snippets(code_snippets, "transports", "mqueue")

            code  = Generation.render_template "typekit", "mqueue", "Registration.hpp", binding
            typekit.save_automatic("transports", "mqueue", "Registration.hpp", code)

            pkg_config = Generation.render_template 'typekit/mqueue/transport-mqueue.pc', binding
            typekit.save_automatic("transports", "mqueue", "#{typekit.name}-transport-mqueue.pc.in", pkg_config)
            code = Generation.render_template "typekit/mqueue/CMakeLists.txt", binding
            typekit.save_automatic("transports", "mqueue", "CMakeLists.txt", code)

            return [], []
        end
    end

    module Type
        def mqueue_compatible?; false end
    end

    module NumericType
        def mqueue_compatible?; true end
    end

    module OpaqueType
    end

    module ContainerType
    end

    module EnumType
    end

    module CompoundType
        def mqueue_compatible?; boost_serialization_compatible? end
    end

    module ArrayType
    end
    Orocos::Generation::Typekit.register_plugin(Plugin)

    end
    end
end

