module OroGen
    module TypekitMarshallers
    module MQueue
    class Plugin
        attr_reader :typekit
        def initialize(typekit)
            @typekit = typekit
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

        def dependencies
            result = []
            typekit.used_typekits.each do |tk|
                next if tk.virtual?
                begin
                    build_dep = Gen::RTT_CPP::BuildDependency.new(
                        tk.name.upcase + "_TRANSPORT_MQUEUE",
                        tk.pkg_transport_name('mqueue'))
                    build_dep.in_context('mqueue', 'include')
                    build_dep.in_context('mqueue', 'link')
                    result << build_dep
                rescue Utilrb::PkgConfig::NotFound => e
                    raise Gen::RTT_CPP::ConfigError, "the MQueue transport for the #{tk.name} typekit cannot be found. It is needed to build the MQueue transport for this project"
                end
            end
	    typekit.used_libraries.each do |pkg|
		needs_link = typekit.linked_used_libraries.include?(pkg)
		result << Gen::RTT_CPP::BuildDependency.new(pkg.name.upcase, pkg.name).
		    in_context('mqueue', 'include')
		if needs_link
		    result.last.in_context('mqueue', 'link')
		end
	    end
            result
        end

        def separate_cmake?; true end

        def self.plugin_name(typekit_name)
            "orogen_typekits::#{typekit_name}MQueueTransportPlugin"
        end

        def generate(typesets)
            headers, impl = [], []
            
            code  = Gen::RTT_CPP.render_template "typekit", "mqueue", "TransportPlugin.hpp", binding
            headers << typekit.save_automatic("transports", "mqueue",
                    "TransportPlugin.hpp", code)
            code  = Gen::RTT_CPP.render_template "typekit", "mqueue", "TransportPlugin.cpp", binding
            impl << typekit.save_automatic("transports", "mqueue",
                    "TransportPlugin.cpp", code)

            code_snippets = typesets.interface_types.map do |type|
                code  = Gen::RTT_CPP.render_template "typekit", "mqueue", "Type.cpp", binding
                [type, code]
            end
            impl += typekit.render_typeinfo_snippets(code_snippets, "transports", "mqueue")

            code  = Gen::RTT_CPP.render_template "typekit", "mqueue", "Registration.hpp", binding
            typekit.save_automatic("transports", "mqueue", "Registration.hpp", code)

            impl = impl.map do |path|
                typekit.cmake_relative_path(path, "transports", "mqueue")
            end
            headers = headers.map do |path|
                typekit.cmake_relative_path(path, "transports", "mqueue")
            end

            pkg_config = Gen::RTT_CPP.render_template 'typekit', "mqueue", "transport-mqueue.pc", binding
            typekit.save_automatic("transports", "mqueue", "#{typekit.name}-transport-mqueue.pc.in", pkg_config)
            code = Gen::RTT_CPP.render_template "typekit", "mqueue", "CMakeLists.txt", binding
            typekit.save_automatic("transports", "mqueue", "CMakeLists.txt", code)
            cmake_build = Gen::RTT_CPP.render_template "typekit", "mqueue", "build.cmake", binding
            typekit.save_automatic("transports", "mqueue", "build.cmake", cmake_build)

            # We generate our own CMake code, no need to export anything to the
            # main typekit code
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

    end
    end
end

