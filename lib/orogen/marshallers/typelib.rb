module Orocos
    module TypekitMarshallers
        module TypelibMarshaller
            class Plugin
                def self.name; "typelib" end
                def name; "typelib" end

                def dependencies(typekit)
                    []
                end

                def separate_cmake?; true end

                def generate(typekit, typesets)
                    impl    = []
                    headers = []

                    typesets.registered_types.each do |type|
                        if type.contains_opaques?
                            if type.opaque?
                                spec = typekit.opaque_specification(type)
                                needs_copy = spec.needs_copy?
                                intermediate = typekit.find_type(spec.intermediate)
                            else
                                # This is not an opaque, but has fields that are opaque themselves
                                needs_copy = true
                                intermediate = typekit.find_type("#{type.name}_m")
                            end

                            code  = Generation.render_template "typekit", "typelib", "OpaqueType.cpp", binding
                            impl << typekit.save_automatic("transports", "typelib", "#{type.method_name}.cpp", code)
                        else
                            code  = Generation.render_template "typekit", "typelib", "Type.cpp", binding
                            impl << typekit.save_automatic("transports", "typelib", "#{type.method_name}.cpp", code)
                        end
                    end

                    code = Generation.render_template "typekit", "typelib", "TypelibMarshallerBase.hpp", binding
                    headers << typekit.save_automatic("transports", "typelib", "TypelibMarshallerBase.hpp", code)
                    code = Generation.render_template "typekit", "typelib", "TypelibMarshallerBase.cpp", binding
                    impl << typekit.save_automatic("transports", "typelib", "TypelibMarshallerBase.cpp", code)
                    code = Generation.render_template "typekit", "typelib", "TypelibMarshaller.hpp", binding
                    headers << typekit.save_automatic("transports", "typelib", "TypelibMarshaller.hpp", code)

                    code = Generation.render_template "typekit", "typelib", "Registration.hpp", binding
                    headers << typekit.save_automatic("transports", "typelib", "Registration.hpp", code)
                    code = Generation.render_template "typekit", "typelib", "TransportPlugin.hpp", binding
                    headers << typekit.save_automatic("transports", "typelib", "TransportPlugin.hpp", code)
                    code = Generation.render_template "typekit", "typelib", "TransportPlugin.cpp", binding
                    impl << typekit.save_automatic("transports", "typelib", "TransportPlugin.cpp", code)
                    code = Generation.render_template "typekit", "typelib", "CMakeLists.txt", binding
                    typekit.save_automatic("transports", "typelib", "CMakeLists.txt", code)

                    return [], []
                end
            end
            Orocos::Generation::Typekit.register_plugin(Plugin)
        end
    end
end

