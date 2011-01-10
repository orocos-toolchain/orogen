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

                    code_snippets = typesets.interface_types.map do |type|
                        if type.contains_opaques?
                            needs_copy =
                                if type.opaque?
                                    typekit.opaque_specification(type).needs_copy?
                                else true
                                end

                            intermediate = typekit.intermediate_type_for(type)
                            code  = Generation.render_template "typekit", "typelib", "OpaqueType.cpp", binding
                        else
                            code  = Generation.render_template "typekit", "typelib", "Type.cpp", binding
                        end
                        [type, code]
                    end
                    impl += typekit.render_typeinfo_snippets(code_snippets, "transports", "typelib")

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
                    pkg_config = Generation.render_template "typekit", "typelib", "transport-typelib.pc", binding
                    typekit.save_automatic("transports", "typelib", "#{typekit.name}-transport-typelib.pc.in", pkg_config)
                    code = Generation.render_template "typekit", "typelib", "CMakeLists.txt", binding
                    typekit.save_automatic("transports", "typelib", "CMakeLists.txt", code)

                    return [], []
                end
            end
            Orocos::Generation::Typekit.register_plugin(Plugin)
        end
    end
end

