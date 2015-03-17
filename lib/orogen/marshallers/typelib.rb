module OroGen
    module TypekitMarshallers
        module TypelibMarshaller
            class Plugin
                def self.name; "typelib" end
                def name; "typelib" end

                attr_reader :typekit
                def initialize(typekit)
                    @typekit = typekit
                end

                def dependencies
		    result = []
		    typekit.used_libraries.each do |pkg|
			needs_link = typekit.linked_used_libraries.include?(pkg)
			result << Gen::RTT_CPP::BuildDependency.new(pkg.name.upcase, pkg.name).
			    in_context('typelib', 'include')
			if needs_link
			    result.last.in_context('typelib', 'link')
			end
		    end
                    result
                end

                def separate_cmake?; true end

                def generate(typesets)
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
                            code  = Gen::RTT_CPP.render_template "typekit", "typelib", "OpaqueType.cpp", binding
                        else
                            code  = Gen::RTT_CPP.render_template "typekit", "typelib", "Type.cpp", binding
                        end
                        [type, code]
                    end
                    impl += typekit.render_typeinfo_snippets(code_snippets, "transports", "typelib")

                    code = Gen::RTT_CPP.render_template "typekit", "typelib", "Registration.hpp", binding
                    headers << typekit.save_automatic("transports", "typelib", "Registration.hpp", code)
                    code = Gen::RTT_CPP.render_template "typekit", "typelib", "TransportPlugin.hpp", binding
                    headers << typekit.save_automatic("transports", "typelib", "TransportPlugin.hpp", code)
                    code = Gen::RTT_CPP.render_template "typekit", "typelib", "TransportPlugin.cpp", binding
                    impl << typekit.save_automatic("transports", "typelib", "TransportPlugin.cpp", code)
                    pkg_config = Gen::RTT_CPP.render_template "typekit", "typelib", "transport-typelib.pc", binding
                    typekit.save_automatic("transports", "typelib", "#{typekit.name}-transport-typelib.pc.in", pkg_config)
                    code = Gen::RTT_CPP.render_template "typekit", "typelib", "CMakeLists.txt", binding
                    typekit.save_automatic("transports", "typelib", "CMakeLists.txt", code)
                    cmake_build = Gen::RTT_CPP.render_template "typekit", "typelib", "build.cmake", binding
                    typekit.save_automatic("transports", "typelib", "build.cmake", cmake_build)

                    return [], []
                end
            end
        end
    end
end

