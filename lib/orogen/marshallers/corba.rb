module Orocos
    module TypekitMarshallers
    module Corba
    class Plugin
        def initialize
            Typelib::Type          .extend(TypekitMarshallers::Corba::Type)
            Typelib::NumericType   .extend(TypekitMarshallers::Corba::NumericType)
            Typelib::ContainerType .extend(TypekitMarshallers::Corba::ContainerType)
            Typelib::EnumType      .extend(TypekitMarshallers::Corba::EnumType)
            Typelib::CompoundType  .extend(TypekitMarshallers::Corba::CompoundType)
            Typelib::ArrayType     .extend(TypekitMarshallers::Corba::ArrayType)
        end

        def self.name; "corba" end
        def name; "corba" end

        def dependencies(typekit)
            result = []
            typekit.used_typekits.each do |tk|
                build_dep = Orocos::Generation::BuildDependency.new(
                    tk.name.upcase + "_TRANSPORT_CORBA",
                    tk.pkg_corba_name)
                build_dep.in_context('corba', 'include')
                build_dep.in_context('corba', 'link')
                result << build_dep
            end
            result
        end

        def separate_cmake?; true end

        def generate(typekit, typesets)
            headers, impl = [], []
            
            idl = Orocos::Generation.render_template "typekit", "corba", "Types.idl", binding
            idl_file = typekit.save_automatic("transports", "corba",
                "#{typekit.name}Types.idl", idl)

            code  = Generation.render_template "typekit", "corba", "Convertions.hpp", binding
            headers << typekit.save_automatic("transports", "corba",
                    "Convertions.hpp", code)
            code  = Generation.render_template "typekit", "corba", "Convertions.cpp", binding
            impl << typekit.save_automatic("transports", "corba",
                    "Convertions.cpp", code)

            code  = Generation.render_template "typekit", "corba", "TransportPlugin.hpp", binding
            headers << typekit.save_automatic("transports", "corba",
                    "TransportPlugin.hpp", code)
            code  = Generation.render_template "typekit", "corba", "TransportPlugin.cpp", binding
            impl << typekit.save_automatic("transports", "corba",
                    "TransportPlugin.cpp", code)

            typesets.converted_types.each do |type|
                code  = Generation.render_template "typekit", "corba", "Type.cpp", binding
                impl << typekit.save_automatic("transports", "corba",
                        "#{type.name_as_word}.cpp", code)
            end
            typesets.opaque_types.each do |opdef|
                type = opdef.type
                intermediate_type = typekit.find_type(opdef.intermediate)
                code  = Generation.render_template "typekit", "corba", "Type.cpp", binding
                impl << typekit.save_automatic("transports", "corba",
                        "#{type.name_as_word}.cpp", code)
            end

            code  = Generation.render_template "typekit", "corba", "Registration.hpp", binding
            typekit.save_automatic("transports", "corba", "Registration.hpp", code)

            pkg_config = Generation.render_template 'typekit/corba/transport-corba.pc', binding
            typekit.save_automatic("transports", "corba", "#{typekit.name}-transport-corba.pc.in", pkg_config)
            code = Generation.render_template "typekit/corba/CMakeLists.txt", binding
            typekit.save_automatic("transports", "corba", "CMakeLists.txt", code)

            #code  = Generation.render_template "typekit/corba/Transport.hpp", binding
            #typekit.save_automatic("corba",
            #        "#{typekit.name}CorbaTransport.hpp", code)

            #corba_hpp = Generation.render_template "typekit/corba/TypekitCorba.hpp", binding
            #impl << typekit.save_automatic("corba", "#{component.name}TypekitCorba.hpp", corba_hpp)
            #corba_impl_hpp = Generation.render_template "typekit/corba/TypekitCorbaImpl.hpp", binding
            #impl << typekit.save_automatic("corba", "#{component.name}TypekitCorbaImpl.hpp", corba_impl_hpp)
            #corba_cpp = Generation.render_template "typekit/corba/TypekitCorba.cpp", binding
            #impl << typekit.save_automatic("corba", "#{component.name}TypekitCorba.cpp", corba_cpp)
            #pkg_config = Generation.render_template 'typekit/corba/transport-corba.pc', binding
            #impl << typekit.save_automatic("corba", "#{component.name}-transport-corba.pc.in", pkg_config)

            return [], []
        end
    end

    module Type
        def corba_name(plain_name = false)
            if inlines_code?
                normalize_cxxname(basename)
            elsif contains_opaques?
                "orogen#{namespace('::')}Corba::#{normalize_cxxname(basename).gsub(/[^\w]/, '_')}_m"
            else
                "orogen#{namespace('::')}Corba::#{normalize_cxxname(basename).gsub(/[^\w]/, '_')}"
            end
        end

        def corba_arg_type; "#{corba_name} const&" end
        def corba_ref_type; "#{corba_name}&" end

        def to_corba(typekit, result, *args)
            STDERR.puts "to_corba not implemented for #{name}"
            result
        end
        def from_corba(typekit, result, *args)
            STDERR.puts "from_corba not implemented for #{name}"
            result
        end

        def inline_fromCorba(result, value, indent)
            "#{indent}#{result} = #{value};\n"
        end
        def inline_toCorba(result, value, indent)
            "#{indent}#{result} = #{value};\n"
        end
    end

    module NumericType
        def corba_name(plain_name = false)
	    if integer?
		if name == "/bool"
		    "CORBA::Boolean"
                elsif size == 1
                    "CORBA::Octet"
                elsif size == 2
                    "CORBA::#{'U' if unsigned?}Short"
                elsif size == 4
                    "CORBA::#{'U' if unsigned?}Long"
                elsif size == 8
                    "CORBA::#{'U' if unsigned?}LongLong"
                else
                    raise "unexpected integer size #{size}"
		end
	    else
                if size == 4
                    "CORBA::Float"
                elsif size == 8
                    "CORBA::Double"
                else
                    raise "unexpected floating-point size #{size}"
                end
	    end
        end
    end

    ::Typelib::specialize_model '/std/string' do
        def inlines_code?; true end
        def corba_name(plain_name = false); "char" end
        def corba_arg_type; "char const*" end
        def corba_ref_type; "CORBA::String_member&" end

        def inline_toCorba(result, value, indent)
            "#{indent}#{result} = #{value}.c_str();"
        end
    end

    module ContainerType
        def corba_name(plain_name = false)
            if plain_name
                return super
            end

            if deference.corba_name == "CORBA::Octet"
                "_CORBA_Unbounded_Sequence_Octet"
            else
                "_CORBA_Unbounded_Sequence< #{deference.corba_name} >"
            end
        end
        def corba_arg_type; "#{corba_name} const&" end
        def corba_ref_type; "#{corba_name}&" end

        def to_corba(typekit, result, indent)
            collection_name, element_type = container_kind, deference.name
            element_type = registry.build(element_type)

            result << "#{indent}corba.length(value.size());\n"
            # Special case for array of bytes, we can do a simple memcpy
            # (maybe extend that later)
            if collection_name == "/std/vector" && 
                element_type < NumericType &&
                element_type.integer? &&
                element_type.size == 1

                result << "if (!value.empty()) #{indent}memcpy(&corba[0], &value[0], value.size());"
            else
                allocate_index do |element_idx|
                    result << <<-EOT
    #{indent}size_t #{element_idx} = 0;
    #{indent}for(#{cxx_name}::const_iterator it = value.begin(); it != value.end(); ++it, ++#{element_idx})
    #{indent}{
                    EOT

                    if element_type.inlines_code?
                        result << element_type.inline_toCorba("corba[#{element_idx}]", "(*it)", indent)
                    elsif element_type < ArrayType
                        result << indent << "    toCORBA(corba[#{element_idx}], *it, #{element_type.length});\n";
                    else
                        result << indent << "    toCORBA(corba[#{element_idx}], *it);\n";
                    end

                    result << "#{indent}}\n";
                end
            end
	    result
        end
        def from_corba(typekit, result, indent)
            collection_name, element_type = container_kind, deference.name
            element_type = registry.build(element_type)

            if container_kind != "/std/vector"
                raise NotImplementedError, "from_corba is not implemented for other containers than std::vector, got #{container_kind}"
            end

            allocate_index do |element_idx|
                result << "#{indent}size_t const size_#{element_idx} = corba.length();\n"
                result << "#{indent}value.resize(size_#{element_idx});\n"
                result << <<-EOT
#{indent}for(size_t #{element_idx} = 0; #{element_idx} < size_#{element_idx}; ++#{element_idx})
#{indent}{
                EOT

                if element_type.inlines_code?
                    result << element_type.inline_fromCorba("value[#{element_idx}]", "corba[#{element_idx}]", indent)
                elsif element_type < ArrayType
                    result << "#{indent}    fromCORBA(value[#{element_idx}], #{element_type.length}, corba[#{element_idx}]);\n";
                else
                    result << "#{indent}    fromCORBA(value[#{element_idx}], corba[#{element_idx}]);\n";
                end

                result << "#{indent}}\n";
            end
	    result
        end
    end

    module EnumType
	def to_corba(typekit, result, indent)
            seen_values = Set.new
            namespace = namespace('::')
            result << indent << "switch(value) {\n"
            keys.each do |name, value|
                next if seen_values.include?(value)
                seen_values << value

                result << indent << "  case #{namespace}#{name}:\n"
                result << indent << "    corba = orogen#{namespace}Corba::#{name};\n"
                result << indent << "    break;\n"
            end
            result << indent << "}\n"
	end
	def from_corba(typekit, result, indent)
            seen_values = Set.new
            namespace = namespace('::')
            result << indent << "switch(corba) {\n"
            keys.each do |name, value|
                next if seen_values.include?(value)
                seen_values << value

                result << indent << "  case orogen#{namespace}Corba::#{name}:\n"
                result << indent << "    value = #{namespace}#{name};\n"
                result << indent << "    break;\n"
            end
            result << indent << "}\n"
	end
    end

    module CompoundType
        def to_corba(typekit, result, indent)
            code_copy(typekit, result, indent, "corba", "value", "toCORBA", true) do |field_name, field_type|
                if field_type.inlines_code?
                    field_type.inline_toCorba("corba.#{field_name}", "value.#{field_name}", indent)
                end
            end
        end
        def from_corba(typekit, result, indent)
            code_copy(typekit, result, indent, "value", "corba", "fromCORBA", true) do |field_name, field_type|
                if field_type.inlines_code?
                    field_type.inline_fromCorba("value.#{field_name}", "corba.#{field_name}", indent)
                end
            end
        end
    end

    module ArrayType
        def corba_arg_type; "#{deference.corba_name} const*" end
        def corba_ref_type; "#{deference.corba_name}*" end

        def to_corba(typekit, result, indent)
            code_copy(typekit, result, indent, "corba", "value", "toCORBA") do |type, _|
                type.inlines_code?
            end
        end
        def from_corba(typekit, result, indent)
            code_copy(typekit, result, indent, "value", "corba", "fromCORBA") do |type, _|
                type.inlines_code?
            end
        end
    end
    Orocos::Generation::Typekit.register_plugin(Plugin)

    end
    end
end

