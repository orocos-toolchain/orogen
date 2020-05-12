# frozen_string_literal: true

module OroGen
    module TypekitMarshallers
        module Corba
            class Plugin
                attr_reader :typekit
                def initialize(typekit)
                    @typekit = typekit

                    Typelib::Type          .extend(TypekitMarshallers::Corba::Type)
                    Typelib::NumericType   .extend(TypekitMarshallers::Corba::NumericType)
                    Typelib::ContainerType .extend(TypekitMarshallers::Corba::ContainerType)
                    Typelib::EnumType      .extend(TypekitMarshallers::Corba::EnumType)
                    Typelib::CompoundType  .extend(TypekitMarshallers::Corba::CompoundType)
                    Typelib::ArrayType     .extend(TypekitMarshallers::Corba::ArrayType)
                    Typelib::OpaqueType    .extend(TypekitMarshallers::Corba::OpaqueType)
                end

                def self.name
                    "corba"
                end

                def name
                    "corba"
                end

                def dependencies
                    result = []
                    typekit.used_typekits.each do |tk|
                        next if tk.virtual?

                        build_dep = Gen::RTT_CPP::BuildDependency.new(
                            tk.name.upcase + "_TRANSPORT_CORBA",
                            tk.pkg_transport_name("corba")
                        )
                        build_dep.in_context("corba", "include")
                        build_dep.in_context("corba", "link")
                        result << build_dep
                    end
                    typekit.used_libraries.each do |pkg|
                        needs_link = typekit.linked_used_libraries.include?(pkg)
                        result << Gen::RTT_CPP::BuildDependency.new(pkg.name.upcase, pkg.name)
                                                               .in_context("corba", "include")
                        if needs_link
                            result.last.in_context("corba", "link")
                        end
                    end
                    result
                end

                def separate_cmake?
                    true
                end

                # Complete list of typekits whose include path must be provided for IDL
                # generation
                #
                # OroGen generates one IDL file per typekit, which means that during IDL
                # generation, include paths should be transitive, including typekits
                # whose types are not directly used by this typekit
                #
                # For this to happen, orogen saves the complete list of dependent
                # typekits in the corba transport's pkg-config file, under the corba
                # variable. This method computes a merged list.
                #
                # @return [Set<String>]
                def corba_idl_requires
                    used_typekits = typekit.used_typekits.find_all { |tk| !tk.virtual? }
                    used_typekits.each_with_object(Set.new) do |tk, all|
                        all << tk.name
                        pkg = tk.transport_pkg("corba")
                        all.merge(pkg.corba_idl_requires.split(","))
                    end
                end

                def generate(typesets)
                    corba_plugin = self
                    headers = []
                    impl = []

                    idl_registry = typesets.minimal_registry.dup
                    opaques = idl_registry.each.find_all(&:contains_opaques?)
                    opaques.each do |t|
                        idl_registry.remove(t)
                    end
                    idl_registry.clear_aliases

                    idl = Gen::RTT_CPP.render_template "typekit", "corba", "Types.idl", binding
                    idl_file = typekit.save_automatic("transports", "corba",
                                                      "#{typekit.name}Types.idl", idl)

                    code = Gen::RTT_CPP.render_template "typekit", "corba", "Convertions.cpp", binding
                    impl << typekit.save_automatic("transports", "corba",
                                                   "Convertions.cpp", code)

                    code = Gen::RTT_CPP.render_template "typekit", "corba", "TransportPlugin.hpp", binding
                    headers << typekit.save_automatic("transports", "corba",
                                                      "TransportPlugin.hpp", code)
                    code = Gen::RTT_CPP.render_template "typekit", "corba", "TransportPlugin.cpp", binding
                    impl << typekit.save_automatic("transports", "corba",
                                                   "TransportPlugin.cpp", code)

                    code_snippets = typesets.interface_types.map do |type|
                        target_type = typekit.intermediate_type_for(type)
                        code = Gen::RTT_CPP.render_template "typekit", "corba", "Type.cpp", binding
                        [type, code]
                    end
                    impl += typekit.render_typeinfo_snippets(code_snippets, "transports", "corba")

                    code = Gen::RTT_CPP.render_template "typekit", "corba", "Registration.hpp", binding
                    typekit.save_automatic("transports", "corba", "Registration.hpp", code)

                    impl = impl.map do |path|
                        typekit.cmake_relative_path(path, "transports", "corba")
                    end.sort
                    headers = headers.map do |path|
                        typekit.cmake_relative_path(path, "transports", "corba")
                    end.sort

                    pkg_config = Gen::RTT_CPP.render_template "typekit", "corba", "transport-corba.pc", binding
                    typekit.save_automatic("transports", "corba", "#{typekit.name}-transport-corba.pc.in", pkg_config)
                    code = Gen::RTT_CPP.render_template "typekit", "corba", "CMakeLists.txt", binding
                    typekit.save_automatic("transports", "corba", "CMakeLists.txt", code)

                    # We generate our own CMake code, no need to export anything to the
                    # main typekit code
                    [[], []]
                end
            end

            module Utils
                def strip_leading_underscore(name)
                    name.split("::", -1).map do |field|
                        field.gsub(/^_/, "")
                    end.join("::")
                end
            end

            module Type
                include Utils

                def corba_name
                    if inlines_code?
                        normalize_cxxname(basename)
                    elsif contains_opaques?
                        "#{corba_namespace}::#{normalize_idl_name(basename)}_m"
                    else
                        "#{corba_namespace}::#{normalize_idl_name(basename)}"
                    end
                end

                def normalize_idl_name(basename)
                    strip_leading_underscore(normalize_cxxname(basename.gsub(/[^\w]/, "_")))
                end

                def corba_namespace
                    "orogen#{strip_leading_underscore(namespace('::'))}Corba"
                end

                def to_corba_signature(typekit, options = Hash.new)
                    target_type = typekit.intermediate_type_for(self)
                    "bool #{options[:namespace]}toCORBA( #{target_type.corba_ref_type} corba, #{arg_type} value )"
                end

                def to_corba_array_signature(typekit, options = Hash.new)
                    target_type = typekit.intermediate_type_for(self)
                    "bool #{options[:namespace]}toCORBA( #{target_type.corba_ref_type} corba, #{arg_type} value, int length )"
                end

                def from_corba_signature(typekit, options = Hash.new)
                    target_type = typekit.intermediate_type_for(self)
                    "bool #{options[:namespace]}fromCORBA( #{ref_type} value, #{target_type.corba_arg_type} corba )"
                end

                def from_corba_array_signature(typekit, options = Hash.new)
                    target_type = typekit.intermediate_type_for(self)
                    "bool #{options[:namespace]}fromCORBA( #{ref_type} value, int length, #{target_type.corba_arg_type} corba )"
                end

                def corba_arg_type
                    "#{corba_name} const&"
                end

                def corba_ref_type
                    "#{corba_name}&"
                end

                def to_corba(typekit, result, *args)
                    raise NotImplementedError
                end

                def from_corba(typekit, result, *args)
                    raise NotImplementedError
                end

                def inline_fromCorba(result, value, indent) # rubocop:disable Naming/MethodName
                    "#{indent}#{result} = #{value};\n"
                end

                def inline_fromAny(any_var, corba_var, indent) # rubocop:disable Naming/MethodName
                    "#{indent}if (!(#{any_var} >>= #{corba_var})) return false;"
                end

                def inline_toCorba(result, value, indent) # rubocop:disable Naming/MethodName
                    "#{indent}#{result} = #{value};\n"
                end

                def inline_toAny(any_var, corba_var, indent) # rubocop:disable Naming/MethodName
                    "#{indent}#{any_var} <<= #{corba_var};"
                end
            end

            module NumericType
                def corba_name
                    if integer?
                        if name == "/bool"
                            "CORBA::Boolean"
                        elsif size == 1
                            if unsigned?
                                "CORBA::Octet"
                            else
                                "CORBA::Char"
                            end
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

            ::Typelib::specialize_model "/uint8_t" do
                def inline_fromAny(any_var, corba_var, indent) # rubocop:disable Naming/MethodName
                    "#{indent}#{any_var} >>= CORBA::Any::to_octet(#{corba_var});"
                end

                def inline_toAny(any_var, corba_var, indent) # rubocop:disable Naming/MethodName
                    "#{indent}#{any_var} <<= CORBA::Any::from_octet(#{corba_var});"
                end
            end

            ::Typelib::specialize_model "/int8_t" do
                def inline_fromAny(any_var, corba_var, indent) # rubocop:disable Naming/MethodName
                    "#{indent}#{any_var} >>= CORBA::Any::to_char(#{corba_var});"
                end

                def inline_toAny(any_var, corba_var, indent) # rubocop:disable Naming/MethodName
                    "#{indent}#{any_var} <<= CORBA::Any::from_char(#{corba_var});"
                end
            end

            ::Typelib::specialize_model "/bool" do
                def inline_fromAny(any_var, corba_var, indent) # rubocop:disable Naming/MethodName
                    "#{indent}#{any_var} >>= CORBA::Any::to_boolean(#{corba_var});"
                end

                def inline_toAny(any_var, corba_var, indent) # rubocop:disable Naming/MethodName
                    "#{indent}#{any_var} <<= CORBA::Any::from_boolean(#{corba_var});"
                end
            end

            ::Typelib::specialize_model "/std/string" do
                def inlines_code?
                    true
                end

                def corba_name
                    "char const*"
                end

                def corba_arg_type
                    "char const*"
                end

                def corba_ref_type
                    "char const*&"
                end

                def inline_toCorba(result, value, indent) # rubocop:disable Naming/MethodName
                    "#{indent}#{result} = #{value}.c_str();"
                end

                def to_corba(typekit, result, indent)
                    result << "#{indent}corba = value.c_str();\n"
                end

                def from_corba(typekit, result, indent)
                    result << "#{indent}value = corba;\n"
                end
            end

            module ContainerType
                def corba_name
                    container_kind = self.container_kind.gsub /.*\//, ""
                    element_name   = deference.name.gsub(/[^\w]/, "_")
                    typedef_name = container_kind + "_" + element_name

                    if deference.corba_name !~ /^orogen::/
                        # This type is mapped into the root namespace
                        "orogen::Corba::#{typedef_name}_"
                    else
                        "#{corba_namespace}::#{typedef_name}_"
                    end
                end

                def corba_arg_type
                    "#{corba_name} const&"
                end

                def corba_ref_type
                    "#{corba_name}&"
                end

                def corba_namespace
                    deference.corba_namespace
                end

                def to_corba(typekit, result, indent)
                    collection_name = container_kind
                    element_type = deference.name
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
                                result << indent << "    toCORBA(corba[#{element_idx}], reinterpret_cast< #{element_type.deference.cxx_name} const*>(*it), #{element_type.length});\n"
                            else
                                result << indent << "    toCORBA(corba[#{element_idx}], *it);\n"
                            end

                            result << "#{indent}}\n"
                        end
                    end
                    result
                end

                def from_corba(typekit, result, indent)
                    collection_name = container_kind
                    element_type = deference.name
                    element_type = registry.build(element_type)

                    if container_kind != "/std/vector"
                        raise NotImplementedError, "from_corba is not implemented for other containers than std::vector, got #{container_kind}"
                    end

                    allocate_index do |element_idx|
                        result << "#{indent}size_t const size_#{element_idx} = corba.length();\n"
                        result << "#{indent}value.resize(size_#{element_idx});\n"

                        if collection_name == "/std/vector" &&
                           element_type < NumericType

                            result << "if (!value.empty()) #{indent}memcpy(&value[0], &corba[0], size_#{element_idx});"
                        else
                            result << <<-EOT
    #{indent}for(size_t #{element_idx} = 0; #{element_idx} < size_#{element_idx}; ++#{element_idx})
    #{indent}{
                    EOT

                            if element_type.inlines_code?
                                result << element_type.inline_fromCorba("value[#{element_idx}]", "corba[#{element_idx}]", indent)
                            elsif element_type < ArrayType
                                result << "#{indent}    fromCORBA(reinterpret_cast<#{element_type.deference.cxx_name}*>(value[#{element_idx}]), #{element_type.length}, corba[#{element_idx}]);\n"
                            else
                                result << "#{indent}    fromCORBA(value[#{element_idx}], corba[#{element_idx}]);\n"
                            end

                            result << "#{indent}}\n"
                        end
                    end
                    result
                end
            end

            module EnumType
                include Utils

                def to_corba(typekit, result, indent)
                    seen_values = Set.new
                    namespace = namespace("::")
                    stripped_namespace = strip_leading_underscore(namespace)
                    result << indent << "switch(value) {\n"
                    keys.each do |name, value|
                        next if seen_values.include?(value)

                        seen_values << value

                        stripped_name = strip_leading_underscore(name)
                        result << indent << "  case #{namespace}#{name}:\n"
                        result << indent << "    corba = orogen#{stripped_namespace}Corba::#{stripped_name};\n"
                        result << indent << "    break;\n"
                    end
                    result << <<-EOT
#{indent}  default:
#{indent}    RTT::log(RTT::Error) << "orogen_typekits::toCORBA() invalid value '" << (int)value << "' for enum '#{cxx_name}'" << RTT::endlog();
#{indent}    return false;
EOT
                    result << indent << "}\n"
                end

                def from_corba(typekit, result, indent)
                    seen_values = Set.new
                    namespace = namespace("::")
                    stripped_namespace = strip_leading_underscore(namespace)
                    result << indent << "switch(corba) {\n"
                    keys.each do |name, value|
                        next if seen_values.include?(value)

                        seen_values << value

                        stripped_name = strip_leading_underscore(name)
                        result << indent << "  case orogen#{stripped_namespace}Corba::#{stripped_name}:\n"
                        result << indent << "    value = #{namespace}#{name};\n"
                        result << indent << "    break;\n"
                    end
                    result << <<-EOT
#{indent}  default:
#{indent}    RTT::log(RTT::Error) << "orogen_typekits::fromCORBA() invalid value '" << (int)corba << "' for enum '#{cxx_name}'" << RTT::endlog();
#{indent}    return false;
EOT
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
                def corba_arg_type
                    "#{deference.corba_name} const*"
                end

                def corba_ref_type
                    "#{deference.corba_name}*"
                end

                def to_corba(typekit, result, indent)
                    element_type = deference.name
                    element_type = registry.build(element_type)

                    # Special case for array of numerics, we can do a simple memcpy
                    if  element_type < NumericType
                        result << "#{indent}const int array_size = length * sizeof(#{element_type.cxx_name});\n"
                        result << "#{indent}memcpy(corba, value, array_size);"
                    else
                        code_copy(typekit, result, indent, "corba", "value", "toCORBA") do |type, _|
                            type.inlines_code?
                        end
                    end
                end

                def from_corba(typekit, result, indent)
                    element_type = deference.name
                    element_type = registry.build(element_type)

                    # Special case for array of numerics, we can do a simple memcpy
                    if  element_type < NumericType
                        result << "#{indent}const int array_size = length * sizeof(#{element_type.cxx_name});\n"
                        result << "#{indent}memcpy(value, corba, array_size);"
                    else
                        code_copy(typekit, result, indent, "value", "corba", "fromCORBA") do |type, _|
                            type.inlines_code?
                        end
                    end
                end
            end

            module OpaqueType
                def to_corba(typekit, result, indent)
                    spec        = typekit.opaque_specification(name)
                    target_type = typekit.intermediate_type_for(self)
                    result << typekit.code_toIntermediate(target_type, spec.needs_copy?, "    ")
                    result << "#{indent}if (!toCORBA(corba, intermediate)) return false;"
                end

                def from_corba(typekit, result, indent)
                    spec        = typekit.opaque_specification(name)
                    target_type = typekit.intermediate_type_for(self)
                    if spec.needs_copy?
                        result << <<~EOCODE
                            #{indent}#{target_type.cxx_name} intermediate;
                            #{indent}if (!fromCORBA(intermediate, corba))
                            #{indent}    return false;
                            #{typekit.code_fromIntermediate(target_type, true, indent)}
                        EOCODE
                    else
                        result << <<~EOCODE
                            #{indent}UNIQUE_PTR< #{target_type.cxx_name} > intermediate(new #{target_type.cxx_name});
                            #{indent}if (!fromCORBA(*intermediate, corba))
                            #{indent}    return false;
                            #{typekit.code_fromIntermediate(target_type, false, indent)}
                        EOCODE
                    end
                end
            end
        end
    end
end
