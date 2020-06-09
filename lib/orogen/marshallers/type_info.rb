module OroGen
    module TypekitMarshallers
    module TypeInfo
        class Plugin
            class << self
                attr_predicate :rtt_scripting?, true
            end
            @rtt_scripting = true

            def self.name; "type_info" end
            def name; "type_info" end

            attr_reader :typekit
            def initialize(typekit)
                @typekit = typekit

                Typelib::Type          .extend(TypekitMarshallers::TypeInfo::Type)
                Typelib::NumericType   .extend(TypekitMarshallers::TypeInfo::NumericType)
                Typelib::ContainerType .extend(TypekitMarshallers::TypeInfo::ContainerType)
                Typelib::EnumType      .extend(TypekitMarshallers::TypeInfo::EnumType)
                Typelib::CompoundType  .extend(TypekitMarshallers::TypeInfo::CompoundType)
                Typelib::ArrayType     .extend(TypekitMarshallers::TypeInfo::ArrayType)
            end

            def dependencies
            end

            def separate_cmake?; false end

            def generate(typesets)
                headers, impl = [], []

                arrays, plain = typesets.registered_types.
                    find_all { |t| !t.contains_opaques? }.
                    partition { |t| t < Typelib::ArrayType }

                code_snippets = []
                code_snippets += plain.find_all { |t| !t.contains_opaques? }.map do |type|
                    c = Gen::RTT_CPP.render_template "typekit", "type_info", "Info.cpp", binding
                    [type, c]
                end

                # For arrays, we must define one type info object per element
                # type, not per type
                arrays_of = arrays.inject(Hash.new) do |h, t|
                    h[t.deference] = t; h
                end
                code_snippets += arrays_of.values.find_all { |t| !t.contains_opaques? }.map do |type|
                    c = Gen::RTT_CPP.render_template "typekit", "type_info", "ArrayInfo.cpp", binding
                    [type.deference.name_as_word + "[]", c]
                end

                code_snippets += typesets.registered_types.find_all { |t| t.contains_opaques? }.map do |type|
                    needs_copy =
                        if type.opaque? then typekit.opaque_specification(type).needs_copy?
                        else true
                        end

                    intermediate_type = typekit.intermediate_type_for(type)
                    c  = Gen::RTT_CPP.render_template "typekit", "type_info", "OpaqueInfo.cpp", binding
                    [type, c]
                end

                impl += typekit.render_typeinfo_snippets(code_snippets, "type_info")

                code  = Gen::RTT_CPP.render_template "typekit", "type_info", "TypeInfo.hpp", binding
                typekit.save_automatic("type_info",
                        "Registration.hpp", code)

                return headers, impl
            end
        end

        module Type
            def boost_serialization_compatible?; false end
            def info_type_header
                raise NotImplementedError, "don't know what to use to handle #{self}"
            end
            
            def info_type
                raise NotImplementedError, "don't know what to use to handle #{self}"
            end
        end

        module NumericType
            def boost_serialization_compatible?; true end
            def info_type_header
                "rtt/types/TemplateTypeInfo.hpp"
            end
            
            def info_type
                "RTT::types::TemplateTypeInfo"
            end
        end

        module EnumType
            def boost_serialization_compatible?; true end
            def info_type_header
                "rtt/types/TemplateTypeInfo.hpp"
            end
            
            def info_type
                "RTT::types::TemplateTypeInfo"
            end
        end

        module CompoundType
            def boost_serialization_compatible?
                dependencies.all?(&:boost_serialization_compatible?)
            end
            def to_boost_serialization
                result = []
                each_field do |field_name, field_type|
                    if field_type < Typelib::ArrayType
                        result << "a & make_nvp(\"#{field_name}\", boost::serialization::make_array(b.#{field_name}, #{field_type.length}));"
                    else
                        result << "a & make_nvp(\"#{field_name}\", b.#{field_name});"
                    end
                end
                result.join("\n");
            end

            def info_type_header
                "rtt/types/StructTypeInfo.hpp"
            end
            
            def info_type
                "RTT::types::StructTypeInfo"
            end
        end

        module ContainerType
            def boost_serialization_compatible?
                dependencies.all?(&:boost_serialization_compatible?)
            end

            def info_type_header
                "rtt/types/SequenceTypeInfo.hpp"
            end
            
            def info_type
                "RTT::types::SequenceTypeInfo"
            end
        end

        module ArrayType
            def boost_serialization_compatible?
                deference.boost_serialization_compatible?
            end

            def info_type_header
                "rtt/types/CArrayTypeInfo.hpp"
            end
            
            def info_type
                "RTT::types::CArrayTypeInfo"
            end
        end
    end
    end
end

