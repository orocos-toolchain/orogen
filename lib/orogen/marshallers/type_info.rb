module Orocos
    module TypekitMarshallers
    module TypeInfo
        class Plugin
            def initialize
                Typelib::Type          .extend(TypekitMarshallers::TypeInfo::Type)
                Typelib::NumericType   .extend(TypekitMarshallers::TypeInfo::NumericType)
                Typelib::ContainerType .extend(TypekitMarshallers::TypeInfo::ContainerType)
                Typelib::EnumType      .extend(TypekitMarshallers::TypeInfo::EnumType)
                Typelib::CompoundType  .extend(TypekitMarshallers::TypeInfo::CompoundType)
                Typelib::ArrayType     .extend(TypekitMarshallers::TypeInfo::ArrayType)
            end

            def self.name; "type_info" end
            def name; "type_info" end

            def dependencies(typekit)
            end

            def separate_cmake?; false end

            def generate(typekit, typesets)
                headers, impl = [], []

                typesets.converted_types.each do |type|
                    code  = Generation.render_template "typekit/type_info/Info.cpp", binding
                    impl << typekit.save_automatic("type_info",
                        "#{type.name_as_word}.cpp", code)
                end
                typesets.array_types.each do |type|
                    code  = Generation.render_template "typekit/type_info/ArrayInfo.cpp", binding
                    impl << typekit.save_automatic("type_info",
                        "#{type.deference.name_as_word}Array.cpp", code)
                end
                typesets.opaque_types.each do |opdef|
                    type = opdef.type
                    intermediate_type = typekit.find_type(opdef.intermediate)
                    code  = Generation.render_template "typekit/type_info/OpaqueInfo.cpp", binding
                    impl << typekit.save_automatic("type_info",
                        "#{type.name_as_word}.cpp", code)
                end

                code  = Generation.render_template "typekit/type_info/TypeInfo.hpp", binding
                typekit.save_automatic("type_info",
                        "Registration.hpp", code)

                return headers, impl
            end
        end

        module Type
            def info_type_header
                raise NotImplementedError, "don't know what to use to handle #{self}"
            end
            
            def info_type
                raise NotImplementedError, "don't know what to use to handle #{self}"
            end
        end

        module NumericType
            def info_type_header
                "rtt/types/TemplateTypeInfo.hpp"
            end
            
            def info_type
                "RTT::types::TemplateTypeInfo"
            end
        end

        module EnumType
            def info_type_header
                "rtt/types/TemplateTypeInfo.hpp"
            end
            
            def info_type
                "RTT::types::TemplateTypeInfo"
            end
        end

        module CompoundType
            def to_boost_serialization
                result = []
                each_field do |field_name, field_type|
                    if field_type < Typelib::ArrayType
                        result << "a & make_nvp(\"#{field_name}\", make_array(b.#{field_name}, #{field_type.length}));"
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
            def info_type_header
                "rtt/types/SequenceTypeInfo.hpp"
            end
            
            def info_type
                "RTT::types::SequenceTypeInfo"
            end
        end

        module ArrayType
            def info_type_header
                "rtt/types/CArrayTypeInfo.hpp"
            end
            
            def info_type
                "RTT::types::CArrayTypeInfo"
            end
        end
        Orocos::Generation::Typekit.register_plugin(Plugin)
    end
    end
end

