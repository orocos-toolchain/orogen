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

            def name
                "type_info"
            end

            def dependencies(typekit)
            end

            def generate(typekit, typesets)
                headers, impl = [], []

                typesets.registered_types.each do |type|
                    code  = Generation.render_template "typekit/type_info/Info.cpp", binding
                    impl << Generation.save_automatic("typekit", "type_info",
                        "#{typekit.component.name}_#{type.name_as_word}.cpp", code)
                end
                typesets.array_types.each do |type|
                    code  = Generation.render_template "typekit/type_info/ArrayInfo.cpp", binding
                    impl << Generation.save_automatic("typekit", "type_info",
                        "#{typekit.component.name}_#{type.deference.name_as_word}Array.cpp", code)
                end

                code  = Generation.render_template "typekit/type_info/TypeInfo.hpp", binding
                Generation.save_automatic("typekit", "type_info",
                        "#{typekit.component.name}TypeInfo.hpp", code)

                return headers, impl
            end
        end

        module Type
            def info_type_header
                raise NotImplementedError, "don't know what to use !"
            end
            
            def info_type
                raise NotImplementedError, "don't know what to use !"
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
    end
    end
end

