module Orocos
    module TypekitMarshallers
        module ROS
            class Plugin
                def self.name; "ros" end
                def name; "ros" end

                def dependencies(typekit)
                    result = []
                    typekit.used_typekits.each do |tk|
                        next if tk.virtual?
                        build_dep = Orocos::Generation::BuildDependency.new(
                            tk.name.upcase + "_TRANSPORT_ROS",
                            tk.pkg_transport_name('ros'))
                            build_dep.in_context('ros', 'include')
                            build_dep.in_context('ros', 'link')
                            result << build_dep
                    end
                    typekit.used_libraries.each do |pkg|
                        needs_link = typekit.linked_used_libraries.include?(pkg)
                        result << Orocos::Generation::BuildDependency.new(pkg.name.upcase, pkg.name).
                            in_context('ros', 'include')
                        if needs_link
                            result.last.in_context('ros', 'link')
                        end
                    end
                    build_dep = Orocos::Generation::BuildDependency.new(
                        "OROCOS_RTT_ROS",
                        "orocos-rtt-ros-${OROCOS_TARGET}")
                    build_dep.in_context('ros', 'include')
                    build_dep.in_context('ros', 'link')
                    result << build_dep
                    result
                end

                def separate_cmake?; true end

                attr_reader :typekit

                def ros_typename(type)
                    if type <= Typelib::NumericType
                        if type.name == "/bool"
                            "bool"
                        elsif type.integer?
                            "#{'u' if type.unsigned?}int#{type.size * 8}"
                        else
                            "float#{type.size * 8}"
                        end
                    elsif type <= Typelib::EnumType
                        # TODO: compute minimal size ?
                        "int32"
                    elsif type.name == "/std/string"
                        "string"
                    end
                end

                def ros_base_type?(type)
                    !!ros_typename(type)
                end

                def ros_message_name(type, absolute = false)
                    if base_type = ros_typename(type)
                        base_type
                    elsif type <= Typelib::ArrayType || type <= Typelib::ContainerType
                        "#{ros_message_name(type.deference)}[]"
                    else
                        source_typekit = typekit.imported_typekits_for(type.name).first
                        if source_typekit || absolute
                            prefix = "#{(source_typekit || typekit).name}_msgs/"
                        end
                        "#{prefix}" + type.name.
                            split("/").
                            map { |n| "#{n[0, 1].upcase}#{n[1..-1]}" }.
                            join("").
                            gsub(/[^\w]/, "")
                    end
                end

                def ros_cxx_type(type)
                    if type < Typelib::EnumType
                        "boost::int32_t"
                    elsif type < Typelib::ArrayType || type < Typelib::ContainerType
                        "std::vector< #{ros_cxx_type(type.deference)} >"
                    elsif type < Typelib::NumericType
                        if type.integer?
                            "boost::#{type.name[1..-1]}"
                        else
                            type.name[1..-1]
                        end
                    else
                        ros_message_name(type, true).gsub(/\//, '::')
                    end
                end

                def ros_ref_type(type)
                    "#{ros_cxx_type(type)}&"
                end
                def ros_arg_type(type)
                    "#{ros_cxx_type(type)} const&"
                end

                def ros_exported_type?(type)
                    type = typekit.intermediate_type_for(type)
                    (type < Typelib::CompoundType) && !ros_base_type?(type)
                end

                def generate(typekit, typesets)
                    @typekit = typekit
                    headers, impl = Array.new, Array.new

                    rosmsg_registry = typesets.minimal_registry.dup

                    # All the generated messages. Used to generate the CMake
                    # file
                    all_messages = Array.new
                    typesets.converted_types.each do |type|
                        type_name = type.name
                        msg_name  = ros_message_name(type)

                        if type.opaque?
                            type = typekit.intermediate_type_for(type)
                        end

                        if ros_exported_type?(type) && !typekit.m_type?(type)
                            msg = Orocos::Generation.render_template "typekit", "ros", "Type.msg", binding
                            typekit.save_automatic("transports", "ros", "msg", "#{msg_name}.msg", msg)
                            all_messages << msg_name
                        end
                    end
                    all_messages = all_messages.sort

                    convert_types = Set.new
                    typesets.converted_types.each do |type|
                        next if ros_base_type?(type) || typekit.m_type?(type)
                        convert_types << [type, type]

                        target_type = typekit.intermediate_type_for(type)
                        if target_type != type
                            convert_types << [target_type, type]
                        end
                    end
                    convert_types = convert_types.sort_by { |t0, t1| [t0.name, t1.name] }

                    convert_array_types = Set.new
                    typesets.array_types.each do |type|
                        type = type.deference
                        next if ros_base_type?(type) || typekit.m_type?(type)
                        convert_array_types << [type, type]

                        target_type = typekit.intermediate_type_for(type)
                        if target_type != type
                            convert_array_types << [target_type, type]
                        end
                    end
                    convert_array_types = convert_array_types.sort_by { |t0, t1| [t0.name, t1.name] }

                    code  = Generation.render_template "typekit", "ros", "Convertions.hpp", binding
                    headers << typekit.save_automatic("transports", "ros",
                                                      "Convertions.hpp", code)
                    code  = Generation.render_template "typekit", "ros", "Convertions.cpp", binding
                    impl << typekit.save_automatic("transports", "ros",
                                                   "Convertions.cpp", code)

                    code  = Generation.render_template "typekit", "ros", "TransportPlugin.hpp", binding
                    headers << typekit.save_automatic("transports", "ros",
                                                      "TransportPlugin.hpp", code)
                    code  = Generation.render_template "typekit", "ros", "TransportPlugin.cpp", binding
                    impl << typekit.save_automatic("transports", "ros",
                                                   "TransportPlugin.cpp", code)

                    code_snippets = typesets.interface_types.map do |type|
                        next if !ros_exported_type?(type)
                        code  = Generation.render_template "typekit", "ros", "Type.cpp", binding
                        [type, code]
                    end.compact
                    impl += typekit.render_typeinfo_snippets(code_snippets, "transports", "ros")

                    code  = Generation.render_template "typekit", "ros", "Registration.hpp", binding
                    typekit.save_automatic("transports", "ros", "Registration.hpp", code)

                    impl = impl.map do |path|
                        typekit.cmake_relative_path(path, "transports", "ros")
                    end.sort
                    headers = headers.map do |path|
                        typekit.cmake_relative_path(path, "transports", "ros")
                    end.sort

                    cmake_config = Generation.render_template "typekit", "ros", "config.cmake.in", binding
                    typekit.save_automatic("transports", "ros", "#{typekit.name}_msgs-config.cmake.in", cmake_config)

                    pkg_config = Generation.render_template "typekit", "ros", "transport-ros.pc", binding
                    typekit.save_automatic("transports", "ros", "#{typekit.name}-transport-ros.pc.in", pkg_config)

                    cmake = Generation.render_template "typekit", "ros", "CMakeLists.txt", binding
                    typekit.save_automatic("transports", "ros", "CMakeLists.txt", cmake)

                    return [], []
                end
            end
            Orocos::Generation::Typekit.register_plugin(Plugin)

            module TypeExtension
                def call_to_ros(ros_var, value_var)
                    "toROS(#{ros_var}, #{value_var})"
                end
                def call_from_ros(value_var, ros_var)
                    "fromROS(#{value_var}, #{ros_var})"
                end
            end
            module OpaqueTypeExtension
                # Convert a C++ value into the corresponding ROS value
                # The C++ value is called 'value' and is a const-ref to the C++
                # type. The ROS value is called 'ros' and is a ref.
                #
                # The method must return the string that will be used for
                # convertion
                def to_ros(typekit, buffer, indent)
                    needs_copy  = typekit.opaque_specification(self).needs_copy?
                    target_type = typekit.intermediate_type_for(self)
                    if needs_copy
                        buffer <<
                            "#{indent}#{target_type.cxx_name} temp;\n" <<
                            "#{indent}orogen_typekits::toIntermediate(temp, value);\n" <<
                            "#{indent}#{target_type.call_to_ros("ros", "temp")};\n"
                    else
                        buffer <<
                            "#{indent}#{target_type.arg_type} temp = orogen_typekits::toIntermediate(value);\n" <<
                            "#{indent}#{target_type.call_to_ros("ros", "temp")};\n"
                    end
                end
                # Convert a C++ value into the corresponding ROS value
                # The C++ value is called 'value' and is a ref to the C++
                # type. The ROS value is called 'ros' and is a const-ref.
                #
                # The method must return the string that will be used for
                # convertion
                def from_ros(typekit, buffer, indent)
                    needs_copy  = typekit.opaque_specification(self).needs_copy?
                    target_type = typekit.intermediate_type_for(self)
                    if needs_copy
                        buffer <<
                            "#{indent}#{target_type.cxx_name} temp;\n" <<
                            "#{indent}#{target_type.call_from_ros("temp", "ros")};\n"
                            "#{indent}orogen_typekits::fromIntermediate(value, temp);\n"
                    else
                        buffer <<
                            "#{indent}std::auto_ptr< #{target_type.cxx_name} > temp(new #{target_type.cxx_name});\n" <<
                            "#{indent}#{target_type.call_from_ros("*temp", "ros")};\n"
                            "#{indent}if (orogen_typekits::fromIntermediate(value, temp.get())) temp.release();\n"
                    end
                end
            end
            module ArrayTypeExtension
                def call_to_ros(ros_var, value_var)
                    "toROS(#{ros_var}, #{value_var}, #{length})"
                end
                def call_from_ros(value_var, ros_var)
                    "fromROS(#{value_var}, #{ros_var}, #{length})"
                end
                # Convert a C++ value into the corresponding ROS value
                # The C++ value is called 'value' and is a const-ref to the C++
                # type. The ROS value is called 'ros' and is a ref.
                #
                # The method must return the string that will be used for
                # convertion
                def to_ros(typekit, buffer, indent)
                    buffer <<
                        "#{indent}ros.resize(#{length});\n" <<
                        "#{indent}for (size_t idx = 0; idx < #{length}; ++idx)\n" <<
                        "#{indent}#{deference.call_to_ros("ros[idx]", "value[idx]")};\n"
                end
                # Convert a C++ value into the corresponding ROS value
                # The C++ value is called 'value' and is a ref to the C++
                # type. The ROS value is called 'ros' and is a const-ref.
                #
                # The method must return the string that will be used for
                # convertion
                def from_ros(typekit, buffer, indent)
                    buffer <<
                        "#{indent}for (size_t idx = 0; idx < #{length}; ++idx)\n" <<
                        "#{indent}#{deference.call_from_ros("value[idx]", "ros[idx]")};\n"
                end
            end
            module ContainerTypeExtension
                # Convert a C++ value into the corresponding ROS value
                # The C++ value is called 'value' and is a const-ref to the C++
                # type. The ROS value is called 'ros' and is a ref.
                #
                # The method must return the string that will be used for
                # convertion
                def to_ros(typekit, buffer, indent)
                    if deference.kind_of?(Typelib::NumericType)
                        buffer << "#{indent}ros = value;"
                    else
                        buffer <<
                            "#{indent}ros.resize(value.size());\n" <<
                            "#{indent}for (size_t idx = 0; idx < value.size(); ++idx)\n" <<
                            "#{indent}#{deference.call_to_ros("ros[idx]", "value[idx]")};\n"
                    end
                end
                # Convert a C++ value into the corresponding ROS value
                # The C++ value is called 'value' and is a ref to the C++
                # type. The ROS value is called 'ros' and is a const-ref.
                #
                # The method must return the string that will be used for
                # convertion
                def from_ros(typekit, buffer, indent)
                    if deference.kind_of?(Typelib::NumericType)
                        buffer << "#{indent}value = ros;"
                    else
                        buffer <<
                            "#{indent}value.resize(ros.size());\n" <<
                            "#{indent}for (size_t idx = 0; idx < ros.size(); ++idx)\n" <<
                            "#{indent}#{deference.call_from_ros("value[idx]", "ros[idx]")};\n"
                    end
                end
            end
            module CompoundTypeExtension
                # Convert a C++ value into the corresponding ROS value
                # The C++ value is called 'value' and is a const-ref to the C++
                # type. The ROS value is called 'ros' and is a ref.
                #
                # The method must return the string that will be used for
                # convertion
                def to_ros(typekit, buffer, indent)
                    each_field do |field_name, field_type|
                        buffer << "#{indent}#{field_type.call_to_ros("ros.#{field_name}", "value.#{field_name}")};\n"
                    end
                end
                # Convert a C++ value into the corresponding ROS value
                # The C++ value is called 'value' and is a ref to the C++
                # type. The ROS value is called 'ros' and is a const-ref.
                #
                # The method must return the string that will be used for
                # convertion
                def from_ros(typekit, buffer, indent)
                    each_field do |field_name, field_type|
                        buffer << "#{indent}#{field_type.call_from_ros("value.#{field_name}", "ros.#{field_name}")};\n"
                    end
                end
            end
            Typelib::Type.extend TypeExtension
            Typelib::OpaqueType.extend OpaqueTypeExtension
            Typelib::ContainerType.extend ContainerTypeExtension
            Typelib::ArrayType.extend ArrayTypeExtension
            Typelib::CompoundType.extend CompoundTypeExtension
        end
    end
end
