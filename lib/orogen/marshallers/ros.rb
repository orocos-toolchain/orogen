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
                            "int#{type.size * 8}"
                        else
                            "float#{type.size * 8}"
                        end
                    elsif type <= Typelib::EnumType
                        # TODO: compute minimal size ?
                        "int32"
                    elsif type.name == "/std/string"
                        "string"
                    elsif type.name == "/base/Time" # YUK
                        "time"
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
                    if type < Typelib::ArrayType || type < Typelib::ContainerType
                        "std::vector< #{ros_cxx_type(type.deference)} >"
                    elsif type < Typelib::NumericType
                        "boost::" + ros_message_name(type, true).gsub(/\//, '::') + "_t"
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

                def generate(typekit, typesets)
                    @typekit = typekit
                    headers, impl = Array.new, Array.new

                    rosmsg_registry = typesets.minimal_registry.dup

                    # All the generated messages. Used to generate the CMake
                    # file
                    all_messages = Array.new

                    rosmsg_registry.each do |type|
                        type_name = type.name
                        msg_name  = ros_message_name(type)

                        if type.opaque?
                            type = typekit.intermediate_type_for(type)
                        end

                        if type <= Typelib::CompoundType && !typekit.m_type?(type)
                            msg = Orocos::Generation.render_template "typekit", "ros", "Type.msg", binding
                            typekit.save_automatic("transports", "ros", "msg", "#{msg_name}.msg", msg)
                            all_messages << msg_name
                        end
                    end

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
                        next if !(type < Typelib::CompoundType)
                        code  = Generation.render_template "typekit", "ros", "Type.cpp", binding
                        [type, code]
                    end.compact
                    impl += typekit.render_typeinfo_snippets(code_snippets, "transports", "ros")

                    code  = Generation.render_template "typekit", "ros", "Registration.hpp", binding
                    typekit.save_automatic("transports", "ros", "Registration.hpp", code)

                    impl = impl.map do |path|
                        typekit.cmake_relative_path(path, "transports", "ros")
                    end
                    headers = headers.map do |path|
                        typekit.cmake_relative_path(path, "transports", "ros")
                    end

                    pkg_config = Generation.render_template "typekit", "ros", "transport-ros.pc", binding
                    typekit.save_automatic("transports", "ros", "#{typekit.name}-transport-ros.pc.in", pkg_config)

                    cmake = Generation.render_template "typekit", "ros", "CMakeLists.txt", binding
                    typekit.save_automatic("transports", "ros", "CMakeLists.txt", cmake)

                    return [], []
                end
            end
            Orocos::Generation::Typekit.register_plugin(Plugin)

            module TypeExtension
                def to_ros(*args)
                end
                def from_ros(*args)
                end
            end
            Typelib::Type.extend TypeExtension
        end
    end
end
