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

                def generate(typekit, typesets)
                    rosmsg_registry = typesets.minimal_registry.dup

                    # All the generated messages. Used to generate the CMake
                    # file
                    all_messages = Array.new

                    rosmsg_registry.each do |type|
                        type_name = type.name
                        msg_name  = MsgGeneration.message_name_for(typekit.name, type)

                        if type.opaque?
                            type = typekit.intermediate_type_for(type)
                        end

                        if type <= Typelib::CompoundType && !typekit.m_type?(type)
                            msg = Orocos::Generation.render_template "typekit", "ros", "Type.msg", binding
                            typekit.save_automatic("transports", "ros", "msg", "#{msg_name}.msg", msg)
                            all_messages << msg_name
                        end
                    end

                    cmake = Generation.render_template "typekit", "ros", "CMakeLists.txt", binding
                    typekit.save_automatic("transports", "ros", "CMakeLists.txt", cmake)

                    return [], []
                end
            end
            Orocos::Generation::Typekit.register_plugin(Plugin)

            module MsgGeneration
                def self.message_name_for(typekit_name, type)
                    if base_type = ros_typename(type)
                        base_type
                    elsif type <= Typelib::ArrayType || type <= Typelib::ContainerType
                        "#{message_name_for(typekit_name, type.deference)}[]"
                    else
                        type.name.
                            split("/").
                            map { |n| "#{n[0, 1].upcase}#{n[1..-1]}" }.
                            join("").
                            gsub(/[^\w]/, "")
                    end
                end

                def self.ros_typename(type)
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

                def generate_type(type)
                    if base_type = ros_base_type(type)
                    end
                end
            end
        end
    end
end

