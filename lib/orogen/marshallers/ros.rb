module OroGen
    module TypekitMarshallers
        module ROS
            extend Logger::Hierarchy

            class Gen::RTT_CPP::Typekit
                def ros_mappings(mappings)
                    plugin = find_plugin('ros')
                    if !plugin
                        plugin = enable_plugin('ros')
                        plugin.enabled = false
                    end

                    plugin.ros_mappings(mappings)
                    self
                end
            end
            class Gen::RTT_CPP::ImportedTypekit
                def ros_mappings(mappings)
                    self
                end
            end

            # Loads a rosmap file
            #
            # These files are generated for each ROS transport. They contain
            # the mapping from an oroGen type name to the corresponding ROS
            # message name.
            #
            # @return [Hash<String,String>] mapping from the oroGen type
            #   name to the corresponding ROS message name
            def self.load_rosmap(path)
                result = Hash.new
                File.readlines(path).each do |line|
                    line = line.strip
                    next if line.empty?
                    type_name, ros_msg_name, ros_field_name = line.split(/\s+/)
                    result[type_name] = ros_msg_name
                end
                result
            end

            def self.load_rosmap_by_package_name(name)
                pkg = Utilrb::PkgConfig.new("#{name}-transport-ros-#{OroGen.orocos_target}")
                if pkg.rosmap
                    load_rosmap(pkg.rosmap)
                else raise ArgumentError, "the oroGen project #{name} does not have a rosmap"
                end
            end

            DEFAULT_BOXED_MSG_MAPPINGS = Hash.new
            DEFAULT_TYPE_TO_MSG = Hash.new

            DEFAULT_BOXED_MSG_MAPPINGS['std_msgs/Time'] = 'time'
            [8, 16, 32, 64].each do |int_size|
                DEFAULT_BOXED_MSG_MAPPINGS["std_msgs/Int#{int_size}"]  = "int#{int_size}"
                DEFAULT_TYPE_TO_MSG["/int#{int_size}_t"] = "std_msgs/Int#{int_size}"
                DEFAULT_BOXED_MSG_MAPPINGS["std_msgs/UInt#{int_size}"]  = "uint#{int_size}"
                DEFAULT_TYPE_TO_MSG["/uint#{int_size}_t"] = "std_msgs/UInt#{int_size}"
            end
            DEFAULT_BOXED_MSG_MAPPINGS['std_msgs/Bool']  = 'bool'
            DEFAULT_TYPE_TO_MSG['/bool'] = 'std_msgs/Bool'
            DEFAULT_BOXED_MSG_MAPPINGS['std_msgs/Float32'] = 'float32'
            DEFAULT_TYPE_TO_MSG['/float'] = 'std_msgs/Float32'
            DEFAULT_BOXED_MSG_MAPPINGS['std_msgs/Float64'] = 'float64'
            DEFAULT_TYPE_TO_MSG['/double'] = 'std_msgs/Float64'
            DEFAULT_BOXED_MSG_MAPPINGS['std_msgs/String'] = 'string'
            DEFAULT_TYPE_TO_MSG['/std/string'] = 'std_msgs/String'

            # Typekit generation plugin to handle ROS types
            class Plugin
                def self.name; "ros" end
                def name; "ros" end

                def initialize(typekit)
                    @typekit = typekit
                    @type_to_msg = DEFAULT_TYPE_TO_MSG.dup
                    @boxed_msg_mappings = DEFAULT_BOXED_MSG_MAPPINGS.dup
                    self.enabled = true

                    Typelib::Type.extend TypeExtension
                    Typelib::OpaqueType.extend OpaqueTypeExtension
                    Typelib::ContainerType.extend ContainerTypeExtension
                    Typelib::ArrayType.extend ArrayTypeExtension
                    Typelib::CompoundType.extend CompoundTypeExtension
                end

                # @return [Boolean] true if the plugin is enabled, i.e. if the
                #   transport should be generated
                attr_predicate :enabled?, true

                # Mappings from oroGen types to ROS message names
                #
                # @see ros_mappings
                attr_reader :type_to_msg

                # Mappings from ROS boxed messages to the base type they are
                # wrapping
                #
                # Base ROS types cannot be used as messages. One has to first
                # box them into a std_msgs message. This registers these
                # mappings. oroGen then always generate a convertion function
                # for the boxed type whenever the type_to_msg specify a
                # message that should be boxed.
                #
                # @see ros_mappings
                attr_reader :boxed_msg_mappings

                # True if some custom oroGen-to-ROS convertions have been
                # registered
                def has_custom_convertions?
                    !type_to_msg.empty?
                end

                # Checks whether a ROS message name is a boxing type
                def boxed_ros_msg?(msg_name)
                    !!boxed_msg_mappings[msg_name]
                end

                # Define custom mappings from typegen types to ROS messages
                #
                # @param [Hash{String=>String}] mappings mapping from the name of a type
                # known to oroGen to the ROS message that should be used to
                # represent it on the ROS side. The ROS message name should be
                # fully qualified
                def ros_mappings(mappings)
                    mappings.each do |typename, ros_msg_name|
                        type = typekit.find_type(typename)
                        type_to_msg[type.name] = ros_msg_name
                    end
                end

                def load_ros_mappings
                    typekit.used_typekits.each do |tk|
                        next if tk.virtual?
                        begin
                            # Yuk ! Not dependent on the architecture, and
                            # hardcoded pkg-config stuff behaviour
                            raw_mapping = ROS.load_rosmap_by_package_name(tk.name)
                            raw_mapping.delete_if { |type_name, _| !typekit.registry.include?(type_name) }
                            ros_mappings(raw_mapping)
                        rescue Utilrb::PkgConfig::NotFound
                        end
                    end
                end

                # Generates the set of compilation dependencies for the ROS
                # transport
                def dependencies
                    result = []
                    typekit.used_typekits.each do |tk|
                        next if tk.virtual?
                        build_dep = Gen::RTT_CPP::BuildDependency.new(
                            tk.name.upcase + "_TRANSPORT_ROS",
                            tk.pkg_transport_name('ros'))
                            build_dep.in_context('ros', 'include')
                            build_dep.in_context('ros', 'link')
                            result << build_dep
                    end
                    typekit.used_libraries.each do |pkg|
                        needs_link = typekit.linked_used_libraries.include?(pkg)
                        result << Gen::RTT_CPP::BuildDependency.new(pkg.name.upcase, pkg.name).
                            in_context('ros', 'include')
                        if needs_link
                            result.last.in_context('ros', 'link')
                        end
                    end
                    build_dep = Gen::RTT_CPP::BuildDependency.new(
                        "OROCOS_RTT_ROS",
                        "orocos-rtt-ros-${OROCOS_TARGET}")
                    build_dep.in_context('ros', 'include')
                    build_dep.in_context('ros', 'link')
                    result << build_dep
                    result
                end

                # Generates the set of compilation dependencies for the ROS
                # transport
                def separate_cmake?; true end

                # The underlying typekit
                attr_reader :typekit

                # Returns true if +type+ is a base type in ROS
                def ros_base_type?(type)
                    type < Typelib::NumericType || type < Typelib::EnumType
                end

                # Returns the type name that should be used in a field in
                # another ROS message
                #
                # For "normal" messages, this is the message name. For boxed
                # messages (e.g. std_msgs/Time), it is the corresponding base
                # type
                def ros_field_name(type, absolute = false)
                    if !type_to_msg.has_key?(type.name) && (type <= Typelib::ArrayType || type <= Typelib::ContainerType)
                        "#{ros_field_name(type.deference, absolute)}[]"
                    else
                        msg_name = ros_message_name(type, absolute)
                        boxed_msg_mappings[msg_name] || msg_name
                    end
                end

                # Returns the ROS package name generated for a given typekit
                #
                # @param [#name,#to_str] the typekit or typekit name
                # @return [String] the corresponding ROS package name
                def ros_package_name_for_typekit(typekit)
                    if typekit.respond_to?(:name)
                        typekit = typekit.name
                    end
                    "orogen_#{typekit}_msgs"
                end

                # Returns the ROS message name for +type+
                #
                # @param [TypeClass] type the type as a typelib Type class
                # @param [Boolean] absolute if true, the message name will
                # always contain its namespace. Otherwise, it contains its
                # namespace only if not generated by +typekit+. This is required
                # by the ROS message generator
                def ros_message_name(type, absolute = false)
                    if type < Typelib::EnumType
                        "std_msgs/Int32"
                    elsif msg_name = type_to_msg[type.name]
                        return msg_name
                    elsif type <= Typelib::ArrayType || type <= Typelib::ContainerType
                        "#{ros_message_name(type.deference, absolute)}[]"
                    else
                        source_typekit = typekit.imported_typekits_for(type.name).first
                        if source_typekit || absolute
                            prefix = "#{ros_package_name_for_typekit(source_typekit || typekit)}/"
                        end
                        "#{prefix}" + type.name.
                            split("/").
                            map { |n| "#{n[0, 1].upcase}#{n[1..-1]}" }.
                            join("").
                            gsub(/[^\w]/, "")
                    end
                end

                # Returns the C++ type name for the ROS message that represents
                # +type+
                #
                # @param [Boolean] do_unboxing if true, the returned type is the
                # unboxed version of the message (i.e. std_msgs/Time would map
                # to ros::Time). If false, the base message type name is kept
                def ros_cxx_type(type, do_unboxing = true)
                    explicit_mapping = type_to_msg[type.name]
                    if !explicit_mapping && (type < Typelib::ArrayType || type < Typelib::ContainerType)
                        return "std::vector< #{ros_cxx_type(type.deference, do_unboxing)} >"
                    end

                    msg_name = ros_message_name(type, true)
                    if do_unboxing && (unboxed = boxed_msg_mappings[msg_name])
                        msg_name = unboxed
                    end

                    if type < Typelib::NumericType && do_unboxing
                        if type.integer?
                            if msg_name == "bool" then "bool"
                            else "boost::#{msg_name}_t"
                            end
                        elsif type.size == 4
                            return "float"
                        elsif type.size == 8
                            return "double"
                        else raise ArgumentError, "don't know what to use to represent #{type} on the ROS C++ mapping"
                        end
                    elsif type < Typelib::EnumType
                        return "boost::int32_t"
                    elsif msg_name == "time"
                        return "ros::Time"
                    elsif msg_name == 'string'
                        return 'std::string'
                    else
                        return msg_name.gsub(/\//, '::')
                    end
                end

                # Returns a reference signature to the C++ type for the ROS
                # message that represents +type+
                def ros_ref_type(type, unbox = true)
                    "#{ros_cxx_type(type, unbox)}&"
                end

                # Returns the argument signature the C++ type for the ROS
                # message that represents +type+
                def ros_arg_type(type, unbox = true)
                    "#{ros_cxx_type(type, unbox)} const&"
                end

                ARRAY_TYPES = [Typelib::ContainerType, Typelib::ArrayType]

                def multidimensional_array?(type)
                    ARRAY_TYPES.any? { |array_t| type < array_t } &&
                        ARRAY_TYPES.any? { |array_t| type.deference < array_t }
                end

                # Returns whether +type+ should be exported as a ROS message
                def ros_converted_type?(type)
                    type = typekit.intermediate_type_for(type)
                    return true if type_to_msg.has_key?(type.name)
                    return if ros_base_type?(type)

                    if type < Typelib::CompoundType
                        ros_exported_type?(type)
                    elsif multidimensional_array?(type)
                        return false
                    elsif ARRAY_TYPES.any? { |array_type| type < array_type }
                        return (ros_converted_type?(type.deference) || type.deference <= Typelib::EnumType)
                    else true
                    end
                end

                # Returns whether +type+ should be exported as a ROS message
                def ros_exported_type?(type)
                    type = typekit.intermediate_type_for(type)
                    return true if type_to_msg.has_key?(type.name)
                    return if ros_base_type?(type)
                    return if !(type < Typelib::CompoundType)
                    # ROS cannot represent arrays in arrays. Filter those out
                    type.recursive_dependencies.each do |t|
                        next if type_to_msg.has_key?(t.name)
                        if multidimensional_array?(t)
                            return
                        end
                    end
                    return true
                end

                def generate_type_convertion_list(typeset)
                    convert_types = Array.new
                    typeset.each do |type|
                        next if !ros_converted_type?(type)
                        convert_types << [type, type]

                        target_type = typekit.intermediate_type_for(type)
                        if target_type != type
                            convert_types << [target_type, type]
                        end
                    end
                    convert_types.to_set.to_a.sort_by { |t0, t1| [t0.name, t1.name] }
                end

                def generate_disabled(typesets, convert_types, convert_array_types, user_converted_types)
                    rosmap = Gen::RTT_CPP.render_template "typekit", "ros", "rosmap", binding
                    rosmap = typekit.save_automatic("transports", "ros", "#{typekit.name}.rosmap", rosmap)

                    pkg_config = Gen::RTT_CPP.render_template "typekit", "ros", "transport-ros-disabled.pc", binding
                    typekit.save_automatic("transports", "ros", "#{typekit.name}-transport-ros.pc.in", pkg_config)

                    cmake = Gen::RTT_CPP.render_template "typekit", "ros", "CMakeLists-disabled.txt", binding
                    typekit.save_automatic("transports", "ros", "CMakeLists.txt", cmake)
                    return [], []
                end

                # Do the code generation for this ROS transport
                def generate(typesets)
                    load_ros_mappings

                    @typekit = typekit

                    rosmsg_registry = typesets.minimal_registry.dup

                    # All the generated messages. Used to generate the CMake
                    # file
                    all_messages = Array.new
                    typesets.converted_types.each do |type|
                        # We are reusing an existing ROS message
                        next if type_to_msg.has_key?(type.name)
                        type_name = type.name
                        msg_name  = ros_message_name(type)

                        if ros_exported_type?(type) && !typekit.m_type?(type)
                            if type.opaque?
                                type = typekit.intermediate_type_for(type)
                            end

                            msg = Gen::RTT_CPP.render_template "typekit", "ros", "Type.msg", binding
                            typekit.save_automatic("transports", "ros", "msg", "#{msg_name}.msg", msg)
                            all_messages << msg_name
                        end
                    end

                    all_messages = all_messages.sort

                    convert_types = generate_type_convertion_list(
                        typesets.converted_types.
                            find_all { |t| !type_to_msg[t.name] && !typekit.m_type?(t)})
                    convert_array_types = generate_type_convertion_list(
                        typesets.array_types.map(&:deference).
                            find_all { |t| !typekit.m_type?(t)})
                    user_converted_types = generate_type_convertion_list(
                        typesets.converted_types.
                            find_all { |t| type_to_msg[t.name] })
                    user_converted_types.delete_if do |type, ros_type|
                        ros_cxx_type(type) == type.cxx_name.gsub(/^::/, '')
                    end

                    if !enabled?
                        return generate_disabled(typesets, convert_types, convert_array_types, user_converted_types)
                    end

                    # Have a look the user_converted_types whether we need to
                    # generate some unboxing functions automatically
                    convert_boxed_types = user_converted_types.find_all do |type, ros_type|
                        boxed_ros_msg?(ros_message_name(ros_type, true))
                    end.to_set

                    rejected = []
                    code_snippets = typesets.interface_types.map do |type|
                        if !ros_exported_type?(type)
                            rejected << type
                            next
                        end

                        if (mapped = type_to_msg[type.name]) && boxed_ros_msg?(mapped)
                            convert_boxed_types << [type, type]
                        end
                        code  = Gen::RTT_CPP.render_template "typekit", "ros", "Type.cpp", binding
                        [type, code]
                    end.compact

                    headers, impl = Array.new, Array.new

                    convert_boxed_types = convert_boxed_types.sort_by { |t1, t2| [t1.name, t2.name] }
                    code  = Gen::RTT_CPP.render_template "typekit", "ros", "Convertions.hpp", binding
                    headers << typekit.save_automatic("transports", "ros",
                                                      "Convertions.hpp", code)
                    code  = Gen::RTT_CPP.render_template "typekit", "ros", "Convertions.cpp", binding
                    impl << typekit.save_automatic("transports", "ros",
                                                   "Convertions.cpp", code)

                    code  = Gen::RTT_CPP.render_template "typekit", "ros", "TransportPlugin.hpp", binding
                    headers << typekit.save_automatic("transports", "ros",
                                                      "TransportPlugin.hpp", code)
                    code  = Gen::RTT_CPP.render_template "typekit", "ros", "TransportPlugin.cpp", binding
                    impl << typekit.save_automatic("transports", "ros",
                                                   "TransportPlugin.cpp", code)
                    if !user_converted_types.empty?
                        # We need to generate a user part with the convertion
                        # functions. Reuse the templates !
                        
                        code  = Gen::RTT_CPP.render_template "typekit", "ros", "ROSConvertions.hpp", binding
                        headers << typekit.save_user("ROSConvertions.hpp", code)
                        code  = Gen::RTT_CPP.render_template "typekit", "ros", "ROSConvertions.cpp", binding
                        impl << typekit.save_user("ROSConvertions.cpp", code)
                        Gen::RTT_CPP.create_or_update_symlink(
                            headers.last, File.join(typekit.automatic_dir, "transports", "ros", "ROSConvertions.hpp"))
                    end

                    impl += typekit.render_typeinfo_snippets(code_snippets, "transports", "ros")

                    if !rejected.empty?
                        ROS.warn "#{rejected.size} types cannot be exported to ROS, you will not able to publish/subscribe ports of these types to/from ROS: #{rejected.map(&:name).sort.join(", ")}"
                    end

                    code  = Gen::RTT_CPP.render_template "typekit", "ros", "Registration.hpp", binding
                    typekit.save_automatic("transports", "ros", "Registration.hpp", code)

                    impl = impl.map do |path|
                        typekit.cmake_relative_path(path, "transports", "ros")
                    end.sort
                    headers = headers.map do |path|
                        typekit.cmake_relative_path(path, "transports", "ros")
                    end.sort

                    cmake_config = Gen::RTT_CPP.render_template "typekit", "ros", "config.cmake.in", binding
                    typekit.save_automatic("transports", "ros", "orogen_#{typekit.name}_msgs-config.cmake.in", cmake_config)

                    rosmap = Gen::RTT_CPP.render_template "typekit", "ros", "rosmap", binding
                    rosmap = typekit.save_automatic("transports", "ros", "#{typekit.name}.rosmap", rosmap)

                    pkg_config = Gen::RTT_CPP.render_template "typekit", "ros", "transport-ros.pc", binding
                    typekit.save_automatic("transports", "ros", "#{typekit.name}-transport-ros.pc.in", pkg_config)

                    cmake = Gen::RTT_CPP.render_template "typekit", "ros", "CMakeLists.txt", binding
                    typekit.save_automatic("transports", "ros", "CMakeLists.txt", cmake)

                    return [], []
                end
            end

            module TypeExtension
                # Call to generate the C++ call required to convert this type
                # into the corresponding ROS message.
                #
                # @param [String] ros_var the name of the C++ local variable
                # representing the ROS message
                # @param [String] value_var the name of the C++ local variable
                # representing the typegen type
                def call_to_ros(ros_var, value_var)
                    "toROS(#{ros_var}, #{value_var})"
                end

                # Call to generate the C++ call required to convert this type
                # from the corresponding ROS message.
                #
                # @param [String] value_var the name of the C++ local variable
                # representing the typegen type
                # @param [String] ros_var the name of the C++ local variable
                # representing the ROS message
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
                        "#{indent}for (size_t idx = 0; idx < length; ++idx)\n" <<
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
                        "#{indent}for (size_t idx = 0; idx < length; ++idx)\n" <<
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
        end
    end
end
