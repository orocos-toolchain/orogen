module Typelib
    class Type
        def self.normalize_typename(name)
            "/" + Typelib.split_typename(name).map do |part|
                normalize_typename_part(part)
            end.join("/")
        end

        def self.normalize_typename_part(name)
            # Remove all trailing array modifiers first
            if name =~ /(.*)(\[\d+\]+)$/
                name, array_modifiers = $1, $2
            end

            name, template_arguments = Typelib::GCCXMLLoader.parse_template(name)
            template_arguments.map! do |arg|
                arg =
                    if arg !~ /^\d+$/ && arg[0, 1] != "/"
                        "/#{arg}"
                    else arg
                    end
            end

            if !template_arguments.empty?
                "#{name}<#{template_arguments.join(",")}>#{array_modifiers}"
            else
                "#{name}#{array_modifiers}"
            end
        end

        def self.normalize_cxxname(name)
            if name =~ /::/
                raise InternalError, "normalize_cxxname called with a C++ type name (#{name})"
            end

            if name =~ /(.*)((?:\[\d+\])+)$/
                name = $1
                suffix = $2
            else
                suffix = ''
            end

            converted = Typelib.split_typename(name).map do |p|
                normalize_cxxname_part(p)
            end
            if converted.size == 1
                "#{converted.first}#{suffix}"
            else
                "::" + converted.join("::") + suffix
            end
        end

        def self.normalize_cxxname_part(name)
            name, template_arguments = Typelib::GCCXMLLoader.parse_template(name)

            name = name.gsub('/', '::')
            if name =~ /^::(.*)/
                name = $1
            end

            if !template_arguments.empty?
                template_arguments.map! do |arg|
                    if arg !~ /^\d+$/
                        normalize_cxxname(arg)
                    else arg
                    end
                end

                "#{name}< #{template_arguments.join(", ")} >"
            else name
            end
        end

        def self.cxx_name
            normalize_cxxname(name)
        end
        def self.cxx_basename
            normalize_cxxname(basename)
        end
        def self.cxx_namespace
            namespace('::')
        end

        def self.contains_opaques?
            if @contains_opaques.nil?
                @contains_opaques = contains?(Typelib::OpaqueType)
            end
            @contains_opaques
        end
    end
    class NumericType
	def self.cxx_name
	    if integer?
		if name == "/bool"
		    "bool"
                elsif name == "/char"
                    "char"
                elsif name == "/unsigned char"
                    "unsigned char"
		else
		    "boost::#{'u' if unsigned?}int#{size * 8}_t"
		end
	    else
		basename
	    end
	end

    end
    class ContainerType
        def self.cxx_name
            if name =~ /</
                normalize_cxxname(container_kind) + "< " + deference.cxx_name + " >"
            else
                normalize_cxxname(container_kind)
            end
        end

    end
    class Registry
        # Returns true if +type+ is handled by the typekit that is included in
        # the RTT itself, and false otherwise.
        #
        # This is used in property bags and in the interface definition, as --
        # among the simple types -- only these can be used directly in
        # interfaces.
        def self.base_rtt_type?(type)
            if type.name == "/std/string"
                return true
            elsif !(type <= Typelib::NumericType)
                return false
            end

            if type.integer?
                type.name == "/bool" || type.size == 4
            else
                type.name == "/double"
            end
        end

        # Returns the typename used by RTT to register the given type
        def self.rtt_typename(type)
            if !@typelib_to_rtt_mappings
                cxx_types = Typelib::Registry.new
                Typelib::Registry.add_standard_cxx_types(cxx_types)
                @typelib_to_rtt_mappings = {
                    cxx_types.get('bool') => 'bool',
                    cxx_types.get('int') => 'int',
                    cxx_types.get('unsigned int') => 'uint',
                    cxx_types.get('float') => 'float',
                    cxx_types.get('double') => 'double',
                    cxx_types.get('char') => 'char'
                }
            end

            if type.name == "/std/string"
                return "string"
            elsif !(type <= Typelib::NumericType)
                return type.name
            end

            if type.name == "/bool" then return 'bool'
            elsif mapped = @typelib_to_rtt_mappings.find { |typelib, rtt| typelib == type }
                return mapped[1]
            else
                raise ArgumentError, "#{type.name} is (probably) not registered on the RTT type system"
            end
        end
    end
end


