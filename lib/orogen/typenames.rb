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
                raise Orocos::Generation::InternalError, "normalize_cxxname called with a C++ type name (#{name})"
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
end


