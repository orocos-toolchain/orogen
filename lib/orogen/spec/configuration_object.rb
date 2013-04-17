require 'utilrb/module/attr_predicate'
require 'utilrb/value_set'

module Orocos
    module Spec
        # Representation of a task's attribute or property
	class ConfigurationObject
            # The task on which this property is attached
            attr_reader :task
	    # The property name
	    attr_reader :name

	    # The property type, as a Typelib::Type object from the underlying
	    # component's type registry
	    attr_reader :type

            #If this property coudl be set Dynamic this returns true
            def dynamic?
                @dynamic
            end

            # The name of the type this property is using, for consistency with
            # the +type+ attribute
            def type_name; type.name end

            # The type name as registered on RTT
            def orocos_type_name; Typelib::Registry.rtt_typename(type) end

	    # The property's default value
	    attr_reader :default_value

            #Returns the operation if this is an dynamic property, nil othrwise
            attr_reader :operation

            # The property default value, formatted for as a C++ value
            def cxx_default_value
                if type < Typelib::EnumType
                    type.namespace('::') + default_value.to_s
                else
                    default_value.inspect
                end
            end

	    # Create a new property with the given name, type and default value
	    def initialize(task, name, type, default_value)
                name = name.to_s
		if name !~ /^\w+$/
                    raise ArgumentError, "property names need to be valid C++ identifiers, i.e. contain only alphanumeric characters and _ (got #{name})"
		end

                type = task.project.find_interface_type(type)
                Orocos.validate_toplevel_type(type)
                @dynamic = false
		@task, @name, @type, @default_value = task, name, type, default_value
                @operation = nil
	    end

            def dynamic
                if task.find_operation("set#{name.capitalize}")
                    raise ArgumentError, "an operation of name set#{name.capitalize} already Exists, this means you cannot register and dynamic peoperty with the name #{name}"
                end
                @operation = task.operation("set#{name.capitalize}").
                    returns("bool").
                    argument("value", type_name).
                    doc("Dynamic Property setter of #{name}")


                @dynamic = true
                self
            end

            def pretty_print(pp)
                default = if value = self.default_value
                              ", default: #{value}"
                          end

                pp.text "#{name}:#{type.name}#{default}"
                if doc
                    pp.text ", doc: #{doc}"
                end
            end

	    # call-seq:
	    #	doc new_doc -> self
            #	doc ->  current_doc
	    #
	    # Gets/sets a string describing this object
	    dsl_attribute(:doc) { |value| value.to_s }
	end
    end
end

