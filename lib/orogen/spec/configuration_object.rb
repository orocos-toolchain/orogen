require 'utilrb/module/attr_predicate'
require 'utilrb/value_set'

module Orocos
    module Spec
        # Representation of a task's attribute or property
	class ConfigurationObject
            # The task on which this property is attached
            attr_accessor :task
	    # The property name
	    attr_reader :name

	    # The property type, as a Typelib::Type object from the underlying
	    # component's type registry
	    attr_reader :type

            #If this property coudl be set Dynamic this returns true
            def dynamic?; !!@setter_operation end

            # An operation that can be used to set the property. This is non-nil
            # only for dynamic properties
            #
            # @return [Orocos::Spec::Operation]
            attr_accessor :setter_operation

            # The name of the type this property is using, for consistency with
            # the +type+ attribute
            def type_name; type.name end

            # The type name as registered on RTT
            def orocos_type_name; Typelib::Registry.rtt_typename(type) end

	    # The property's default value
	    attr_reader :default_value

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
                @setter_operation = nil
	    end

            def dynamic
                @setter_operation = task.find_operation("set#{name.capitalize}")
                if !@setter_operation
                    @setter_operation = task.operation("set#{name.capitalize}").
                        returns("bool").
                        argument("value", type_name).
                        doc("Dynamic Property setter of #{name}")
                end

                self
            end

            def pretty_print(pp)
                default = if value = self.default_value
                              ", default: #{value}"
                          end

                if doc
                    first_line = true
                    doc.split("\n").each do |line|
                        pp.breakable if !first_line
                        first_line = false
                        pp.text "# #{line}"
                    end
                    pp.breakable
                end
                pp.text "#{name}:#{type.name}#{default}"
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

