require 'utilrb/module/attr_predicate'
require 'utilrb/value_set'

module OroGen
    module Spec
        # Representation of a task's attribute or property
	class ConfigurationObject
            # The task on which this property is attached
            attr_accessor :task
	    # The property name
	    attr_reader :name

	    # The property type, as a Typelib::Type object from the underlying
	    # project's type registry
	    attr_reader :type

            #If this property coudl be set Dynamic this returns true
            def dynamic?; !!@setter_operation end

            # An operation that can be used to set the property. This is non-nil
            # only for dynamic properties. 
            # 
            # @return [Spec::Operation,nil]
            attr_accessor :setter_operation

            # The name of the type this property is using, for consistency with
            # the +type+ attribute
            def type_name; type.name end

            # The type name as registered on RTT
            def orocos_type_name; Typelib::Registry.rtt_typename(type) end

	    # The property's default value
	    attr_reader :default_value

	    # Create a new property with the given name, type and default value
	    def initialize(task, name, type, default_value)
                name = name.to_s
		if name !~ /^\w+$/
                    raise ArgumentError, "property names need to be valid C++ identifiers, i.e. contain only alphanumeric characters and _ (got #{name})"
		end

                type = task.project.find_interface_type(type)
                OroGen.validate_toplevel_type(type)
                @dynamic = false
		@task, @name, @type, @default_value = task, name, type, default_value
                @setter_operation = nil
                @doc = nil
	    end

            def dynamic
                @setter_operation = task.find_operation("__orogen_set#{name.capitalize}")
                if !@setter_operation
                    @setter_operation = task.operation("__orogen_set#{name.capitalize}").
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

            # Converts this model into a representation that can be fed to e.g.
            # a JSON dump, that is a hash with pure ruby key / values.
            #
            # The generated hash has the following keys:
            #
            #     name: the attribute name
            #     type: the type (as marshalled with Typelib::Type#to_h)
            #     dynamic: boolean value indicating whether this can be set
            #       dynamically or not
            #     doc: the documentation string
            #     default: the default value. Not present if there is none.
            #
            # @return [Hash]
            def to_h
                result = Hash[
                    name: name,
                    type: type.to_h,
                    dynamic: !!dynamic?,
                    doc: (doc || "")]
                if value = self.default_value
                    if value.respond_to?(:to_simple_value)
                        result[:default] = value.to_simple_value
                    else
                        result[:default] = value
                    end
                end
                result
            end
	end
    end
end

