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

            # The name of the type this property is using, for consistency with
            # the +type+ attribute
            def type_name; type.name end

            # The type object that will actually be used in Ruby scripts
            # 
            # If the type is an opaque, it is the corresponding intermediate
            # type. Otherwise, #type is returned
            attr_reader :typelib_type

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

                @typelib_type = task.project.intermediate_type_for(type)
		@task, @name, @type, @default_value = task, name, type, default_value
	    end

            def pretty_print(pp)
                pp.text "#{name}:#{type.name}"
                if doc
                    pp.text ": #{doc}"
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

