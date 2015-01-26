module OroGen
    module Spec
        # Representation of a RTT operation. Instances of this object are
        # usually created through TaskContext#operation. The generated code will
        # expect the class implementation (user-visible part) to define one
        # method, to serve the call, with almost the same name that the method
        # itself.
        #
        # For instance, the following definition
        #   operation('MyMethod')
        #
        # will require the user-visible part to define
        #   [return value] myMethod([arguments]);
        #
        # (note that the first character of the method name has been set to
        # lowercase to generate the C++ method name)
        #
        # The argument list of the C++ method (the first one) can be defined
        # using Callable#argument. Its return type by using #returns. The
        # default method signature is no return type (i.e. void) and no
        # arguments.
        #
        # The name of the C++ method can be changed #method_name.
        #
        # For instance,
        #   operation('MyMethod').
        #     argument('x', 'double', 'the target X value').
        #     argument('y', 'double', 'the target Y value').
        #     method_name('move').
        #     returns('double')
        #
        # will require the user-visible part to define
        #   double move(double x, double y);
	class Operation
	    # The TaskContext instance this operation is part of
	    attr_reader :task
	    # The operation name
	    attr_reader :name
            # True if this operation runs its associated C++ method in caller
            # thread (default is false)
            #
            # See also #runs_in_caller_thread and #runs_in_callee_thread
            attr_reader :in_caller_thread

            RTT_ARGUMENT_COUNT_LIMIT = 4

	    def initialize(task, name)
                name = name.to_s
		if name !~ /^\w+$/
                    raise ArgumentError, "#{self.class.name.downcase} names need to be valid C++ identifiers, i.e. contain only alphanumeric characters and _ (got #{name})"
		end

		@task = task
		@name = name
                @return_type = [nil, 'void', ""]
		@arguments = []
                @in_caller_thread = false
                @doc = nil

                super()
	    end

            # Declares that the C++ method associated with this operation should
            # be executed in the caller thread (default is callee thread)
            #
            # See also #runs_in_callee_thread and #in_caller_thread
            def runs_in_caller_thread
                @in_caller_thread = true
                self
            end

            # Declares that the C++ method associated with this operation should
            # be executed in the caller thread
            #
            # See also #runs_in_callee_thread and #in_caller_thread
            def runs_in_callee_thread
                @in_caller_thread = false
                self
            end

	    # call-seq:
	    #	doc new_doc -> self
            #	doc ->  current_doc
	    #
	    # Gets/sets a string describing this object
	    dsl_attribute(:doc) { |value| value.to_s }

	    # The set of arguments of this operation, as an array of [name, type,
	    # doc] elements. The +type+ objects are Typelib::Type instances.
            # 
            # See #argument
	    attr_reader :arguments

            # This version of find_interface_type returns both a Typelib::Type object and
            # a normalized version for +name+. It does accept const and
            # reference qualifiers in +name+.
            def find_interface_type(qualified_type)
                if qualified_type.respond_to?(:name)
                    qualified_type = qualified_type.name
                end
                type_name = OroGen.unqualified_cxx_type(qualified_type)
                typelib_type_name = ::Typelib::GCCXMLLoader.cxx_to_typelib(type_name)
		type      = task.project.find_interface_type(typelib_type_name)
                OroGen.validate_toplevel_type(type)
                return type, qualified_type.gsub(type_name, type.cxx_name)
            end

            # Defines the next argument of this operation. +name+ is the argument
            # name and +type+ is either the type name as a string, or a
            # Typelib::Type object. In both cases, the required type must be
            # defined in the task context, either because it is part of its own
            # typekit or because it has been imported by a
            # Project#load_typekit call.
            #
            # Note that RTT does not support having more than 4
            # arguments for an operation, and trying that will therefore raise an
            # error
	    def argument(name, qualified_type, doc = "")
                if arguments.size >= RTT_ARGUMENT_COUNT_LIMIT
                    raise ArgumentError, "RTT does not support having more than #{RTT_ARGUMENT_COUNT_LIMIT} arguments for an operation"
                end

                type, qualified_type = find_interface_type(qualified_type)
		arguments << [name, type, doc, qualified_type]
		self
	    end

            # Shortcut for #arg
            def arg(*args, &block)
                argument(*args, &block)
            end

	    # The return type of this operation, as a [type_object,
            # qualified_cxx_type] pair.
            #
            # See #returns
	    attr_reader :return_type

            # Sets the return type for this operation. +type+ can either be the
            # type name or a Typelib::Type object. In both cases, the required
            # type must be defined in the underlying project, either because it
            # is part of its own typekit or because it has been imported by a
            # Project#load_typekit call.
	    def returns(type, doc = "")
                @return_type =
                    if type
                        type, qualified_type = find_interface_type(type)
                        [type, qualified_type, doc]
                    else [nil, 'void', doc]
                    end

		self
	    end

            # Returns true if this operation's signature is not void
            def has_return_value?
                !!@return_type.first
            end

            def pretty_print(pp)
                pp.text name
                pp.nest(2) do
                    if !self.doc
                        pp.breakable
                        pp.text self.doc
                    end
                    if !self.return_type[2].empty?
                        pp.breakable
                        pp.text "Returns: #{self.return_type[2]}"
                    end
                    arguments.map do |name, type, doc, qualified_type|
                        pp.breakable
                        pp.text "#{name}: #{doc}"
                    end
                end
            end

            attr_predicate :hidden?, true
            # Converts this model into a representation that can be fed to e.g.
            # a JSON dump, that is a hash with pure ruby key / values.
            #
            # The generated hash has the following keys:
            #
            #     name: the operation name
            #     returns: the operation return type. It is not present if the
            #       operation does not return anything
            #         type: the return type as marshalled with
            #           Typelib::Type#to_h
            #         doc: the return type documentation
            #
            #     arguments: the list of arguments as an array of
            #         name: the argument name
            #         type: the argument type as marshalled with
            #           Typelib::Type#to_h
            #         doc: the argument documentation
            #
            # @return [Hash]
            def to_h
                result = Hash[name: name, doc: (doc || "")]
                if has_return_value?
                    result[:returns] = Hash[type: self.return_type[0].to_h, doc: self.return_type[2]]
                end
                result[:arguments] = arguments.map do |name, type, doc, qualified_type|
                    Hash[name: name, type: type.to_h, doc: doc]
                end
                result
            end
	end
    end
end


