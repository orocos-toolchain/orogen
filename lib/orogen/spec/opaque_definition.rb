module OroGen
    module Spec
        # Data structure that represents the definition for an opaque type
        #
        # See Typekit#opaque_type
        class OpaqueDefinition
            # The Typelib::Type subclass that represents this opaque
            attr_reader :type
            # The Typelib::Type subclass that represents the intermediate type
            # used to marshal this opaque
            attr_reader :intermediate
            # Option hash
            attr_reader :options
            # If non-nil, this is a block which, when called, will return the
            # C++ code needed to convert +type+ into +intermediate+. If nil, it
            # is assumed that the user will provide that code.
            attr_reader :code_generator

            # Backtrace at the definition point
            attr_accessor :caller

            def initialize(type, intermediate, options, code_generator)
                if !type || !intermediate
                    raise ArgumentError, "trying to create an opaque definition with nil types"
                end

                @type, @intermediate, @options, @code_generator =
                    type, intermediate, options, code_generator
            end
            # The set of paths that should be added to -I to the generated
            # +type+ to +intermediate+ convertion.
            def includes; options[:include] end
            # If true, the opaque needs to be copied into the intermediate. If
            # false, the convertion does not require a copy.
            def needs_copy?; !!options[:needs_copy] end
            # If true, the convertion function is provided by the user, and
            # orogen should therefore generate the corresponding templates.
            def generate_templates?; !code_generator end
        end
    end
end


