# frozen_string_literal: true

module OroGen
    module Spec
        module DynamicPort
            def instanciate(name, type = nil)
                m = dup
                m.instance_variable_set :@name, name
                if type
                    if type.respond_to?(:to_str)
                        type = task.project.find_type(type)
                    end
                    m.instance_variable_set :@type, type
                end
                m
            end

            def dynamic?
                true
            end

            def each_interface_type
                return enum_for(__method__) unless block_given?

                super if type
            end

            def pretty_print(pp)
                pp.text "[dyn,#{self.class < InputPort ? "in" : "out"}]#{name}:#{type ? type.name : "any type"}"
            end
        end

        # Specification for a dynamic output port.
        #
        # Dynamic ports are not statically present, but will be created at
        # runtime. They are added by TaskContext#dynamic_output_port.
        class DynamicOutputPort < OutputPort
            include DynamicPort
        end

        # Specification for a dynamic input port.
        #
        # Dynamic ports are not statically present, but will be created at
        # runtime. They are added by TaskContext#dynamic_input_port.
        class DynamicInputPort < InputPort
            include DynamicPort

            def task_trigger # :nodoc:
                @trigger_port = true
            end
        end
    end
end
