module OroGen
    module Loaders
        # Class that allows to load an oroGen file to build a {Spec::Project}
        #
        # It basically ignores any statements that are not part of the spec,
        # assuming that they are part of the code-generation side of things
        class Project < BasicObject
            # @param [Spec::Project] spec the project model we are building
            def initialize(spec)
                @spec = spec
            end

            def method_missing(m, *args, &block)
                if spec.respond_to?(m)
                    spec.send(m, *args, &block)
                end
            end

            def __load__(file, verbose = true)
                self.deffile = File.expand_path(file)
                Kernel.eval_dsl_file(deffile, self, [], verbose)
                self
            end

            def __eval__(name, file_contents, verbose = true)
                self.deffile = "#{name}.orogen"
                Kernel.eval_dsl_file_content(deffile, file_contents, self, [], verbose)
                self
            end
        end
    end
end
