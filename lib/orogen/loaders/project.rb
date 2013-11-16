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

            def import_types_from(typekit)
                # Filter out the files ... For now, by ignoring stuff that is
                # not a typekit
                if typekit.respond_to?(:to_str) && !@spec.loader.has_typekit?(typekit)
                    return
                end
                super
            end

            def method_missing(m, *args, &block)
                if @spec.respond_to?(m)
                    @spec.send(m, *args, &block)
                end
            end

            def __load__(file, verbose = true)
                deffile = ::File.expand_path(file)
                deftext = ::File.read(deffile)
                @load_doc = File.file?(deffile)
                ::Kernel.eval_dsl_file_content(deffile, deftext, self, [], verbose)
                self
            end

            def __eval__(name, file_contents, verbose = true)
                deffile = "#{name}.orogen"
                deftext = file_contents
                @load_doc = ::File.file?(deffile)
                ::Kernel.eval_dsl_file_content(deffile, deftext, self, [], verbose)
                self
            end
        end
    end
end
