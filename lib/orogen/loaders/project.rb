module OroGen
    module Loaders
        class TaskContext < BasicObject
            def initialize(load_doc, spec)
                @load_doc = load_doc
                @spec = spec
            end

            def method_missing(m, *args, &block)
                @spec.send(m, *args, &block)
            end

            def __normalize_typename(type)
                if type.respond_to?(:to_str)
                    ::Typelib::GCCXMLLoader.cxx_to_typelib(type)
                else type
                end
            end

            def property(name, type, *args, &block)
                model = super(name, __normalize_typename(type), *args, &block)
                if @load_doc
                    ::OroGen::Spec.load_documentation(model, /^property/)
                end
                model
            end
            def input_port(name, type, *args, &block)
                model = super(name, __normalize_typename(type), *args, &block)
                if @load_doc
                    ::OroGen::Spec.load_documentation(model, /^input_port/)
                end
                model
            end
            def output_port(name, type, *args, &block)
                model = super(name, __normalize_typename(type), *args, &block)
                if @load_doc
                    ::OroGen::Spec.load_documentation(model, /^output_port/)
                end
                model
            end
            def dynamic_input_port(name, type, *args, &block)
                type = if type
                           __normalize_typename(type)
                       end

                model = super(name, type, *args, &block)
                if @load_doc
                    ::OroGen::Spec.load_documentation(model, /^dynamic_input_port/)
                end
                model
            end
            def dynamic_output_port(name, type, *args, &block)
                type = if type
                           __normalize_typename(type)
                       end

                model = super(name, type, *args, &block)
                if @load_doc
                    ::OroGen::Spec.load_documentation(model, /^dynamic_output_port/)
                end
                model
            end
            def operation(*args, &block)
                model = super
                if @load_doc
                    ::OroGen::Spec.load_documentation(model, /^operation/)
                end
                model
            end
            def ro_ptr(typename)
                super(__normalize_typename(typename))
            end
            def shared_ptr(typename)
                super(__normalize_typename(typename))
            end
        end

        class BlackHole < BasicObject
            # This is a black hole, we don't get any information from the orogen
            # file for the typekit
            def method_missing(*args)
            end
        end

        # Class that allows to load an oroGen file to build a {Spec::Project}
        #
        # It basically ignores any statements that are not part of the spec,
        # assuming that they are part of the code-generation side of things
        class Project < BasicObject
            # @param [Spec::Project] spec the project model we are building
            def initialize(spec)
                @spec = spec
            end

            # For backward-compatibility reasons
            def using_typekit(typekit)
                import_types_from(typekit)
            end

            def import_types_from(typekit)
                # Filter out the files ... For now, by ignoring stuff that is
                # not a typekit
                if typekit.respond_to?(:to_str) && !@spec.loader.has_typekit?(typekit)
                    return
                end
                super
            end

            def task_context(*args, &block)
                load_doc = @load_doc
                model = super(*args) do
                    ::OroGen::Loaders::TaskContext.new(load_doc, self).instance_eval(&block)
                end
                if @load_doc
                    Spec.load_documentation(model, /^task_context/)
                end
                model
            end

            def method_missing(m, *args, &block)
                if @spec.respond_to?(m)
                    @spec.send(m, *args, &block)
                end
            end

            def self.const_missing(m)
                ::Kernel.const_get(m)
            end

            def find_task_context(name)
                @spec.loader.task_model_from_name(name)
            rescue OroGen::TaskModelNotFound
            end

            def has_typekit?(name)
                @spec.loader.has_typekit?(name)
            end

            def __load__(file, verbose = (::OroGen.logger.level == ::Logger::DEBUG))
                deffile = ::File.expand_path(file)
                __eval__(deffile, File.read(deffile), verbose)
            end

            def __eval__(deffile, deftext, verbose = (::OroGen.logger.level == ::Logger::DEBUG))
                if !deffile
                    @load_doc = false
                    instance_eval deftext
                else
                    @load_doc = ::File.file?(deffile)
                    instance_eval deftext, deffile, 1
                end
                self
            rescue ::Exception => e
                if true || verbose then ::Kernel.raise
                else
                    this_level = ::Kernel.caller.size
                    until_here = e.backtrace[-(this_level-1)..-1] || []
                    subcalls = e.backtrace[0, e.backtrace.size - this_level - 1] || []
                    subcalls.delete_if { |line| line =~ /eval|method_missing/ && line !~ /\.orogen/ }
                    subcalls = subcalls.map { |line| line.gsub(/:in `(?:block in )?__eval__'/, '') }
                    ::Kernel.raise e, e.message, (subcalls + until_here)
                end
            end

            def typekit
                BlackHole.new
            end
        end
    end
end
