module Orocos
    module Generation
        # Instances of this class represent a typekit that has been imported
        # using Component#using_typekit.
        class ImportedTypekit
            attr_reader :main_project
            attr_reader :name
            attr_reader :pkg
            attr_reader :registry
            attr_reader :typelist

            def initialize(main_project, name, pkg, registry, typelist)
                @main_project, @name, @pkg, @registry, @typelist = main_project, name, pkg, registry, typelist
            end
            def pkg_name
                pkg.name
            end
            def pkg_corba_name
                Utilrb::PkgConfig.new(pkg.name.gsub('typekit', 'transport-corba')).name
            end
            def include_dirs
                pkg.include_dirs
            end

            def includes?(type)
                typename = if type.respond_to?(:name) then type.name
                           else type.to_str
                           end
                typelist.include?(typename)
            end
        end

        # Instances of this class represent a task library loaded in a
        # component, i.e.  a set of TaskContext defined externally and imported
        # using #using_task_library.
        #
        # For the task contexts imported this way,
        # TaskContext#external_definition?  returns true.
        class ImportedProject < Component
            # The main Component instance that groups all the imported task
            # libraries
            attr_reader :main_project
            # The pkg-config file defining this task library
            attr_reader :pkg

            # Import in the +base+ component the task library whose orogen
            # specification is included in +file+
            def self.load(main_project, pkg, file)
                new(main_project, pkg).load(file)
            end

            def initialize(main_project, pkg = nil)
                @main_project = main_project
                @pkg = pkg
                super()

		if pkg && main_project && main_project.has_typekit?(name)
                    using_typekit pkg.project_name
		end
            end

            def include_dirs
                pkg.include_dirs
            end

            def load_task_library(name)
                main_project.load_task_library(name)
            end
            def load_typekit(name)
                main_project.load_typekit(name)
            end

            def task_context(name, &block) # :nodoc:
                task = super
                task.external_definition = true
                task
            end

	    def find_type(type)
		if type
		    if type.respond_to?(:to_str)
                        type = type.gsub('::', '/')
                        type = Typelib::Type.normalize_typename(type)
                        begin
                            registry.get(type)
                        rescue Typelib::NotFound
                            typekit(true)
                            registry.get(type)
                        end
		    elsif type.kind_of?(Class) && type <= Typelib::Type
                        type
                    else
			raise ArgumentError, "expected a type object, got #{type}"
		    end
		end
	    end

            # Simply ignore type export directives
            def export_types(*args); self end
            def type_export_policy(*args); self end

            # True if this task library defines a typekit
            attr_predicate :has_typekit?, true

            def import_types_from(name, *args)
                if main_project && main_project.has_typekit?(name)
                    using_typekit name
                else
                    using_typekit self.name
                end
                import_types_from(*args) unless args.empty?
            end

            def typekit(create = nil, &block) # :nodoc:
                if !create && !block_given?
                    super
                else
                    @typekit = using_typekit(name)
                    self.has_typekit = true
                    @typekit
                end
            end

            # Task library objects represent an import, and as such they cannot
            # be generated.  This method raises NotImplementedError
            def generate; raise NotImplementedError end
            # Task library objects represent an import, and as such they cannot
            # be generated.  This method raises NotImplementedError
            def generate_build_system; raise NotImplementedError end
        end
    end
end

