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
                @main_project, @name, @pkg, @registry, @typelist =
                    main_project, name, pkg, registry, typelist
            end

            def has_opaques?
                if @has_opaques.nil?
                    @has_opaques = registry.any? { |t| t.opaque? }
                end
                @has_opaques
            end

            def pkg_name
                pkg.name
            end
            def perform_pending_loads; end
            def pkg_transport_name(transport_name)
                Utilrb::PkgConfig.new(pkg.name.gsub('typekit', "transport-#{transport_name}")).name
            end
            def pkg_corba_name
                pkg_transport_name('corba')
            end
            def include_dirs
                pkg.include_dirs
            end
            # Returns the full path to the types/ subdir in the include
            # directory
            def types_dir
                File.join(pkg.includedir, name, 'types')
            end

            def using_library(*args); end
            def using_typekit(*args); end

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
            # The pkg-config file defining this oroGen project
            attr_reader :pkg
            # The pkg-config file for the task library of this oroGen project
            def tasklib_pkg
                @tasklib_pkg ||= Utilrb::PkgConfig.new("#{name}-tasks-#{Orocos::Generation.orocos_target}")
            end

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

            def using_task_library(name)
                load_task_library(name)
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

            def import_types_from(name, *args)
                if main_project && main_project.has_typekit?(name)
                    using_typekit name
                else
                    using_typekit self.name
                end
            end

            def typekit(create = nil, &block) # :nodoc:
                if @typekit.nil?
                    @typekit =
                        if main_project && main_project.has_typekit?(name)
                            using_typekit(name)
                        else nil
                        end
                end
                @typekit
            end

            def deployment(name)
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

