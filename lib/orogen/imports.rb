module Orocos
    module Generation
        # Instances of this class represent a toolkit that has been imported
        # using Component#using_toolkit.
        class ImportedToolkit
            attr_reader :name
            attr_reader :pkg
            attr_reader :registry
            attr_reader :typelist

            def initialize(name, pkg, registry, typelist)
                @name, @pkg, @registry, @typelist = name, pkg, registry, typelist
            end
            def pkg_name
                pkg.name
            end
            def pkg_corba_name
                Utilrb::PkgConfig.new(pkg.name.gsub('toolkit', 'transport-corba')).name
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
        class TaskLibrary < Component
            # The component in which the library has been imported, or nil if
            # there is none
            attr_reader :base_component
            # The pkg-config file defining this task library
            attr_reader :pkg

            # Import in the +base+ component the task library whose orogen
            # specification is included in +file+
            def self.load(base, pkg, file)
                new(base, pkg).load(file)
            end

            def initialize(component, pkg)
                @base_component, @pkg = component, pkg
                super()
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
			registry.get(type)
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


            # True if this task library defines a toolkit
            attr_predicate :has_toolkit?, true

            def using_toolkit(name)
                super
                base_component.using_toolkit name if base_component
            end

            def import_types_from(name, *args)
                if Utilrb::PkgConfig.has_package?("#{name}-toolkit-#{orocos_target}")
                    using_toolkit name
                else
                    using_toolkit self.name
                end
                import_types_from(*args) unless args.empty?
            end

            def toolkit(create = nil, &block) # :nodoc:
                if !create && !block_given?
                    super
                else
                    using_toolkit name
                    self.has_toolkit = true
                    nil
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

