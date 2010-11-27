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
            attr_reader :interface_typelist

            attr_reader :opaques
            attr_reader :opaque_registry

            def self.from_raw_data(main, name, pkg, registry_xml, typelist_txt)
                typekit_registry = Typelib::Registry.new
                Typelib::Registry.add_standard_cxx_types(typekit_registry)
                typekit_registry.merge_xml(registry_xml)

                raw_typelist = typelist_txt.split("\n").map(&:strip)
                typekit_typelist, typekit_interface_typelist = [], []
                raw_typelist.each do |decl|
                    decl =~ /^([^\s]*)(?:\s+(\d+))?$/
                    type, is_interface = $1, $2
                    typekit_typelist << type
                    if !is_interface || is_interface != '0'
                        typekit_interface_typelist << type
                    end
                end

                typekit = self.new(main, name,
                              pkg, typekit_registry,
                              typekit_typelist,
                              typekit_interface_typelist)

                # Now initialize the opaque definitions
                doc = Nokogiri::XML(registry_xml)
                doc.xpath('//opaque').each do |opaque_entry|
                    base_type_name  = opaque_entry['name']
                    inter_type_name = opaque_entry['marshal_as']
                    includes        = opaque_entry['includes']
                    needs_copy      = opaque_entry['needs_copy']
                    spec = OpaqueDefinition.new(
                        typekit_registry.get(base_type_name),
                        inter_type_name,
                        { :includes => includes.split(':'), :needs_copy => (needs_copy == '1') },
                        nil)

                    typekit.opaque_registry.merge(typekit_registry.minimal(base_type_name))
                    typekit.opaques << spec
                end

                typekit
            end

            def has_opaques?
                registry.any? { |t| includes?(t) && t.contains_opaques? }
            end

            def interface_type?(typename)
                typename = typename.name if typename.respond_to?(:name)
                interface_typelist.include?(typename)
            end

            def initialize(main_project, name, pkg, registry, typelist, interface_typelist)
                @main_project, @name, @pkg, @registry = main_project, name, pkg, registry
                @typelist = typelist
                @interface_typelist = interface_typelist
                @opaques = Array.new
                @opaque_registry = Typelib::Registry.new
            end

            def has_opaques?
                if @has_opaques.nil?
                    @has_opaques = registry.any? { |t| t.opaque? }
                end
                @has_opaques
            end

            # Simply ignore, we load the whole typekit anyway
            def opaque_type(*args); end

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
                pkg.include_dirs.to_set
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
            def virtual?; false end
        end

        class RTTTypekit < ImportedTypekit
            RTT_INTERFACE_TYPELIST = %w{/bool /double /float /int32_t}
            def initialize(*args)
                super
                @typelist = registry.each.
                    find_all { |t| t < Typelib::NumericType || t < Typelib::NullType }.
                    map(&:name).to_set
                @interface_typelist = RTT_INTERFACE_TYPELIST
            end

            def pkg_name; end
            def pkg_transport_name(transport_name); end
            def include_dirs; Set.new end
            def types_dir; nil end
            def virtual?; true end
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
                main_project.using_task_library(name)
            end

            def task_context(name, &block) # :nodoc:
                task = super
                task.external_definition = true
                task
            end

            def find_task_context(name)
                begin
                    super
                rescue ArgumentError
                    if main_project
                        main_project.find_task_context(name)
                    else raise
                    end
                end
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

            # Task library objects represent an import, and as such they cannot
            # be generated.  This method raises NotImplementedError
            def generate; raise NotImplementedError end
            # Task library objects represent an import, and as such they cannot
            # be generated.  This method raises NotImplementedError
            def generate_build_system; raise NotImplementedError end
        end
    end
end

