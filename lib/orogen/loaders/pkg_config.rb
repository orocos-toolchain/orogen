module OroGen
    module Loaders
        # A loader that accesses the information from the pkg-config files
        # installed by oroGen.
        #
        # It allows to query oroGen-related information from the installed
        # oroGen projects, as well as load the corresponding models. It assumes
        # that the set of installed orogen projects is self-consistent.
        #
        # It accesses the local file system
        class PkgConfig < Base
            include Logger::Hierarchy

            Project = Struct.new :pkg, :orogen_path
            TaskLibrary = Struct.new :pkg
            Type = Struct.new :name, :exported

            def to_s; "#<OroGen::Loaders::PkgConfig(#{object_id.to_s(16)}) #{orocos_target}>" end

            # @return [String] the name of the orocos target we are looking for
            attr_reader :orocos_target
            # @return [Hash<String,Project>] the set of known projects
            attr_reader :available_projects
            # @return [Set] the set of known typekits
            attr_reader :available_typekits
            # @return [Hash<String,Utilrb::PkgConfig>] the set of known task libraries
            attr_reader :available_task_libraries
            # @return [Hash<String,String>] the set of known task
            #   models, as a mapping from the task model to the name of the task
            #   library that defines it
            attr_reader :available_task_models
            # @return [Hash<String,Utilrb::PkgConfig>] the set of known
            #   deployments
            attr_reader :available_deployments
            # @return [Hash<String,Set<String>>] the set of known
            #   deployments on a per-task-model basis
            attr_reader :available_deployed_tasks
            # @return [Hash<String,Type>] the set of known
            #   deployments on a per-task-model basis
            attr_reader :available_types
            # @return [Hash<String,String>] the path to the binary on a
            #   per-deployment basis
            attr_reader :deployment_binfiles

            # @param [String] orocos_target the orocos target we are loading for
            # @param root_loader the root loader. Other loaders might pass it
            #   instead of themselves to the {Spec::Project} objects so that
            #   they use the root loader to get their dependencies. The
            #   pkg-config loader assumes that we have a self-consistent set of
            #   packages on disk (since it is installed packages), and therefore
            #   does not use it.
            def initialize(orocos_target, root_loader = self)
                @orocos_target = orocos_target
                super(root_loader)
            end

            def clear
                super
                update
            end

            # Registers a project from its pkg-config object
            #
            # @param [Utilrb::PkgConfig] the pkg-config from the project
            #   (orogen-project-NAME)
            # @return [Project] the registered project
            def add_project_from(pkg)
            end

            def has_project?(name)
                available_projects.include?(name)
            end

            def has_typekit?(name)
                available_typekits.include?(name)
            end

            def project_model_text_from_name(name)
                begin
                    pkg = Utilrb::PkgConfig.get("orogen-project-#{name}", minimal: true)
                rescue Utilrb::PkgConfig::NotFound
                    raise ProjectNotFound, "cannot find a project called #{name}"
                end

                project = pkg.project_name
                if project.empty?
                    warn "#{pkg.name}.pc does not have a project_name field"
                elsif pkg.deffile.empty?
                    warn "#{pkg.name}.pc does not have a deffile field"
                end
                return File.read(pkg.deffile), pkg.deffile
            end

            def typekit_model_text_from_name(name)
                begin
                    pkg = Utilrb::PkgConfig.get("#{name}-typekit-#{orocos_target}", minimal: true)
                rescue Utilrb::PkgConfig::NotFound
                    raise TypekitNotFound, "cannot find a typekit called #{name}"
                end

                registry = File.read(pkg.type_registry)
                typelist = File.join(File.dirname(pkg.type_registry), "#{name}.typelist")
                typelist = File.read(typelist)
                return registry, typelist
            end

            def find_task_library_from_task_model_name(model_name)
                available_task_models[model_name]
            end

            def find_project_from_deployment_name(deployment_name)
                available_deployments[deployment_name]
            end

            def find_deployments_from_deployed_task_name(name)
                available_deployed_tasks[name] || Set.new
            end

            def update
                @available_projects = Set.new
                @available_task_libraries = Set.new
                @available_deployments = Hash.new
                @deployment_binfiles = Hash.new
                @available_deployed_tasks = Hash.new
                @available_task_models = Hash.new
                @available_typekits = Set.new
                @available_types = Hash.new

                all_packages = Utilrb::PkgConfig.each_package.to_a

                project_pkgconfig = Hash.new

                all_packages.grep(/^orogen-project-/).each do |pkg_name|
                    pkg = Utilrb::PkgConfig.get(pkg_name, minimal: true)
                    project_pkgconfig[pkg.project_name] = pkg
                    available_projects << pkg.project_name
                end

                all_packages.grep(/-tasks-#{orocos_target}$/).each do |pkg_name|
                    pkg = Utilrb::PkgConfig.get(pkg_name, minimal: true)

                    # Verify that the corresponding orogen project is indeed
                    # available. If not, just ignore the library
                    if !available_projects.include?(pkg.project_name)
                        OroGen.warn "found task library #{pkg.project_name}, but the corresponding oroGen project could not be found. Consider deleting #{pkg.path}."
                        next
                    end

                    available_task_libraries << pkg.project_name
                    pkg.task_models.split(",").
                        each { |class_name| available_task_models[class_name] = pkg.project_name }
                end

                all_packages.grep(/^orogen-\w+$/).each do |pkg_name|
                    pkg = Utilrb::PkgConfig.get(pkg_name, minimal: true)
                    deployment_name = pkg_name.gsub(/^orogen-/, '')

                    # Verify that the corresponding orogen project is indeed
                    # available. If not, just ignore the library
                    if !available_projects.include?(pkg.project_name)
                        OroGen.warn "found deployment #{deployment_name}, but the corresponding oroGen project #{pkg.project_name} could not be found. Consider deleting #{pkg.path}."
                        next
                    elsif !pkg.deployed_tasks
                        # oroGen has a bug, in which it installed the pkg-config
                        # file for deployments that were not meant to be
                        # installed.
                        #
                        # Workaround
                        next
                    end

                    available_deployments[deployment_name] = pkg.project_name
                    deployment_binfiles[deployment_name] = pkg.binfile
                    pkg.deployed_tasks.split(',').each do |deployed_task_name|
                        available_deployed_tasks[deployed_task_name] ||= Set.new
                        available_deployed_tasks[deployed_task_name] << deployment_name
                    end
                end

                all_packages.grep(/-typekit-#{orocos_target}$/).each do |pkg_name|
                    pkg = Utilrb::PkgConfig.get(pkg_name, minimal: true)

                    if !has_project?(pkg.project_name)
                        OroGen.warn "found typekit #{pkg.project_name}, but the corresponding oroGen project could not be found. Consider deleting #{pkg.path}."
                        next
                    elsif !project_pkgconfig[pkg.project_name].type_registry
                        OroGen.warn "found typekit #{pkg.project_name}, but the corresponding oroGen project does not have a typekit. Consider deleting #{pkg.path}."
                        next
                    end
                    available_typekits << pkg.project_name

                    typelist = pkg.type_registry.gsub(/tlb$/, 'typelist')
                    typelist, typelist_exported =
                        OroGen::Spec::Typekit.parse_typelist(File.read(typelist))
                    typelist = typelist - typelist_exported
                    typelist.compact.each do |typename|
                        if existing = available_types[typename]
                            OroGen.info "#{typename} is defined by both #{existing[0]} and #{pkg.project_name}, registering #{existing[0]} as the responsible typekit"
                        else
                            available_types[typename] = Type.new(pkg.project_name, false)
                        end
                    end
                    typelist_exported.compact.each do |typename|
                        if existing = available_types[typename]
                            OroGen.info "#{typename} is defined by both #{existing[0]} and #{pkg.project_name}, registering #{pkg.project_name} as it exports the type but #{existing[0]} does not"
                        end
                        available_types[typename] = Type.new(pkg.project_name, true)
                    end
                end
            end

            # Returns the typekit that defines the given type
            #
            # @param [#name,String] type the type name
            # @param [Boolean] exported if true, the method will raise if the
            #   type is defined but not exported.
            # @return [Spec::Typekit]
            # @raise NotTypekitType, NotExportedType
            def typekit_for(type, exported = true)
                typename = if type.respond_to?(:name)
                               type.name
                           else type
                           end

                if typekits = typekits_by_type_name[typename]
                    if exported
                        if export_tk = typekits.find_all { |tk| tk.interface_type?(typename) }.first
                            return export_tk
                        end
                    else
                        return typekits.first
                    end
                end

                typekit = available_types[typename]

                if !typekit
                    raise NotTypekitType.new(typename), "no type #{typename} has been registered in an oroGen project"
                elsif exported && !typekit.exported
                    typekits = imported_typekits_for(typename)
                    raise NotExportedType.new(typename, typekits), "the type #{typename} is registered in the #{typekits.map(&:name).sort.join(", ")} typekit, but it is not exported to the RTT type system"
                else
                    typekit_model_from_name(typekit.name)
                end
            end

            def each_available_project_name(&block)
                return available_projects.each(&block)
            end

            def each_available_typekit_name(&block)
                return available_typekits.each(&block)
            end

            def each_available_deployment_name(&block)
                return available_deployments.each_key(&block)
            end
        end
    end
end
