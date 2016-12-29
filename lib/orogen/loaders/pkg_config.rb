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
            # @return [Hash<String,Boolean>] a cache of project presence flags
            attr_reader :available_projects
            # @return [Set] the set of known typekits
            attr_reader :available_typekits
            # @return [Hash<String,String>] the set of known task
            #   models, as a mapping from the task model to the name of the task
            #   library that defines it
            attr_reader :available_task_models
            # @return [Hash<String,DeploymentInfo>] the set of known
            #   deployments
            attr_reader :available_deployments
            # @return [Hash<String,Set<String>>] the set of known
            #   deployments on a per-task-model basis
            attr_reader :available_deployed_tasks
            # @return [Hash<String,Type>] the set of known
            #   deployments on a per-task-model basis
            attr_reader :available_types

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
                @available_projects = Hash.new
                @available_typekits = Hash.new
                @available_task_models = Hash.new
                @available_deployments = Hash.new
                @available_deployed_tasks = Hash.new
                @available_types = nil
            end

            # Registers a project from its pkg-config object
            #
            # @param [Utilrb::PkgConfig] the pkg-config from the project
            #   (orogen-project-NAME)
            # @return [Project] the registered project
            def add_project_from(pkg)
            end
            
            def find_pkgconfig(name, minimal: true)
                Utilrb::PkgConfig.get(name, minimal: minimal)
            rescue Utilrb::PkgConfig::NotFound
            end

            def has_pkgconfig?(name)
                !!find_pkgconfig(name, minimal: true)
            end

            def has_project?(name)
                if available_projects.has_key?(name)
                    available_projects[name]
                else
                    available_projects[name] = has_pkgconfig?("orogen-project-#{name}")
                end
            end

            def has_typekit?(name)
                if available_typekits.has_key?(name)
                    available_typekits[name]
                elsif !has_pkgconfig?("#{name}-typekit-#{orocos_target}")
                    available_typekits[name] = false
                elsif !(project_pkg = find_pkgconfig("orogen-project-#{name}"))
                    available_typekits[name] = false
                else
                    available_typekits[name] = !!project_pkg.type_registry
                end
            end

            def project_model_text_from_name(name)
                begin
                    pkg = Utilrb::PkgConfig.get("orogen-project-#{name}", minimal: true)
                    available_projects[name] = true
                rescue Utilrb::PkgConfig::NotFound
                    available_projects[name] = false
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
                    available_typekits[name] = true
                rescue Utilrb::PkgConfig::NotFound
                    available_typekits[name] = false
                    raise TypekitNotFound, "cannot find a typekit called #{name}"
                end

                registry = File.read(pkg.type_registry)
                typelist = File.join(File.dirname(pkg.type_registry), "#{name}.typelist")
                typelist = File.read(typelist)
                return registry, typelist
            end

            def find_task_library_from_task_model_name(model_name)
                if available_task_models.has_key?(model_name)
                    return available_task_models[model_name]
                end

                project_name = model_name.split('::').first
                project_pkg = find_pkgconfig("#{project_name}-tasks-#{orocos_target}")

                if project_pkg
                    project_pkg.task_models.split(',').each do |project_task_name|
                        available_task_models[project_task_name] = project_name
                    end
                end

                if available_task_models.has_key?(model_name)
                    available_task_models[model_name]
                else
                    available_task_models[model_name] = nil
                end
            end

            DeploymentInfo = Struct.new :project_name, :binfile

            def find_project_from_deployment_name(deployment_name)
                if available_deployments.has_key?(deployment_name)
                    return available_deployments[deployment_name].project_name
                end
                
                if deployment_pkg = find_pkgconfig("orogen-#{deployment_name}")
                    if has_project?(deployment_pkg.project_name)
                        info = DeploymentInfo.new(deployment_pkg.project_name, deployment_pkg.binfile)
                        available_deployments[deployment_name] = info
                        info.project_name
                    else
                        available_deployments[deployment_name] = nil
                    end
                else
                    available_deployments[deployment_name] = nil
                end
            end

            def find_deployment_binfile(deployment_name)
                if find_project_from_deployment_name(deployment_name)
                    available_deployments[deployment_name].binfile
                end
            end

            def load_available_deployed_tasks
                @available_deployed_tasks = Hash.new

                Utilrb::PkgConfig.each_package(/^orogen-\w+$/) do |pkg_name|
                    pkg = Utilrb::PkgConfig.get(pkg_name, minimal: true)
                    deployment_name = pkg_name.gsub(/^orogen-/, '')

                    if !pkg.deployed_tasks
                        # oroGen has a bug, in which it installed the pkg-config
                        # file for deployments that were not meant to be
                        # installed.
                        #
                        # Workaround
                        next
                    end

                    pkg.deployed_tasks.split(',').each do |deployed_task_name|
                        available_deployed_tasks[deployed_task_name] ||= Set.new
                        available_deployed_tasks[deployed_task_name] << deployment_name
                    end
                end
            end

            def find_deployments_from_deployed_task_name(name)
                if !available_deployed_tasks
                    load_available_deployed_tasks
                end

                if available_deployed_tasks.has_key?(name)
                    return available_deployed_tasks[name]
                else return Set.new
                end
            end

            def load_available_types
                @available_types = Hash.new

                Utilrb::PkgConfig.each_package(/-typekit-#{orocos_target}$/) do |pkg_name|
                    pkg = Utilrb::PkgConfig.get(pkg_name, minimal: true)
                    next if !pkg.type_registry

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

                if !available_types
                    load_available_types
                end

                typekit = available_types[typename]
                if !typekit
                    raise NotTypekitType.new(typename), "no type #{typename} has been registered in an oroGen project"
                elsif exported && !typekit.exported
                    begin
                        typekits = imported_typekits_for(typename)
                        raise NotExportedType.new(typename, typekits), "the type #{typename} is registered in the #{typekits.map(&:name).sort.join(", ")} typekit, but it is not exported to the RTT type system"
                    rescue DefinitionTypekitNotFound
                        raise NotExportedType.new(typename, []), "the type #{typename} is not exported to the RTT type system"
                    end
                else
                    typekit_model_from_name(typekit.name)
                end
            end

            def each_available_project_name
                return enum_for(__method__) if !block_given?
                rx = /^orogen-project-/
                Utilrb::PkgConfig.each_package(rx) do |pkg_name|
                    yield(pkg_name.gsub(rx, ''))
                end
            end

            def each_available_typekit_name(&block)
                return enum_for(__method__) if !block_given?
                rx = /-typekit-#{orocos_target}$/
                Utilrb::PkgConfig.each_package(rx) do |pkg_name|
                    yield(pkg_name.gsub(rx, ''))
                end
            end

            def each_available_task_model_name(&block)
                return enum_for(__method__) if !block_given?
                each_available_project_name do |project_name|
                    begin
                        pkg = Utilrb::PkgConfig.new("#{project_name}-tasks-#{orocos_target}")
                    rescue Utilrb::PkgConfig::NotFound
                        next
                    end
                    pkg.task_models.split(',').each do |model_name|
                        yield(model_name, project_name)
                    end
                end
            end

            def each_available_deployment_name(&block)
                return enum_for(__method__) if !block_given?
                rx = /^orogen-\w+$/
                Utilrb::PkgConfig.each_package(rx) do |pkg_name|
                    yield(pkg_name.gsub(/^orogen-/, ''))
                end
            end

            def each_available_deployed_task_name(&block)
                return enum_for(__method__) if !block_given?

                if !available_deployed_tasks
                    load_available_deployed_tasks
                end
                available_deployed_tasks.each do |deployed_task_name, deployments|
                    deployments.each do |deployment_name|
                        yield(deployed_task_name, deployment_name)
                    end
                end
            end

            # Enumerate the types available on this system
            #
            # @yieldparam [String] type_name the name of the type
            # @yieldparam [String] typekit_name the name of one typekit that
            #   defines it
            # @yieldparam [Boolean] exported whether this type is exported or
            #   not
            def each_available_type_name(&block)
                return enum_for(__method__) if !block_given?

                if !available_types
                    load_available_types
                end
                available_types.each do |type_name, type_info|
                    yield(type_name, type_info.name, type_info.exported)
                end
            end
        end
    end
end
