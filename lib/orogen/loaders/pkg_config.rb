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
        class PkgConfig
            Project = Struct.new :pkg, :orogen_path
            TaskLibrary = Struct.new :pkg
            Type = Struct.new :name, :exported

            # @return [String] the name of the orocos target we are looking for
            attr_reader :orocos_target
            # @return [Hash<String,Project>] the set of known projects
            attr_reader :available_projects
            # @return [Hash<String,Utilrb::PkgConfig>] the set of known typekits
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

            # @param [String] orocos_target the orocos target we are loading for
            # @param root_loader the root loader. Other loaders might pass it
            #   instead of themselves to the {Spec::Project} objects so that
            #   they use the root loader to get their dependencies. The
            #   pkg-config loader assumes that we have a self-consistent set of
            #   packages on disk (since it is installed packages), and therefore
            #   does not use it.
            def initialize(orocos_target, root_loader = nil)
                @orocos_target = orocos_target
                update
            end

            # Registers a project from its pkg-config object
            #
            # @param [Utilrb::PkgConfig] the pkg-config from the project
            #   (orogen-project-NAME)
            # @return [Project] the registered project
            def add_project_from(pkg)
                project = pkg.project_name
                if project.empty?
                    OroGen.warn "#{pkg.name}.pc does not have a project_name field"
                end
                if description = available_projects[project]
                    return description
                end

                if pkg.deffile.empty?
                    OroGen.warn "#{pkg.name}.pc does not have a deffile field"
                else
                    available_projects[pkg.project_name] = Project.new(pkg, pkg.deffile)
                end
            end

            # Tests if a typekit with that name exists
            #
            # @param [String] the typekit name
            # @return [Boolean]
            def has_typekit?(name)
                available_typekits.has_key?(name)
            end

            # Returns the project model corresponding to the given name
            #
            # @param [String] the project name
            # @option options [Boolean] :define_dummy_types (false) if true,
            #   unknown types will be automatically created as null types on the
            #   underlying typekit. If false, they will generate a
            #   Typekit::NotFound exception
            # @raise [OroGen::NotFound] if there is no task library with that
            #   name. This does including having a project with that name if the
            #   project defines no tasks.
            # @return [OroGen::Spec::Project]
            def project_model_from_name(name, options = Hash.new)
                name = name.to_str
                options = Kernel.validate_options options,
                    :define_dummy_types => false

                if !(project = available_projects[name])
                    raise OroGen::NotFound, "cannot find an installed oroGen project called #{name}"
                end

                OroGen.info "loading oroGen project #{name} on #{self.name}"
                lib = Spec::Project.new(self)
                if has_typekit?(name)
                    lib.typekit = typekit_from_name(name)
                else lib.typekit = Spec::Typekit.new(lib.name)
                end
                lib.typekit.define_dummy_types = options[:define_dummy_types]
                lib.__load__(project.orogen_path)
                lib
            end

            # Returns the task library model corresponding to the given name
            # @param (see project_model_from_name)
            # @raise (see project_model_from_name)
            # @return (see project_model_from_name)
            def task_library_model_from_name(name, options = Hash.new)
                project = project_model_from_name(name, options)
                if project.self_tasks.empty?
                    raise OroGen::NotFound, "there is an oroGen project called #{name}, but it defines no tasks"
                end
                project
            end

            # Returns the task model object corresponding to a model name
            #
            # @raise OroGen::NotFound if there are no such model
            def task_model_from_name(name)
                tasklib_name = available_task_models[name]
                if !tasklib_name
                    raise OroGen::NotFound, "no task model #{name} is registered"
                end

                tasklib = project_model_from_name(tasklib_name)
                result = tasklib.tasks[name]
                if !result
                    raise InternalError, "while looking up model of #{name}: found project #{tasklib_name}, but this project does not actually have a task model called #{name}"
                end

                result
            end

            # Returns the deployment model for the given deployment name
            #
            # @return [OroGen::Spec::Deployment] the deployment model
            # @raise [OroGen::NotFound] if no deployment with that name exists
            def deployment_model_from_name(name)
                pkg = available_deployments[name]
                if !pkg
                    raise OroGen::NotFound, "there is no deployment called #{name}"
                end

                project = project_model_from_name(name)
                deployment = project.deployers.find { |d| d.name == name }
                if !deployment
                    raise InternalError, "cannot find the deployment called #{name} in #{project.name}. Candidates were #{project.deployers.map(&:name).sort.join(", ")}"
                end
                deployment
            end

            # Returns the deployed task model for the given name
            #
            # @param [String] name the deployed task name
            # @param [String] deployment_name () the name of the deployment in which the
            #   task is defined. It must be given only when more than one deployment
            #   defines a task with the requested name
            # @return [OroGen::Spec::TaskDeployment] the deployed task model
            # @raise [OroGen::NotFound] if no deployed tasks with that name exists
            # @raise [OroGen::NotFound] if deployment_name was given, but the requested
            #   task is not defined in this deployment
            # @raise [OroGen::AmbiguousName] if more than one task exists with that
            #   name. In that case, you will have to provide the deployment name
            #   explicitly using the second argument
            def deployed_task_model_from_name(name, deployment_name = nil)
                if deployment_name
                    deployment = deployment_model_from_name(deployment_name)
                else
                    deployment_names = available_deployed_tasks[name]
                    if !deployment_names
                        raise OroGen::NotFound, "cannot find a deployed task called #{name}"
                    elsif deployment_names.size > 1
                        raise OroGen::AmbiguousName, "more than one deployment defines a deployed task called #{name}: #{deployment_names.map(&:name).sort.join(", ")}"
                    end
                    deployment = deployment_model_from_name(deployment_names.first)
                end

                if !(task = deployment.find_task_by_name(name))
                    if deployment_name
                        raise OroGen::NotFound, "deployment #{deployment_name} does not have a task called #{name}"
                    else
                        raise InternalError, "deployment #{deployment_name} was supposed to have a task called #{name} but does not"
                    end
                end
                task
            end

            # Loads a typekit from its name
            #
            # @param [String] name the typekit name
            # @raise [OroGen::NotFound] if the typekit cannot be found
            def typekit_from_name(name)
                if !(pkg = available_typekits[name])
                    raise OroGen::NotFound, "cannot find an installed typekit called #{name}"
                end
                registry = File.read(pkg.type_registry)
                typelist = File.join(File.dirname(pkg.type_registry), "#{name}.typelist")
                typelist = File.read(typelist)

                Spec::Typekit.from_raw_data(self, name, pkg, registry_xml, typelist_txt)
            end

            def update
                @available_projects = Hash.new
                @available_task_libraries = Hash.new
                @available_deployments = Hash.new
                @available_deployed_tasks = Hash.new
                @available_task_models = Hash.new
                @available_typekits = Hash.new
                @available_types = Hash.new

                Utilrb::PkgConfig.each_package(/^orogen-project-/) do |pkg_name|
                    pkg = Utilrb::PkgConfig.new(pkg_name)
                    add_project_from(pkg)
                end

                Utilrb::PkgConfig.each_package(/-tasks-#{orocos_target}$/) do |pkg_name|
                    pkg = Utilrb::PkgConfig.new(pkg_name)
                    tasklib_name = pkg_name.gsub(/-tasks-#{orocos_target}$/, '')

                    # Verify that the corresponding orogen project is indeed
                    # available. If not, just ignore the library
                    if !available_projects.has_key?(pkg.project_name)
                        OroGen.warn "found task library #{tasklib_name}, but the corresponding oroGen project #{pkg.project_name} could not be found. Consider deleting #{pkg.path}."
                        next
                    end

                    available_task_libraries[tasklib_name] = pkg
                    pkg.task_models.split(",").
                        each { |class_name| available_task_models[class_name] = tasklib_name }
                end

                Utilrb::PkgConfig.each_package(/^orogen-\w+$/) do |pkg_name|
                    pkg = Utilrb::PkgConfig.new(pkg_name)
                    deployment_name = pkg_name.gsub(/^orogen-/, '')

                    # Verify that the corresponding orogen project is indeed
                    # available. If not, just ignore the library
                    if !available_projects.has_key?(pkg.project_name)
                        OroGen.warn "found deployment #{deployment_name}, but the corresponding oroGen project #{pkg.project_name} could not be found. Consider deleting #{pkg.path}."
                        next
                    end

                    available_deployments[deployment_name] = pkg
                    pkg.deployed_tasks.split(',').each do |deployed_task_name|
                        available_deployed_tasks[deployed_task_name] ||= Set.new
                        available_deployed_tasks[deployed_task_name] << deployment_name
                    end
                end

                Utilrb::PkgConfig.each_package(/-typekit-#{orocos_target}$/) do |pkg_name|
                    pkg = Utilrb::PkgConfig.new(pkg_name)
                    typekit_name = pkg_name.gsub(/-typekit-#{orocos_target}$/, '')

                    if !available_projects.has_key?(pkg.project_name)
                        OroGen.warn "found typekit #{typekit_name}, but the corresponding oroGen project #{pkg.project_name} could not be found. Consider deleting #{pkg.path}."
                        next
                    elsif !available_projects[pkg.project_name].pkg.type_registry
                        OroGen.warn "found typekit #{typekit_name}, but the corresponding oroGen project #{pkg.project_name} does not have a typekit. Consider deleting #{pkg.path}."
                        next
                    end
                    available_typekits[typekit_name] = pkg

                    typelist = pkg.type_registry.gsub(/tlb$/, 'typelist')
                    typelist, typelist_exported =
                        OroGen::Spec::Typekit.parse_typelist(File.read(typelist))
                    typelist = typelist - typelist_exported
                    typelist.compact.each do |typename|
                        if existing = available_types[typename]
                            OroGen.info "#{typename} is defined by both #{existing[0]} and #{typekit_name}"
                        else
                            available_types[typename] = Type.new(typekit_name, false)
                        end
                    end
                    typelist_exported.compact.each do |typename|
                        if existing = available_types[typename]
                            OroGen.info "#{typename} is defined by both #{existing[0]} and #{typekit_name}"
                        end
                        available_types[typename] = Type.new(typekit_name, true)
                    end
                end
            end
        end
    end
end
