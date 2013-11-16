module OroGen
    module Loaders
        # Definition of the base loader API
        class Base
            # Set of projects loaded so far
            #
            # @return [Hash<String,Spec::Project>]
            attr_reader :loaded_projects

            # Set of task models that are known to us
            attr_reader :loaded_task_models

            # Set of typekits loaded so far
            #
            # @return [Hash<String,Spec::Typekit>]
            attr_reader :loaded_typekits

            # The loader that should be used to resolve dependencies
            attr_reader :root_loader

            def initialize(root_loader)
                @loaded_projects = Hash.new
                @loaded_typekits = Hash.new
                @loaded_task_models = Hash.new
                @root_loader = root_loader
            end

            # Returns the project model corresponding to the given name
            #
            # @param [String] the project name
            # @option options [Boolean] :define_dummy_types (false) if true,
            #   unknown types will be automatically created as null types on the
            #   underlying typekit. If false, they will generate a
            #   Typekit::NotFound exception
            # @raise [OroGen::NotFound] if there is no project with that
            #   name.
            # @return [OroGen::Spec::Project]
            def project_model_from_name(name, options = Hash.new)
                if project = loaded_projects[name]
                    return project
                end

                name = name.to_str
                options = Kernel.validate_options options,
                    :define_dummy_types => false

                text, path = project_model_text_from_name(name)

                OroGen.info "loading oroGen project #{name}"
                project = Spec::Project.new(root_loader)
                if has_typekit?(name)
                    project.typekit = typekit_from_name(name)
                else project.typekit = Spec::Typekit.new(name)
                end
                project.typekit.define_dummy_types = options[:define_dummy_types]
                Loaders::Project.new(project).__eval__(text, path)
                register_project_info(project)
                loaded_projects[name] = project
            end

            # Returns the task library model corresponding to the given name
            # @param (see project_model_from_name)
            # @raise [OroGen::NotFound] if there is no task library with that
            #   name. This does including having a project with that name if the
            #   project defines no tasks.
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
            # @param [String] name the task model name
            # @return [Spec::TaskContext]
            # @raise [OroGen::NotFound] if there are no such model
            def task_model_from_name(name)
                tasklib_name = find_task_library_from_task_model_name(name)
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
            # @param [String] name the deployment name
            # @return [OroGen::Spec::Deployment] the deployment model
            # @raise [OroGen::NotFound] if no deployment with that name exists
            def deployment_model_from_name(name)
                pkg = find_project_from_deployment_name(name)
                if !pkg
                    raise OroGen::NotFound, "there is no deployment called #{name}"
                end

                project = project_model_from_name(pkg.project_name)
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
                    deployment_names = find_deployments_from_deployed_task_name(name)
                    if deployment_names.empty?
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
            # @return [Spec::Typekit] the typekit
            # @raise [OroGen::NotFound] if the typekit cannot be found
            def typekit_from_name(name)
                if typekit = loaded_typekits[name]
                    return typekit
                end

                registry_xml, typelist_txt = typekit_model_text_from_name(name)
                loaded_typekits[name] = Spec::Typekit.from_raw_data(name, registry_xml, typelist_txt)
            end

            # Registers this project's subobjects
            def register_project_info(project)
                loaded_task_models.merge! project.tasks
            end

            def project_model_text_from_name(name)
                raise NotImplementedError
            end

            def typekit_model_text_from_name(name)
                raise NotImplementedError
            end

            def has_typekit?(name)
                raise NotImplementedError
            end

            def find_task_library_from_task_model_name(name)
                raise NotImplementedError
            end

            def find_project_from_deployment_name(name)
                raise NotImplementedError
            end

            def find_deployments_from_deployed_task_name(name)
                raise NotImplementedError
            end
        end
    end
end
