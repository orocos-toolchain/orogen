# frozen_string_literal: true

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

            # Set of deployment models that are known to us
            attr_reader :loaded_deployment_models

            # The registry that includes types from all loaded typekits
            attr_reader :registry

            # The list of types that can be used on an oroGen interface
            attr_reader :interface_typelist

            # A mapping from type names to the typekits that define them
            attr_reader :typekits_by_type_name

            # Set of typekits loaded so far
            #
            # @return [Hash<String,Spec::Typekit>]
            attr_reader :loaded_typekits

            # The loader that should be used to resolve dependencies
            attr_reader :root_loader

            # Sets the behaviour of the type resolution on unknown types
            #
            # @return [Boolean]
            attr_predicate :define_dummy_types?, true

            # Set of callbacks that are called whenever a new typekit gets
            # loaded
            #
            # @return [Array<#call>]
            attr_reader :typekit_load_callbacks

            # Set of callbacks that are called whenever a new typekit gets
            # loaded
            #
            # @return [Array<#call>]
            attr_reader :project_load_callbacks

            def initialize(root_loader = self)
                @root_loader = root_loader || self
                root_loader.added_child(self) if root_loader != self
                @typekit_load_callbacks = []
                @project_load_callbacks = []
                clear
            end

            def clear
                @loaded_projects = {}
                @loaded_typekits = {}
                @loaded_task_models = {}
                @loaded_deployment_models = {}
                @typekits_by_type_name = {}
                @registry = Typelib::Registry.new
                @interface_typelist = Set.new
            end

            def added_child(loader); end

            # Returns the project model corresponding to the given name
            #
            # @param [String] the project name
            # @raise [ProjectNotFound] if there is no project with that
            #   name.
            # @return [OroGen::Spec::Project]
            def project_model_from_name(name)
                if (project = loaded_projects[name])
                    return project
                elsif root_loader.has_loaded_project?(name)
                    loaded_projects[name] = project =
                        root_loader.project_model_from_name(name)
                    return project
                end

                name = name.to_str

                text, path = project_model_text_from_name(name)
                OroGen.info "loading oroGen project #{name}"
                project_model_from_text(text, name: name, path: path)
            end

            def project_model_from_text(text, name: nil, path: nil)
                project = Spec::Project.new(root_loader)
                project.typekit =
                    if has_typekit?(name)
                        typekit_model_from_name(name)
                    else
                        Spec::Typekit.new(root_loader, name)
                    end

                Loaders::Project.new(project).__eval__(path, text)
                if name && (project.name != name)
                    raise ArgumentError,
                          "got project #{project.name} while loading #{name}"
                end

                register_project_model(project)
                project
            end

            # Registers a callback that should be called with newly registered
            # projects
            #
            # @param [Boolean] initial_events if true, the callbacks will be
            #   called instantly with the projects that have already been loaded
            def on_project_load(initial_events = true, &block)
                project_load_callbacks << block
                if initial_events
                    current_set = loaded_projects.values.dup
                    current_set.each do |p|
                        block.call(p)
                    end
                end
                block
            end

            # Removes the given callback from the listeners to {on_project_load}
            #
            # @param [Object] callback the value returned by {on_project_load}
            #   for the callback that should be removed
            def remove_project_load_callback(callback)
                project_load_callbacks.delete(callback)
            end

            # Returns the task library model corresponding to the given name
            # @param (see project_model_from_name)
            # @raise [ProjectNotFound] if there is no task library with that
            #   name. This does including having a project with that name if the
            #   project defines no tasks.
            # @return (see project_model_from_name)
            def task_library_model_from_name(name)
                project = project_model_from_name(name)
                if project.self_tasks.empty?
                    raise ProjectNotFound,
                          "there is an oroGen project called #{name}, "\
                          "but it defines no tasks"
                end
                project
            end

            # Returns the task model object corresponding to a model name
            #
            # @param [String] name the task model name
            # @return [Spec::TaskContext]
            # @raise [TaskModelNotFound] if there are no such model
            # @raise (see project_model_from_name)
            def task_model_from_name(name)
                if (model = loaded_task_models[name])
                    return model
                end

                tasklib_name = find_task_library_from_task_model_name(name)
                unless tasklib_name
                    raise TaskModelNotFound, "no task model #{name} is registered"
                end

                tasklib = project_model_from_name(tasklib_name)
                result = tasklib.tasks[name]
                unless result
                    raise InternalError,
                          "while looking up model of #{name}: found project "\
                          "#{tasklib_name}, but this project does not actually "\
                          "have a task model called #{name}"
                end

                result
            end

            # Returns the deployment model for the given deployment name
            #
            # @param [String] name the deployment name
            # @return [OroGen::Spec::Deployment] the deployment model
            # @raise [DeploymentModelNotFound] if no deployment with that name exists
            def deployment_model_from_name(name)
                if (model = loaded_deployment_models[name])
                    return model
                end

                project_name = find_project_from_deployment_name(name)
                unless project_name
                    raise DeploymentModelNotFound,
                          "there is no deployment called #{name} on #{self}"
                end

                project = project_model_from_name(project_name)
                deployment = project.deployers[name]
                unless deployment
                    raise InternalError,
                          "cannot find the deployment called #{name} "\
                          "in #{project.name}. Candidates were "\
                          "#{project.deployers.map(&:name).sort.join(', ')}"
                end
                deployment
            end

            # Returns the deployed task model for the given name
            #
            # @param [String] name the deployed task name
            # @param [String] deployment_name () the name of the deployment in
            #   which the task is defined. It must be given only when more than
            #   one deployment defines a task with the requested name
            # @return [OroGen::Spec::TaskDeployment] the deployed task model
            # @raise [DeployedTaskModelNotFound] if no deployed tasks with that
            #   name exists
            # @raise [DeployedTaskModelNotFound] if deployment_name was given,
            #   but the requested task is not defined in this deployment
            # @raise [OroGen::AmbiguousName] if more than one task exists with
            #   That name. In that case, you will have to provide the deployment
            #   Name explicitly using the second argument
            def deployed_task_model_from_name(name, deployment_name = nil)
                if deployment_name
                    deployment = deployment_model_from_name(deployment_name)
                else
                    deployment_names = find_deployments_from_deployed_task_name(name)
                    if deployment_names.empty?
                        raise DeployedTaskModelNotFound,
                              "cannot find a deployed task called #{name}"
                    elsif deployment_names.size > 1
                        raise AmbiguousName,
                              "more than one deployment defines a deployed task called "\
                              "#{name}: #{deployment_names.map(&:name).sort.join(', ')}"
                    end
                    deployment = deployment_model_from_name(deployment_names.first)
                end

                unless (task = deployment.find_task_by_name(name))
                    if deployment_name
                        raise DeployedTaskModelNotFound,
                              "deployment #{deployment_name} does not have a task "\
                              "called #{name}"
                    else
                        raise InternalError,
                              "deployment #{deployment_name} was supposed to have "\
                              "a task called #{name} but does not"
                    end
                end
                task
            end

            # Loads a typekit from its name
            #
            # @param [String] name the typekit name
            # @return [Spec::Typekit] the typekit
            # @raise [TypekitNotFound] if the typekit cannot be found
            def typekit_model_from_name(name)
                if (typekit = loaded_typekits[name])
                    return typekit
                elsif root_loader.has_loaded_typekit?(name)
                    loaded_typekits[name] = typekit =
                        root_loader.typekit_model_from_name(name)
                    return typekit
                end

                registry_xml, typelist_txt = typekit_model_text_from_name(name)
                typekit = Spec::Typekit.from_raw_data(
                    root_loader, name, registry_xml, typelist_txt
                )
                if typekit.name != name
                    raise InternalError,
                          "inconsistency: got typekit #{typekit.name} while "\
                          "loading #{name}"
                end

                register_typekit_model(typekit)
                typekit
            end

            # Registers information from this typekit
            #
            # Callbacks registered by {#on_typekit_load} gets called with the
            # new typekit as argument
            def register_typekit_model(typekit)
                if loaded_typekits.key?(typekit.name)
                    raise AlreadyRegistered,
                          "there is already a typekit called #{typekit.name} "\
                          "registered on #{self}"
                end

                loaded_typekits[typekit.name] = typekit
                return root_loader.register_typekit_model(typekit) if root_loader != self

                registry.merge typekit.registry

                typekit.registry.each(with_aliases: false) do |type|
                    self_type = registry.get(type.name)
                    self_type.metadata.add("orogen:typekits", typekit.name)
                    if typekit.include?(type.name)
                        self_type.metadata.add("orogen:definition_typekits", typekit.name)
                    end

                    if type.contains_opaques?
                        intermediate_type_name = typekit.intermediate_type_name_for(type)
                        intermediate_type = registry.get(intermediate_type_name)
                        self_type
                            .metadata
                            .add("orogen:intermediate_type", intermediate_type_name)
                        intermediate_type
                            .metadata.add("orogen:intermediate_type_of", type.name)
                        unless type.opaque?
                            intermediate_type
                                .metadata.set("orogen:generated_type", "true")
                        end
                    end
                end

                @interface_typelist |= typekit.interface_typelist
                typekit.registry.each(with_aliases: true) do |typename, _|
                    typekits_by_type_name[typename] ||= Set.new
                    typekits_by_type_name[typename] << typekit
                end
                typekit_load_callbacks.each do |callback|
                    callback.call(typekit)
                end
            end

            def register_type_model(type, interface = true)
                registry.merge type.registry.minimal(type.name)
                interface_typelist << type.name if interface
            end

            # Registers a callback that should be called with newly registered
            # typekits
            #
            # @param [Boolean] initial_events if true, the callbacks will be
            #   called instantly with the typekits that have already been loaded
            def on_typekit_load(initial_events = true, &block)
                typekit_load_callbacks << block
                return unless initial_events

                current_set = loaded_typekits.values.dup
                current_set.each do |tk|
                    block.call(tk)
                end
            end

            # Resolves a type object
            #
            # @param [#name,String] type the type to be resolved
            # @return [Model<Typelib::Type>] the corresponding type in
            #   {#registry}
            # @raise Typelib::NotFound if the type cannot be found
            def resolve_type(type, options = {})
                typename =
                    if type.respond_to?(:name)
                        type.name
                    else type
                    end
                registry.get(typename)
            rescue Typelib::NotFound => e
                unless define_dummy_types? || options[:define_dummy_type]
                    raise e, "#{e.message} using #{self}", e.backtrace
                end

                type = registry.create_null(typename)
                register_type_model(type, interface: true)
                type
            end

            # Returns the typekit object that defines this type
            #
            # @option options [Boolean] :definition_typekits (true) if true,
            #   only the typekits that actually have the type in their typelist
            #   are returned. Otherwise, every typekit that have it in their
            #   registry are returned.
            #
            # @return [Set<Spec::Typekit>] the list of typekits
            # @raise [DefinitionTypekitNotFound] if no typekits define this type
            def imported_typekits_for(typename, definition_typekits: true)
                typename = typename.name if typename.respond_to?(:name)
                unless (typekits = typekits_by_type_name[typename])
                    raise DefinitionTypekitNotFound,
                          "#{typename} is not defined by any typekits loaded so far"
                end

                return typekits unless definition_typekits

                definition_typekits = typekits.find_all { |tk| tk.include?(typename) }
                if definition_typekits.empty?
                    raise DefinitionTypekitNotFound,
                          "typekits #{typekits.map(&:name).sort.join(', ')} have "\
                          "#{typename} in their registries, but it seems that they "\
                          "got it from another typekit that cannot be found"
                end
                definition_typekits.to_set
            end

            # Returns the type object for +typename+, validating that we can use
            # it in a task interface, i.e. that it will be registered in the
            # RTT's typeinfo system
            def resolve_interface_type(typename)
                type = resolve_type(typename)
                if type < Typelib::ArrayType
                    raise InvalidInterfaceType.new(type),
                          "static arrays are not valid interface types. "\
                          "Use an array in a structure or a std::vector"
                elsif !interface_type?(type)
                    typekits = imported_typekits_for(type.name)
                    raise NotExportedType.new(type, typekits),
                          "#{type.name}, defined in the "\
                          "#{typekits.map(&:name).join(', ')} typekits, is never exported"
                end
                type
            end

            # Tests whether the given type can be used on an interface
            #
            # @param [#name,String] typename the type
            # @return [Boolean]
            def interface_type?(typename)
                typename = typename.name if typename.respond_to?(:name)
                interface_typelist.include?(typename)
            end

            # Tests whether the given type can be used on an interface
            #
            # @param (see Spec::Typekit#intermediate_type?)
            # @return (see Spec::Typekit#intermediate_type?)
            def intermediate_type?(type)
                type = resolve_type(type)
                !type.metadata.get("orogen:intermediate_type_of").empty?
            end

            # Returns the opaque type that is paired with the given type
            #
            # @param (see Spec::Typekit#opaque_type_for)
            # @raise (see Spec::Typekit#opaque_type_for)
            # @return (see Spec::Typekit#opaque_type_for)
            def opaque_type_for(type)
                type = resolve_type(type)
                opaques = type.metadata.get("orogen:intermediate_type_of")
                registry.get(opaques.first || type.name)
            end

            # Returns the intermediate type that is paired with the given type
            #
            # @param (see Spec::Typekit#opaque_type_for)
            # @raise (see Spec::Typekit#opaque_type_for)
            # @return (see Spec::Typekit#opaque_type_for)
            def intermediate_type_for(type)
                type = resolve_type(type)
                intermediates = type.metadata.get("orogen:intermediate_type")
                registry.get(intermediates.first || type.name)
            end

            # Returns whether this type is a m-type (intermediate type generated
            # by oroGen)
            #
            # @param (see Spec::Typekit#m_type?)
            # @raise (see Spec::Typekit#m_type?)
            # @return (see Spec::Typekit#m_type?)
            def m_type?(type)
                type = resolve_type(type)
                type.metadata.get("orogen:generated_type") == ["true"]
            end

            # Registers this project's subobjects
            def register_project_model(project)
                if loaded_projects.key?(project.name)
                    raise AlreadyRegistered,
                          "there is already a project called #{project.name} "\
                          "registered on #{self}"
                end

                loaded_projects[project.name] = project
                return root_loader.register_project_model(project) if root_loader != self

                project.tasks.each do |_, task_model|
                    register_task_context_model(task_model)
                end
                project.deployers.each do |_, deployer_model|
                    register_deployment_model(deployer_model)
                end
                project_load_callbacks.each do |callback|
                    callback.call(project)
                end
            end

            def has_loaded_project?(name)
                loaded_projects.key?(name)
            end

            # Registers a new task model
            #
            # @param [Spec::TaskContext] model
            # @return [void]
            def register_task_context_model(model)
                loaded_task_models[model.name] = model
            end

            # Registers a new deployment model
            #
            # @param [Spec::Deployment] model
            # @return [void]
            def register_deployment_model(model)
                loaded_deployment_models[model.name] = model
            end

            # Returns the textual representation of a project model
            #
            # @param [String] the project name
            # @raise [ProjectNotFound] if there is no project with that
            #   name.
            # @return [(String,String)] the model as text, as well as a path to
            #   the model file (or nil if there is no such file)
            def project_model_text_from_name(_name)
                raise NotImplementedError
            end

            # Returns the textual representation of a typekit
            #
            # @param [String] the typekit name
            # @raise [TypekitNotFound] if there is no typekit with that name
            # @return [(String,String)] the typekit registry as XML and the
            #   typekit's typelist
            def typekit_model_text_from_name(_name)
                raise NotImplementedError
            end

            # Tests if a project with that name exists
            #
            # @param [String] name the project name
            # @return [Boolean]
            def has_project?(name)
                loaded_projects.key?(name)
            end

            # Tests if a typekit with that name has been loaded
            #
            # @param [String] name the typekit name
            # @return [Boolean]
            def has_loaded_typekit?(name)
                loaded_typekits.key?(name)
            end

            # Tests if a typekit with that name exists
            #
            # @param [String] name the typekit name
            # @return [Boolean]
            def has_typekit?(name)
                loaded_typekits.key?(name)
            end

            # Returns the task library name in which a task model is defined
            #
            # @param [String] model_name the name of the task model to look for
            # @return [String,nil]
            def find_task_library_from_task_model_name(name)
                if (m = /^(\w+)::/.match(name))
                    return m[1]
                end

                raise ArgumentError,
                      "#{self.class} uses the default name-based resolution to "\
                      "resolve the task library from the task name '#{name}', but "\
                      "'#{name}' does not follow the expected convention "\
                      "${project_name}::${task_name}"
            end

            # Returns the project that defines the given deployment
            #
            # @param [String] deployment_name the deployment we are looking for
            # @return [String,nil]
            def find_project_from_deployment_name(name); end

            # Returns the set of deployments that contain a certain task
            #
            # @param [String] name
            # @return [Set<String>]
            def find_deployments_from_deployed_task_name(name); end

            # Enumerates the names of all available projects
            #
            # @yieldparam [String] project_name
            def each_available_project_name
                return enum_for(__method__) unless block_given?

                nil
            end

            def typelib_type_for(t)
                if t.respond_to?(:name)
                    return t unless t.contains_opaques?

                    t = t.name
                end

                unless registry.include?(t)
                    raise Typelib::NotFound,
                          "#{t} cannot be found in the currently loaded registries"
                end

                type = registry.get(t)
                if type.contains_opaques?
                    intermediate_type_for(type)
                elsif type.null?
                    # 't' is an opaque type and there are no typelib marshallers
                    # to convert it to something we can manipulate, raise
                    raise Typelib::NotFound,
                          "#{t} is a null type and there are no typelib marshallers "\
                          "registered in RTT to convert it to a typelib-compatible type"
                else type
                end
            end

            def inspect
                to_s
            end
        end
    end
end
