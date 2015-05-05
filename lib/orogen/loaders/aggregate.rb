module OroGen
    module Loaders
        # A launcher that aggregates other launchers
        class Aggregate < Base
            class DuplicateLoader < ArgumentError; end

            # @return [Array]
            attr_reader :loaders

            def initialize(root_loader = self)
                @loaders = Array.new
                super(root_loader)
            end

            def added_child(loader)
                add(loader)
            end

            def clear
                super
                loaders.each do |l|
                    l.clear
                end
            end

            def add(loader)
                if loaders.include?(loader)
                    raise DuplicateLoader, "#{loader} is already a child of #{self}"
                end
                @loaders << loader
            end

            def remove(loader)
                @loaders.delete loader
            end

            def project_model_from_name(name)
                if project = loaded_projects[name]
                    return project
                end

                OroGen::Loaders.debug "Aggregate: resolving #{name} on #{loaders.map(&:to_s).join(",")}"
                loaders.each do |l|
                    begin
                        # We assume that the sub-loaders are created with self
                        # as root loader. They will therefore register
                        # themselves on self
                        return l.project_model_from_name(name)
                    rescue ProjectNotFound => e
                        Loaders.debug "  not available on #{l}: #{e}"
                    end
                end
                raise ProjectNotFound, "there is no project named #{name} on #{self}"
            end

            def find_task_library_from_task_model_name(name)
                loaders.each do |l|
                    next if !l.respond_to?(:find_task_library_from_task_model_name)
                    if tasklib_name = l.find_task_library_from_task_model_name(name)
                        return tasklib_name
                    end
                end
                nil
            end

            def find_project_from_deployment_name(name)
                loaders.each do |l|
                    next if !l.respond_to?(:find_project_from_deployment_name)
                    if project_name = l.find_project_from_deployment_name(name)
                        return project_name
                    end
                end
                nil
            end

            def find_deployments_from_deployed_task_name(name)
                loaders.each do |l|
                    next if !l.respond_to?(:find_deployments_from_deployed_task_name)
                    if deployment_name = l.find_deployments_from_deployed_task_name(name)
                        return deployment_name
                    end
                end
                nil
            end

            def typekit_model_from_name(name)
                if typekit = loaded_typekits[name]
                    return typekit
                end

                loaders.each do |l|
                    begin
                        # We assume that the sub-loaders are created with self
                        # as root loader. They will therefore register
                        # themselves on self
                        return l.typekit_model_from_name(name)
                    rescue TypekitNotFound
                    end
                end
                raise TypekitNotFound, "there is no typekit named #{name} on #{self}"
            end

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

                loaders.each do |l|
                    next if !l.respond_to?(:typekit_for)
                    begin
                        return l.typekit_for(type, exported)
                    rescue NotExportedType, NotTypekitType
                    end
                end
                raise NotTypekitType.new(type), "#{type} is not defined by any typekit known to #{self}"
            end

            def has_typekit?(name)
                loaders.any? { |l| l.has_typekit?(name) }
            end

            def has_project?(name)
                loaders.any? { |l| l.has_project?(name) }
            end

            # Enumerates the names of all available projects
            #
            # @yieldparam [String] project_name
            def each_available_project_name
                return enum_for(__method__) if !block_given?
                seen = Set.new
                loaders.each do |l|
                    l.each_available_project_name do |name|
                        if !seen.include?(name)
                            seen << name
                            yield(name)
                        end
                    end
                end
                nil
            end

            def deployment_model_from_name(name)
                if deployment = loaded_deployment_models[name]
                    return deployment
                end

                OroGen::Loaders.debug do
                    "Aggregate: resolving deployment #{name} on #{loaders.map(&:to_s).join(",")}"
                end

                loaders.each do |l|
                    begin
                        # We assume that the sub-loaders are created with self
                        # as root loader. They will therefore register
                        # themselves on self
                        return l.deployment_model_from_name(name)
                    rescue DeploymentModelNotFound => e
                        Loaders.debug "  not available on #{l}: #{e}"
                    end
                end
                raise DeploymentModelNotFound, "there is no deployment named #{name} on #{self}"
            end

            def to_s
                "#<#{self.class.name}(#{object_id.to_s(16)}): #{loaders.map(&:to_s).join(",")}>"
            end
        end
    end
end

