module OroGen
    module Loaders
        class Files < Base
            attr_reader :available_projects
            attr_reader :available_typekits

            def initialize(root_loader = self)
                @available_projects = Hash.new
                @available_typekits = Hash.new
                super
            end

            def clear
                @available_projects.clear
                @available_typekits.clear
                super
            end

            def register_orogen_file(path, name = nil)
                name ||= File.basename(path, ".orogen")
                available_projects[name] = path
            end

            def project_model_text_from_name(name)
                path = available_projects[name]
                if !path
                    raise ProjectNotFound, "no project called #{name} on #{self}"
                end

                return File.read(path), path
            end

            def has_project?(name)
                available_projects.has_key?(name)
            end

            def has_typekit?(name)
                available_typekits.has_key?(name)
            end

            def register_typekit(path, name)
                typelist = File.join(path, "#{name}.typelist")
                tlb      = File.join(path, "#{name}.tlb")
                available_typekits[name] = [typelist, tlb]
            end

            def typekit_model_text_from_name(name)
                typelist, tlb = available_typekits[name]
                if !typelist
                    raise TypekitNotFound, "#{self} has not typekit named #{name}"
                end
                return File.read(tlb), File.read(typelist)
            end

            def deployment_model_from_name(name)
                if model = loaded_deployment_models[name]
                    return model
                end

                each_project do |project|
                    if m = project.find_deployment_by_name(name)
                        loaded_deployment_models[name] = m
                        return m
                    end
                end
                raise DeploymentModelNotFound, "there is no deployment called #{name} on #{self}"
            end

            def has_typekit?(name)
                available_typekits.has_key?(name)
            end

            def to_s;
                "#<OroGen::Loaders::Files(#{object_id.to_s(16)}) projects=#{available_projects.keys.sort.join(",")} typekits=#{available_typekits.keys.sort.join(",")}>"
            end

            def each_project
                return enum_for(__method__) if !block_given?

                available_projects.each_key do |project_name|
                    project = begin
                                  root_loader.project_model_from_name(project_name)
                              rescue Exception => e
                                  OroGen.warn "could not load #{project_name}: #{e.message}"
                                  next
                              end

                    yield(project)
                end
                nil
            end

            def find_project_from_deployment_name(name)
                each_project do |project|
                    if project.find_deployment_by_name(name)
                        return project.name
                    end
                end
                nil
            end

            def each_available_project_name(&block)
                return available_projects.each_key(&block)
            end

            def each_available_typekit_name(&block)
                return available_typekits.each_key(&block)
            end

            def each_available_deployment_name(&block)
                return enum_for(__method__) if !block_given?
                each_project do |project|
                    project.each_deployment do |spec|
                        yield(spec.name)
                    end
                end
                nil
            end
        end
    end
end

