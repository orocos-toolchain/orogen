module OroGen
    module Loaders
        class Files < Base
            attr_reader :available_projects
            attr_reader :available_typekits

            def initialize(root_loader = self)
                super
                @available_projects = Hash.new
                @available_typekits = Hash.new
            end

            def register_orogen_file(path, name = nil)
                name ||= File.basename(path, ".orogen")
                available_projects[name] = path
            end

            def project_model_text_from_name(name)
                path = available_projects[name]
                if !path
                    raise OroGen::NotFound, "no project called #{name} on #{self}"
                end

                return File.read(path), path
            end

            def register_typekit(path, name)
                typelist = File.join(path, "#{name}.typelist")
                tlb      = File.join(path, "#{name}.tlb")
                available_typekits[name] = [typelist, tlb]
            end

            def typekit_model_text_from_name(name)
                typelist, tlb = available_typekits[name]
                if !typelist
                    raise OroGen::NotFound, "#{self} has not typekit named #{name}"
                end
                return File.read(tlb), File.read(typelist)
            end

            def has_typekit?(name)
                available_typekits.has_key?(name)
            end
        end
    end
end
