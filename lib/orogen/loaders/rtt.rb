module OroGen
    module Loaders
        module RTT
            DIR = File.join(File.expand_path(File.dirname(__FILE__)), 'rtt')
            STANDARD_TASK_SPECS = { "rtt" => DIR, "ocl" => DIR }
            STANDARD_TYPEKIT_SPECS = { "orocos" => DIR }
            def self.loader
                loader = Files.new
                STANDARD_TASK_SPECS.each do |name, dir|
                    loader.register_orogen_file(File.join(dir, "#{name}.orogen"), name)
                end
                STANDARD_TYPEKIT_SPECS.each do |name, dir|
                    loader.register_typekit(dir, name)
                end
                loader
            end

            def self.standard_tasks
                if !@standard_tasks
                    loader = self.loader
                    STANDARD_TASK_SPECS.each do |name, dir|
                        loader.project_model_from_name(name)
                    end
                    @standard_tasks = loader.loaded_task_models
                end
                @standard_tasks
            end

            def self.standard_typekits
                if !@standard_typekits
                    loader = self.loader
                    @standard_typekits = STANDARD_TYPEKIT_SPECS.map do |name, _|
                        loader.typekit_model_from_name(name)
                    end
                end
                return @standard_typekits
            end

            def self.setup_loader(loader)
                standard_typekits.each do |tk|
                    loader.register_typekit_objects(tk)
                end
                loader.loaded_task_models.merge!(standard_tasks)
            end
        end
    end
end

