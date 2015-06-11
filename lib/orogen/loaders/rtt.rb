module OroGen
    module Loaders
        module RTT
            DIR = File.join(File.expand_path(File.dirname(__FILE__)), 'rtt')
            STANDARD_PROJECT_SPECS = { "RTT" => DIR, "OCL" => DIR }
            STANDARD_TYPEKIT_SPECS = { "orocos" => DIR }
            def self.loader
                loader = Files.new
                STANDARD_PROJECT_SPECS.each do |name, dir|
                    loader.register_orogen_file(File.join(dir, "#{name}.orogen"), name)
                end
                STANDARD_TYPEKIT_SPECS.each do |name, dir|
                    loader.register_typekit(dir, name)
                end
                loader
            end

            def self.standard_projects
                if !@standard_projects
                    loader = self.loader
                    @standard_projects = STANDARD_PROJECT_SPECS.map do |name, dir|
                        loader.project_model_from_name(name)
                    end
                end
                @standard_projects
            end

            def self.standard_typekits
                if !@standard_typekits
                    loader = self.loader
                    @standard_typekits = STANDARD_TYPEKIT_SPECS.map do |name, _|
                        typekit = loader.typekit_model_from_name(name)
                        typekit.virtual = true
                        # Some fine-tuning :( Super-HACK
                        if name == 'orocos'
                            type = typekit.registry.create_container '/std/string', '/std/string'
                            type.metadata.set 'orogen_include', 'string'
                            typekit.registry.alias '/string', '/std/string'
                        end
                        typekit
                    end
                end
                return @standard_typekits
            end

            def self.setup_loader(loader)
                standard_typekits.each do |tk|
                    loader.register_typekit_model(tk)
                    # One additional step for us: register the types in
                    # tk.typelist manually. This is needed as we use the
                    # typelist to register non-normalized names
                    tk.typelist.each do |typename|
                        loader.typekits_by_type_name[typename] ||= Array.new
                        loader.typekits_by_type_name[typename] << tk
                    end
                end
                standard_projects.each do |proj|
                    loader.register_project_model(proj)
                end
            end
        end
    end
end

