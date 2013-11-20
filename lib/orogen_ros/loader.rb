module Orocos
    module ROS
        class Loader < OroGen::Loaders::Base
            attr_reader :search_path
            attr_reader :packs

            def initialize(root_loader = self)
                @search_path = Array.new
                @packs = Array.new

                super(root_loader)

                root_loader.on_typekit_load do |tk|
                    ROS.load_rosmap_by_package_name(tk.name)
                end
            end

            def find_rosmap_by_package_name(name)
                OroGen::TypekitMarshallers::ROS.load_rosmap_by_package_name(name)
            rescue ArgumentError
            rescue Utilrb::PkgConfig::NotFound
                # Nothing installed, look into the pack_paths
                packs.each do |dir|
                    rosmap_path = File.join(dir, "#{name}.rosmap")
                    if File.file?(rosmap_path)
                        return Orocos::TypekitMarshallers::ROS.load_rosmap(rosmap_path)
                    end
                end
                nil
            end

            def load_rosmap_by_package_name(name)
                rosmaps = find_rosmap_by_package_name(name)
                return if !rosmaps

                rosmaps = [rosmaps, Orocos::TypekitMarshallers::ROS::DEFAULT_TYPE_TO_MSG]
                rosmaps.each do |rosmap|
                    orogen_to_ros_mappings.merge! rosmap
                    rosmap.each do |type_name, ros_name, _|
                        set = (ros_to_orogen_mappings[ros_name] ||= Set.new)
                        set << type_name
                    end
                end
            end

            def find_project_file_from_name(name)
                search_path.each do |dir|
                    file = File.join(dir, "#{name}.#{ROS.spec_file_suffix}")
                    if File.file?(file)
                        return file
                    end
                end
                nil
            end

            def has_project?(name)
                !!find_project_file_from_name(name)
            end

            def has_typekit?(name)
                false
            end

            # Returns the project model corresponding to the given name
            #
            # @param [String] the project name
            # @raise [OroGen::NotFound] if there is no project with that
            #   name.
            # @return [OroGen::Spec::Project]
            def project_model_from_name(name)
                if project = loaded_projects[name]
                    return project
                end

                name = name.to_str

                text, path = project_model_text_from_name(name)

                OroGen.info "loading model for ROS package #{name}"
                project = Package.new(root_loader)
                project.typekit = Spec::Typekit.new(root_loader, name)

                # ROS packages don't have the same issue than oroGen, namely
                # there is no code generation so we don't need an intermediate
                # as oroGen needs
                project.__eval__(path, text)
                register_project_model(project)
                project
            end

            def project_model_text_from_name(name)
                path = find_project_file_from_name(name)
                if !path
                    raise OroGen::NotFound, "no oroGen model defined for the ROS package #{name}"
                end
                return File.read(path), path
            end

            def typekit_model_text_from_name(name)
                raise OroGen::NotFound, "ROS packages define no typekits (was looking for #{name})"
            end
        end
    end
end
