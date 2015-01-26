module OroGen
    module ROS
        class Loader < OroGen::Loaders::Base
            attr_reader :search_path
            attr_reader :packs

            # @return [String] the suffix of files that contain ROS package
            #   models
            attr_accessor :spec_file_suffix
            # @return [Hash<String,String>] mapping from a package name to its
            #   full path. This is used as both a cache for {ROS.rospack_find},
            #   and as a way to register paths for launchfiles on systems that
            #   do not have ROS installed
            attr_reader :package_paths
            # @return [Hash<String,String>] mapping from typelib type names to
            #   the corresponding ROS message name
            attr_reader :orogen_to_ros_mappings
            # @return [Hash<String,String>] mapping from ROS message names to
            #   typelib type names
            attr_reader :ros_to_orogen_mappings

            def initialize(root_loader = self)
                @spec_file_suffix = ".orogen"

                super(root_loader)

                root_loader.on_typekit_load do |tk|
                    load_rosmap_by_package_name(tk.name)
                end
            end

            def clear
                super
                @search_path = Array.new
                @packs = Array.new
                @package_paths = Hash.new
                @orogen_to_ros_mappings = Hash.new
                @ros_to_orogen_mappings = Hash.new
            end

            def to_s; "#<#{self.class.name} packs=#{packs.inspect} search_path=#{search_path.inspect}>" end

            def find_rosmap_by_package_name(name)
                OroGen::TypekitMarshallers::ROS.load_rosmap_by_package_name(name)
            rescue ArgumentError
            rescue Utilrb::PkgConfig::NotFound
                # Nothing installed, look into the pack_paths
                packs.each do |dir|
                    rosmap_path = File.join(dir, "#{name}.rosmap")
                    if File.file?(rosmap_path)
                        return OroGen::TypekitMarshallers::ROS.load_rosmap(rosmap_path)
                    end
                end
                nil
            end

            def load_rosmap_by_package_name(name)
                rosmaps = find_rosmap_by_package_name(name)
                return if !rosmaps

                rosmaps = [rosmaps, OroGen::TypekitMarshallers::ROS::DEFAULT_TYPE_TO_MSG]
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
                    file = File.join(dir, "#{name}#{spec_file_suffix}")
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

                ROS.info "loading model for ROS package #{name}"
                project = Spec::Package.new(root_loader, self)
                project.typekit = OroGen::Spec::Typekit.new(root_loader, name)

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
                    raise OroGen::ProjectNotFound, "could not find an oroGen model for the ROS package #{name} in #{search_path.inspect}"
                end
                return File.read(path), path
            end

            def typekit_model_text_from_name(name)
                raise OroGen::TypekitNotFound, "ROS packages define no typekits (was looking for #{name})"
            end

            # Manually registers the path to a package
            #
            # @return [void]
            def register_package_path(package_name, path)
                package_paths[package_name] = path
            end

            # Find the path of a ros package
            # @return [String] Path to the rospackage
            def rospack_find(package_name)
                if path = package_paths[package_name]
                    return path
                end

                # Look for it in the packs
                packs.each do |pack_dir|
                    pkg_dir = File.join(pack_dir, package_name)
                    if File.directory?(pkg_dir)
                        package_paths[package_name] = pkg_dir
                        return pkg_dir
                    end
                end

                package_path = (`rospack find #{package_name}` || '').strip
                if package_path.empty?
                    raise ArgumentError, "rospack cannot find package #{package_name}"
                end
                package_paths[package_name] = package_path
            end

            # Finds the path to a given ROS launch file
            # @return [String]
            def roslaunch_find(package_name, name)
                package_path = rospack_find(package_name)
                launchfile_name = "#{name}.launch"
                path = File.join(package_path, "launch", launchfile_name)
                alternative_path = File.join(package_path, launchfile_name)
                if File.file?(path)
                    return path
                elsif File.file?(alternative_path)
                    return alternative_path
                else raise ArgumentError, "package #{package_name} has no launch file #{launchfile_name} (looked for #{path} and #{alternative_path})"
                end
            end

            def map_message_type_to_orogen(message_type)
                orogen_types = find_all_types_for(message_type)
                if orogen_types.empty?
                    raise ArgumentError, "there are not oroGen equivalent for #{message_type}"
                end
                orogen_types.first
            end

            # Loads all known mappings from the oroGen types to the ROS messages.
            # Builds a reverse mapping as well
            def load_all_rosmaps(typekits)
                orogen_to_ros_mappings.clear
                ros_to_orogen_mappings.clear

                typekits.each do |name|
                    find_rosmap_by_package_name(name)
                end

                rosmaps.each do |rosmap|
                    orogen_to_ros_mappings.merge! rosmap
                    rosmap.each do |type_name, ros_name, _|
                        set = (ros_to_orogen_mappings[ros_name] ||= Set.new)
                        set << type_name
                    end
                end
                nil
            end

            # Get the list of oroGen types that can be used to communicate with a
            # given ROS message name
            #
            # At first call, it calls load_all_rosmaps to load all the known
            # mappings
            def find_all_types_for(message_name)
                ros_to_orogen_mappings[message_name] || Set.new
            end

            # Check if a given ROS message type can be accessed on this side
            #
            # At first call, it calls load_all_rosmaps to load all the known
            # mappings
            def compatible_message_type?(message_type)
                ros_to_orogen_mappings.has_key?(message_type)
            end
        end
    end
end
