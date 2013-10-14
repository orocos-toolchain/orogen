require 'utilrb'

module Orocos
    module ROS

        @spec_file_suffix = ".orogen"
        @spec_search_directories = ["."]

        # This class wraps ros tooling and set some default configurations
        class << self
            attr_reader :spec_file_suffix
            attr_accessor :spec_search_directories

            # Find the path of a ros package
            # @return [String] Path to the rospackage
            def rospack_find(package_name)
                package_path = (`rospack find #{package_name}` || '').strip
                if package_path.empty?
                    raise ArgumentError, "rospack cannot find package #{package_name}"
                end
                package_path
            end

            # Find the path of a rosnode binary
            # @return [String] Path to the rosnode binary
            #
            def rosnode_find(package_name, binary_name)
                package_path = rospack_find(package_name)

                bin_path = File.join(package_path, 'bin', binary_name)
                if !File.file?(bin_path)
                    raise ArgumentError, "there is no node called #{binary_name} in #{package_name} (looked in #{bin_path})"
                end
                bin_path
            end
           
            # @return [Array] List of running ros nodes 
            def rosnode_list
                running_nodes = (`rosnode list` || []).split("\n")
            end

            # @return [Bool] True if a ros node of given name is currenlty running
            def rosnode_running?(node_name)
                 # making sure node name conform to 
                 # pattern "/nodename"
                 node_name = "/#{node_name}".sub(/^\/\//,'/')
                 rosnode_list.include?(node_name)
            end

            # Run the launch from the package +package_name+ given by +launch_name+
            # @return [Int] Pid of the roslaunch process
            def roslaunch(package_name, launch_name)
                args = ["#{package_name}", "#{launch_name}"]
                pid = Utilrb.spawn "roslaunch", "__name:=#{launch_name}", *args
                pid
            end

        end
    end
end
