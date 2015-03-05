module OroGen
    module ROS
        OROGEN_ROS_LIB_DIR = File.expand_path(File.dirname(__FILE__))

        # Find the path of a rosnode binary
        # @return [String] Path to the rosnode binary
        def self.rosnode_find(package_name, binary_name)
            package_path = rospack_find(package_name)

            bin_path = File.join(package_path, 'bin', binary_name)
            if !File.file?(bin_path)
                raise ArgumentError, "there is no node called #{binary_name} in #{package_name} (looked in #{bin_path})"
            end
            bin_path
        end
       
        # List running ROS nodes using the ROS tooling
        # @return [Array] List of running ros nodes 
        def self.rosnode_list
            # Redirect error to stdout, since rosnode does not support
            # proper error reporting
            running_nodes = (`rosnode list 2>&1` || []).split("\n")

            # Handle ROS error message
            if running_nodes.size == 1 && running_nodes.first =~ /ERROR/
                raise InternalError, "cannot query node list. Master node is not available."
            end
            running_nodes
        end

        # Test whether a ROS node of the given name is running
        # @return [Bool] True if a ros node of given name is currenlty running
        def self.rosnode_running?(node_name)
            # making sure node name conform to 
            # pattern "/nodename"
            rosnode_list.include?(rosnode_normalize_name(node_name))
        end

        # Normalize the rosnode name
        # see #normalize_name
        # @return [String] (prefixed) node name
        def self.rosnode_normalize_name(node_name)
            normalize_name(node_name)
        end

        # Normalize the topic name (to match port names)
        # see #normalize_name
        # @return [String] (prefixed) node name
        def self.normalize_topic_name(topic_name)
            normalize_name(topic_name)
        end

        # Normalize the name, i.e. make sure the
        # name is(!) prefixed with a '/'
        def self.normalize_name(name)
             "/#{name}".sub(/^\/+/,'/')
        end

        # Locate the launch file in a given ros package 
        # @return [String] absolute path to the launch file
        # @throws [ArgumentError] if the launch file cannot be found in the ros package
        def self.roslaunch_find(package_name, launch_name)
            package_path = rospack_find(package_name)
            launch_path = File.join(package_path, "launch")

            launch_name = launch_name.gsub(/\.launch$/,"")
            launch_name += ".launch"

            launch_path = File.join(launch_path, launch_name)
            if !File.file?(launch_path)
                raise ArgumentError, "there is no launch_file called #{launch_name} in #{package_name} (looked in #{launch_path})"
            end
            launch_path
        end
    end
end
