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

        # Find the package of the rosnode with the given name.
        # Will only be useful after a call to Orocos::ROS#load
        # @return [Orocos::ROS::Spec::Project] The package (project) or nil if the package could not be found
        def self.rosnode_findpackage(name)
            # see description, a call to this function is only useful after call to #load
            return nil if !loaded?

            available_nodes.each do |node_name, node|
                if rosnode_normalize_name(name) == rosnode_normalize_name(node_name)
                    return node.project
                end
            end
            nil
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

        # Run the launch from the package +package_name+ given by +launch_name+
        # @options [Hash] Options are forwarded to Utilrb.spawn, e.g.
        #     :working_directory
        #     :nice
        #     :redirect
        # @return [Int] Pid of the roslaunch process
        def self.roslaunch(package_name, launch_name, options = Hash.new)
            launch_name = launch_name.gsub(/\.launch/,'')
            launch_name = launch_name + ".launch"
            arguments = [package_name, launch_name]
            arguments += [options]

            pid = Utilrb.spawn "roslaunch", "__name:=#{launch_name}", *arguments
            pid
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

        # Get the roscore process id
        # @return [Int] Pid of the roscore process, if it has been started by this Ruby process,
        #     false otherwise
        def self.roscore_pid
            @roscore_pid || 0
        end

        # Test whether roscore is available or not
        # @return [Boolean] True if roscore is available, false otherwise
        def self.roscore_available?
            begin
                !rosnode_list.empty?
            rescue InternalError => e
                false
            end
        end

        # Start the roscore process
        # @return[INT] Pid of the roscore process see #roscore_pid
        def self.roscore_start(*args)
            options = args.last.kind_of?(Hash) ? args.pop : Hash.new
            options, unknown_options = Kernel.filter_options options,
                :redirect => File.join("/var/tmp/roscore.log")

            args << options

            if !roscore_available?
                @roscore_pid = Utilrb.spawn "roscore", *args
                ::Process.detach(@roscore_pid)
                @roscore_pid
            elsif !@roscore_pid
                warn "roscore is already running, but is not controlled by this process"
            else
                info "roscore is already running, pid '#{@roscore_pid}'"
            end

            if unknown_options[:wait]
                while !roscore_available?
                    sleep 0.1
                end
            end
        end

        # Shutdown roscore if controlled by this process, otherwise
        # calls to this function will return false
        # This will only work if roscore has been started by the same ruby process
        # @throw [ArgumentError] if trying to shutdown an already dead roscore
        # @return [Boolean] True if roscore has been shutdown, false if not
        def self.roscore_shutdown
            begin
                if @roscore_pid
                    info "roscore will be shutdown"
                    status = ::Process.kill('INT',@roscore_pid)
                    @roscore_pid = nil
                    return status
                end
            rescue Errno::ESRCH
                raise ArgumentError, "trying to shutdown roscore, which is not running anymore with pid '#{@roscore_pid}'"
            end

            warn "roscore is not controlled by this process; no shutdown will be performed"
            false
        end

        # @return [String] the type name that should be used on the oroGen
        #   side to represent the given ROS message
        #
        # @param [String] message_type the ROS message type name
        def self.map_message_type_to_orogen(message_type)
            default_loader.map_message_type_to_orogen(message_type)
        end

        def self.default_loader
            if !@default_loader
                loader = Loader.new(Orocos.default_loader)
                loader.search_path << Orocos::ROS::OROGEN_ROS_LIB_DIR
                loader.project_model_from_name 'ros'
                @default_loader = loader
            end
            @default_loader
        end

        # Helper method for initialize
        def self.load
            @loaded = true
        end

        def self.loaded?; !!@loaded end
        
        def self.clear
            @default_loader = nil
            @loaded = false
        end
    end
end
