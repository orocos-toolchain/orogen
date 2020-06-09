module OroGen
    def self.each_orogen_plugin_path(&block)
        (ENV['OROGEN_PLUGIN_PATH'] || "").split(':').each(&block)
    end

    def self.each_orogen_plugin_dir
        each_orogen_plugin_path do |p|
            if File.directory?(p)
                yield(p)
            end
        end
    end

    def self.each_orogen_plugin_file(type)
        each_orogen_plugin_path do |path|
            if File.file?(path)
                yield(path)
            else
                Dir.glob(File.join(path, type, '*.rb')).each do |file|
                    yield(file)
                end
            end
        end
    end

    def self.load_orogen_plugin(*path)
        original_load_path = $LOAD_PATH.dup
        each_orogen_plugin_dir do |dir|
            $LOAD_PATH << dir
        end

        path = File.join(*path)
        if File.extname(path) != ".rb"
            path = "#{path}.rb"
        end

        each_orogen_plugin_dir do |dir|
            path = File.join(dir, path)
            if File.file?(path)
                info "loading plugin #{path}"
                require path
                return
            end
        end
        raise ArgumentError, "cannot load plugin #{path}: not found in #{ENV['OROGEN_PLUGIN_PATH']}"

    ensure
        if original_load_path
            $LOAD_PATH.clear
            $LOAD_PATH.concat(original_load_path)
        end
    end

    def self.load_orogen_plugins(*type)
        original_load_path = $LOAD_PATH.dup
        type = File.join(*type)
        each_orogen_plugin_dir do |dir|
            $LOAD_PATH << dir
        end
        each_orogen_plugin_file(type) do |file|
            info "loading plugin #{file}"
            begin
                require file
            rescue Exception => e
                warn "could not load plugin #{file}: #{e.message}"
                e.backtrace.each do |line|
                    warn "  #{line}"
                end
            end
        end
    ensure
        if original_load_path
            $LOAD_PATH.clear
            $LOAD_PATH.concat(original_load_path)
        end
    end

    # Load a separate typelib registry containing the types defined by the given
    # oroGen project
    def self.registry_of(typekit_name)
        registry = Typelib::Registry.new
        typekit_pkg =
            Utilrb::PkgConfig.new("#{typekit_name}-typekit-#{Gen::RTT_CPP.orocos_target}")

        tlb = typekit_pkg.type_registry
        if tlb
            registry.import(tlb)
        end

        registry
    end
end

