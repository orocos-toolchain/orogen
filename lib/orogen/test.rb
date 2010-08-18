require 'test/unit'
require 'fileutils'
require 'orogen'

module Orocos
    module Generation
        # This module includes common methods used by orogen's test suite. The
        # basic idea is to have a set of methods that allow:
        #  - each module to be built and installed in a separate directory
        #  - that working directory to be automatically removed at teardown
        #
        # Moreover, two environment variables allow to change the cleanup
        # behaviour.
        #
        # Normally, when a module build is requested, whatever files that are in
        # the module's working directory is deleted, and that same directory is
        # removed at teardown.
        #
        # If TEST_KEEP_WC is set, the module's working directory won't be
        # deleted at teardown.
        #
        # If TEST_DONT_CLEAN is set, the module's working directory is not
        # cleaned. It speeds up the testing process when the test files don't
        # change but orogen does change.
	module Test
	    include Orocos
	    include Orocos::Generation

	    TEST_DIR      = File.expand_path('../../test', File.dirname(__FILE__))
            TEST_DATA_DIR = File.join( TEST_DIR, 'data' )
            WC_ROOT  = File.join(TEST_DIR, 'wc')

	    attr_reader :working_directory

            def prefix_directory
                File.join(WC_ROOT, "prefix", *subdir)
            end

            attr_reader :subdir

	    def setup
                @subdir = Array.new
                @old_pkg_config = ENV['PKG_CONFIG_PATH'].dup if ENV['PKG_CONFIG_PATH']
		super if defined? super
	    end

	    def teardown
                clear_wc
                ENV['PKG_CONFIG_PATH'] = @old_pkg_config
		super if defined? super
	    end

	    def create_wc(*subdir)
                required_wc = File.join(TEST_DIR, 'wc', *subdir)
		if working_directory != required_wc
		    @working_directory = required_wc
		    FileUtils.mkdir_p working_directory
                    @subdir = subdir
		end
	    end

            def clear_wc
		unless ENV['TEST_KEEP_WC']
		    if File.directory?(WC_ROOT)
			FileUtils.rm_rf WC_ROOT
                        @working_directory = nil
		    end
		end
            end

	    def copy_in_wc(file, destination = nil)
		if destination
		    destination = File.expand_path(destination, working_directory)
		    FileUtils.mkdir_p destination
		end

		FileUtils.cp File.expand_path(file, TEST_DIR), (destination || working_directory)
	    end

	    def in_wc(*subdir, &block)
		Dir.chdir(File.join(working_directory, *subdir), &block)
	    end
            def install
                in_wc do
                    Dir.chdir("build") do
                        if !system("make", "install")
                            raise "failed to install"
                        end
                    end
                end
            end
            def in_prefix(&block)
                old_pkgconfig = ENV['PKG_CONFIG_PATH']
                in_wc do
                    Dir.chdir("build") do
                        if !system("make", "install")
                            raise "failed to install"
                        end
                    end

                    ENV['PKG_CONFIG_PATH'] += ":" + File.join(prefix_directory, 'lib', 'pkgconfig')
                    Dir.chdir(prefix_directory, &block)
                end
            ensure
                ENV['PKG_CONFIG_PATH'] = old_pkgconfig
            end

	    def compile_wc(component = nil, *subdir)
		in_wc(*subdir) do
                    if component
                        unless component.deffile
                            component.deffile = File.join(working_directory, "#{component.name}.orogen")
                        end
                        component.generate
                    end

		    yield if block_given?

		    FileUtils.mkdir('build') unless File.directory?('build')
		    Dir.chdir('build') do
                        make_cmd = ["make"]
                        if ENV['TEST_MAKE_OPTIONS']
                            make_cmd.concat(ENV['TEST_MAKE_OPTIONS'].split(','))
                        end
			if !system("cmake -DCMAKE_BUILD_TYPE=Debug -DCMAKE_INSTALL_PREFIX=#{prefix_directory} ..")
			    raise "failed to configure"
			elsif !system(*make_cmd)
			    raise "failed to build"
			end
		    end
		end
	    end

            def build_typegen(name, header_files, transports)
                @working_directory = File.join(TEST_DIR, 'wc', name)
                header_files = header_files.map do |file|
                    File.join(TEST_DATA_DIR, file)
                end

                if !ENV['TEST_DONT_CLEAN'] || !File.directory?(working_directory)
                    # Copy +dirname+ in place of wc
                    FileUtils.rm_rf working_directory
                    FileUtils.mkdir_p working_directory
                end

                in_wc do
                    FileUtils.mkdir_p "types"
                    header_files.each do |file|
                        FileUtils.cp file, "types"
                    end
                    cmdline = ["typegen"]
                    if !transports.empty?
                        cmdline << "--transports=#{transports.join(",")}"
                    end
                    cmdline << "-o" << "typekit_output" << name << "types"

                    if !system(*cmdline)
                        raise "typegen failed"
                    end
                end

                compile_wc(nil, "typekit_output")
            end

            def build_test_component(dirname, transports = [], test_bin = nil, wc_dirname = nil)
                source             = File.join(TEST_DATA_DIR, dirname)
                @working_directory = File.join(TEST_DIR, 'wc', wc_dirname || dirname)
                @subdir = [dirname]

                if !ENV['TEST_DONT_CLEAN'] || !File.directory?(working_directory)
                    # Copy +dirname+ in place of wc
                    FileUtils.rm_rf working_directory
                    FileUtils.mkdir_p File.dirname(working_directory)
                    FileUtils.cp_r source, working_directory
                end

                component = nil
                in_wc do
                    spec = Dir.glob("*.orogen").to_a.first
                    component = Component.load(spec)
                    component.enable_transports(*transports)

                    compile_wc(component) do
                        FileUtils.cp 'templates/CMakeLists.txt', 'CMakeLists.txt'
                        File.open('CMakeLists.txt', 'a') do |io|
                            yield(io) if block_given?
                        end
                    end
                end

                if test_bin
                    in_prefix do
                        assert(system(test_bin))
                    end
                end
                component
            end


            def compile_and_test(component, test_bin)
                compile_wc(component) do
                    FileUtils.cp 'templates/CMakeLists.txt', 'CMakeLists.txt'
                    File.open('CMakeLists.txt', 'a') do |io|
                        yield(io) if block_given?
                    end
                end

                in_prefix do
                    output = nil
                    assert(system(test_bin))
                end
            end
	end
    end
end

