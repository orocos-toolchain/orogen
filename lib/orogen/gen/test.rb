require 'orogen/test'
require 'orogen/gen'
require 'fileutils'

module OroGen
    module Gen
    module RTT_CPP
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
	module SelfTest
            include OroGen::Gen
            include OroGen::Gen::RTT_CPP

	    attr_reader :working_directory

            def prefix_directory
                File.join(path_to_wc_root, "prefix", *subdir)
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
                Utilrb::PkgConfig.clear_cache
		super if defined? super
                flexmock_teardown
	    end

	    def create_wc(*subdir)
                required_wc = File.join(path_to_test, 'wc', *subdir)
		if working_directory != required_wc
		    @working_directory = required_wc
		    FileUtils.mkdir_p working_directory
                    @subdir = subdir
		end
	    end

            def clear_wc
		if ENV['TEST_KEEP_WC'] != '1' && ENV['TEST_DONT_CLEAN'] != '1'
		    if File.directory?(path_to_wc_root)
			FileUtils.rm_rf path_to_wc_root
                        @working_directory = nil
		    end
		end
            end

	    def copy_in_wc(file, destination = nil)
		if destination
		    destination = File.expand_path(destination, working_directory)
		    FileUtils.mkdir_p destination
		end

		FileUtils.cp File.expand_path(file, path_to_test), (destination || working_directory)
	    end

	    def in_wc(*subdir, &block)
		Dir.chdir(File.join(working_directory, *subdir), &block)
	    end

            def redirect_to_logfile
                Hash[[STDOUT,STDERR] => [logfile, 'a']]
            end

            def install
                in_wc do
                    Dir.chdir("build") do
                        assert(call_make('install'), "failed to install, see #{logfile} for more details")
                    end
                end
            end
            def in_prefix(&block)
                old_pkgconfig = ENV['PKG_CONFIG_PATH']
                in_wc do
                    Dir.chdir("build") do
                        assert(call_make('install'), "failed to install, see #{logfile} for more details")
                    end

                    ENV['PKG_CONFIG_PATH'] += ":" + File.join(prefix_directory, 'lib', 'pkgconfig')
                    Dir.chdir(prefix_directory, &block)
                end
            ensure
                ENV['PKG_CONFIG_PATH'] = old_pkgconfig
            end

            def logfile
                File.join(working_directory, 'test.log')
            end

	    def compile_wc(project = nil, *subdir)
		in_wc(*subdir) do
                    if project
                        unless project.deffile
                            project.deffile = File.join(working_directory, "#{project.name}.orogen")
                        end
                        project.generate
                    end

		    yield if block_given?

		    FileUtils.mkdir('build') unless File.directory?('build')
		    Dir.chdir('build') do
			assert(system("cmake -DCMAKE_BUILD_TYPE=Debug -DCMAKE_INSTALL_PREFIX=#{prefix_directory} ..", redirect_to_logfile), "failed to configure, see #{logfile} for more details")
                        assert(call_make, "failed to build, see #{logfile} for more details")
		    end
		end
	    end

            def call_make(*args)
                make_cmd = ["make"]
                if ENV['TEST_MAKE_OPTIONS']
                    make_cmd.concat(ENV['TEST_MAKE_OPTIONS'].split(','))
                end
                system(*make_cmd, *args, redirect_to_logfile)
            end

            def build_typegen(name, header_files, transports)
                @working_directory = File.join(path_to_test, 'wc', name)
                header_files = [*header_files].map do |file|
                    File.join(path_to_data, file)
                end

                if ENV['TEST_DONT_CLEAN'] != '1' || !File.directory?(working_directory)
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

                    assert(system(*cmdline, redirect_to_logfile), "typegen failed, log file in #{logfile}")
                end

                compile_wc(nil, "typekit_output")
            end

            def build_test_project(dirname, transports = [], test_bin = nil, wc_dirname = nil)
                source             = File.join(path_to_data, dirname)
                @working_directory = File.join(path_to_test, 'wc', wc_dirname || dirname)
                @subdir = [dirname]

                if ENV['TEST_DONT_CLEAN'] != '1' || !File.directory?(working_directory)
                    # Copy +dirname+ in place of wc
                    FileUtils.rm_rf working_directory
                    FileUtils.mkdir_p File.dirname(working_directory)
                    FileUtils.cp_r source, working_directory
                end

                project = nil
                in_wc do
                    spec = Dir.glob("*.orogen").to_a.first
                    project = Project.load(spec)
                    project.enable_transports(*transports)

                    compile_wc(project) do
                        FileUtils.cp 'templates/CMakeLists.txt', 'CMakeLists.txt'
                        File.open('CMakeLists.txt', 'a') do |io|
                            yield(io) if block_given?
                        end
                    end
                end

                if test_bin
                    in_prefix do
                        assert(system(test_bin, redirect_to_logfile), "failed to run test program #{test_bin}, see #{logfile} for output")
                    end
                end
                project
            end


            def compile_and_test(project, test_bin)
                compile_wc(project) do
                    FileUtils.cp 'templates/CMakeLists.txt', 'CMakeLists.txt'
                    File.open('CMakeLists.txt', 'a') do |io|
                        yield(io) if block_given?
                    end
                end

                in_prefix do
                    assert(system(test_bin, redirect_to_logfile), "failed to run test program #{test_bin}, see #{logfile} for output")
                end
            end
	end
    end
    end
    Minitest::Test.include OroGen::Gen::RTT_CPP::SelfTest
    Gen::RTT_CPP.enable
end

