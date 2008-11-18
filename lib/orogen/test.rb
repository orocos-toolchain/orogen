require 'test/unit'
require 'fileutils'
require 'orogen'

module Orocos
    module Generation
	module Test
	    include Orocos
	    include Orocos::Generation

	    TEST_DIR = File.expand_path('../../test', File.dirname(__FILE__))
	    attr_reader :working_directory

	    def setup
		super if defined? super
	    end

	    def teardown
		unless ENV['TEST_KEEP_WC']
		    if working_directory && File.directory?(working_directory)
			FileUtils.rm_rf working_directory
		    end
		end

		super if defined? super
	    end

	    def create_wc
		if !working_directory
		    @working_directory = File.join(TEST_DIR, 'wc')
		    FileUtils.mkdir_p working_directory
		end
	    end

	    def copy_in_wc(file, destination = nil)
		create_wc
		if destination
		    destination = File.expand_path(destination, working_directory)
		    FileUtils.mkdir_p destination
		end

		FileUtils.cp File.expand_path(file, TEST_DIR), (destination || working_directory)
	    end

	    def in_wc(&block)
		create_wc
		Dir.chdir(working_directory, &block)
	    end
            def in_prefix(&block)
		create_wc
                old_pkgconfig = ENV['PKG_CONFIG_PATH']
                in_wc do
                    Dir.chdir("build") do
                        if !system("make", "install")
                            raise "failed to install"
                        end
                    end

                    ENV['PKG_CONFIG_PATH'] += ":" + File.join(working_directory, 'prefix', 'lib', 'pkgconfig')
                    Dir.chdir("prefix", &block)
                end
            ensure
                ENV['PKG_CONFIG_PATH'] = old_pkgconfig
            end

	    def compile_wc(component)
		in_wc do
                    component.instance_variable_set(:@deffile, File.join(working_directory, "#{component.name}.orogen"))
		    component.generate

		    yield if block_given?
		    FileUtils.mkdir('build') unless File.directory?('build')
		    Dir.chdir('build') do
			if !system("cmake -DCMAKE_BUILD_TYPE=Debug -DCMAKE_INSTALL_PREFIX=#{working_directory}/prefix ..")
			    raise "failed to configure"
			elsif !system("make")
			    raise "failed to build"
			end
		    end
		end
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

