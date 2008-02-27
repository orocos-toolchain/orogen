require 'test/unit'
require 'orocos/generation/build'
require 'fileutils'

module Orocos
    class Generation
	module Test
	    include Orocos

	    TEST_DIR = File.expand_path('../../../test', File.dirname(__FILE__))
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
		    File.mkdir_p destination
		end

		FileUtils.cp File.expand_path(file, TEST_DIR), (destination || working_directory)
	    end

	    def in_wc(&block)
		create_wc
		Dir.chdir(working_directory, &block)
	    end

	    def compile_wc(generation)
		in_wc do
		    generation.build_system
		    yield if block_given?
		    FileUtils.mkdir('build') unless File.directory?('build')
		    Dir.chdir('build') do
			if !system("cmake ..")
			    raise "failed to configure"
			elsif !system("make")
			    raise "failed to build"
			end
		    end
		end
	    end
	end
    end
end

