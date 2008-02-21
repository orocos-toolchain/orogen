require 'fileutils'
require 'erb'

module Orocos
    module Generation
	@templates = Hash.new
	class << self
	    attr_reader :templates
	end

	def self.template_path(*path)
	    reldir = File.join('..', 'templates', *path)
	    File.expand_path(reldir, File.dirname(__FILE__))
	end

	def self.load_template(*path)
	    if template = templates[path]
		template
	    else
		template_data   = begin
				      File.open(template_path(*path))
				  rescue Errno::ENOENT
				      raise ArgumentError, "template #{File.join(*path)} does not exist"
				  end

		templates[path] = ERB.new(template_data.read)
	    end
	end

	def self.save_automatic(*args)
	    save_user '.orocos', *args
	end

	def self.save_user(*args)
	    if args.size < 2
		raise ArgumentError, "expected at least 2 arguments, got #{args.size}"
	    end

	    data      = args.pop
	    file_name = args.pop
	    dir_name  = File.expand_path(File.join(*args))
	    FileUtils.mkdir_p(dir_name)
	    File.open(File.join(dir_name, file_name), 'w') do |io|
		io.write data
	    end
	end
    end
end

