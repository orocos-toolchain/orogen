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

		templates[path] = ERB.new(template_data.read, nil, nil, path.join('_').gsub(/[\/\.-]/, '_'))
	    end
	end

	def self.render_template(name, binding)
	    template = load_template name
	    template.result(binding)
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

	# Returns the C++ code which changes the current namespace from +old+
	# to +new+
	def self.adapt_namespace(old, new, indent_size = 4)
	    old = old.split('/').delete_if { |v| v.empty? }
	    new = new.split('/').delete_if { |v| v.empty? }
	    indent = old.size * indent_size

	    result = ""

	    while !old.empty? && old.first == new.first
		old.shift
		new.shift
	    end
	    while !old.empty?
		indent -= indent_size
		result << " " * indent + "}\n"
		old.shift
	    end
	    while !new.empty?
		result << "#{" " * indent}namespace #{new.first} {\n"
		indent += indent_size
		new.shift
	    end

	    result
	end
    end
end

