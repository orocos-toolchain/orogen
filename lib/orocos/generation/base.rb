require 'logger'
require 'fileutils'
require 'erb'
require 'typelib'

class Module
    # call-seq:
    #   dsl_attribute(name)
    #   dsl_attribute(name) { |value| ... }
    #
    # This defines a +name+ instance method on the given class which accepts zero or one argument
    #
    # Without any argument, it acts as a getter for the +@name+ attribute. With
    # one argument, it acts instead as a setter for the same attribute and
    # returns self. If a block has been given to +dsl_attribute+, any new value
    # is passed to the block, whose return value is actually saved in the
    # instance variable.  This block can therefore both filter the value
    # (convert it to a desired form) and validate it.
    #
    # The goal of this method is to have a nicer way to handle attribute in DSLs: instead
    # of 
    #
    #    model = create_model do
    #	    self.my_model_attribute = 'bla'
    #
    #	    if (my_model_attribute)
    #		<do something>
    #	    end
    #	 end
    #
    # (or worse, using set_ and get_ prefixes), we can do
    #
    #    model = create_model do
    #	    my_model_attribute 'bla'
    #
    #	    if (my_model_attribute)
    #		<do something>
    #	    end
    #	 end
    #
    def dsl_attribute(name, &filter_block)
	class_eval do
	    define_method(name) do |*value|
		if value.empty?
		    instance_variable_get("@#{name}")
		elsif value.size > 1
		    raise ArgumentError, "expected 0 or 1 argument (got #{value.size})"
		elsif filter_block
		    instance_variable_set("@#{name}", filter_block.call(value.first))
		    self
		else
		    instance_variable_set("@#{name}", value.first)
		    self
		end
	    end
	end
    end
end


module Orocos
    module Generation
	class << self
	    attr_reader :logger
	end
	@logger = Logger.new(STDOUT)
	logger.level = Logger::WARN
 
	@templates = Hash.new
	class << self
	    # The set of templates already loaded as a path => ERB object hash
	    attr_reader :templates
	end

	# call-seq:
	#   template_path(path1, path2, ..., file_name)
	#
	# Returns the full path for the template path1/path2/.../file_name.
	# Templates names are the path relative to the template base directory,
	# which is the orocos/templates directory directly in Orocos.rb
	# sources.
	def self.template_path(*path)
	    reldir = File.join('..', 'templates', *path)
	    File.expand_path(reldir, File.dirname(__FILE__))
	end

	# call-seq:
	#   load_template path1, path2, ..., file_name => erb object
	#
	# Loads the template file located at
	# template_dir/path1/path2/.../file_name and return the ERB object
	# generated from it. template_dir is the templates/ directory located
	# in Orocos.rb sources.
	#
	# A template is only loaded once. See Generation.templates.
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

	# call-seq:
	#   render_template path1, path2, file_name, binding
	#
	# Render the template found at path1/path2/file_name and render it
	# using the provided binding
	def self.render_template(*args)
	    binding = args.pop
	    template = load_template(*args)
	    template.result(binding)
	end

	def self.save_generated(overwrite, *args) # :nodoc:
	    if args.size < 2
		raise ArgumentError, "expected at least 2 arguments, got #{args.size}"
	    end

	    data      = args.pop
	    file_path = File.expand_path(File.join(*args))
	    dir_name  = File.dirname(file_path)
	    FileUtils.mkdir_p(dir_name)

	    if File.exists?(file_path)
		if File.read(file_path) != data
		    if overwrite
			logger.info "  overwriting #{file_path}"
		    else
			logger.warn "  will not overwrite #{file_path}"
			return
		    end
		else
		    logger.debug "  #{file_path} has not changed"
		    return
		end
	    else
		logger.info "  creating #{file_path}"
	    end

	    File.open(file_path, 'w') do |io|
		io.write data
	    end
	end

	# call-seq:
	#   save_automatic path1, path2, ..., file_name, data
	#
	# Save the provided data in the path1/path2/.../file_name file of the
	# automatically-generated part of the component (i.e. under .orocos)
	def self.save_automatic(*args)
	    save_generated true, '.orocos', *args
	end
	
	# call-seq:
	#   save_public_automatic path1, path2, ..., file_name, data
	#
	# Save the provided data in the path1/path2/file_name file of the
	# user-written part of the component. It differs from save_user because
	# it will happily overwrite an existing file.
	def self.save_public_automatic(*args)
	    save_generated true, *args
	end
	
	# call-seq:
	#   save_user path1, path2, ..., file_name, data
	#
	# Save the provided data in the path1/path2/file_name file of the
	# user-written part of the component, if the said file does
	# not exist yet
	def self.save_user(*args)
	    save_generated false, *args

	    # Save the template in path1/path2/.../orogen/file_name
	    args = args.dup
	    args.insert(-3, "orogen")
	    save_generated true, *args
	end

	# Returns the C++ code which changes the current namespace from +old+
	# to +new+. +indent_size+ is the count of indent spaces between
	# namespaces.
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

