require 'logger'
require 'fileutils'
require 'erb'

class Module
    def dsl_attribute(name, &block)
	assignation = if block
			  validator_iv = "@dsl_attribute_#{name}_validator"
			  instance_variable_set(validator_iv, block)
			  "@#{name} = self.class.instance_variable_get(#{validator_iv}).call(value.first)"
		      else
			  "@#{name} = value.first"
		      end

	class_eval <<-EOD
	def #{name}(*value)
	    if value.empty?
		@#{name}
	    elsif value.size > 1
		raise ArgumentError, "expected 0 or 1 argument (got \#{value.size})"
	    else
		#{assignation}
	    end
	end
	EOD
    end
end


module Orocos
    class Generation
	class << self
	    attr_reader :logger
	end
	@logger = Logger.new(STDOUT)
	logger.level = Logger::WARN

	attr_reader :tasks
	def initialize(&block)
	    @tasks = []

	    instance_eval(&block)
	end

	dsl_attribute :name

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

	def self.save_generated(overwrite, *args)
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
			logger.info "  will not overwrite #{file_path}"
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

	def self.save_automatic(*args)
	    save_generated true, '.orocos', *args
	end

	def self.save_public_automatic(*args)
	    save_generated true, *args
	end

	def self.save_user(*args)
	    save_generated false, *args
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

