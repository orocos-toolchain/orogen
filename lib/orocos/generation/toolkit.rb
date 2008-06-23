require 'typelib'
require 'tempfile'
require 'find'
require 'orocos/generation/base'

module Typelib
    class Type
	def self.to_orocos_decomposition(result, path, indent = "    ")
	    orocos_type = registry.orocos_equivalent(self).basename
	    result << indent << "target_bag.add( new Property<#{orocos_type}>(\"#{path}\", \"\", value#{path}) );"
	end

	def self.code_to_corba(result, path = "", indent = "    ")
	    result << indent << "result#{path} = value#{path};"
	end
	def self.code_from_corba(result, path = "", indent = "    ")
	    code_to_corba(result, path, indent)
	end

	def self.to_orocos_composition
	end
    end
    class CompoundType
	def self.convertion_code_helper(method, result, path, indent)
	    each_field do |name, type|
		type.send(method, result, "#{path}.#{name}", indent) << "\n"
	    end
	end
	def self.to_orocos_decomposition(result, path, indent = "    ")
	    convertion_code_helper(:to_orocos_decomposition, result, path, indent)
	    result
	end
	def self.code_to_corba(result, path = "", indent = "    ")
	    convertion_code_helper(:code_to_corba, result, path, indent)
	    result
	end

	def self.to_orocos_composition
	end
    end
    class ArrayType
	def self.convertion_code_helper(method, result, path, indent)
	    result << indent << "for (i = 0; i < #{length}; ++i) {\n" 
	    deference.send(method, result, path + "[i]", indent + "    ") << "\n"
	    result << indent << "}"
	end
	def self.to_orocos_decomposition(result, path, indent = "    ")
	    convertion_code_helper(:to_orocos_decomposition, result, path, indent)
	end
	def self.code_to_corba(result, path = "", indent = "    ")
	    convertion_code_helper(:code_to_corba, result, path, indent)
	end

	def self.to_orocos_composition
	end
    end
    class Registry
	OROCOS_KNOWN_TYPES = ['char', 'int', 'unsigned int', 'float', 'double']
	OROCOS_KNOWN_CONVERTIONS = {
	    'short' => 'int',
	    'unsigned short' => 'unsigned int' }

	attr_reader :orocos_type_equivalence
	def build_orocos_type_equivalence
	    @orocos_type_equivalence = Hash.new

	    orocos_known_types = OROCOS_KNOWN_TYPES.map { |name| get(name) }
	    orocos_known_convertions = OROCOS_KNOWN_CONVERTIONS.map { |from, to| [get(from), get(to)] }
	    each_type do |type|
		next if type < CompoundType || type < ArrayType

		if eqv = orocos_known_types.find { |known_t| known_t == type }
		    orocos_type_equivalence[type] = eqv
		elsif eqv = orocos_known_convertions.find { |from_t, to_t| from_t == type }
		    orocos_type_equivalence[type] = eqv[1]
		end
	    end
	end

	def orocos_equivalent(user_type)
	    if !orocos_type_equivalence
		build_orocos_type_equivalence
	    end

	    if type = orocos_type_equivalence[user_type]
		type
	    else
		raise TypeError, "#{user_type.name} does not have an equivalent in the Orocos RTT toolkit"
	    end
	end
    end
end

module Orocos
    module Generation
	class Toolkit
	    attr_reader :component
	    attr_reader :imports, :loads
	    attr_reader :registry
            attr_reader :preloaded_registry

	    dsl_attribute :blob_threshold do |value|
		value = Integer(value)
		if value == 0; nil
		else value
		end
	    end

	    dsl_attribute :name do |new|
		new = new.to_s
		if new !~ /^\w+$/
		    raise "toolkit names can only contain alphanumeric characters and _"
		end
		new
	    end

	    def initialize(component, name, &block)
		@component = component
		if name
		    self.name name
		end

		@corba_enabled = true
		@imports, @loads = [], []
		@registry = Typelib::Registry.new
		@preloaded_registry = Typelib::Registry.new

		instance_eval(&block) if block_given?
	    end

            def preload(file)
                cpp_source = Tempfile.new("preload")

                cpp_source.puts "#include <#{file}>"
                cpp_source.flush

                puts cpp_source.path
                puts File.read(cpp_source.path)

                load(cpp_source.path, true)
            ensure
                cpp_source.close if cpp_source
            end

	    def load(file, preload = false)
		file = File.expand_path(file)
		if !File.file?(file)
		    raise ArgumentError, "no such file or directory #{file}"
		end

		loads << file
		registry.import(file, 'c')
                preloaded_registry.import(file, 'c') if preload
		component.registry.import(file, 'c')
	    end

	    def import(other_toolkit)
		raise NotImplementedError
	    end

	    def corba_enabled?; @corba_enabled end

	    def disable_corba
		@corba_enabled = false
	    end

	    def to_code
		toolkit = self

                registry = self.registry.minimal(preloaded_registry)
		type_header = Generation.render_template('toolkit/types.hpp', binding)

		generated_types = []
		registry.each_type do |type|
		    if type < Typelib::CompoundType
			generated_types << type
		    end
		end

		if corba_enabled?
		    corba  = Generation.render_template 'toolkit/corba.hpp', binding
		    idl    = Orocos::Generation.render_template "toolkit/corba.idl", binding
		end
		header = Orocos::Generation.render_template "toolkit/header.hpp", binding
		namespace = '/'
		source = Orocos::Generation.render_template "toolkit/toolkit.cpp", binding

		return type_header, header, source, corba, idl
	    end

	    def generate
		toolkit = self

		types, hpp, cpp, corba, idl = to_code
		if toolkit.corba_enabled?
		    Generation.save_automatic("toolkit", "#{name}ToolkitCorba.hpp", corba)
		    Generation.save_automatic("toolkit", "#{name}Toolkit.idl", idl)
		end
		Generation.save_automatic("toolkit", "#{name}ToolkitTypes.hpp", types)
		Generation.save_automatic("toolkit", "#{name}Toolkit.hpp", hpp)
		Generation.save_automatic("toolkit", "#{name}Toolkit.cpp", cpp)

		pkg_config = Generation.render_template 'toolkit/toolkit.pc', binding
		Generation.save_automatic("toolkit", "#{toolkit.name}-toolkit.pc.in", pkg_config)
	    end
	end
    end
end

