require 'typelib'
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
	end
	def self.code_to_corba(result, path = "", indent = "    ")
	    convertion_code_helper(:code_to_corba, result, path, indent)
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
    class Generation
	class Toolkit
	    attr_reader :component
	    attr_reader :name, :imports, :loads
	    attr_reader :registry
	    def self.validate_name(name)
		if !name || name.empty?
		    raise "empty toolkit name"
		elsif name !~ /^\w+$/
		    raise "toolkit names can only contain alphanumeric characters and _"
		end

		# TODO: check that this toolkit name is not already used ?
	    end

	    def initialize(component, name)
		Toolkit.validate_name name
		@component = component
		@name = name

		@corba_enabled = true
		@imports, @loads = [], []
		@registry = Typelib::Registry.new
	    end

	    def load(file)
		file = File.expand_path(file)
		loads << file
		registry.import(file)
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

		type_header = Orocos::Generation.render_template('toolkit/types.hpp', binding)

		generated_types = []
		registry.each_type do |type|
		    if type < Typelib::CompoundType
			generated_types << type
		    end
		end

		if corba_enabled?
		    corba  = Orocos::Generation.render_template 'toolkit/corba.hpp', binding
		    idl    = Orocos::Generation.render_template "toolkit/corba.idl", binding
		end
		header = Orocos::Generation.render_template "toolkit/header.hpp", binding
		namespace = '/'
		source = Orocos::Generation.render_template "toolkit/toolkit.cpp", binding

		return type_header, header, source, corba, idl
	    end
	end

	# call-seq:
	#   component.toolkit(toolkit_name = component.name) do
	#      ... toolkit setup ...
	#   end => toolkit
	#
	#   component.toolkit => current toolkit or nil
	#
	# The first form defines the type toolkit this component defines and
	# builds a Toolkit object based what the code block does. The given
	# code block should call Toolkit instance methods to set up that new
	# object
	#
	# The second form returns a Toolkit object if one is defined, and nil
	# otherwise.
	def toolkit(toolkit_name = name, &block)
	    if !block
		return @toolkit
	    end

	    toolkit_name = toolkit_name.to_s
	    self.name(toolkit_name) unless self.name

	    @toolkit = Toolkit.new(self, toolkit_name)
	    @toolkit.instance_eval(&block)

	    component = self
	    types, hpp, cpp, corba, idl = toolkit.to_code
	    if toolkit.corba_enabled?
		Generation.save_automatic("toolkit", "#{name}ToolkitCorba.hpp", corba)
		Generation.save_automatic("toolkit", "#{name}Toolkit.idl", idl)
	    end
	    Generation.save_automatic("toolkit", "#{name}ToolkitTypes.hpp", types)
	    Generation.save_automatic("toolkit", "#{name}Toolkit.hpp", hpp)
	    Generation.save_automatic("toolkit", "#{name}Toolkit.cpp", cpp)

	    cmake = Generation.render_template "toolkit/CMakeLists.txt", binding
	    Generation.save_automatic("toolkit", "CMakeLists.txt", cmake)

	    pkg_config = Generation.render_template 'toolkit/toolkit.pc', binding
	    Generation.save_automatic("toolkit", "#{toolkit.name}-toolkit.pc.in", pkg_config)

	    @toolkit
	end
    end
end

