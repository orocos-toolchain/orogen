$LOAD_PATH.unshift File.expand_path('../lib', File.dirname(__FILE__))
require 'orogen/test'

class TC_GenerationTypekit < Test::Unit::TestCase
    include Orocos::Generation::Test

    TRANSPORTS = ['corba', 'typelib', ['typelib', 'mqueue']]
    def self.transport_tests(name)
        TRANSPORTS.each do |transport|
            define_method("test_#{name}_#{[*transport].last}") do
                send(name, *transport)
            end
        end
        define_method("test_#{name}_none") do
            send(name)
        end
        define_method("test_#{name}_all") do
            send(name, *TRANSPORTS.flatten)
        end
    end

    def test_orocos_type_equivalence
	registry = Typelib::Registry.new
        Typelib::Registry.add_standard_cxx_types(registry)

	assert_equal(registry.get('int'), registry.base_rtt_type_for(registry.get('int32_t')))
	assert_equal(registry.get('unsigned int'), registry.base_rtt_type_for(registry.get('uint32_t')))
	assert_equal(registry.get('int'), registry.base_rtt_type_for(registry.get('int16_t')))
	assert_equal(registry.get('unsigned int'), registry.base_rtt_type_for(registry.get('uint16_t')))
	assert_equal(registry.get('int'), registry.base_rtt_type_for(registry.get('short')))
	assert_equal(registry.get('unsigned int'), registry.base_rtt_type_for(registry.get('unsigned short')))
	assert_equal(registry.get('double'), registry.base_rtt_type_for(registry.get('float')))
    end

    def test_base_types_are_imported
        typekit = Orocos::Generation::Typekit.new
        assert(typekit.imported_type?('/nil'))
        assert(typekit.imported_type?('/int32_t'))
    end

    def test_self_types_is_initially_empty
        typekit = Orocos::Generation::Typekit.new
        assert_equal [], typekit.self_types.to_a
    end

    def test_normalized_registry_is_initially_empty
        typekit = Orocos::Generation::Typekit.new
        assert_equal [], typekit.normalize_registry.to_a
    end

    def test_typekit_load
	component = Component.new
        component.name 'test_typekit_load'

        # Load a file that does not exist
        assert_raises(ArgumentError) do
            component.typekit do
                load 'does_not_exist.h'
            end
        end

        # Load a file with errors
        assert_raises(ArgumentError) do
            typekit = component.typekit(true)
            typekit.load File.join(TEST_DATA_DIR, 'exists')
            typekit.perform_pending_loads
        end
    end

    def check_output_file(basedir, name)
        output   = File.read(File.join(prefix_directory, name))
        expected = File.read(File.join(TEST_DATA_DIR, basedir, name))
        assert_equal(expected, output)
    end

    def opaque(*transports)
        build_test_component('modules/typekit_opaque', transports, "bin/test") do |cmake|
            transports.each do |transport_name|
                cmake << "ADD_DEFINITIONS(-DWITH_#{transport_name.upcase})\n"
            end

            if transports.include?('typelib')
                cmake << <<-EOT
pkg_check_modules(TYPELIB REQUIRED typelib)
include_directories(${TYPELIB_INCLUDE_DIRS})
link_directories(${TYPELIB_LIBRARY_DIRS})
                EOT
            end

            if transports.include?('corba')
                cmake << <<-EOT
find_package(OrocosCORBA REQUIRED COMPONENTS Typekit)
include_directories(${OrocosCORBA_INCLUDE_DIRS})
add_definitions(${OrocosCORBA_CFLAGS_OTHER})
link_directories(${OrocosCORBA_LIBRARY_DIRS})
                EOT
            end

            cmake << <<-EOF
link_directories(${CMAKE_INSTALL_PREFIX}/lib/orocos/plugins ${CMAKE_INSTALL_PREFIX}/lib/orocos/types)

ADD_EXECUTABLE(test test.cpp)
list(APPEND CMAKE_PREFIX_PATH ${OrocosRTT_PREFIX})
target_link_libraries(test opaque-typekit-${OROCOS_TARGET})
target_link_libraries(test ${OROCOS_COMPONENT_LIBRARIES})
include_directories(${CMAKE_SOURCE_DIR}/.orogen/typekit)
include_directories(${CMAKE_BINARY_DIR}/.orogen/typekit)
find_package( RTTPlugin COMPONENTS rtt-typekit rtt-marshalling)
target_link_libraries(test ${RTT_PLUGIN_rtt-marshalling_LIBRARY})
target_link_libraries(test ${RTT_PLUGIN_rtt-typekit_LIBRARY})
INSTALL(TARGETS test RUNTIME DESTINATION bin)
            EOF

            transports.each do |transport_name|
                cmake << "\ntarget_link_libraries(test opaque-transport-#{transport_name}-${OROCOS_TARGET})"
            end
            if transports.include?('corba')
                cmake << "\ntarget_link_libraries(test ${OrocosCORBA_LIBRARIES} omniDynamic4)"
            end
	end

        #check_output_file('modules/typekit_opaque', 'opaque.xml')
        #check_output_file('modules/typekit_opaque', 'opaque.cpf')
        #check_output_file('modules/typekit_opaque', 'composed_opaque.xml')
        #check_output_file('modules/typekit_opaque', 'composed_opaque.cpf')
        #check_output_file('modules/typekit_opaque', 'shared_ptr__opaque_type.xml')
        #check_output_file('modules/typekit_opaque', 'shared_ptr__opaque_type.cpf')
        #check_output_file('modules/typekit_opaque', 'shared_ptr__shared_ptr.xml')
        #check_output_file('modules/typekit_opaque', 'shared_ptr__shared_ptr.cpf')
        #check_output_file('modules/typekit_opaque', 'readonlypointer.xml')
        #check_output_file('modules/typekit_opaque', 'readonlypointer.cpf')
    end
    transport_tests 'opaque'

    def test_opaque_autodef
        build_test_component('modules/typekit_autodef', ['corba'])
    end

    def simple(*transports)
        build_test_component('modules/typekit_simple', transports, "bin/test") do |cmake|
            ENV['CMAKE_LIBRARY_PATH'] = "#{ENV['CMAKE_LIBRARY_PATH']}:#{prefix_directory}"

            transports.each do |transport_name|
                cmake << "ADD_DEFINITIONS(-DWITH_#{transport_name.upcase})\n"
            end
            if transports.include?('typelib')
                cmake << <<-EOT
pkg_check_modules(TYPELIB REQUIRED typelib)
include_directories(${TYPELIB_INCLUDE_DIRS})
link_directories(${TYPELIB_LIBRARY_DIRS})
                EOT
            end
            cmake << <<-EOT
include_directories(${OrocosRTT_INCLUDE_DIRS} ${OrocosCORBA_INCLUDE_DIRS})
include_directories(${CMAKE_SOURCE_DIR}/.orogen/typekit)
include_directories(${CMAKE_BINARY_DIR}/.orogen/typekit)
link_directories(${OrocosCORBA_LIBRARY_DIRS} ${OrocosRTT_LIBRARY_DIRS})
link_directories(${CMAKE_INSTALL_PREFIX}/lib/orocos/plugins ${CMAKE_INSTALL_PREFIX}/lib/orocos/types)
list(APPEND CMAKE_PREFIX_PATH ${OrocosRTT_PREFIX})

add_executable(test test.cpp)
target_link_libraries(test simple-typekit-${OROCOS_TARGET})
target_link_libraries(test ${OrocosRTT_LIBRARIES})
find_package( RTTPlugin COMPONENTS rtt-typekit rtt-marshalling)
target_link_libraries(test ${RTT_PLUGIN_rtt-marshalling_LIBRARY})
target_link_libraries(test ${RTT_PLUGIN_rtt-typekit_LIBRARY})
install(TARGETS test RUNTIME DESTINATION bin)
            EOT

            transports.each do |transport_name|
                cmake << "target_link_libraries(test simple-transport-#{transport_name}-${OROCOS_TARGET})\n"
            end
        end

        # The simple.h header should be installed in orocos/typekit/simple.h
        assert File.exists?( File.join(prefix_directory, "include", "orocos", "simple", "types", "simple", "simple.h") )

        #check_output_file('modules/typekit_simple', 'basic.cpf')
        #check_output_file('modules/typekit_simple', 'basic.xml')
        #check_output_file('modules/typekit_simple', 'simple_vector.cpf')
        #check_output_file('modules/typekit_simple', 'simple_vector.xml')
        #check_output_file('modules/typekit_simple', 'complex_vector.cpf')
        #check_output_file('modules/typekit_simple', 'complex_vector.xml')
        #check_output_file('modules/typekit_simple', 'complex_array.cpf')
        #check_output_file('modules/typekit_simple', 'complex_array.xml')
    end
    transport_tests 'simple'

    def dependencies(*transports)
        # Install a fake library
        libprefix = File.join(prefix_directory, "libs/typekit_dependencies_lib")
        FileUtils.mkdir_p File.join(libprefix, "include")
        FileUtils.mkdir_p File.join(libprefix, "lib", "pkgconfig")
        FileUtils.cp File.join(TEST_DATA_DIR, "modules", "typekit_dependencies_lib", "tkdeps_lib.h"), File.join(libprefix, "include")
        File.open(File.join(libprefix, "lib", "pkgconfig", "tkdeps_lib.pc"), 'w') do |io|
            io << "Name: Blablabla\n"
            io << "Description: Blablabla\n"
            io << "Version: 0\n"
            io << "Cflags: -I#{libprefix}/include\n"
            io << "Libs: \n"
        end
        ENV['PKG_CONFIG_PATH'] += ":#{libprefix}/lib/pkgconfig"

        # Install the parent typekit (the one that will be imported in the main
        # typekit)
        build_test_component 'modules/typekit_dependencies_parent', transports
        install
        ENV['PKG_CONFIG_PATH'] += ":" + File.join(prefix_directory, 'lib', 'pkgconfig')

        # And now the final one ...
        build_test_component('modules/typekit_dependencies', transports)
    end
    transport_tests 'dependencies'
end

