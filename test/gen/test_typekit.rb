require 'orogen/gen/test'

class TC_GenerationTypekit < Minitest::Test
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

    def test_self_types_is_initially_empty
        typekit = OroGen::Gen::RTT_CPP::Typekit.new
        assert_equal [], typekit.self_types.to_a
    end

    def test_normalized_registry_is_initially_empty
        typekit = OroGen::Gen::RTT_CPP::Typekit.new
        assert_equal [], typekit.normalize_registry.to_a
    end

    def test_typekit_load_should_raise_LoadError_if_the_file_does_not_exist
	project = Project.new
        project.name 'test_typekit_load'
        project.deffile = File.join(path_to_wc_root, 'test_typekit_load', 'test_typekit_load.orogen')

        # Load a file that does not exist
        assert_raises(LoadError) do
            project.typekit do
                load 'does_not_exist.h'
            end
        end
    end

    def test_typekit_load_should_raise_ArgumentError_if_the_file_has_errors
	project = Project.new
        project.name 'test_typekit_load'
        project.deffile = File.join(path_to_wc_root, 'test_typekit_load', 'test_typekit_load.orogen')

        # Load a file with errors
        assert_raises(ArgumentError) do
            typekit = project.typekit(true)
            typekit.load File.join(path_to_data, 'exists')
            typekit.perform_pending_loads
        end
    end

    def check_output_file(basedir, name)
        output   = File.read(File.join(prefix_directory, name))
        expected = File.read(File.join(path_to_data, basedir, name))
        assert_equal(expected, output)
    end

    def opaque(*transports)
        build_test_project('modules/typekit_opaque', transports, "bin/test") do |cmake|
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
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/.orogen/typekit)
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
        build_test_project('modules/typekit_autodef', ['corba'])
    end

    def simple(*transports)
        build_test_project('modules/typekit_simple', transports, "bin/test") do |cmake|
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
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/.orogen/typekit)
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
        assert File.exists?( File.join(prefix_directory, "include", "orocos", "simple", "simple.h") )

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
        FileUtils.cp File.join(path_to_data, "modules", "typekit_dependencies_lib", "tkdeps_lib.h"), File.join(libprefix, "include")
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
        build_test_project 'modules/typekit_dependencies_parent', transports
        install
        ENV['PKG_CONFIG_PATH'] += ":" + File.join(prefix_directory, 'lib', 'pkgconfig')

        # And now the final one ...
        build_test_project('modules/typekit_dependencies', transports)
    end
    transport_tests 'dependencies'

    def test_parse_typelist_single_type_without_export_boolean
        typelist, interface_typelist = ImportedTypekit.parse_typelist("/string")
        assert_equal ['/string'], typelist
        assert_equal ['/string'], interface_typelist
    end
    def test_parse_typelist_array_type_without_export_boolean
        typelist, interface_typelist = ImportedTypekit.parse_typelist("/string[9]")
        assert_equal ['/string[9]'], typelist
        assert_equal ['/string[9]'], interface_typelist
    end
    def test_parse_typelist_type_with_spaces_in_name_without_export_boolean
        typelist, interface_typelist = ImportedTypekit.parse_typelist("/unsigned int")
        assert_equal ['/unsigned int'], typelist
        assert_equal ['/unsigned int'], interface_typelist
    end
    def test_parse_typelist_type_with_spaces_in_name_exported
        typelist, interface_typelist = ImportedTypekit.parse_typelist("/unsigned int 1")
        assert_equal ['/unsigned int'], typelist
        assert_equal ['/unsigned int'], interface_typelist
    end
    def test_parse_typelist_type_with_spaces_in_name_not_exported
        typelist, interface_typelist = ImportedTypekit.parse_typelist("/unsigned int 0")
        assert_equal ['/unsigned int'], typelist
        assert_equal [], interface_typelist
    end
end


describe OroGen::Gen::RTT_CPP::Typekit do
    describe "#filter_unsupported_types" do
        attr_reader :typekit
        before do
            @typekit = OroGen::Gen::RTT_CPP::Typekit.new
        end

        it "rejects multi-dimensional arrays" do
            reg = Typelib::Registry.import File.join(path_to_data, 'typekit', 'multi_dimensional_array.h')
            typekit.filter_unsupported_types(reg)
            assert !reg.include?("/double[2][4]")
        end
        it "rejects std::vector<bool>" do
            reg = Typelib::Registry.import File.join(path_to_data, 'typekit', 'std_vector_bool.h')
            typekit.filter_unsupported_types(reg)
            assert !reg.include?("/std/vector</bool>")
        end
        it "rejects pointers" do
            reg = Typelib::Registry.import File.join(path_to_data, 'typekit', 'pointer.h')
            typekit.filter_unsupported_types(reg)
            assert !reg.include?("/double*")
        end
        it "rejects compounds whose field name does not start with an alphanumeric character" do
            reg = Typelib::Registry.import File.join(path_to_data, 'typekit', 'compound_with_field_not_starting_with_alphanumeric_character.h')
            typekit.filter_unsupported_types(reg)
            assert !reg.include?("/Test")
        end
        it "rejects the types that depend on rejected types" do
            reg = Typelib::Registry.import File.join(path_to_data, 'typekit', 'rejected_dependencies.h')
            typekit.filter_unsupported_types(reg)
            assert !reg.include?("/CompoundTest")
            assert !reg.include?("/VectorTest")
            assert !reg.include?("/ArrayTest")
        end
    end
end
