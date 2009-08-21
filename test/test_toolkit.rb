require 'orogen/test'

class TC_GenerationToolkit < Test::Unit::TestCase
    include Orocos::Generation::Test

    def test_orocos_type_equivalence
	registry = Typelib::Registry.new

	assert_equal(registry.get('int'), registry.orocos_equivalent(registry.get('int32_t')))
	assert_equal(registry.get('unsigned int'), registry.orocos_equivalent(registry.get('uint32_t')))
	assert_equal(registry.get('int'), registry.orocos_equivalent(registry.get('int16_t')))
	assert_equal(registry.get('unsigned int'), registry.orocos_equivalent(registry.get('uint16_t')))
	assert_equal(registry.get('int'), registry.orocos_equivalent(registry.get('short')))
	assert_equal(registry.get('unsigned int'), registry.orocos_equivalent(registry.get('unsigned short')))

	assert_raises(TypeError) { registry.orocos_equivalent(registry.get('int64_t')) }
    end

    def test_toolkit_load
	component = Component.new
        component.name 'test_toolkit_load'

        # Load a file with errors
        assert_raises(ArgumentError) do
            component.toolkit do
                load File.join(TEST_DATA_DIR, 'exists')
            end
        end

        # Load a file that does not exist
        assert_raises(ArgumentError) do
            component.toolkit do
                load 'does_not_exist.h'
            end
        end
    end

    def check_output_file(basedir, name)
        output   = File.read(File.join(prefix_directory, name))
        expected = File.read(File.join(TEST_DATA_DIR, basedir, name))
        assert_equal(expected, output)
    end

    def test_opaque(with_corba = true)
        build_test_component('modules/toolkit_opaque', with_corba, "bin/test") do |cmake|
            cmake << "\nADD_DEFINITIONS(-DWITH_CORBA)" if with_corba
            cmake << "\nADD_EXECUTABLE(test test.cpp)"
            cmake << "\nTARGET_LINK_LIBRARIES(test opaque-toolkit-${OROCOS_TARGET})"
            cmake << "\nTARGET_LINK_LIBRARIES(test ${OROCOS_COMPONENT_LIBRARIES})"
            cmake << "\nINSTALL(TARGETS test RUNTIME DESTINATION bin)"
            cmake << "\n"
	end

        check_output_file('modules/toolkit_opaque', 'opaque.xml')
        check_output_file('modules/toolkit_opaque', 'opaque.cpf')
        check_output_file('modules/toolkit_opaque', 'composed_opaque.xml')
        check_output_file('modules/toolkit_opaque', 'composed_opaque.cpf')
        check_output_file('modules/toolkit_opaque', 'shared_ptr__opaque_type.xml')
        check_output_file('modules/toolkit_opaque', 'shared_ptr__opaque_type.cpf')
        check_output_file('modules/toolkit_opaque', 'shared_ptr__shared_ptr.xml')
        check_output_file('modules/toolkit_opaque', 'shared_ptr__shared_ptr.cpf')
        check_output_file('modules/toolkit_opaque', 'readonlypointer.xml')
        check_output_file('modules/toolkit_opaque', 'readonlypointer.cpf')
    end
    def test_opaque_without_corba; test_opaque(false) end

    def test_opaque_autodef(with_corba = true)
        build_test_component('modules/toolkit_autodef', with_corba)
    end

    def test_opaque_validation
        # First, check that the actual opaque module generates properly
        create_wc("modules/toolkit_opaque_validation_ok")
        FileUtils.rm_rf working_directory
        FileUtils.cp_r File.join(TEST_DATA_DIR, "modules/toolkit_opaque"), working_directory

        component = Component.new
        in_wc do
            component.load 'opaque.orogen'
            assert_nothing_raised { component.generate }
        end

        # Second, check that it fails if an invalid file is loaded
        create_wc("modules/toolkit_opaque_validation_fail")
        FileUtils.rm_rf working_directory
        FileUtils.cp_r File.join(TEST_DATA_DIR, "modules/toolkit_opaque"), working_directory

        component = Component.new
        in_wc do
            component.load 'opaque.orogen'
            component.toolkit.load File.join(TEST_DATA_DIR, 'opaque_invalid.h')
            assert_raises(NotImplementedError) { component.generate }
        end
    end

    def test_simple(with_corba = true)
        build_test_component('modules/toolkit_simple', with_corba, "bin/test") do |cmake|
             cmake << "\nADD_DEFINITIONS(-DWITH_CORBA)" if with_corba
             cmake << "\nADD_EXECUTABLE(test test.cpp)"
             cmake << "\nTARGET_LINK_LIBRARIES(test simple-toolkit-${OROCOS_TARGET})"
             cmake << "\nTARGET_LINK_LIBRARIES(test ${OROCOS_COMPONENT_LIBRARIES})"
             cmake << "\nINSTALL(TARGETS test RUNTIME DESTINATION bin)"
             cmake << "\n"
        end

        check_output_file('modules/toolkit_simple', 'simple.cpf')
        check_output_file('modules/toolkit_simple', 'simple.xml')

        # The simple.h header should be installed in orocos/toolkit/simple.h
        assert File.exists?( File.join(prefix_directory, "include", "orocos", "simple", "simple.h") )
    end
    def test_simple_without_corba; test_simple(false) end

    def test_dependencies(with_corba = true)
        # Install a fake library
        libprefix = File.join(prefix_directory, "libs/toolkit_dependencies_lib")
        FileUtils.mkdir_p File.join(libprefix, "include")
        FileUtils.mkdir_p File.join(libprefix, "lib", "pkgconfig")
        FileUtils.cp File.join(TEST_DATA_DIR, "modules", "toolkit_dependencies_lib", "tkdeps_lib.h"), File.join(libprefix, "include")
        File.open(File.join(libprefix, "lib", "pkgconfig", "tkdeps_lib.pc"), 'w') do |io|
            io << "Name: Blablabla\n"
            io << "Description: Blablabla\n"
            io << "Version: 0\n"
            io << "Cflags: -I#{libprefix}/include\n"
            io << "Libs: \n"
        end
        ENV['PKG_CONFIG_PATH'] += ":#{libprefix}/lib/pkgconfig"

        # Install the parent toolkit (the one that will be imported in the main
        # toolkit)
        build_test_component 'modules/toolkit_dependencies_parent', with_corba
        install
        ENV['PKG_CONFIG_PATH'] += ":" + File.join(prefix_directory, 'lib', 'pkgconfig')

        # And now the final one ...
        component = build_test_component('modules/toolkit_dependencies', with_corba)

        deps = component.toolkit.dependencies
        assert_equal ["tkdeps_lib", "tkdeps_parent-toolkit-gnulinux"], deps.to_a.map(&:name).sort
    end
    def test_dependencies_without_corba
        test_toolkit_dependencies(false)
    end
end

