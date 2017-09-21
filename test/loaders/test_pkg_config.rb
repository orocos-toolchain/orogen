require 'orogen/test'

describe OroGen::Loaders::PkgConfig do
    attr_reader :pkg_config
    attr_reader :fixtures_prefix

    def stub_pkgconfig_package(pkg_name, pkg)
        @pkg_config ||= Hash.new
        pkg_config[pkg_name] = pkg
        flexmock(Utilrb::PkgConfig).should_receive(:get).
            with(pkg_name, Hash).and_return(pkg)
    end

    def stub_pkgconfig_each_package(filter)
        @pkg_config ||= Hash.new
        mock = flexmock(Utilrb::PkgConfig).should_receive(:each_package).with(filter, Proc)
        packages = pkg_config.find_all { |name, pkg| filter === name }
        if !packages.empty?
            mock.and_iterates(*packages.map(&:first))
        end
    end

    def stub_orogen_pkgconfig_final
        stub_pkgconfig_each_package /^orogen-project-/
        stub_pkgconfig_each_package /-tasks-oroarch$/
        stub_pkgconfig_each_package /^orogen-\w+$/
        stub_pkgconfig_each_package /-typekit-oroarch$/
        flexmock(Utilrb::PkgConfig).should_receive(:get).with(String, Hash).and_return { raise Utilrb::PkgConfig::NotFound.new("not found") }
    end

    def stub_orogen_pkgconfig(name, task_models = Array.new, deployed_tasks = Array.new)
        deffile = File.join(fixtures_prefix, 'deffile', "base.orogen")
        type_registry = File.join(fixtures_prefix, "typekit", "#{name}.tlb")
        type_registry = nil if !File.exists?(type_registry)
        pkg = flexmock(
            :project_name => name,
            :deffile => deffile,
            :type_registry => type_registry,
            :task_models => task_models.join(","),
            :deployed_tasks => deployed_tasks.join(","),
            :binfile => "/path/to/binfile/#{name}")
        stub_pkgconfig_package("orogen-project-#{name}", pkg)
        stub_pkgconfig_package("#{name}-typekit-oroarch", pkg)
        stub_pkgconfig_package("#{name}-tasks-oroarch", pkg)
        stub_pkgconfig_package("orogen-#{name}", pkg)
        return pkg
    end

    before do
        @fixtures_prefix = File.join(File.dirname(__FILE__), '..', 'fixtures', 'pkgconfig_loader')
    end
    after do
        flexmock_teardown
    end

    describe "#has_project?" do
        let(:loader) { OroGen::Loaders::PkgConfig.new('oroarch') }

        it "returns true if the project is available and caches the result" do
            stub_orogen_pkgconfig 'test'
            stub_orogen_pkgconfig_final
            assert loader.has_project?('test')
            flexmock(Utilrb::PkgConfig).should_receive(:get).never
            assert loader.has_project?('test')
        end
        it "returns false if the project is not available and caches the result" do
            stub_orogen_pkgconfig_final
            assert !loader.has_project?('test')
            flexmock(Utilrb::PkgConfig).should_receive(:get).never
            assert !loader.has_project?('test')
        end
    end

    describe "#has_typekit?" do
        let(:loader) { OroGen::Loaders::PkgConfig.new('oroarch') }

        it "returns true if the typekit is available and caches the result" do
            stub_orogen_pkgconfig 'base'
            stub_orogen_pkgconfig_final
            assert loader.has_typekit?('base')
            flexmock(Utilrb::PkgConfig).should_receive(:get).never
            assert loader.has_typekit?('base')
        end
        it "returns false if the typekit is not available and caches the result" do
            stub_orogen_pkgconfig_final
            assert !loader.has_typekit?('base')
            flexmock(Utilrb::PkgConfig).should_receive(:get).never
            assert !loader.has_typekit?('base')
        end
        it "returns false if the corresponding project does not exist" do
            pkg = flexmock(
                :project_name => 'base',
                :deffile => File.join(fixtures_prefix, 'deffile', "base.orogen"),
                :type_registry => nil,
                :task_models => "",
                :deployed_tasks => "",
                :path => "bla")
            stub_pkgconfig_package("base-typekit-oroarch", pkg)
            stub_orogen_pkgconfig_final
            assert !loader.has_typekit?('base')
        end
        it "returns false if the corresponding project does not have a type_registry field" do
            pkg = flexmock(
                :project_name => 'base',
                :deffile => File.join(fixtures_prefix, 'deffile', "base.orogen"),
                :type_registry => nil,
                :task_models => "",
                :deployed_tasks => "",
                :path => "bla")
            stub_pkgconfig_package("orogen-project-base", pkg)
            stub_pkgconfig_package("base-typekit-oroarch", pkg)
            stub_orogen_pkgconfig_final
            assert !loader.has_typekit?('base')
        end
    end

    describe "#each_available_project_name" do
        it "enumerates the projects" do
            stub_orogen_pkgconfig 'test'
            stub_orogen_pkgconfig_final
            loader = OroGen::Loaders::PkgConfig.new('oroarch')
            assert loader.has_project?('test')
            assert_equal ['test'], loader.each_available_project_name.to_a
        end
    end

    describe "#each_available_typekit_name" do
        it "enumerates the typekits" do
            stub_orogen_pkgconfig 'test'
            stub_orogen_pkgconfig_final
            loader = OroGen::Loaders::PkgConfig.new('oroarch')
            assert_equal ['test'], loader.each_available_typekit_name.to_a
        end
    end

    describe "#each_available_deployment_name" do
        it "enumerates the deployments" do
            stub_orogen_pkgconfig 'base', ["base::Task"], ["deployment1", "deployment2"]
            stub_orogen_pkgconfig 'test', ["test::Task"], ["deployment1", "deployment3"]
            stub_orogen_pkgconfig_final
            loader = OroGen::Loaders::PkgConfig.new('oroarch')
            assert_equal Set['base', 'test'], loader.each_available_deployment_name.to_set
        end
    end

    describe "#load_available_types" do
        it "stores the typename-to-typekit information" do
            stub_orogen_pkgconfig 'base'
            stub_orogen_pkgconfig_final
            loader = OroGen::Loaders::PkgConfig.new('oroarch')
            assert loader.has_typekit?('base')
            loader.load_available_types
            assert(typekit = loader.available_types['/base/JointLimits'])
            assert_equal 'base', typekit.name
            assert typekit.exported
            assert(typekit = loader.available_types['/base/JointLimitRange'])
            assert_equal 'base', typekit.name
            assert !typekit.exported
        end

        it "prefers a typekit that exports the type over a typekit that does not" do
            stub_orogen_pkgconfig 'base'
            stub_orogen_pkgconfig 'not_exporting'
            stub_orogen_pkgconfig_final
            loader = OroGen::Loaders::PkgConfig.new('oroarch')
            assert loader.has_typekit?('base')
            loader.load_available_types
            assert(typekit = loader.available_types['/base/JointLimits'])
            assert_equal 'base', typekit.name
            assert typekit.exported
        end

        it "prefers a typekit that exports the type over a typekit that does not" do
            stub_orogen_pkgconfig 'not_exporting'
            stub_orogen_pkgconfig 'base'
            stub_orogen_pkgconfig_final
            loader = OroGen::Loaders::PkgConfig.new('oroarch')
            assert loader.has_typekit?('base')
            loader.load_available_types
            assert(typekit = loader.available_types['/base/JointLimits'])
            assert_equal 'base', typekit.name
            assert typekit.exported
        end
    end

    describe "#typekit_for" do
        before do
            stub_orogen_pkgconfig 'base'
            stub_orogen_pkgconfig_final
        end
        let(:loader) { OroGen::Loaders::PkgConfig.new('oroarch') }

        it "loads the available types if not done already" do
            flexmock(loader).should_receive(:load_available_types).once.pass_thru
            loader.typekit_for('/base/JointLimits')
        end

        it "raises if the required type cannot be found" do
            assert_raises(OroGen::NotTypekitType) { loader.typekit_for('/base/does_not_exist') }
        end

        it "raises if the required type is not exported and 'exported' is true" do
            assert_raises(OroGen::NotExportedType) { loader.typekit_for('/base/JointLimitRange', true) }
        end

        it "accepts a type object as argument" do
            typekit = loader.typekit_for(flexmock(name: '/base/JointLimits'), false)
            assert_kind_of OroGen::Spec::Typekit, typekit
            assert_equal 'base', typekit.name
        end

        it "resolves non-exported types using the typekit's already loaded typekit-to-type mappings" do
            typekit = loader.typekit_for('/base/JointLimitRange', false)
            flexmock(loader).should_receive(:typekit_model_from_name).never
            assert_same typekit, loader.typekit_for('/base/JointLimitRange', false)
            assert_raises(OroGen::NotExportedType) do
                loader.typekit_for('/base/JointLimitRange', true)
            end
        end

        it "resolves exported types using the typekit's already loaded typekit-to-type mappings" do
            typekit = loader.typekit_for('/base/JointLimits', true)
            flexmock(loader).should_receive(:typekit_model_from_name).never
            assert_same typekit, loader.typekit_for('/base/JointLimits', true)
        end

        it "annotates the error with known typekits that define but do not export the type" do
            flexmock(loader).should_receive(:imported_typekits_for).with('/base/JointLimitRange').
                and_return([tk = flexmock(name: 'Test')])
            e = assert_raises(OroGen::NotExportedType) do
                loader.typekit_for('/base/JointLimitRange', true)
            end
            assert_match /Test/, e.message
            assert_equal [tk], e.typekits
        end

        it "returns the defining typekit if the type is exported and exported is false" do
            typekit = loader.typekit_for('/base/JointLimits', false)
            assert_kind_of OroGen::Spec::Typekit, typekit
            assert_equal 'base', typekit.name
        end

        it "returns the defining typekit if the type is exported and exported is true" do
            typekit = loader.typekit_for('/base/JointLimits', true)
            assert_kind_of OroGen::Spec::Typekit, typekit
            assert_equal 'base', typekit.name
        end

        it "returns the defining typekit if the type is not exported and exported is false" do
            typekit = loader.typekit_for('/base/JointLimitRange', false)
            assert_kind_of OroGen::Spec::Typekit, typekit
            assert_equal 'base', typekit.name
        end
    end

    it "should not register the type library if the corresponding project does not exist" do
        pkg = flexmock(
            :project_name => 'base',
            :deffile => File.join(fixtures_prefix, 'deffile', "base.orogen"),
            :type_registry => nil,
            :task_models => "",
            :deployed_tasks => "",
            :path => "bla")
        stub_pkgconfig_package("base-tasks-oroarch", pkg)
        stub_orogen_pkgconfig_final
        loader = OroGen::Loaders::PkgConfig.new('oroarch')
        assert !loader.available_typekits['base']
    end

    describe "#find_task_library_from_task_model_name" do
        before do
            pkg = flexmock(
                :project_name => 'base',
                :deffile => File.join(fixtures_prefix, 'deffile', "base.orogen"),
                :task_models => "base::Task,base::other::Task",
                :deployed_tasks => "",
                :path => "bla",
                :binfile => '/path/to/binfile')
            stub_pkgconfig_package("orogen-project-base", pkg)
            stub_orogen_pkgconfig_final
        end
        let(:loader) { OroGen::Loaders::PkgConfig.new('oroarch') }

        it "returns the project name if the task's namespace is a known project that lists the model" do
            assert_equal "base", loader.find_task_library_from_task_model_name("base::Task")
            flexmock(Utilrb::PkgConfig).should_receive(:get).never
            assert_equal "base", loader.find_task_library_from_task_model_name("base::Task")
        end
        it "returns nil if the task's namespace is not a known project" do
            assert_nil loader.find_task_library_from_task_model_name("undefined_project::Task")
            flexmock(Utilrb::PkgConfig).should_receive(:get).never
            assert_nil loader.find_task_library_from_task_model_name("undefined_project::Task")
        end
        it "returns nil if the task's project does not list the task in its description file" do
            assert_nil loader.find_task_library_from_task_model_name("base::UndefinedTask")
            flexmock(Utilrb::PkgConfig).should_receive(:get).never
            assert_nil loader.find_task_library_from_task_model_name("base::UndefinedTask")
        end
        it "handles nested namespaces" do
            assert_equal 'base', loader.find_task_library_from_task_model_name("base::other::Task")
            flexmock(Utilrb::PkgConfig).should_receive(:get).never
            assert_equal 'base', loader.find_task_library_from_task_model_name("base::other::Task")
        end
    end

    describe "#find_deployments_from_deployed_task_name" do
        it "resolves the deployments that provide the corresponding task name" do
            stub_orogen_pkgconfig 'base', ["base::Task"], ["deployment1", "deployment2"]
            stub_orogen_pkgconfig 'test', ["test::Task"], ["deployment1", "deployment3"]
            stub_orogen_pkgconfig_final
            loader = OroGen::Loaders::PkgConfig.new('oroarch')
            assert_equal 'base', loader.find_project_from_deployment_name('base')
            assert_equal ['base', 'test'].to_set, loader.find_deployments_from_deployed_task_name('deployment1')
            assert_equal ['base'].to_set, loader.find_deployments_from_deployed_task_name('deployment2')

            flexmock(Utilrb::PkgConfig).should_receive(:get).never
            assert_equal 'base', loader.find_project_from_deployment_name('base')
            assert_equal ['base', 'test'].to_set, loader.find_deployments_from_deployed_task_name('deployment1')
            assert_equal ['base'].to_set, loader.find_deployments_from_deployed_task_name('deployment2')
        end

        it "ignores deployments whose project does not exist" do
            pkg = flexmock(
                :project_name => 'base',
                :deffile => File.join(fixtures_prefix, 'deffile', "base.orogen"),
                :type_registry => nil,
                :task_models => "",
                :deployed_tasks => "",
                :path => "bla",
                :binfile => '/path/to/binfile')
            stub_pkgconfig_package("orogen-base", pkg)
            stub_orogen_pkgconfig_final
            loader = OroGen::Loaders::PkgConfig.new('oroarch')
            assert !loader.find_project_from_deployment_name('base')
            assert !loader.find_project_from_deployment_name('deployment1')
        end
    end

    describe "find_deployment_binfile" do
        it "returns the binary file of a known deployment" do
            stub_orogen_pkgconfig 'base', ["base::Task"], ["deployment1", "deployment2"]
            stub_orogen_pkgconfig_final
            loader = OroGen::Loaders::PkgConfig.new('oroarch')
            assert_equal '/path/to/binfile/base', loader.find_deployment_binfile('base')
        end
        it "returns nil for an unknown deployment" do
            stub_orogen_pkgconfig_final
            loader = OroGen::Loaders::PkgConfig.new('oroarch')
            assert !loader.find_deployment_binfile('base')
        end
    end

    describe "#project_model_text_from_name" do
        it "should return the model content and path from the project name" do
            pkg = stub_orogen_pkgconfig 'base'
            stub_orogen_pkgconfig_final
            loader = OroGen::Loaders::PkgConfig.new('oroarch')
            assert_equal [File.read(pkg.deffile), pkg.deffile],
                loader.project_model_text_from_name('base')
        end
        it "should raise OroGen::ProjectNotFound for unknown projects" do
            stub_orogen_pkgconfig_final
            loader = OroGen::Loaders::PkgConfig.new('oroarch')
            assert_raises(OroGen::ProjectNotFound) { loader.project_model_text_from_name('base') }
        end
    end

    describe "#typekit_model_text_from_name" do
        it "should return the typekit tlb and typelist" do
            stub_orogen_pkgconfig 'base'
            stub_orogen_pkgconfig_final
            loader = OroGen::Loaders::PkgConfig.new('oroarch')
            tlb = File.read(File.join(fixtures_prefix, "typekit", "base.tlb"))
            typelist = File.read(File.join(fixtures_prefix, "typekit", "base.typelist"))
            assert_equal [tlb, typelist],
                loader.typekit_model_text_from_name('base')
        end
        it "should raise OroGen::TypekitNotFound for unknown projects" do
            stub_orogen_pkgconfig_final
            loader = OroGen::Loaders::PkgConfig.new('oroarch')
            assert_raises(OroGen::TypekitNotFound) { loader.typekit_model_text_from_name('base') }
        end
    end

    describe "#task_library_path_from_name" do
        before do
            FileUtils.mkdir_p(@env_dir = Dir.mktmpdir)
            @loader = OroGen::Loaders::PkgConfig.new('oroarch')
        end
        after do
            FileUtils.rm_rf @env_dir
        end

        it "resolves an existing library from the pkg-config description" do
            pkg  = flexmock(name: 'test-tasks-oroarch', library_dirs: [@env_dir])
            path = File.join(@env_dir, 'libtest-tasks-oroarch.so')
            FileUtils.touch path
            stub_pkgconfig_package 'test-tasks-oroarch', pkg
            assert_equal path, @loader.task_library_path_from_name('test')
        end

        it "raises if the task library does not exist" do
            assert_raises(OroGen::TaskLibraryNotFound) do
                @loader.task_library_path_from_name('test')
            end
        end

        it "raises if the library cannot be found on disk" do
            pkg  = flexmock(name: 'test-tasks-oroarch', library_dirs: [@env_dir])
            path = File.join(@env_dir, 'libtest-tasks-oroarch.so')
            stub_pkgconfig_package 'test-tasks-oroarch', pkg
            assert_raises(OroGen::LibraryNotFound) do
                @loader.task_library_path_from_name('test')
            end
        end
    end

    describe "#typekit_library_path_from_name" do
        before do
            FileUtils.mkdir_p(@env_dir = Dir.mktmpdir)
            @loader = OroGen::Loaders::PkgConfig.new('oroarch')
        end
        after do
            FileUtils.rm_rf @env_dir
        end

        it "resolves an existing library from the pkg-config description" do
            pkg  = flexmock(name: 'test-typekit-oroarch', library_dirs: [@env_dir])
            path = File.join(@env_dir, 'libtest-typekit-oroarch.so')
            FileUtils.touch path
            stub_pkgconfig_package 'test-typekit-oroarch', pkg
            assert_equal path, @loader.typekit_library_path_from_name('test')
        end

        it "raises if the typekit does not exist" do
            assert_raises(OroGen::TypekitNotFound) do
                @loader.typekit_library_path_from_name('test')
            end
        end

        it "raises if the library cannot be found on disk" do
            pkg  = flexmock(name: 'test-typekit-oroarch', library_dirs: [@env_dir])
            path = File.join(@env_dir, 'libtest-typekit-oroarch.so')
            stub_pkgconfig_package 'test-typekit-oroarch', pkg
            assert_raises(OroGen::LibraryNotFound) do
                @loader.typekit_library_path_from_name('test')
            end
        end
    end

    describe "#transport_library_path_from_name" do
        before do
            FileUtils.mkdir_p(@env_dir = Dir.mktmpdir)
            @loader = OroGen::Loaders::PkgConfig.new('oroarch')
        end
        after do
            FileUtils.rm_rf @env_dir
        end

        it "resolves an existing library from the pkg-config description" do
            pkg  = flexmock(name: 'test-transport-trsp-oroarch', library_dirs: [@env_dir])
            path = File.join(@env_dir, 'libtest-transport-trsp-oroarch.so')
            FileUtils.touch path
            stub_pkgconfig_package 'test-transport-trsp-oroarch', pkg
            assert_equal path, @loader.transport_library_path_from_name('test', 'trsp')
        end

        it "raises if the transport does not exist" do
            assert_raises(OroGen::TransportNotFound) do
                @loader.transport_library_path_from_name('test', 'trsp')
            end
        end

        it "raises if the library cannot be found on disk" do
            pkg  = flexmock(name: 'test-transport-trsp-oroarch', library_dirs: [@env_dir])
            path = File.join(@env_dir, 'libtest-transport-trsp-oroarch.so')
            stub_pkgconfig_package 'test-transport-trsp-oroarch', pkg
            assert_raises(OroGen::LibraryNotFound) do
                @loader.transport_library_path_from_name('test', 'trsp')
            end
        end
    end
end

