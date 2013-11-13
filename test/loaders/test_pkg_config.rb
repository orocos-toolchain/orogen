require 'orogen'
require 'minitest/spec'
require 'flexmock'

describe OroGen::Loaders::PkgConfig do
    include FlexMock::ArgumentTypes
    include FlexMock::MockContainer

    attr_reader :pkg_config
    attr_reader :fixtures_prefix

    def stub_pkgconfig_package(pkg_name, pkg)
        @pkg_config ||= Hash.new
        pkg_config[pkg_name] = pkg
        flexmock(Utilrb::PkgConfig).should_receive('get').
            with(pkg_name, nil, Hash).and_return(pkg)
    end

    def stub_pkgconfig_each_package(filter)
        @pkg_config ||= Hash.new
        mock = flexmock(Utilrb::PkgConfig).should_receive(:each_package).with(filter, Proc)
        pkg_config.each do |name, pkg|
            if filter === name
                mock.and_yield(name)
            end
        end
    end

    def stub_orogen_pkgconfig_final
        stub_pkgconfig_each_package /^orogen-project-/
        stub_pkgconfig_each_package /-tasks-oroarch$/
        stub_pkgconfig_each_package /^orogen-\w+$/
        stub_pkgconfig_each_package /-typekit-oroarch$/
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
            :deployed_tasks => deployed_tasks.join(","))
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

    it "should be able to enumerate the projects" do
        pkg = stub_orogen_pkgconfig 'base'
        stub_orogen_pkgconfig_final
        loader = OroGen::Loaders::PkgConfig.new('oroarch')
        assert(project = loader.available_projects['base'])
        assert_equal pkg, project.pkg
        assert_equal pkg.deffile, project.orogen_path
    end

    it "should be able to enumerate the typekits" do
        pkg = stub_orogen_pkgconfig 'base'
        stub_orogen_pkgconfig_final
        loader = OroGen::Loaders::PkgConfig.new('oroarch')
        assert_equal loader.available_projects['base'].pkg, loader.available_typekits['base']
    end

    it "should be able to enumerate the types" do
        pkg = stub_orogen_pkgconfig 'base'
        stub_orogen_pkgconfig_final
        loader = OroGen::Loaders::PkgConfig.new('oroarch')
        assert(typekit = loader.available_types['/base/JointLimits'])
        assert_equal 'base', typekit.name
        assert typekit.exported
        assert(typekit = loader.available_types['/base/JointLimitRange'])
        assert_equal 'base', typekit.name
        assert !typekit.exported
    end

    it "should neither register the typekit nor the types if the project does not declare a typekit" do
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
        loader = OroGen::Loaders::PkgConfig.new('oroarch')
        assert !loader.available_typekits['base']
        assert !loader.available_types['/base/JointLimits']
    end

    it "should neither register the typekit nor the types if the corresponding project does not exist" do
        pkg = flexmock(
            :project_name => 'base',
            :deffile => File.join(fixtures_prefix, 'deffile', "base.orogen"),
            :type_registry => nil,
            :task_models => "",
            :deployed_tasks => "",
            :path => "bla")
        stub_pkgconfig_package("base-typekit-oroarch", pkg)
        stub_orogen_pkgconfig_final
        loader = OroGen::Loaders::PkgConfig.new('oroarch')
        assert !loader.available_typekits['base']
        assert !loader.available_types['/base/JointLimits']
    end

    it "should register the task library and the tasks" do
        pkg = stub_orogen_pkgconfig 'base', ["base::Task"]
        stub_orogen_pkgconfig_final
        loader = OroGen::Loaders::PkgConfig.new('oroarch')
        assert_equal pkg, loader.available_task_libraries['base']
        assert_equal 'base', loader.available_task_models['base::Task']
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
        assert !loader.available_types['/base/JointLimits']
    end

    it "should register the deployments" do
        pkg = stub_orogen_pkgconfig 'base', ["base::Task"], ["deployment1", "deployment2"]
        stub_orogen_pkgconfig_final
        loader = OroGen::Loaders::PkgConfig.new('oroarch')
        assert_equal pkg, loader.available_deployments['base']
        assert_equal ['base'].to_set, loader.available_deployed_tasks['deployment1']
        assert_equal ['base'].to_set, loader.available_deployed_tasks['deployment2']
    end

    it "should not register the deployments if the corresponding project does not exist" do
        pkg = flexmock(
            :project_name => 'base',
            :deffile => File.join(fixtures_prefix, 'deffile', "base.orogen"),
            :type_registry => nil,
            :task_models => "",
            :deployed_tasks => "",
            :path => "bla")
        stub_pkgconfig_package("orogen-base", pkg)
        stub_orogen_pkgconfig_final
        loader = OroGen::Loaders::PkgConfig.new('oroarch')
        assert !loader.available_deployments['base']
        assert !loader.available_deployed_tasks['deployment1']
    end
end

