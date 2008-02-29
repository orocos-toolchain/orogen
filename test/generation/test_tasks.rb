require 'orocos/generation/test'
require 'orocos/generation/tasks'

class TC_GenerationTasks < Test::Unit::TestCase
    include Orocos::Generation::Test
    TEST_DATA_DIR = File.join( TEST_DIR, 'generation', 'data' )

    def test_task
	in_wc do
	    component = Generation.new do
		name 'test'
	    end

	    name = 'task_name'
	    doc  = 'task doc'

	    task = component.task_context(name)
	    assert_raises(ArgumentError) { component.task_context(name) }

	    assert_kind_of(Generation::TaskContext, task)
	    assert_equal(name, task.name)

	    # Check name validation
	    assert_raises(ArgumentError) { component.task_context("bla bla") }
	    assert_raises(ArgumentError) { component.task_context("bla(bla") }
	    assert_raises(ArgumentError) { component.task_context("bla!bla") }
	    assert_raises(ArgumentError) { component.task_context("bla/bla") }
	end
    end

    def test_task_property
	in_wc do
	    component = Generation.new do
		name 'test'
	    end

	    name = 'property_name'
	    doc  = 'property doc'
	    type = '/std/vector<double>'

	    task = component.task_context(name)
	    property = task.property(name, type).
		doc(doc)

	    assert_kind_of(Generation::Property, property)
	    assert_equal(name, property.name)
	    assert_equal(type, property.type.full_name)
	    assert_equal(doc,  property.doc)
	end
    end

    def test_task_method
	in_wc do
	    component = Generation.new do
		name 'test'
	    end

	    name = 'method_name'
	    doc  = 'method doc'

	    task = component.task_context(name)

	    meth = task.method("#{name}-1").
		doc("#{doc}-1")

	    assert_kind_of(Generation::Method, meth)
	    assert_equal(name, meth.name)
	    assert_equal(nil,  meth.return_type)
	    assert_equal(doc,  meth.doc)
	    assert(meth.arguments.empty?)
	end
    end
end

