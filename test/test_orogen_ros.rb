require 'orogen_ros/test'

describe Orogen::ROS do
    include Orogen::ROS::SelfTest

    before do
        # Code that is run before each test
    end
    after do
        # Code that is run after each test
    end

    # We advise you to create one more "describe" subblock per method on the
    # tested class. The before/after blocks from the parent scope are executed,
    # and you can add some specific to this specific method
    describe "#my_method" do
        before do
        end
        after do
        end
        it "should do something right" do
        end
    end
end
