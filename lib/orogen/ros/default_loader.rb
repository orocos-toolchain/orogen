module OroGen
    module ROS
        # A ROS loader that loads oroGen's built-in default definitions for ROS
        class DefaultLoader < OroGen::ROS::Loader
            def clear
                super
                if !search_path.include?(OroGen::ROS::OROGEN_ROS_LIB_DIR)
                    search_path << OroGen::ROS::OROGEN_ROS_LIB_DIR
                end
                project_model_from_name 'ros'
            end
        end
    end
end

