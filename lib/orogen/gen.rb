require 'orogen'

module OroGen
    module Gen
        extend Logger::Hierarchy

        module RTT_CPP
            extend Logger::Hierarchy

            ConfigError = OroGen::ConfigError
            OROGEN_LIB_DIR = OroGen::OROGEN_LIB_DIR

            ConfigurationObject = Spec::ConfigurationObject
            Attribute           = Spec::Attribute
            Property            = Spec::Property

            Operation           = Spec::Operation

            Port                = Spec::Port
            OutputPort          = Spec::OutputPort
            InputPort           = Spec::InputPort
            DynamicInputPort    = Spec::DynamicInputPort
            DynamicOutputPort   = Spec::DynamicInputPort

            TaskContext         = Spec::TaskContext
        end
    end
    Generation = Gen::RTT_CPP
end

require 'orogen/gen/enable'
require 'orogen/gen/base'
require 'orogen/gen/templates'
require 'orogen/gen/typekit'
require 'orogen/marshallers'
require 'orogen/gen/deployment'
require 'orogen/gen/tasks'
require 'orogen/gen/project'
require 'orogen/gen/imports'
OroGen::Gen::RTT_CPP::Typekit.register_plugin(OroGen::TypekitMarshallers::ROS::Plugin)
OroGen::Gen::RTT_CPP::Typekit.register_plugin(OroGen::TypekitMarshallers::Corba::Plugin)
OroGen::Gen::RTT_CPP::Typekit.register_plugin(OroGen::TypekitMarshallers::MQueue::Plugin)
OroGen::Gen::RTT_CPP::Typekit.register_plugin(OroGen::TypekitMarshallers::TypeInfo::Plugin)
OroGen::Gen::RTT_CPP::Typekit.register_plugin(OroGen::TypekitMarshallers::TypelibMarshaller::Plugin)

OroGen::Gen::RTT_CPP::Deployment.register_global_initializer(
    :qt,
    global_scope: <<~QT_GLOBAL_SCOPE,
        static int QT_ARGC = 1;
        static char const* QT_ARGV[] = { "orogen", nullptr };
        #include <pthread.h>
        #include <QApplication>
        void* qt_thread_main(void*)
        {
            QApplication *qapp = new QApplication(QT_ARGC, const_cast<char**>(QT_ARGV));
            qapp->setQuitOnLastWindowClosed(false);
            reinterpret_cast<QCoreApplication*>(qapp)->exec();
            return NULL;
        }
    QT_GLOBAL_SCOPE
    init: <<~QT_INIT_CODE,
        pthread_t qt_thread;
        pthread_create(&qt_thread, NULL, qt_thread_main, NULL);
    QT_INIT_CODE
    exit: <<~QT_EXIT_CODE,
        QApplication::instance()->exit();
        pthread_join(qt_thread, NULL);
    QT_EXIT_CODE
    tasks_cmake: <<~QT_DEPLOYMENT_CMAKE,
        find_package(Qt4 REQUIRED)
        include(${QT_USE_FILE})
        include_directories(${QT_INCLUDE_DIR})
        link_directories(${QT_LIBRARY_DIR})
        set(CMAKE_AUTOMOC true)
    QT_DEPLOYMENT_CMAKE
    deployment_cmake: <<~QT_DEPLOYMENT_CMAKE,
        find_package(Qt4 REQUIRED)
        include(${QT_USE_FILE})
        include_directories(${QT_INCLUDE_DIR})
        link_directories(${QT_LIBRARY_DIR})
        target_link_libraries(<%= deployer.name %> ${QT_LIBRARIES})
        set(CMAKE_AUTOMOC true)
    QT_DEPLOYMENT_CMAKE
)
