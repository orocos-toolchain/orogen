#include <rtt/os/main.h>

#include <boost/program_options.hpp>
#include <iostream>

#ifdef OROGEN_SERVICE_DISCOVERY_ACTIVATED
#include <service_discovery/service_discovery.h>
#endif // OROGEN_SERVICE_DISCOVERY_ACTIVATED

#include <rtt/typekit/RealTimeTypekit.hpp>
<% if deployer.transports.include?('corba') %>
#include <rtt/transports/corba/TransportPlugin.hpp>
<% end %>
<% if deployer.transports.include?('mqueue') %>
#include <rtt/transports/mqueue/TransportPlugin.hpp>
<% end %>

<% if component.typekit || !component.used_typekits.empty? %>#include <rtt/types/TypekitPlugin.hpp><% end %>
<% if typekit = component.typekit %>
#include "typekit/Plugin.hpp"
<% deployer.transports.each do |transport_name| %>
#include "typekit/transports/<%= transport_name %>/TransportPlugin.hpp"
<% end %>
<% end %>
<% deployer.used_typekits.each do |tk| %>
    <% next if tk.virtual? %>
#include <<%= tk.name %>/typekit/Plugin.hpp>
    <% deployer.transports.each do |transport_name| %>
#include <<%= tk.name %>/transports/<%= transport_name %>/TransportPlugin.hpp>
    <% end %>
<% end %>

<% task_activities = deployer.task_activities.
        sort_by(&:name) %>

<% task_activities.each do |task| %>
#include <<%= task.context.header_file %>>
<% end %>
<% if deployer.corba_enabled? %>
#include <rtt/transports/corba/ApplicationServer.hpp>
#include <rtt/transports/corba/TaskContextServer.hpp>
#include <rtt/transports/corba/CorbaDispatcher.hpp>
#include <signal.h>
<% end %>
<% if deployer.transports.include? 'ros' %>
#include <ros/ros.h>
<% end %>

<% require 'set'
    activity_headers = task_activities.
        map { |t| t.activity_type.header }.
        compact.
        to_set %>
<% activity_headers.to_a.sort.each do |header| %>
#include <<%= header %>>
<% end %>

<% if deployer.browse %>
#include <ocl/TaskBrowser.hpp>
<% end %>

#include <rtt/Logger.hpp>
#include <rtt/base/ActivityInterface.hpp>

namespace po = boost::program_options;

class Deinitializer
{
    friend Deinitializer& operator << (Deinitializer&, RTT::base::ActivityInterface&);

    std::vector<RTT::base::ActivityInterface*> m_activities;

#ifdef OROGEN_SERVICE_DISCOVERY_ACTIVATED
    friend Deinitializer& operator << (Deinitializer&, servicediscovery::ServiceDiscovery&);

    std::vector<servicediscovery::ServiceDiscovery*> m_service_discoveries;
#endif


public:
    ~Deinitializer()
    {
        for (std::vector<RTT::base::ActivityInterface*>::const_iterator it = m_activities.begin();
                it != m_activities.end(); ++it)
        {
            (*it)->stop();
        }

#ifdef OROGEN_SERVICE_DISCOVERY_ACTIVATED
        for(std::vector<servicediscovery::ServiceDiscovery*>::iterator sit = m_service_discoveries.begin();
                sit != m_service_discoveries.end(); ++sit)
        {
            (*sit)->stop();
            delete *sit;
        }
#endif 
    }
};

Deinitializer& operator << (Deinitializer& deinit, RTT::base::ActivityInterface& activity)
{
    deinit.m_activities.push_back(&activity);
    return deinit;
}

#ifdef OROGEN_SERVICE_DISCOVERY_ACTIVATED
Deinitializer& operator << (Deinitializer& deinit, servicediscovery::ServiceDiscovery& service_discovery)
{
    deinit.m_service_discoveries.push_back(&service_discovery);
    return deinit;
}
#endif 

<% if deployer.corba_enabled? %>
int sigint_com[2];
void sigint_quit_orb(int)
{
    uint8_t dummy = 0;
    unsigned int sent = 0;
    while(sent < sizeof(uint8_t))
    {
	int ret = write(sigint_com[1], &dummy, sizeof(uint8_t));
	if(ret < 0)
	{
	    std::cerr << "Failed to signal quit to orb" << std::endl;
	    break;
	}
	sent += ret;
    }
}
<% end %>

int ORO_main(int argc, char* argv[])
{
   po::options_description desc("Options");

   desc.add_options()
        ("help", "show all available options supported by this deployment")
        ("prefix", po::value<std::string>(), "Sets a prefix for all TaskContext names")
#ifdef OROGEN_SERVICE_DISCOVERY_ACTIVATED
        ("sd-domain", po::value<std::string>(), "set service discovery domain")
#endif // OROGEN_SERVICE_DISOCVERY_ACTIVATED
        ("rename", po::value< std::vector<std::string> >(), "rename a task of the deployment: --rename oldname:newname");

   po::variables_map vm;
   po::store(po::parse_command_line(argc, argv, desc), vm);
   po::notify(vm);

   if(vm.count("help")) {
       std::cout << desc << std::endl;
       return 0;
   }


   <% if deployer.loglevel %>
   if ( log().getLogLevel() < Logger::<%= deployer.loglevel %> ) {
       log().setLogLevel( Logger::<%= deployer.loglevel %> );
   }
   <% end %>

   RTT::types::TypekitRepository::Import( new RTT::types::RealTimeTypekitPlugin );
   <% if deployer.transports.include?('corba') %>
   RTT::types::TypekitRepository::Import( new RTT::corba::CorbaLibPlugin );
   <% end %>
   <% if deployer.transports.include?('mqueue') %>
   RTT::types::TypekitRepository::Import( new RTT::mqueue::MQLibPlugin );
   <% end %>

   <% if component.typekit %>
   RTT::types::TypekitRepository::Import( new orogen_typekits::<%= component.name %>TypekitPlugin );
   <% deployer.transports.each do |transport_name| %>
   RTT::types::TypekitRepository::Import( new <%= typekit.transport_plugin_name(transport_name) %> );
   <% end %>
   <% end %>
   <% deployer.used_typekits.each do |tk| %>
   <% next if tk.virtual? %>
   RTT::types::TypekitRepository::Import( new orogen_typekits::<%= tk.name %>TypekitPlugin );
       <% deployer.transports.each do |transport_name| %>
   RTT::types::TypekitRepository::Import( new <%= Orocos::Generation::Typekit.transport_plugin_name(transport_name, tk.name) %> );
       <% end %>
   <% end %>

<% if deployer.corba_enabled? %>
    RTT::corba::ApplicationServer::InitOrb(argc, argv);
<% end %>

    std::string prefix = "";

    if( vm.count("prefix")) 
        prefix = vm["prefix"].as<std::string>();

    std::string task_name;

    std::map<std::string, std::string> rename_map;

    if ( vm.count("rename") ) {

        const std::vector< std::string>& ren_vec = vm["rename"].as<std::vector <std::string> >();

        for ( unsigned int i = 0; i < ren_vec.size(); i++) {

            const std::string& ren_str = ren_vec.at(i);

            unsigned int colon_pos = ren_str.find(':');
            if ( colon_pos == std::string::npos ) continue;

            rename_map.insert( std::pair<std::string, std::string>( 
                ren_str.substr(0,colon_pos), ren_str.substr(colon_pos+1) ));
        }
    }    

<% task_activities.each do |task| %>
    task_name = "<%= task.name %>";
    if (rename_map.count(task_name))
        task_name = rename_map[task_name];
    else
        task_name = prefix + task_name;
    
    <%= task.context.class_name %> task_<%= task.name%>(task_name);
    <%= task.generate_activity_setup %>
    task_<%= task.name %>.setActivity(activity_<%= task.name %>);
    <% task.properties.sort_by { |prop| prop.name }.each do |prop|
        if prop.value %>
    dynamic_cast< RTT::Property<  <%= prop.interface_object.type.cxx_name %> >*>(
            task_<%= task.name %>.properties()->getProperty("<%= prop.name %>"))
        ->set(<%= prop.value.inspect %>);
        <% end %>
    <% end %>

    <% if deployer.corba_enabled? %>
    RTT::corba::TaskContextServer::Create( &task_<%= task.name%> );
    <% if task.realtime? %>
    RTT::corba::CorbaDispatcher::Instance( task_<%= task.name %>.ports(), ORO_SCHED_RT, RTT::os::LowestPriority );
    <% else %>
    RTT::corba::CorbaDispatcher::Instance( task_<%= task.name %>.ports(), ORO_SCHED_OTHER, RTT::os::LowestPriority );
    <% end %>
    <% end %>

<% end %>

   Deinitializer deinit;

#ifdef OROGEN_SERVICE_DISCOVERY_ACTIVATED
    if( vm.count("sd-domain") ) {
<% task_activities.each do |task| %>
    servicediscovery::ServiceConfiguration sd_conf_<%= task.name%>(prefix + "<%= task.name %>", vm["sd-domain"].as<std::string>());
    sd_conf_<%= task.name%>.setDescription("IOR", RTT::corba::TaskContextServer::getIOR(&task_<%= task.name%>));
    servicediscovery::ServiceDiscovery* sd_<%= task.name%> = new servicediscovery::ServiceDiscovery();
    deinit << *sd_<%= task.name%>;
    sd_<%= task.name%>->start(sd_conf_<%= task.name%>);
<% end %>
    }
#endif // OROGEN_SERVICE_DISCOVERY_ACTIVATED

<% if !deployer.loggers.empty?
        deployer.loggers.sort_by { |filename, _| filename }.each do |filename, logger|
            logger.config.each do |type, reported_activity, args| %>
                task_<%= logger.task.name %>.connectPeers(&task_<%= reported_activity.name %>);
                task_<%= logger.task.name %>.report<%= type %>(<%= args.map { |v| "\"#{v}\"" }.join(", ") %>);
            <% end %>
        <% end %>
<% end %>


<% deployer.peers.sort_by { |a, b| [a.name, b.name] }.each do |a, b| %>
    task_<%= a.name %>.connectPeers(&task_<%= b.name %>);
<% end %>

<% deployer.connections.
    sort_by { |src, dst, policy| [src.name, dst.name] }.
    each do |src, dst, policy|
        if src.kind_of?(TaskDeployment) %>
            task_<%= src.activity.name %>.connectPorts(&task_<%= dst.activity.name %>);
        <% else %>
        {
            <%= policy.to_code("policy") %>
            RTT::base::OutputPortInterface* src = dynamic_cast<RTT::base::OutputPortInterface*>(
                    task_<%= src.activity.name %>.ports()->getPort("<%= src.name %>"));
            RTT::base::InputPortInterface* dst = dynamic_cast<RTT::base::InputPortInterface*>(
                    task_<%= dst.activity.name %>.ports()->getPort("<%= dst.name %>"));
            src->createConnection(*dst, policy);
        }
        <% end %>
    <% end %>

    // Start some activities
<% task_activities.each do |task| %>
    deinit << *activity_<%= task.name%>;

    <% if task.start?
        if task.context.needs_configuration? %>
    if (!task_<%= task.name %>.configure())
    {
        RTT::log(RTT::Error) << "cannot configure <%= task.name %>" << RTT::endlog();
        return -1;
    }
        <% end %>
    if (!task_<%= task.name %>.start())
    {
        RTT::log(RTT::Error) << "cannot start <%= task.name %>" << RTT::endlog();
        return -1;
    }
    <% end %>
<% end %>

<% if deployer.transports.include? 'ros' %>
    RTT::log(RTT::Info)<<"Initializing ROS node"<<RTT::endlog();
    if(!ros::isInitialized()){
        int argc =__os_main_argc();
        char ** argv = __os_main_argv();
        ros::init(argc,argv,"rtt",ros::init_options::AnonymousName);
      if(ros::master::check())
          ros::start();
      else{
          RTT::log(RTT::Error)<<"No ros::master available"<<RTT::endlog();
          ros::shutdown();
          return false;
      }   
    }
    static ros::AsyncSpinner spinner(1); // Use 1 threads
    spinner.start();
    RTT::log(RTT::Info)<<"ROS node spinner started"<<RTT::endlog();
<% end %>

<% if deployer.corba_enabled? %>
    /** Setup shutdown procedure on SIGINT. We use a pipe-based channel to do
        so, as we can't shutdown the ORB from the signal handler */
    if (pipe(sigint_com) == -1)
    {
        std::cerr << "failed to setup SIGINT handler: " << strerror(errno) << std::endl;
        return 1;
    }

    struct sigaction sigint_handler;
    sigint_handler.sa_handler = &sigint_quit_orb;
    sigemptyset(&sigint_handler.sa_mask);
    sigint_handler.sa_flags     = 0;
    sigint_handler.sa_restorer  = 0;
    if (-1 == sigaction(SIGINT, &sigint_handler, 0))
    {
        std::cerr << "failed to install SIGINT handler" << std::endl;
        return 1;
    }
    sigset_t unblock_sigint;
    sigemptyset(&unblock_sigint);
    sigaddset(&unblock_sigint, SIGINT);
    if (-1 == sigprocmask(SIG_UNBLOCK, &unblock_sigint, NULL))
    {
        std::cerr << "failed to install SIGINT handler" << std::endl;
        return 1;
    }
    RTT::corba::TaskContextServer::ThreadOrb();
    while (true)
    {
        uint8_t dummy;
        int read_count = read(sigint_com[0], &dummy, 1);
        if (read_count == 1)
            break;
    }

    RTT::corba::TaskContextServer::ShutdownOrb();
    RTT::corba::TaskContextServer::DestroyOrb();
<% elsif deployer.browse %>
    OCL::TaskBrowser browser(& task_<%= deployer.browse.name %>);
    browser.loop();
<% end %>

    return 0;
}

