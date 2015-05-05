#include <rtt/os/main.h>

#include <boost/program_options.hpp>
#include <iostream>
#include <rtt/internal/GlobalEngine.hpp>
#include <rtt/TaskContext.hpp>

<% if deployer.corba_enabled? %>
#ifdef OROGEN_SERVICE_DISCOVERY_ACTIVATED
#include <service_discovery/ServiceDiscovery.hpp>
#endif // OROGEN_SERVICE_DISCOVERY_ACTIVATED
<% end %>

#include <rtt/typekit/RealTimeTypekit.hpp>
<% if deployer.transports.include?('corba') %>
#include <rtt/transports/corba/TransportPlugin.hpp>
<% end %>
<% if deployer.transports.include?('mqueue') %>
#include <rtt/transports/mqueue/TransportPlugin.hpp>
<% end %>

<% if project.typekit || !project.used_typekits.empty? %>
#include <rtt/types/TypekitPlugin.hpp>
<% end %>
<% if typekit = project.typekit %>
#include <<%= typekit.name %>/typekit/Plugin.hpp>
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

<% task_activities = deployer.task_activities.sort_by(&:name) %>
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

namespace orogen
{
<% task_activities.each do |task| %>
    extern RTT::TaskContext* create_<%= task.task_model.name.gsub(/[^\w]/, '_') %>(std::string const& instance_name);
<% end %>
}

namespace po = boost::program_options;

class Deinitializer
{
    friend Deinitializer& operator << (Deinitializer&, RTT::base::ActivityInterface&);

    std::vector<RTT::base::ActivityInterface*> m_activities;

<% if deployer.corba_enabled? %>
#ifdef OROGEN_SERVICE_DISCOVERY_ACTIVATED
    friend Deinitializer& operator << (Deinitializer&, servicediscovery::avahi::ServiceDiscovery&);

    std::vector<servicediscovery::avahi::ServiceDiscovery*> m_service_discoveries;
#endif
<% end %>


public:
    ~Deinitializer()
    {
        for (std::vector<RTT::base::ActivityInterface*>::const_iterator it = m_activities.begin();
                it != m_activities.end(); ++it)
        {
            (*it)->stop();
        }

<% if deployer.corba_enabled? %>
#ifdef OROGEN_SERVICE_DISCOVERY_ACTIVATED
        for(std::vector<servicediscovery::avahi::ServiceDiscovery*>::iterator sit = m_service_discoveries.begin();
                sit != m_service_discoveries.end(); ++sit)
        {
            (*sit)->stop();
            delete *sit;
        }
#endif 
<% end %>
    }
};

Deinitializer& operator << (Deinitializer& deinit, RTT::base::ActivityInterface& activity)
{
    deinit.m_activities.push_back(&activity);
    return deinit;
}

<% if deployer.corba_enabled? %>
#ifdef OROGEN_SERVICE_DISCOVERY_ACTIVATED
Deinitializer& operator << (Deinitializer& deinit, servicediscovery::avahi::ServiceDiscovery& service_discovery)
{
    deinit.m_service_discoveries.push_back(&service_discovery);
    return deinit;
}
#endif 
<% end %>

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
<% if deployer.corba_enabled? %>
#ifdef OROGEN_SERVICE_DISCOVERY_ACTIVATED
        ("sd-domain", po::value<std::string>(), "set service discovery domain")
#endif // OROGEN_SERVICE_DISOCVERY_ACTIVATED
<% end %>
        ("with-ros", po::value<bool>()->default_value(false), "also publish the task as ROS node, default is false")
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

   <% if project.typekit %>
   RTT::types::TypekitRepository::Import( new orogen_typekits::<%= project.name %>TypekitPlugin );
   <% deployer.transports.each do |transport_name| %>
   RTT::types::TypekitRepository::Import( new <%= typekit.transport_plugin_name(transport_name) %> );
   <% end %>
   <% end %>
   <% deployer.used_typekits.each do |tk| %>
   <% next if tk.virtual? %>
   RTT::types::TypekitRepository::Import( new orogen_typekits::<%= tk.name %>TypekitPlugin );
       <% deployer.transports.each do |transport_name| %>
   RTT::types::TypekitRepository::Import( new <%= RTT_CPP::Typekit.transport_plugin_name(transport_name, tk.name) %> );
       <% end %>
   <% end %>

<% if deployer.corba_enabled? %>
    RTT::corba::ApplicationServer::InitOrb(argc, argv);
<% end %>

    std::string prefix = "";

    if( vm.count("prefix")) 
        prefix = vm["prefix"].as<std::string>();

    bool with_ros = false;

    if( vm.count("with-ros"))
	with_ros = vm["with-ros"].as<bool>();

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
   
<% if lock_timeout = deployer.get_lock_timeout_no_period %>
    RTT::os::Thread::setLockTimeoutNoPeriod(<%= lock_timeout %>);
<% end %>

<% if lock_factor = deployer.get_lock_timeout_period_factor %>
    RTT::os::Thread::setLockTimeoutPeriodFactor(<%= lock_factor %>);
<% end %>

// Initialize some global threads so that we can properly setup their threading
// parameters
<% has_realtime = task_activities.all? { |t| t.realtime? } %>
<% if has_realtime %>
RTT::internal::GlobalEngine::Instance(ORO_SCHED_RT, RTT::os::LowestPriority);
<% else %>
RTT::internal::GlobalEngine::Instance(ORO_SCHED_OTHER, RTT::os::LowestPriority);
<% end %>

//First Create all Tasks to be able to set some (slave-) activities later on in the second loop
<% task_activities.each do |task| %>
    task_name = "<%= task.name %>";
    if (rename_map.count(task_name))
        task_name = rename_map[task_name];
    else
        task_name = prefix + task_name;
    
    std::auto_ptr<RTT::TaskContext> task_<%= task.name%>(
            orogen::create_<%= task.task_model.name.gsub(/[^\w]/, '_') %>(task_name));

    <% if deployer.corba_enabled? %>
    RTT::corba::TaskContextServer::Create( task_<%= task.name %>.get() );
    <% if task.realtime? %>
    RTT::corba::CorbaDispatcher::Instance( task_<%= task.name %>->ports(), ORO_SCHED_RT, RTT::os::LowestPriority );
    <% else %>
    RTT::corba::CorbaDispatcher::Instance( task_<%= task.name %>->ports(), ORO_SCHED_OTHER, RTT::os::LowestPriority );
    <% end %>
    <% end %>

<% end %>

//Create all Activities afterwards to be sure all tasks are created. The Activitied are also handeld by the deployment because
//the order needs to be known since slav activities are useable
//
<% activity_ordered_tasks.each do |task| %>
    <%= task.generate_activity_setup %>
    <% if timeout = task.stop_timeout %>
    { RTT::os::Thread* thread = dynamic_cast<RTT::os::Thread*>(activity_<%= task.name %>);
        if (thread)
            thread->setStopTimeout(<%= timeout %>);
    }
    <% end %>
    task_<%= task.name %>->setActivity(activity_<%= task.name %>);
<% end %>



   Deinitializer deinit;

<% if deployer.corba_enabled? %>
#ifdef OROGEN_SERVICE_DISCOVERY_ACTIVATED
    if( vm.count("sd-domain") ) {
<% task_activities.each do |task| %>
    servicediscovery::avahi::ServiceConfiguration sd_conf_<%= task.name%>(task_<%= task.name%>->getName(), vm["sd-domain"].as<std::string>());
    sd_conf_<%= task.name%>.setDescription("IOR", RTT::corba::TaskContextServer::getIOR(task_<%= task.name%>.get()));
    sd_conf_<%= task.name%>.setDescription("TASK_MODEL","<%= task.task_model.name %>");
    servicediscovery::avahi::ServiceDiscovery* sd_<%= task.name%> = new servicediscovery::avahi::ServiceDiscovery();
    deinit << *sd_<%= task.name%>;
    sd_<%= task.name%>->start(sd_conf_<%= task.name%>);
<% end %>
    }
#endif // OROGEN_SERVICE_DISCOVERY_ACTIVATED
<% end %>

<% all_peers = deployer.peers.dup.to_a
   all_peers.concat deployer.each_task.inject(Array.new) { |a, m| a.concat m.slaves.map { |s| [m, s] } }
   all_peers.sort_by { |a, b| [a.name, b.name] }.each do |a, b| %>
    task_<%= a.name %>->connectPeers(task_<%= b.name %>.get());
<% end %>

<% deployer.connections.
    sort_by { |src, dst, policy| [src.name, dst.name] }.
    each do |src, dst, policy|
        if src.kind_of?(Spec::TaskDeployment) %>
            task_<%= src.activity.name %>->connectPorts(task_<%= dst.activity.name %>.get());
        <% else %>
        {
            <%= policy.to_code("policy") %>
            RTT::base::OutputPortInterface* src = dynamic_cast<RTT::base::OutputPortInterface*>(
                    task_<%= src.activity.name %>->ports()->getPort("<%= src.name %>"));
            RTT::base::InputPortInterface* dst = dynamic_cast<RTT::base::InputPortInterface*>(
                    task_<%= dst.activity.name %>->ports()->getPort("<%= dst.name %>"));
            src->createConnection(*dst, policy);
        }
        <% end %>
    <% end %>

    // Start some activities
<% task_activities.each do |task| %>
    deinit << *activity_<%= task.name%>;

    <% if task.start?
        if task.context.needs_configuration? %>
    if (!task_<%= task.name %>->configure())
    {
        RTT::log(RTT::Error) << "cannot configure <%= task.name %>" << RTT::endlog();
        return -1;
    }
        <% end %>
    if (!task_<%= task.name %>->start())
    {
        RTT::log(RTT::Error) << "cannot start <%= task.name %>" << RTT::endlog();
        return -1;
    }
    <% end %>
<% end %>

    if(with_ros){
<% if deployer.transports.include? 'ros' %>
        RTT::log(RTT::Info)<<"Initializing ROS node"<<RTT::endlog();
        if(!ros::isInitialized()){
            int argc =__os_main_argc();
            char ** argv = __os_main_argv();
            ros::init(argc,argv,prefix + "<%= deployer.name %>");
          if(ros::master::check())
              ros::start();
          else{
              RTT::log(RTT::Error)<<"No ros::master available"<<RTT::endlog();
              return false;
          }
        }
        static ros::AsyncSpinner spinner(1); // Use 1 threads
        spinner.start();
        RTT::log(RTT::Info)<<"ROS node spinner started"<<RTT::endlog();
<% else %>
        throw std::runtime_error("Requesting to start as ROS node, but the support for 'ros' transport is not available. Recompile with 'ros' transport option!");
<% end %>
    }

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
    <% if has_realtime %>
    RTT::corba::TaskContextServer::ThreadOrb(ORO_SCHED_RT, RTT::os::LowestPriority, 0);
    <% else %>
    RTT::corba::TaskContextServer::ThreadOrb(ORO_SCHED_OTHER, RTT::os::LowestPriority, 0);
    <% end %>
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
    OCL::TaskBrowser browser(task_<%= deployer.browse.name %>.get());
    browser.loop();
<% end %>

    return 0;
}

