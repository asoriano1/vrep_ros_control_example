#include "../v_repLib.h"
#include "vrepRosControl_plugin.h"
#include "vrepRosControl_server.h"

#include <iostream>

#include <urdf/model.h>


#define PLUGIN_VERSION 1

std::string vrepRosControl_pluginName = "vrepRosControl";

std::string robot_description_ = "robot_description"; //change the name of the parameter if it's needed

LIBRARY vrepLib; // the V-REP library that we will dynamically load and bind

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// This is the plugin start routine (called just once, just after the plugin was loaded):
VREP_DLLEXPORT unsigned char v_repStart(void* reservedPointer,int reservedInt)
{
	
	// Dynamically load and bind V-REP functions:
	// ******************************************
	// 1. Figure out this plugin's directory:
	char curDirAndFile[1024];
	getcwd(curDirAndFile, sizeof(curDirAndFile));

	std::string currentDirAndPath(curDirAndFile);

	// 2. Append the V-REP library's name:
	std::string temp(currentDirAndPath);
	temp+="/libv_rep.so";

	// 3. Load the V-REP library:
	vrepLib=loadVrepLibrary(temp.c_str());
	if (vrepLib==NULL)
	{
        std::cout << "Error, could not find or correctly load the V-REP library. Cannot start '" << vrepRosControl_pluginName << "' plugin.\n";
		return(0); // Means error, V-REP will unload this plugin
	}
	if (getVrepProcAddresses(vrepLib)==0)
	{
        std::cout << "Error, could not find all required functions in the V-REP library. Cannot start '" << vrepRosControl_pluginName << "' plugin.\n";
		unloadVrepLibrary(vrepLib);
		return(0); // Means error, V-REP will unload this plugin
	}
	// ******************************************

	// Check the version of V-REP:
	// ******************************************
	int vrepVer;
	simGetIntegerParameter(sim_intparam_program_version,&vrepVer);
	if (vrepVer<30102) // if V-REP version is smaller than 3.01.02
	{
        std::cout << "Sorry, your V-REP copy is somewhat old. Cannot start '" << vrepRosControl_pluginName << "' plugin.\n";
		unloadVrepLibrary(vrepLib);
		return(0); // Means error, V-REP will unload this plugin
	}
	// ******************************************
	
	
	// Initialize the ROS part:
	if(!ROS_server::initialize()) 
	{
        std::cout << "ROS master is not running. Cannot start '" << vrepRosControl_pluginName << "' plugin.\n";
		return (0); //If the master is not running then the plugin is not loaded.
	}
	
	// ********************
	
	// Read urdf from robot_description param
	// ********************
	// Get parameters/settings for controllers from ROS param server
	//model_nh_ = ros::NodeHandle("");
	//const std::string urdf_string = getURDF(robot_description_);
	ros::NodeHandle model_nh_ = ros::NodeHandle("");
	const std::string urdf_string = getURDF(robot_description_, model_nh_);
	
	/*if (!parseTransmissionsFromURDF(urdf_string))
  {
    ROS_ERROR_NAMED("vrep_ros_control", "Error parsing URDF in vrep_ros_control plugin, plugin not active.\n");
    return (0);
  }

  // Load the RobotHWSim abstraction to interface the controllers with the gazebo model
  try
  {
    robot_hw_sim_loader_.reset
      (new pluginlib::ClassLoader<gazebo_ros_control::RobotHWSim>
        ("gazebo_ros_control",
          "gazebo_ros_control::RobotHWSim"));

    robot_hw_sim_ = robot_hw_sim_loader_->createInstance(robot_hw_sim_type_str_);
    urdf::Model urdf_model;
    const urdf::Model *const urdf_model_ptr = urdf_model.initString(urdf_string) ? &urdf_model : NULL;

    if(!robot_hw_sim_->initSim(robot_namespace_, model_nh_, parent_model_, urdf_model_ptr, transmissions_))
    {
      ROS_FATAL_NAMED("gazebo_ros_control","Could not initialize robot simulation interface");
      return;
    }

    // Create the controller manager
    ROS_DEBUG_STREAM_NAMED("ros_control_plugin","Loading controller_manager");
    controller_manager_.reset
      (new controller_manager::ControllerManager(robot_hw_sim_.get(), model_nh_));

    // Listen to the update event. This event is broadcast every simulation iteration.
    update_connection_ =
      gazebo::event::Events::ConnectWorldUpdateBegin
      (boost::bind(&GazeboRosControlPlugin::Update, this));

  }
  catch(pluginlib::LibraryLoadException &ex)
  {
    ROS_FATAL_STREAM_NAMED("gazebo_ros_control","Failed to create robot simulation interface loader: "<<ex.what());
  }

  ROS_INFO_NAMED("gazebo_ros_control", "Loaded gazebo_ros_control.");*/
	
	
	
	

	return(PLUGIN_VERSION); // initialization went fine, we return the version number of this plugin (can be queried with simGetModuleName)
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// This is the plugin end routine (called just once, when V-REP is ending, i.e. releasing this plugin):
VREP_DLLEXPORT void v_repEnd()
{
	ROS_server::shutDown();	// shutdown the ROS_server

	unloadVrepLibrary(vrepLib); // release the library
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// This is the plugin messaging routine (i.e. V-REP calls this function very often, with various messages):
VREP_DLLEXPORT void* v_repMessage(int message,int* auxiliaryData,void* customData,int* replyData)
{ 
	// This is called quite often. Just watch out for messages/events you want to handle
	// Keep following 4 lines at the beginning and unchanged:
	int errorModeSaved;
	simGetIntegerParameter(sim_intparam_error_report_mode,&errorModeSaved);
	simSetIntegerParameter(sim_intparam_error_report_mode,sim_api_errormessage_ignore);
	void* retVal=NULL;

	// Here we can intercept many messages from V-REP (actually callbacks). Only the most important messages are listed here:

	if (message==sim_message_eventcallback_instancepass)
	{ 
		// This message is sent each time the scene was rendered (well, shortly after) (very often)
		// When a simulation is not running, but you still need to execute some commands, then put some code here

		ROS_server::instancePass();
	}

	if (message==sim_message_eventcallback_mainscriptabouttobecalled)
	{ 
		// Main script is about to be run (only called while a simulation is running (and not paused!))
		//
		// This is a good location to execute simulation commands
		
		ROS_server::mainScriptAboutToBeCalled();
	}

	if (message==sim_message_eventcallback_simulationabouttostart)
	{ 
	    // Simulation is about to start
		
		ROS_server::simulationAboutToStart();
	}

	if (message==sim_message_eventcallback_simulationended)
	{ 
		// Simulation just ended
		
		ROS_server::simulationEnded();
	}

	// Keep following unchanged:
	simSetIntegerParameter(sim_intparam_error_report_mode,errorModeSaved); // restore previous settings
	return(retVal);
}

// Get the URDF XML from the parameter server
std::string getURDF(std::string param_name, ros::NodeHandle model_nh_)
{
  std::string urdf_string;

  // search and wait for robot_description on param server
  while (urdf_string.empty())
  {
    std::string search_param_name;
    if (model_nh_.searchParam(param_name, search_param_name))
    {
      ROS_INFO_ONCE_NAMED("vrep_ros_control", "vrep_ros_control plugin is waiting for model"
        " URDF in parameter [%s] on the ROS param server.", search_param_name.c_str());

      model_nh_.getParam(search_param_name, urdf_string);
    }
    else
    {
      ROS_INFO_ONCE_NAMED("vrep_ros_control", "vrep_ros_control plugin is waiting for model"
        " URDF in parameter [%s] on the ROS param server.", robot_description_.c_str());

      model_nh_.getParam(param_name, urdf_string);
    }

    usleep(100000);
  }
  ROS_DEBUG_STREAM_NAMED("vrep_ros_control", "Recieved urdf from param server, parsing...");

  return urdf_string;
}

// Get Transmissions from the URDF
bool parseTransmissionsFromURDF(const std::string& urdf_string)
{
  transmission_interface::TransmissionParser::parse(urdf_string, transmissions_);
  return true;
}
