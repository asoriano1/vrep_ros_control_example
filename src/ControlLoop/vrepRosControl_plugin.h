#pragma once

#define VREP_DLLEXPORT extern "C"

// ROS
#include <ros/ros.h>
#include <pluginlib/class_loader.h>
#include <std_msgs/Bool.h>

// ros_control
//#include <gazebo_ros_control/robot_hw_sim.h>
#include <controller_manager/controller_manager.h>
#include <transmission_interface/transmission_parser.h>

// The 3 required entry points of the V-REP plugin:
VREP_DLLEXPORT unsigned char v_repStart(void* reservedPointer,int reservedInt);
VREP_DLLEXPORT void v_repEnd();
VREP_DLLEXPORT void* v_repMessage(int message,int* auxiliaryData,void* customData,int* replyData);


// Get the URDF XML from the parameter server
std::string getURDF(std::string param_name, ros::NodeHandle model_nh_);
	
// Get Transmissions from the URDF
bool parseTransmissionsFromURDF(const std::string& urdf_string);

// Transmissions in this plugin's scope
std::vector<transmission_interface::TransmissionInfo> transmissions_;
