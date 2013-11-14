#ifndef __DENSO_SIMULATION_DENSO_SIM_H
#define __DENSO_SIMULATION_DENSO_SIM_H

#include <ros/ros.h>
#include <angles/angles.h>
#include <pluginlib/class_list_macros.h>

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>

#include <gazebo_ros_control/robot_hw_sim.h>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>

namespace denso_simulation {

  class DENSOSim {
  public:

    DENSOSim(const std::string &prefix,
        hardware_interface::JointStateInterface &js_interface,
        hardware_interface::EffortJointInterface &ej_interface);

    bool initSim(ros::NodeHandle nh, gazebo::physics::ModelPtr parent_model, std::vector<gazebo_ros_control::JointData> joints);
    void readSim(ros::Time time, ros::Duration period);
    void writeSim(ros::Time time, ros::Duration period);

  private:

    unsigned int n_dof_;
    unsigned int n_actuated_dof_;
    bool initialized_;
    std::string prefix_;

    hardware_interface::JointStateInterface &js_interface_;
    hardware_interface::EffortJointInterface &ej_interface_;

    std::vector<std::string> joint_name_;
    std::vector<double> joint_position_;
    std::vector<double> joint_velocity_;
    std::vector<double> joint_effort_;
    std::vector<double> joint_effort_command_;

    std::vector<gazebo::physics::JointPtr> sim_joints_;
  };
}

#endif // ifndef __DENSO_SIMULATION_DENSO_SIM_H
