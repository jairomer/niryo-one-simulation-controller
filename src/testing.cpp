/**
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *  
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *  
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */ 
#include "testing.hpp"
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <std_msgs/Bool.h>
#include <math.h>
#include <string>
#include <iostream>
#include "simulation.hpp"
#include "physical.hpp"
#include "ros/ros.h"

/** TODO: Assemble a series of tests to assert that the digital twin is processing requests. */
bool digital_twin_integration_tests(int argc, char** argv)
{
    return true;
}


/**
 * Physical twin control from the controller node. 
 * Clocks must be synchronized using ptpd if running on different machines. 
*/
void test_physical_twin_control(int argc, char** argv)
{
    ros::init(argc, argv, "physical_twin_controller");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1); 
    spinner.start();

    static const std::string ARM_GROUP = "arm";
    //static const std::string TOOL_GROUP = "tool_action";

    moveit::planning_interface::MoveGroupInterface arm_group(ARM_GROUP);
    const robot_state::JointModelGroup* joint_model_group = 
        arm_group.getCurrentState()->getJointModelGroup(ARM_GROUP);

    //moveit::planning_interface::MoveGroupInterface tool(TOOL_GROUP);


    ros::Duration delay_seconds(4.5); 
    std::vector<double> standard_pos        = {0, 0, 0, 0, 0, 0};
    std::vector<double> target_position_1   = {90*M_PI/180, -54*M_PI/180, 0 ,0 ,-36*M_PI/180,-90*M_PI/180};
    std::vector<double> target_position_2   = {-90*M_PI/180, -54*M_PI/180, 0, 0, -36*M_PI/180, -90*M_PI/180};
    moveit::planning_interface::MoveGroupInterface::Plan master_plan;
    
    DTGripper simulation_gripper(nh, true);
    PTGripper physical_gripper;
    while (ros::ok())
    {
        simulation_gripper.open();
        physical_gripper.open();
        ROS_INFO("Opening gripper.");
        delay_seconds.sleep();

        if(!arm_group.setJointValueTarget(standard_pos)) {
            ROS_INFO("ERROR: Cannot set Joint Value Target");
            return;
        }    
        if (arm_group.plan(master_plan) != moveit::planning_interface::MoveItErrorCode::SUCCESS) {
            ROS_INFO("ERROR: Planning framework returned a non-zero code.");
            return;             
        }
        arm_group.execute(master_plan);

        ROS_INFO("Waiting %f sec...", delay_seconds.toSec());
        delay_seconds.sleep(); 

        ROS_INFO("Moving to target position 1.");
        if(!arm_group.setJointValueTarget(target_position_1)) {
            ROS_INFO("ERROR: Cannot set Joint Value Target");
            return;
        }    
        if (arm_group.plan(master_plan) != moveit::planning_interface::MoveItErrorCode::SUCCESS) {
            ROS_INFO("ERROR: Planning framework returned a non-zero code.");
            return;             
        }
        arm_group.execute(master_plan);
        delay_seconds.sleep();

        ROS_INFO("Closing gripper.");        
        simulation_gripper.close();
        physical_gripper.close();

        ROS_INFO("Waiting %f sec...", delay_seconds.toSec());
        delay_seconds.sleep();

        ROS_INFO("Moving to target initial pose.");
        if(!arm_group.setJointValueTarget(standard_pos)) {
            ROS_INFO("ERROR: Cannot set Joint Value Target");
            return;
        }    
        if (arm_group.plan(master_plan) != moveit::planning_interface::MoveItErrorCode::SUCCESS) {
            ROS_INFO("ERROR: Planning framework returned a non-zero code.");
            return;             
        }
        arm_group.execute(master_plan);

        ROS_INFO("Moving to target position 2 and opening gripper.");
        if(!arm_group.setJointValueTarget(target_position_2)) {
            ROS_INFO("ERROR: Cannot set Joint Value Target");
            return;
        }    
        if (arm_group.plan(master_plan) != moveit::planning_interface::MoveItErrorCode::SUCCESS) {
            ROS_INFO("ERROR: Planning framework returned a non-zero code.");
            return;             
        }
        arm_group.execute(master_plan);

        simulation_gripper.open();
        physical_gripper.open();
        ROS_INFO("Opening gripper.");

        ROS_INFO("Waiting %f sec...", delay_seconds.toSec());
        delay_seconds.sleep(); 

        ROS_INFO("Comming back to initial position.");
        if(!arm_group.setJointValueTarget(standard_pos)) {
            ROS_INFO("ERROR: Cannot set Joint Value Target");
            return;
        }    
        if (arm_group.plan(master_plan) != moveit::planning_interface::MoveItErrorCode::SUCCESS) {
            ROS_INFO("ERROR: Planning framework returned a non-zero code.");
            return;             
        }
        arm_group.execute(master_plan);
        delay_seconds.sleep();

        //simulation_gripper.open();
        //physical_gripper.open();
        //ROS_INFO("Opening gripper.");
        //ROS_INFO("End effector reference frame: %s", arm_group.getEndEffectorLink().c_str());

        ROS_INFO("Moving to target position 2");
        if(!arm_group.setJointValueTarget(target_position_2)) {
            ROS_INFO("ERROR: Cannot set Joint Value Target");
            return;
        }    
        if (arm_group.plan(master_plan) != moveit::planning_interface::MoveItErrorCode::SUCCESS) {
            ROS_INFO("ERROR: Planning framework returned a non-zero code.");
            return;             
        }
        arm_group.execute(master_plan);
        delay_seconds.sleep();
        
        ROS_INFO("Closing gripper.");
        //ROS_INFO("End effector reference frame: %s", arm_group.getEndEffectorLink().c_str());
        simulation_gripper.close();
        physical_gripper.close();

        ROS_INFO("Waiting %f sec...", delay_seconds.toSec());
        delay_seconds.sleep(); 

        ROS_INFO("Comming back to initial position.");
        if(!arm_group.setJointValueTarget(standard_pos)) {
            ROS_INFO("ERROR: Cannot set Joint Value Target");
            return;
        }    
        if (arm_group.plan(master_plan) != moveit::planning_interface::MoveItErrorCode::SUCCESS) {
            ROS_INFO("ERROR: Planning framework returned a non-zero code.");
            return;             
        }
        arm_group.execute(master_plan);

        ROS_INFO("Moving to target position 1.");
        if(!arm_group.setJointValueTarget(target_position_1)) {
            ROS_INFO("ERROR: Cannot set Joint Value Target");
            return;
        }    
        if (arm_group.plan(master_plan) != moveit::planning_interface::MoveItErrorCode::SUCCESS) {
            ROS_INFO("ERROR: Planning framework returned a non-zero code.");
            return;             
        }
        arm_group.execute(master_plan);

        delay_seconds.sleep();
        simulation_gripper.open();
        physical_gripper.open();
        ROS_INFO("Opening gripper");
        delay_seconds.sleep();
    }
    
    ros::shutdown();
}  
