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
#define _USE_MATH_DEFINES // We want to use the M_PI constant.

#include <iostream>
#include <signal.h>
#include <string>
#include <math.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <std_msgs/Bool.h>


#include "simulation.hpp"
#include "physical.hpp"
#include "ros/ros.h"

void exit_handler(int s)
{
    printf("Exiting");
    ros::shutdown();
    exit(0);
}

/**
 * Sets a linear magnitude to angular. 
 * 
 * m     ->  radians
 * m/s   ->  radians/s
 * m/s^2 ->  radians/s^2
 * 
*/
float ang(float linear) { return linear*M_PI/180; }

/**
 * Print vector to stdout.
*/
template<typename T>
void print_vector(std::vector<T>& v)
{
    for (auto it = v.begin(); it != v.end(); it++)
    {
        std::cout << *it << " ";
    }
    std::cout << std::endl;
}

/**
 * Digital twin control from the controller node.
*/
void test_digital_twin_control_joint(int argc, char** argv) 
{
      /* Initialize ROS. */
    int robot_id = 0;
    ros::init(argc, argv, "sim_controller_" + std::to_string(robot_id) );
    DigitalTwin robot(true, false, false);

    /* Target positions. */
    std::vector<double> stadard_pos = {0, 0, 0, 0, 0, 0};
    std::vector<double> target_position_1 = {90*M_PI/180, -54*M_PI/180, 0 ,0 ,-36*M_PI/180,-90*M_PI/180};
    std::vector<double> target_position_2 = {-90*M_PI/180, -54*M_PI/180, 0, 0, -36*M_PI/180, -90*M_PI/180};

    //std::cout << "Publishing to: " << t.JOINT_POS_SUB << std::endl;
    //std::cout << "Publishing to: " << t.GRIPPER_STATE_SUB << std::endl;
    ros::Duration delay_seconds(5); // Time for the gripper to close/open.

    /* Start dancing. */
    if(robot.publishNewTargetPosition(stadard_pos)) {
        std::cout << "Wrong number of degrees of freedom" << std::endl;
        return;
    }
    delay_seconds.sleep(); 

    std::cout << "Moving to target position 1 and closing gripper." << std::endl;
    if(robot.publishNewTargetPosition(stadard_pos) || robot.openGripper()) {
        std::cout << "Wrong number of degrees of freedom" << std::endl;
        return;
    }
    
    delay_seconds.sleep(); 
    if(robot.publishNewTargetPosition(target_position_1)) {
        std::cout << "Wrong number of degrees of freedom" << std::endl;
        return;
    }
    delay_seconds.sleep(); 
    if(robot.closeGripper()) {
        std::cout << "Gripper not defined." << std::endl;
        return;
    }
    delay_seconds.sleep(); 

    std::cout << "Moving to target initial pose." << std::endl;
    if(robot.publishNewTargetPosition(stadard_pos)) {
        std::cout << "Wrong number of degrees of freedom" << std::endl;
        return;
    }
    delay_seconds.sleep(); 

    std::cout << "Moving to target position 2 and opening gripper." << std::endl;
    if(robot.publishNewTargetPosition(target_position_2)) {
        std::cout << "Wrong number of degrees of freedom" << std::endl;
        return;
    }
    delay_seconds.sleep(); 

    if(robot.openGripper()) {
        std::cout << "Gripper not defined." << std::endl;
        return;
    }
    delay_seconds.sleep();

    std::cout << "Comming back to initial position." << std::endl;
    if(robot.publishNewTargetPosition(stadard_pos)) {
        std::cout << "Wrong number of degrees of freedom" << std::endl;
        return;
    }
    std::cout << "done." << std::endl;

    ros::shutdown(); // Cleanup
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
    
    DTGripper simulation_gripper(nh);
    PTGripper physical_gripper();
    while (ros::ok())
    {
        simulation_gripper.open();
        physical_gripper.open();
        ROS_INFO("Opening gripper.");
        //ROS_INFO("End effector reference frame: %s", arm_group.getEndEffectorLink().c_str());

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

        ROS_INFO("Closing gripper.");        
        simulation_gripper.close();
        physical_gripper.close();
        //ROS_INFO("End effector reference frame: %s", arm_group.getEndEffectorLink().c_str());

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

        ROS_INFO("Moving to target position 2 and opening gripper.");
        if(!arm_group.setJointValueTarget(target_position_2)) {
            ROS_INFO("ERROR: Cannot set Joint Value Target");
            return;
        }    
        if (arm_group.plan(master_plan) != moveit::planning_interface::MoveItErrorCode::SUCCESS) {
            ROS_INFO("ERROR: Planning framework returned a non-zero code.");
            return;             
        }
        
        simulation_gripper.open();
        physical_gripper.close();
        ROS_INFO("Opening gripper.");
        //ROS_INFO("End effector reference frame: %s", arm_group.getEndEffectorLink().c_str());

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

        simulation_gripper.open();
        physical_gripper.close();
        ROS_INFO("Opening gripper.");
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
    }
    
    ros::shutdown();
}  

int main(int argc, char** argv) 
{ 
    struct sigaction sigIntHandler;
    sigIntHandler.sa_handler = exit_handler;
    sigemptyset(&sigIntHandler.sa_mask);
    sigIntHandler.sa_flags = 0;

    sigaction(SIGINT, &sigIntHandler, NULL);
    
    //test_digital_twin_control(argc, argv);
    test_physical_twin_control(argc, argv);
    return 0;
}