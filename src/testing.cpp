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

/** 
 * Verifies that a running instance of the coppeliaSIM model is able to satisfy requirements 
 * R-01 and R-02: 
 * - R-01: Joint state of the six axis robot can be monitored: position, velocity and effort.
 *      R-01_A: Position can be fetched. 
 *      R-01_B: Velocity can be fetched. 
 *      R-01_C: Effort can be fetched. 
 *   
 * - R-02: The gripper tool state can be monitored and modified.
 *      This requirement will be satisfied if the state of the gripper can be fetched and 
 *      if it can be changed remotely.
 *      
*/
bool digital_twin_integration_tests(int argc, char** argv)
{
    ROS_INFO("##################################################");
    ROS_INFO("###       Digital Twin Integration Tests       ###");
    ROS_INFO("##################################################");

    ros::init(argc, argv, "digital_twin_integration_testster");
    ros::NodeHandle nh;

    simulation::NiryoOneDT sim(nh);
    ros::AsyncSpinner spinner(2); 
    spinner.start();
    double WAITING_TIME = 4.5;
    ros::Duration delay_seconds(WAITING_TIME); 
    delay_seconds.sleep();
    /* Test T-01: Test connectivity to the simulation node. */
    bool connected = sim.connected();
    if (!connected) {
        ROS_INFO("Waiting %f", WAITING_TIME);
        connected = sim.connected();
        if (!connected) {
            ROS_INFO("Timeout. Check network connection.");
            ros::shutdown();
            return false;
        }
    }

    ROS_INFO("T-01: Passed!");

    /* Test T-02: Get and check the values for the joint state. 
        - Current position should be the initial position. 
        - Velocity and effort should be zero. 
    */
    std::vector<double> EXPECTED_POS = {0.0, 0.640187, -1.397485, 0.0, 0.0, 0.0};
    std::vector<double> EXPECTED_VEL = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    std::vector<double> EXPECTED_EFF = {0.003, -0.05, -1.72, 0.16, -0.32, 0.008};

    std::vector<double> actual_pos;
    std::vector<double> actual_vel;
    std::vector<double> actual_eff;

    connected = sim.getCurrentPosition  (actual_pos); 
    connected = sim.getCurrentVelocity  (actual_vel); 
    connected = sim.getCurrentEffort    (actual_eff); 

    if (!connected) {
        ROS_INFO("Disconnected from the simulation node.");
        ros::shutdown();
        return false;
    }

    /**
        Check that the current values are within range. 
    */
    for (uint i=0; i<6; ++i) {
        if (EXPECTED_POS[i] > actual_pos[i]+5 || EXPECTED_POS[i] < actual_pos[i]-5) {
            ROS_INFO("Mismatch between expected position and actual position.");
            ros::shutdown();
            return false;
        }
        if (EXPECTED_VEL[i] != actual_vel[i]) {
            ROS_INFO("Mismatch between expected velocity and actual velocity.");
            ros::shutdown();
            return false;
        }
        if (EXPECTED_EFF[i] > actual_eff[i]+5 || EXPECTED_EFF[i] < actual_eff[i]-5) {
            ROS_INFO("Mismatch between expected effort and actual effort.");
            ros::shutdown();
            return false;
        }
    }

    ROS_INFO("T-02: Passed!");

    /* Test T-03: Get value for the state of the gripper and verify we can change it. */
    bool is_gripper_open = sim.isGripperOpen();
    double WAITING_TIME_2 = 1;
    ros::Duration wait_prop(WAITING_TIME_2);

    if (is_gripper_open) { 
        ROS_INFO("Gripper reported by simulation as OPEN. Attempting to close.");
        connected = sim.closeGripper();
        if (!connected) {
            ROS_INFO("Disconnected from the simulation node.");
            ros::shutdown();
            return false;
        }
        ROS_INFO("Waiting %f second for publisher...", WAITING_TIME_2);
        wait_prop.sleep();
        is_gripper_open = sim.isGripperOpen();

        if (is_gripper_open && sim.connected()) {
            ROS_INFO("Could not close the gripper.");
            ros::shutdown();
            return false;
        } 
    }
    else {
        ROS_INFO("Gripper reported by simulation as CLOSED. Attempting to open.");
        connected = sim.openGripper();
        if (!connected) {
            ROS_INFO("Disconnected from the simulation node.");
            ros::shutdown();
            return false;
        }
        ROS_INFO("Waiting %f second for publisher...", WAITING_TIME_2);
        wait_prop.sleep();
        is_gripper_open = sim.isGripperOpen();

        if (!is_gripper_open && sim.connected()) {
            ROS_INFO("Could not open the gripper.");
            ros::shutdown();
            return false;
        } 
    }
    
    ROS_INFO("T-03: Passed!");
        
    ros::shutdown();
    return true;
}


/**
 * Physical twin control from the controller node. 
 * Clocks must be synchronized using ptpd if running on different machines.
 *
 * Verifies requirement R-05 and R-03: 
 * - R-03: The digital twin shall update the state of the model as soon as the new state of the 
 *         physical twin is available. 
 * - R-05: The control node on the digital twin shall be able to control the physical twin remotely.
 * 
 * This test passes if: 
 * - Robot follows the choreography of moving one box from a side of the table to the other. 
 * - The state of the robot is propagated in real time to the simulation model in coppeliaSIM. 
*/
void test_physical_twin_control(int argc, char** argv)
{
    ros::init(argc, argv, "physical_twin_controller");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(2); 
    spinner.start();

    /* Assemble arm group. */
    static const std::string ARM_GROUP = "arm";
    moveit::planning_interface::MoveGroupInterface arm_group(ARM_GROUP);
    const robot_state::JointModelGroup* joint_model_group = 
        arm_group.getCurrentState()->getJointModelGroup(ARM_GROUP);

    /** 
        The choreography will alternate between three positions and wait a time for the gripper 
        to have enought time to correctly open or close on target positions. 

        Ideally, a 2cm cube would be at the target positions for demonstration purposes. 
    */
    ros::Duration delay_seconds(4.5); 
    std::vector<double> standard_pos        = {0, 0, 0, 0, 0, 0};
    std::vector<double> target_position_1   = { 90*M_PI/180, -54*M_PI/180, 0 ,0 ,-36*M_PI/180,-90*M_PI/180};
    std::vector<double> target_position_2   = {-90*M_PI/180, -54*M_PI/180, 0, 0, -36*M_PI/180, -90*M_PI/180};
    moveit::planning_interface::MoveGroupInterface::Plan master_plan;
    
    simulation::Gripper simulation_gripper(nh);
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
