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

/**
 *  
 * Joint State Information: The move_group monitors the /joint_states topic for determining 
 * where each joint of the robot is. 
 *  -> This program has to implement a joint State publisher. 
 *  -> There is a ROS packet that might help:  http://wiki.ros.org/joint_state_publisher
 * 
 * Transform Information: The move_group monitors transform information using the ROS library. 
 * this allows the node to get global information about the robot's pose among other things. 
 * The move_group will listen to TF. To publish TF information, we need to have a 
 * robot_state_publisher node running on the simulation script, or at a higher level. 
 *   -> There is a ROS packet that might help: http://wiki.ros.org/robot_state_publisher
 * 
 * Controller Interface: The move_group talks to the controllers on the robot using the 
 * FollowJointTrajectoryAction interface. This is a ROS action interface. A server on the 
 * robot needs to service this action and move_it will only instantiate a client to talk to 
 * this controller action server on your robot. 
*/ 
#define _USE_MATH_DEFINES // We want to use the M_PI constant.

#include <iostream>
#include <string>
#include <math.h>
#include "simulation.hpp"
#include "ros/ros.h"


/**
 * Simulation control from the controller node. Used for demonstration pur
*/
void demo(int argc, char** argv) 
{
      /* Initialize ROS. */
    int robot_id = 0;
    ros::init(argc, argv, "sim_controller_" + std::to_string(robot_id) );
    simulation::Topic t(robot_id);
    simulation::NiryoOne robot(0, true);

    /* Target positions. */
    std::vector<double> stadard_pos = {0, 0, 0, 0, 0, 0};
    std::vector<double> target_position_1 = {90*M_PI/180, -54*M_PI/180, 0 ,0 ,-36*M_PI/180,-90*M_PI/180};
    std::vector<double> target_position_2 = {-90*M_PI/180, -54*M_PI/180, 0, 0, -36*M_PI/180, -90*M_PI/180};

    std::cout << "Publishing to: " << t.JOINT_POS_SUB << std::endl;
    std::cout << "Publishing to: " << t.GRIPPER_STATE_SUB << std::endl;
    ros::Duration delay_seconds(5); // Time for the gripper to close/open.

    /* Start dancing. */
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
}

int main(int argc, char** argv) 
{ 
    demo(argc, argv);
    
    return 0;
}