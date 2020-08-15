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
#ifndef SIMULATION_HPP
#define SIMULATION_HPP
#include <string>
#include "std_msgs/Bool.h"
#include "ros/ros.h"
#include "std_msgs/Int64.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float64MultiArray.h"
#include "sensor_msgs/JointState.h"
#include <stdexcept>
#include <mutex>
#include <vector>

namespace simulation 
{

class Gripper
{
private:
    std::mutex* mtx; 
    bool is_gripper_open; 
    std_msgs::Bool close_gripper;
    std_msgs::Bool open_gripper;
    ros::Publisher gripper_pub;
    ros::Subscriber gripper_sub;
    const bool OPEN = true; 
    const bool CLOSE = false;
    const char* GRIPPER_STATE_PUB = "/coppeliaSIM/NiryoOne/is_gripper_open";
    const char* GRIPPER_STATE_SUB = "/coppeliaSIM/NiryoOne/gripper_command";

    void gripper_state_callback(const std_msgs::Bool::ConstPtr& msg); 
public:
    Gripper(ros::NodeHandle& nh);
    void open();
    void close();
    bool isOpen();
    ~Gripper();
};




class NiryoOneDT
{
private:

    /* Subscribers */
    ros::Subscriber joint_sub;
    ros::Subscriber sim_time_sub;

    /* Gripper Interface to the simulation. */
    simulation::Gripper* gripper;

    /* Joint and simulation time callbacks. */
    void joint_states_cb(const sensor_msgs::JointState::ConstPtr& msg);
    void sim_time_cb(const std_msgs::Float32::ConstPtr& msg);
    
    /* Mutable Values */
    sensor_msgs::JointState joints;
    float simulation_time;

    /* Constants */
    const uint DEFAULT_QUEUE_SIZE = 1;
    const char* JOINT_STATES    = "/coppeliaSIM/NiryoOne/joint_states";
    const char* JOINT_POS_ORDER = "/coppeliaSIM/NiryoOne/joint_states_order";
    const char* SIM_TIME        = "/coppeliaSIM/NiryoOne/simulation_time";

    /* Mutexes */
    std::mutex* joint_mtx;
    std::mutex* sim_time_mtx;
public:

    NiryoOneDT(ros::NodeHandle& nh);

    ~NiryoOneDT();

    /**
     * Returns the simulation time in seconds. 
    */
    float getSimulationTime();

    /**
     * Stores the lastly received position of the joints of the Niryo One 
     * running on coppeliaSim on the vector given as parameter.
     * 
     * If disconnected from simulation, will return false.
     * Otherwise, true will be return. 
    */
    bool getCurrentPosition(std::vector<double>& current_pos);
    
    /**
     * Stores the lastly received velocity of the joints of the Niryo One 
     * running on coppeliaSim on the vector given as parameter.
     * 
     * If disconnected from simulation, will return false.
     * Otherwise, true will be return. 
    */
    bool getCurrentVelocity(std::vector<double>& current_vel);
    
    /**
     * Stores the lastly received force of the joints of the Niryo One 
     * running on coppeliaSim on the vector given as parameter.
     * 
     * If disconnected from simulation, will return false.
     * Otherwise, true will be return. 
    */
    bool getCurrentEffort(std::vector<double>& current_f);

    /**
     * Opens the gripper of the Niryo One inside the simulation. 
     * 
     * It will return false if no connection and true otherwise.
     */
    bool openGripper();

    /**
     * Closes the gripper of the Niryo One has been initialized
     * with one.
     * 
     * It will return false if no connection and true otherwise.
     */
    bool closeGripper();

    /**
     * Will return true if the gripper is open or false if it is closed. 
    */
    bool isGripperOpen();

    /**
     * Verifies that the current node is connected to a ros master with 
     * an instance of the the simulation node.
    */
    bool connected();
};

} // !simulation 
#endif