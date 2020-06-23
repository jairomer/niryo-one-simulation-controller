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

namespace digital_twin {

class GripperRosControl
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
    const char* GRIPPER_STATE_PUB = "/coppeliaSIM/NiryoOne/isGripperOpenPub";
    const char* GRIPPER_STATE_SUB = "/coppeliaSIM/NiryoOne/GripperCommandSub";
public:

    void gripper_state_callback(const std_msgs::Bool::ConstPtr& msg) 
    {
        mtx->lock();
        is_gripper_open = msg->data;
        mtx->unlock();
    }

    /**
     * The NodeHandle has to be initialized. 
    */
    GripperRosControl(ros::NodeHandle& nh, bool isGripperClosed = false) 
    { 
        if (!nh.ok()) throw std::invalid_argument("NodeHandle not Ok");
        is_gripper_open     = isGripperClosed; 
        close_gripper.data  = CLOSE;
        open_gripper.data   = OPEN;
        
        mtx = new std::mutex();

        gripper_pub = nh.advertise<std_msgs::Bool>(GRIPPER_STATE_SUB, 10);
        gripper_sub = nh.subscribe(GRIPPER_STATE_PUB, 10 , &GripperRosControl::gripper_state_callback, this);
    }

    void open()
    {
        mtx->lock(); 
        gripper_pub.publish(open_gripper);
        is_gripper_open = true;
        mtx->unlock(); 
    }

    void close() 
    { 
        mtx->lock();    
        gripper_pub.publish(close_gripper);
        is_gripper_open = false;
        mtx->unlock();    
    }

    bool isOpen() { return is_gripper_open; }

    ~GripperRosControl() 
    {
        gripper_sub.shutdown();
        gripper_pub.shutdown();
        delete mtx;
    }
};

class NiryoOne 
{
private:
    
    int id;
    bool has_gripper;
    bool subscriber_only; 
    std::mutex* mtx_tp;  
    std::mutex* mtx_cp;
    std::mutex* mtx_sim;
    
    sensor_msgs::JointState joint_status;
    ros::Publisher   target_position_publisher;
    ros::Subscriber  joint_state_sub;
    ros::Subscriber  simulation_time_sub;
    ros::NodeHandle* nh;
    GripperRosControl* gripper;
    double simulation_time;

    /* Constants */
    const uint DEFAULT_QUEUE_SIZE = 10;
    const char* JOINT_STATES    = "/coppeliaSIM/NiryoOne/joint_states";
    const char* JOINT_POS_ORDER = "/coppeliaSIM/NiryoOne/joint_states_order";
    const char* SIM_TIME        = "/coppeliaSIM/NiryoOne/simulationTime";

public:

    /**
     * Set of callbacks called by ROS subscribers to update the current
     * physical parameters of the Niryo One. 
    */
    void joint_states_cb(const sensor_msgs::JointState::ConstPtr& msg) 
    { 
        mtx_cp->lock();
        this->joint_status.position = msg->position;
        this->joint_status.velocity = msg->velocity;
        this->joint_status.effort   = msg->effort;
        mtx_cp->unlock();
    }

    void sim_time_cb(const std_msgs::Float32::ConstPtr& msg) 
    {
        mtx_sim->lock();
        this->simulation_time = msg->data;
        mtx_sim->unlock();
    }

    NiryoOne(bool has_gripper, bool is_griper_closed = false, bool subscriber_only = false) 
    {
        this->subscriber_only = subscriber_only;
        this->has_gripper = has_gripper;
        mtx_tp = new std::mutex();
        mtx_cp = new std::mutex();
        mtx_sim = new std::mutex();
        nh = new ros::NodeHandle;

        if (has_gripper) {
            gripper = new GripperRosControl(*nh, is_griper_closed);
        }
        
        target_position_publisher = 
            nh->advertise<std_msgs::Float64MultiArray>(JOINT_POS_ORDER ,  DEFAULT_QUEUE_SIZE);

        joint_state_sub = 
            nh->subscribe(JOINT_STATES, DEFAULT_QUEUE_SIZE, &NiryoOne::joint_states_cb, this);

        simulation_time_sub = 
            nh->subscribe(SIM_TIME, DEFAULT_QUEUE_SIZE, &NiryoOne::sim_time_cb, this);

    }

    ~NiryoOne()
    {
        delete gripper;
        delete mtx_tp;
        delete mtx_cp;
        delete mtx_sim;
        target_position_publisher.shutdown();
        joint_state_sub.shutdown();
        delete nh;
    }

    int set_subscriber_mode(bool mode) 
    {
        subscriber_only = mode;
    }

    float getSimulationTime() 
    { 
        float ret; 
        mtx_sim->lock();
        ret = simulation_time;
        mtx_sim->unlock();
        return ret; 
    }

    /**
     * Publishes the target position on the joints of the Niryo One 
     * running on coppeliaSim.
     * 
     * A vector of 6 degrees of freedom representing the positions in
     * radians is expected.
     * 
     * Will return -1 if the vector is larger or less than 6. 
     * Will return -2 if the current instance is in subscriber_only mode.
     * Will return 0 otherwise.
    */
    int publishNewTargetPosition(std::vector<double>& positions) 
    {
        if (subscriber_only)
            return -2;
        if (positions.size() != 6)
            return -1;
        
        mtx_tp->lock();
        std_msgs::Float64MultiArray target_position;
        target_position.data = positions;
        target_position_publisher.publish(target_position);
        mtx_tp->unlock();

        return 0;
    }

    /**
     * Stores the lastly received position of the joints of the Niryo One 
     * running on coppeliaSim on the vector given as parameter.
     * 
     * If parameter vector is empty, then -1 will be return.
     * Otherwise, 0 will be return. 
    */
    int getCurrentPosition(std::vector<double>& current_pos)
    {
        if (current_pos.size() != 0) 
            return -1;
        mtx_cp->lock();
        for (int i=0; i<6; ++i) 
            current_pos.push_back(joint_status.position[i]);
        mtx_cp->unlock();
        return 0;
    }
    
    /**
     * Stores the lastly received velocity of the joints of the Niryo One 
     * running on coppeliaSim on the vector given as parameter.
     * 
     * If parameter vector is empty, then -1 will be return.
     * Otherwise, 0 will be return. 
    */
    int getCurrentVelocity(std::vector<double>& current_vel)
    {
        if (current_vel.size() != 0) 
            return -1;
        mtx_cp->lock();
        for (int i=0; i<6; ++i) 
            current_vel.push_back(joint_status.velocity[i]);
        mtx_cp->unlock();
        return 0;
    }
    
    /**
     * Stores the lastly received force of the joints of the Niryo One 
     * running on coppeliaSim on the vector given as parameter.
     * 
     * If parameter vector is empty, then -1 will be return.
     * Otherwise, 0 will be return. 
    */
    int getCurrentForce(std::vector<double>& current_f)
    {
        if (current_f.size() != 0) 
            return -1;
        mtx_cp->lock();
        for (int i=0; i<6; ++i) 
            current_f.push_back(joint_status.effort[i]);
        mtx_cp->unlock();
        return 0;
    }

    /**
     * Opens the gripper of the Niryo One has been initialized
     * with one.
     * 
     * It will return -1 if there is no gripper, -2 if in subscriber only mode 
     * and 0 otherwise.
     */
    int openGripper()
    {
        if(has_gripper && !subscriber_only) {
            gripper->open();
            return 0;
        }
        return -1;
    }

    /**
     * Closes the gripper of the Niryo One has been initialized
     * with one.
     * 
     * It will return -1 if there is no gripper and 0 otherwise.
     */
    int closeGripper()
    {
        if(has_gripper) {
            gripper->close();
            return 0;
        }
        return -1;
    }
};

}; // !simulation

#endif