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
#include "std_msgs/Float64MultiArray.h"
#include <stdexcept>
#include <mutex>
#include <vector>

namespace simulation {

/**
 * This type generates the topics used given a specific robot ID. 
 * It is required to do this if we need to use several robots within a single simulation. 
 * 
 * All these topics should be first generated by coppeliaSIM's ROS interface. 
*/
class Topic 
{
public:
    int robot_id = 0;     /* ID of the controlled robot. */
    std::string TOPIC_ROOT = "/coppeliaSIM/NiryoOne_" + std::to_string(robot_id);
    std::string TIME_PUB = TOPIC_ROOT + "/simulationTime";

    /* Gripper Control Topic Names */
    std::string GRIPPER_STATE_PUB = TOPIC_ROOT + "/isGripperOpenPub";
    std::string GRIPPER_STATE_SUB = TOPIC_ROOT + "/GripperCommandSub";

    /* Robot State Topic Names */
    std::string RUNNING_TIME_TOPIC = TOPIC_ROOT + "/simulationTime";
    std::string JOINT_VEL_PUB     = TOPIC_ROOT + "/targetJointAngularVelocityPub";
    std::string JOINT_VEL_SUB     = TOPIC_ROOT + "/targetJointAngularVelocitySub";
    std::string JOINT_POS_PUB     = TOPIC_ROOT + "/targetJointAngularPositionPub";
    std::string JOINT_POS_SUB     = TOPIC_ROOT + "/targetJointAngularPositionSub";
    std::string JOINT_FORCE_PUB   = TOPIC_ROOT + "/targetJointAngularForcePub";
    std::string JOINT_FORCE_SUB   = TOPIC_ROOT + "/targetJointAngularForceSub";

    Topic(int id) { robot_id = id; }

    Topic(const Topic& t) { robot_id = t.robot_id; }

    Topic(Topic& t) { robot_id = t.robot_id; }

    Topic& operator=(Topic& t) { this->robot_id = t.robot_id; return *this; }
};

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
    GripperRosControl(ros::NodeHandle& nh, Topic& t, bool isGripperClosed = false) 
    { 
        if (!nh.ok()) throw std::invalid_argument("NodeHandle not Ok");
        is_gripper_open     = isGripperClosed; 
        close_gripper.data  = CLOSE;
        open_gripper.data   = OPEN;
        
        mtx = new std::mutex();

        gripper_pub = nh.advertise<std_msgs::Bool>(t.GRIPPER_STATE_SUB, 10);
        gripper_sub = nh.subscribe(t.GRIPPER_STATE_PUB, 10 , &GripperRosControl::gripper_state_callback, this);
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
    std::mutex* mtx_tp;  
    std::mutex* mtx_tv;
    std::mutex* mtx_tf;
    std::mutex* mtx_cp;
    std::mutex* mtx_cv;
    std::mutex* mtx_cf;
    
    /* Robot state targets. */ 
    std_msgs::Float64MultiArray target_position;
    std_msgs::Float64MultiArray target_velocity;
    std_msgs::Float64MultiArray target_force;

    /* Robot state update by the callbacks. */
    std_msgs::Float64MultiArray current_position;
    std_msgs::Float64MultiArray current_velocity;
    std_msgs::Float64MultiArray current_force;

    ros::Publisher   target_position_pub;
    ros::Publisher   target_velocity_pub;
    ros::Publisher   target_force_pub;
    ros::Subscriber  current_position_sub;
    ros::Subscriber  current_velocity_sub;
    ros::Subscriber  current_force_sub;
    ros::NodeHandle* nh;
    GripperRosControl* gripper;

    /* Constants */
    const uint DEFAULT_QUEUE_SIZE = 10;

public:

    /**
     * Set of callbacks called by ROS subscribers to update the current
     * physical parameters of the Niryo One. 
    */
    void position_cb(const std_msgs::Float64MultiArray::ConstPtr& msg) 
    { 
        mtx_cp->lock();
        this->current_position.data = msg->data;
        mtx_cp->unlock();
    }

    void velocity_cb(const std_msgs::Float64MultiArray::ConstPtr& msg)
    {
        mtx_cv->lock();
        this->current_velocity.data = msg->data;
        mtx_cv->unlock();
    }

    void force_cb(const std_msgs::Float64MultiArray::ConstPtr& msg)
    {
        mtx_cf->lock();
        this->current_force.data = msg->data;
        mtx_cf->unlock();
    }

    NiryoOne(int id, bool has_gripper, bool is_griper_closed = false) 
    {
        this->id = id;
        this->has_gripper = has_gripper;
        target_position.data    = {0, 0, 0, 0, 0, 0}; 
        target_force.data       = {0, 0, 0, 0, 0, 0};
        target_velocity.data    = {0, 0, 0, 0, 0, 0}; 
        mtx_tp = new std::mutex();
        mtx_tv = new std::mutex();
        mtx_tf = new std::mutex();
        mtx_cp = new std::mutex();
        mtx_cv = new std::mutex();
        mtx_cf = new std::mutex();
        simulation::Topic t(id);
        nh = new ros::NodeHandle;

        if (has_gripper) {
            gripper = new GripperRosControl(*nh, t, is_griper_closed);
        }
        
        target_position_pub = 
            nh->advertise<std_msgs::Float64MultiArray>(t.JOINT_POS_SUB ,  DEFAULT_QUEUE_SIZE);
        target_force_pub = 
            nh->advertise<std_msgs::Float64MultiArray>(t.JOINT_FORCE_SUB, DEFAULT_QUEUE_SIZE);
        target_velocity_pub = 
            nh->advertise<std_msgs::Float64MultiArray>(t.JOINT_VEL_SUB,   DEFAULT_QUEUE_SIZE);

        nh->subscribe(t.JOINT_POS_PUB, DEFAULT_QUEUE_SIZE,   &NiryoOne::position_cb, this);
        nh->subscribe(t.JOINT_VEL_PUB, DEFAULT_QUEUE_SIZE,   &NiryoOne::velocity_cb, this);
        nh->subscribe(t.JOINT_FORCE_PUB, DEFAULT_QUEUE_SIZE, &NiryoOne::force_cb, this);
    }

    ~NiryoOne()
    {
        delete gripper;
        delete mtx_tp;
        delete mtx_tv;
        delete mtx_tf;
        delete mtx_cp;
        delete mtx_cv;
        delete mtx_cf;
        target_position_pub.shutdown();
        target_force_pub.shutdown();
        target_velocity_pub.shutdown();
        current_position_sub.shutdown();
        current_velocity_sub.shutdown();
        current_force_sub.shutdown();
        delete nh;
    }

    /**
     * Publishes the target position on the joints of the Niryo One 
     * running on coppeliaSim.
     * 
     * A vector of 6 degrees of freedom representing the positions in
     * radians is expected.
     * 
     * Will return -1 if the vector is larger or less than 6. 
     * Will return 0 otherwise.
    */
    int publishNewTargetPosition(std::vector<double>& positions) 
    {
        if (positions.size() != 6)
            return -1;
        
        mtx_tp->lock();
        for (int i=0; i<6; ++i)
            target_position.data[i] = positions[i];
        target_position_pub.publish(target_position);
        mtx_tp->unlock();

        return 0;
    }

     /**
     * Publishes the target velocity on the joints of the Niryo One 
     * running on coppeliaSim.
     * 
     * A vector of 6 degrees of freedom representing the angular velocity in
     * radians per second is expected.
     * 
     * Will return -1 if the vector is larger or less than 6. 
     * Will return 0 otherwise.
    */
    int publishNewTargetVelocity(std::vector<double>& velocities) 
    {
        if (velocities.size() != 6)
            return -1;
        
        mtx_tv->lock();
        for (int i=0; i<6; ++i)
            target_velocity.data[i] = velocities[i];
        target_velocity_pub.publish(target_position);
        mtx_tv->unlock();
        
        return 0;
    }

    /**
     * Publishes the target force on the joints of the Niryo One 
     * running on coppeliaSim.
     * 
     * A vector of 6 degrees of freedom representing the force for each 
     * joint is expected.
     * 
     * Will return -1 if the vector is larger or less than 6. 
     * Will return 0 otherwise.
    */
    int publishNewTargetForce(std::vector<double>& forces) 
    {
        if (forces.size() != 6)
            return -1;
        
        mtx_tf->lock();
        for (int i=0; i<6; ++i)
            target_force.data[i] = forces[i];
        target_force_pub.publish(target_force);
        mtx_tf->unlock();
        
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
            current_pos.push_back(current_position.data[i]);
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
        mtx_cv->lock();
        for (int i=0; i<6; ++i) 
            current_vel.push_back(current_velocity.data[i]);
        mtx_cv->unlock();
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
        mtx_cf->lock();
        for (int i=0; i<6; ++i) 
            current_f.push_back(current_force.data[i]);
        mtx_cf->unlock();
        return 0;
    }

    /**
     * Stores the lastly received target position of the joints of 
     * the Niryo One running on coppeliaSim on the vector given as
     * parameter.
     * 
     * If parameter vector is empty, then -1 will be return.
     * Otherwise, 0 will be return. 
    */
    int getTargetPosition(std::vector<double>& target_pos)
    {
        if (target_pos.size() != 0) 
            return -1;
        mtx_tp->lock();
        for (int i=0; i<6; ++i) 
            target_pos.push_back(target_position.data[i]);
        mtx_tp->unlock();
        return 0;
    }
    
    /**
     * Stores the lastly received target velocity of the joints of
     * the Niryo One running on coppeliaSim on the vector given as ç
     * parameter.
     * 
     * If parameter vector is empty, then -1 will be return.
     * Otherwise, 0 will be return. 
    */
    int getTargetVelocity(std::vector<double>& target_vel)
    {
        if (target_vel.size() != 0) 
            return -1;
        mtx_tv->lock();
        for (int i=0; i<6; ++i) 
            target_vel.push_back(target_velocity.data[i]);
        mtx_tv->unlock();
        return 0;
    }
    
    /**
     * Stores the lastly received target force of the joints of 
     * the Niryo One running on coppeliaSim on the vector given 
     * as parameter.
     * 
     * If parameter vector is empty, then -1 will be return.
     * Otherwise, 0 will be return. 
    */
    int getTargetForce(std::vector<double>& target_f)
    {
        if (target_f.size() != 0) 
            return -1;
        mtx_tf->lock();
        for (int i=0; i<6; ++i) 
            target_f.push_back(target_force.data[i]);
        mtx_tf->unlock();
        return 0;
    }


    /**
     * Opens the gripper of the Niryo One has been initialized
     * with one.
     * 
     * It will return -1 if there is no gripper and 0 otherwise.
     */
    int openGripper()
    {
        if(has_gripper) {
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