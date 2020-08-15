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
#include "simulation.hpp"

namespace simulation 
{

void NiryoOneDT::joint_states_cb(const sensor_msgs::JointState::ConstPtr& msg)
{
    joint_mtx->lock();
    
    joints.position = msg->position;
    joints.velocity = msg->velocity;
    joints.effort   = msg->effort;
    
    joint_mtx->unlock();
}

void NiryoOneDT::sim_time_cb(const std_msgs::Float32::ConstPtr& msg) 
{
    sim_time_mtx->lock();
    simulation_time = msg->data;
    sim_time_mtx->unlock();
}
    

NiryoOneDT::NiryoOneDT(ros::NodeHandle& nh)
{
    if (!nh.ok()) throw std::invalid_argument("NodeHandle not Ok");

    this->gripper       = new simulation::Gripper(nh);
    this->joint_mtx     = new std::mutex();
    this->sim_time_mtx  = new std::mutex();

    this->joint_sub     = nh.subscribe(JOINT_STATES, 1, &simulation::NiryoOneDT::joint_states_cb, this);
    this->sim_time_sub  = nh.subscribe(SIM_TIME, 1, &simulation::NiryoOneDT::sim_time_cb, this);
}

NiryoOneDT::~NiryoOneDT() 
{
    delete gripper;
    delete joint_mtx;
    delete sim_time_mtx;
    this->joint_sub.shutdown();
    this->sim_time_sub.shutdown();
}

float NiryoOneDT::getSimulationTime() 
{
    bool connected = this->connected();
    if (connected) {
        sim_time_mtx->lock();
        float sim_time = simulation_time;
        sim_time_mtx->unlock();
        return sim_time;
    }
    return (float) -1;
}

bool NiryoOneDT::getCurrentPosition(std::vector<double>& current_pos) 
{
    bool connected = this->connected();
    if (connected) {
        joint_mtx->lock();
        for (auto it=joints.position.begin(); it != joints.position.end(); ++it)
        {
            current_pos.push_back(*it);
        }
        joint_mtx->unlock();
    }
    return connected;
}

bool NiryoOneDT::getCurrentVelocity(std::vector<double>& current_vel)
{
    bool connected = this->connected();
    if (connected) {
        joint_mtx->lock();
        for (auto it=joints.velocity.begin(); it != joints.velocity.end(); ++it)
        {
            current_vel.push_back(*it);
        }
        joint_mtx->unlock();
    }
    return connected;
}

bool NiryoOneDT::getCurrentEffort(std::vector<double>& current_eff)
{
    bool connected = this->connected();
    if (connected) {
        joint_mtx->lock();
        for (auto it=joints.effort.begin(); it != joints.effort.end(); ++it)
        {
            current_eff.push_back(*it);
        }
        joint_mtx->unlock();
    }
    return connected;
}

bool NiryoOneDT::openGripper()
{
    bool connected = this->connected();
    if (connected) {
        this->gripper->open();
    }
    return connected;
}

bool NiryoOneDT::closeGripper() 
{
    bool connected = this->connected();
    if (connected) {
        this->gripper->close();
    }
    return connected;
}

bool NiryoOneDT::isGripperOpen() 
{
    return this->gripper->isOpen();
}

bool NiryoOneDT::connected()
{   
    std::string lookup = "/sim_ros_interface";
    std::vector<std::string> node_names;
    ros::master::getNodes(node_names);
    for (auto it=node_names.begin(); it!=node_names.end(); ++it)
    {
        if (lookup.compare(*it) == 0) 
            return true;
    }
    return false;
}

}; // !simulation 
