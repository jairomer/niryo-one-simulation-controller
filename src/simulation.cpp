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

void DTGripper::gripper_state_callback(const std_msgs::Bool::ConstPtr& msg)
{
    this->mtx->lock();
    this->is_gripper_open = msg->data;
    this->mtx->unlock();
}


void DTGripper::open()
{
    this->mtx->lock(); 
    this->gripper_pub.publish(open_gripper);
    this->is_gripper_open = true;
    this->mtx->unlock(); 
}


void DTGripper::close() 
{ 
    this->mtx->lock();    
    this->gripper_pub.publish(close_gripper);
    this->is_gripper_open = false;
    this->mtx->unlock();    
}

DTGripper::~DTGripper() 
{
    this->gripper_sub.shutdown();
    this->gripper_pub.shutdown();
    delete mtx;
}

/**
* Set of callbacks called by ROS subscribers to update the current
* physical parameters of the Niryo One. 
*/
void DigitalTwin::joint_states_cb(const sensor_msgs::JointState::ConstPtr& msg) 
{ 
    this->mtx_cp->lock();
    this->joint_status.position = msg->position;
    this->joint_status.velocity = msg->velocity;
    this->joint_status.effort   = msg->effort;
    this->mtx_cp->unlock();
}

void DigitalTwin::sim_time_cb(const std_msgs::Float32::ConstPtr& msg) 
{
    this->mtx_sim->lock();
    this->simulation_time = msg->data;
    this->mtx_sim->unlock();
}

DigitalTwin::~DigitalTwin()
{
    delete this->gripper;
    delete this->mtx_tp;
    delete this->mtx_cp;
    delete this->mtx_sim;
    this->target_position_publisher.shutdown();
    this->joint_state_sub.shutdown();
    delete this->nh;
}

int DigitalTwin::set_subscriber_mode(bool mode) 
{
    this->subscriber_only = mode;
}

float DigitalTwin::getSimulationTime() 
{ 
    float ret; 
    this->mtx_sim->lock();
    ret = simulation_time;
    this->mtx_sim->unlock();
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
int DigitalTwin::publishNewTargetPosition(std::vector<double>& positions) 
{
    if (this->subscriber_only)
        return -2;
    if (positions.size() != 6)
        return -1;
    
    this->mtx_tp->lock();
    std_msgs::Float64MultiArray target_position;
    target_position.data = positions;
    this->target_position_publisher.publish(target_position);
    this->mtx_tp->unlock();

    return 0;
}

/**
* Stores the lastly received position of the joints of the Niryo One 
* running on coppeliaSim on the vector given as parameter.
* 
* If parameter vector is empty, then -1 will be return.
* Otherwise, 0 will be return. 
*/
int DigitalTwin::getCurrentPosition(std::vector<double>& current_pos)
{
    if (current_pos.size() != 0) 
        return -1;
    this->mtx_cp->lock();
    for (int i=0; i<6; ++i) 
        current_pos.push_back(joint_status.position[i]);
    this->mtx_cp->unlock();
    return 0;
}
    
/**
* Stores the lastly received velocity of the joints of the Niryo One 
* running on coppeliaSim on the vector given as parameter.
* 
* If parameter vector is empty, then -1 will be return.
* Otherwise, 0 will be return. 
*/
int DigitalTwin::getCurrentVelocity(std::vector<double>& current_vel)
{
    if (current_vel.size() != 0) 
        return -1;
    this->mtx_cp->lock();
    for (int i=0; i<6; ++i) 
        current_vel.push_back(joint_status.velocity[i]);
    this->mtx_cp->unlock();
    return 0;
}
    
/**
* Stores the lastly received force of the joints of the Niryo One 
* running on coppeliaSim on the vector given as parameter.
* 
* If parameter vector is empty, then -1 will be return.
* Otherwise, 0 will be return. 
*/
int DigitalTwin::getCurrentForce(std::vector<double>& current_f)
{
    if (current_f.size() != 0) 
        return -1;
    this->mtx_cp->lock();
    for (int i=0; i<6; ++i) 
        current_f.push_back(joint_status.effort[i]);
    this->mtx_cp->unlock();
    return 0;
}

/**
* Opens the gripper of the Niryo One has been initialized
* with one.
* 
* It will return -1 if there is no gripper, -2 if in subscriber only mode 
* and 0 otherwise.
*/
int DigitalTwin::openGripper()
{
    if(this->has_gripper && !this->subscriber_only) {
        this->gripper->open();
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
int DigitalTwin::closeGripper()
{
    if(this->has_gripper) {
        this->gripper->close();
        return 0;
    }
    return -1;
}