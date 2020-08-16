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

Gripper::Gripper(ros::NodeHandle& nh) 
{
    if (!nh.ok()) throw std::invalid_argument("NodeHandle not Ok");
    this->is_gripper_open     = false; 
    this->close_gripper.data  = CLOSE;
    this->open_gripper.data   = OPEN;
    this->mtx = new std::mutex();
    this->gripper_pub = nh.advertise<std_msgs::Bool>(GRIPPER_STATE_SUB, 1);
    this->gripper_sub = nh.subscribe(GRIPPER_STATE_PUB, 1 , &simulation::Gripper::gripper_state_callback, this);
}

void Gripper::gripper_state_callback(const std_msgs::Bool::ConstPtr& msg)
{
    this->mtx->lock();
    this->is_gripper_open = msg->data;
    this->mtx->unlock();
}

bool Gripper::isOpen() 
{
    this->mtx->lock();
    bool open = is_gripper_open; 
    this->mtx->unlock();
    return open;
}


void Gripper::open()
{
    this->mtx->lock(); 
    this->gripper_pub.publish(open_gripper);
    this->is_gripper_open = true;
    this->mtx->unlock(); 
}


void Gripper::close() 
{ 
    this->mtx->lock();    
    this->gripper_pub.publish(close_gripper);
    this->is_gripper_open = false;
    this->mtx->unlock();    
}

Gripper::~Gripper() 
{
    this->gripper_sub.shutdown();
    this->gripper_pub.shutdown();
    delete mtx;
}

}; //!simulation 
