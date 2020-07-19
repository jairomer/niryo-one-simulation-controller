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
#ifndef PHYSICAL_HPP
#define PHYSICAL_HPP
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

class PhysicalTwin
{
private:
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    moveit::planning_interface::MoveGroupInterface arm_group(ARM_GROUP);
    const robot_state::JointModelGroup* joint_model_group,
public:
    static const std::string ARM_GROUP = "arm";
    PhysicalTwin() {
        joint_model_group = 
            arm_group.getCurrentState()->getJointModelGroup(ARM_GROUP);
    }

    moveit::planning_interface::MoveItErrorCode setJoints(std::vector<double>& joints);
};

#endif