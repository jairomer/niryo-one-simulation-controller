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

#include "physical.hpp"

moveit::planning_interface::MoveItErrorCode PhysicalTwin::setJoints(std::vector<double>& joints)
{
    moveit::planning_interface::MoveGroupInterface::Plan master_plan;
    moveit::planning_interface::MoveItErrorCode ret;
    if(!(ret = arm_group.setJointValueTarget(standard_pos))) {
        ROS_INFO("ERROR: Cannot set Joint Value Target");
        return ret;
    }
    if ((ret = arm_group.plan(master_plan)) != moveit::planning_interface::MoveItErrorCode::SUCCESS) {
        ROS_INFO("ERROR: Planning framework returned a non-zero code.");
        return ret;             
    }
    return arm_group.execute(master_plan);
}