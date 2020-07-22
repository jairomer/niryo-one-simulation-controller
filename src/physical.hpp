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
#include <niryo_one_msgs/RobotMoveAction.h>
#include <actionlib/client/simple_action_client.h>

class PTGripper 
{
private:
    actionlib::SimpleActionClient<niryo_one_msgs::RobotMoveAction>* ac;
    niryo_one_msgs::ToolCommand command;
    niryo_one_msgs::RobotMoveActionGoal action;
public:
    PTGripper() {
        ac = new actionlib::SimpleActionClient<niryo_one_msgs::RobotMoveAction>("/niryo_one/commander/robot_action", true);
        command.gripper_open_speed = 100;
        command.tool_id = 12;
        action.goal.cmd.cmd_type = 6;
    }

    ~PTGripper() {
        delete ac;
    }

    /**
        Opens the gripper. 

        Returns true on success and false on timeout.
    */
    bool open();

    /**
        Closes the gripper. 

        Returns true on success and false on timeout.
    */
    bool close();
}; 


#endif