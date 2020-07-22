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


bool PTGripper::open()
{
    command.cmd_type = 1;
    action.goal.cmd.tool_cmd = command;
    this->ac->sendGoal(action.goal);
    return this->ac->waitForResult(ros::Duration(10.0));
}

bool PTGripper::close()
{
    command.cmd_type = 2;
    action.goal.cmd.tool_cmd = command;
    this->ac->sendGoal(action.goal);
    return this->ac->waitForResult(ros::Duration(10.0));
}