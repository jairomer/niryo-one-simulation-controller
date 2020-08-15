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

#include <signal.h>
#include <iostream>
#include "testing.hpp"
#include "ros/ros.h"

void exit_handler(int s)
{
    printf("Exiting");
    ros::shutdown();
    exit(0);
}

int main(int argc, char** argv) 
{ 
    struct sigaction sigIntHandler;
    sigIntHandler.sa_handler = exit_handler;
    sigemptyset(&sigIntHandler.sa_mask);
    sigIntHandler.sa_flags = 0;

    sigaction(SIGINT, &sigIntHandler, NULL);
    
    bool success = digital_twin_integration_tests(argc, argv);
    if (success)
        test_physical_twin_control(argc, argv);
    else
        ROS_INFO("Simulated twin not ready. Exiting.")

    exit(0);
}