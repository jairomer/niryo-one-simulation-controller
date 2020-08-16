#include "utils.hpp"

/**
 * Sets a linear magnitude to angular. 
 * 
 * m     ->  radians
 * m/s   ->  radians/s
 * m/s^2 ->  radians/s^2
 * 
*/
float ang(float linear) { return linear*M_PI/180; }


/**
 * Print vector to stdout.
*/
void print_vector(std::vector<double>& v)
{
    for (auto it = v.begin(); it != v.end(); it++)
    {
        std::cout << *it << " ";
    }
    std::cout << std::endl;
}