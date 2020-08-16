#include <iostream>
#include <math.h>
#include <vector>

/**
 * Sets a linear magnitude to angular. 
 * 
 * m     ->  radians
 * m/s   ->  radians/s
 * m/s^2 ->  radians/s^2
 * 
*/
float ang(float linear);

/**
 * Print vector to stdout.
*/
void print_vector(std::vector<double>& v);