#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <iostream>
#include <math.h>
// #include <correct_math_defines.h>


// Function to cosine interpolate a vector and return a new vector
std::vector<double> cosine_interpolate(double start, double end, int npoints) {
    std::vector<double> new_points = {};
    for (int index=0; index<npoints; index++) {
        double angle = (double)index / (double)(npoints-1) * 3.1415926;
        double new_ratio = (1.0 - cos(angle)) * 0.5;            // in [0, 1]
        double new_point = start + new_ratio * (end - start);   // in [start, end]
        new_points.push_back(new_point);
    }
    return new_points;
}


void print_vector(std::vector<double>& vec) {
  
  std::cout << "[ ";
  for (unsigned int i=0; i<vec.size(); i++) {
    std::cout << vec.at(i) << ' ';
  }
  std::cout << "]" << std::endl;
}


int main(int argc, char *argv[])
{
    std::vector<double> points = cosine_interpolate(1, 5, 10);
    print_vector(points);

    return 0;
}