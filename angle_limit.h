#ifndef ANGLE_LIMIT_H
#define ANGLE_LIMIT_H

#include <math.h>
#include <iostream>
#include <stdio.h>
#include <stdint.h>
#include <fstream>
#include <sys/time.h>

using namespace std;

#define READ_ONLY 0
#define LEG_LENGTH 0.1
#define LEG_NUM 1
// #define Weight 5
#define g 9.8
#define upper_limit_angle 50.*M_PI/180.//TODO
#define lower_limit_angle -0.1//TODO
// #define upper_limit_difference_angle 1.1//TODO
#define upper_limit_difference_angle 3.14//TODO
#define lower_limit_difference_angle -0.35//TODO

class limit{
    private:
    int sns_error = 0;
    public:
    int angle_limit(double sns_angle[]);
};

int limit::angle_limit(double sns_angle[]){
    if(sns_angle[0]-sns_angle[1] > upper_limit_difference_angle || sns_angle[0]-sns_angle[1] < lower_limit_difference_angle){
        sns_error = 1;
        std::cout << "angle difference limit !!!!!!!!!!!!!" << std::endl;
    }
    if(sns_angle[0] > upper_limit_angle || sns_angle[0] < lower_limit_angle){
        sns_error = 1;
        std::cout << "angle1 limit !!!!!!!!!!!!!" << std::endl;
    }
    if(sns_angle[1] > -lower_limit_angle || sns_angle[1] < -upper_limit_angle){
        sns_error = 1;
        std::cout << "angle2 limit !!!!!!!!!!!!!" << std::endl;
    }
    return sns_error;
}

#endif