#ifndef PANTAGRAPH_CTR_H
#define PANTAGRAPH_CTR_H

#include <math.h>
#include <iostream>
#include <stdio.h>
#include <stdint.h>
#include <fstream>
#include <sys/time.h>
#include <map>
#include <vector>
#include "angle_limit.h"
#include "motor_driver/MotorDriver.hpp"

using namespace std;

#define READ_ONLY 0
#define LEG_LENGTH 0.1
#define PANTA_WIDTH 0.084
#define LEG_NUM 1
// #define Weight 5
#define g 9.8


class Custom{
private:
    double amp_sw = 0.01;
    double amp_st = 0.01;
    // double l_0 = 2*0.1*cos(3.141592653/8.);
    double l_0 = 0.17;
    // double l_0 = 0.14;
    double spring_coeff = 1500;
    double damping_coeff = 25;
    double spring_rot_coeff = 10.;
    double damping_rot_coeff = 0.1;
    double omega = 11;
    // double omega = 3.14*1.0;
    int sns_error_check = 0;

    //  this is the torque for joint 1   
    double spring_torque_th1[LEG_NUM];

    //  this is the torque for joint 2
    double spring_torque_th2[LEG_NUM];

    
    // gain parameter with g1, g2
    // const double kappa=0;
    // const double K[4][4] = {{0, 0 ,0 , 0},
    //                         {0, 0, 0 ,0},
    //                         {0 ,0 ,0 , 0},
    //                         {0 ,0 ,0 ,0}};

    // const double delta_12 = 0.0*M_PI;
    // const double delta_13 = 0.5*M_PI;
    // const double delta_34 = 0.0*M_PI;
    // const double deruta[4][4] = {{0,delta_12,delta_13,delta_13+delta_34},
    //                             {-delta_12,0,-delta_12+delta_13,-delta_12+delta_13+delta_34},
    //                             {-delta_13,delta_12-delta_13,0,delta_34},
    //                             {-(delta_13+delta_34),-(-delta_12+delta_13+delta_34),-delta_34,0}};



public:
    double phi[LEG_NUM]={0};
    // void RobotControl(double dt, vector<int> motor_ids, std::map<int, motor_driver::motorState> motor_state_map);
    void RobotControl(double dt, vector<int> motor_ids, std::map<int, motor_driver::motorState> motor_state_map,double cmd_array[][2][5]);
    void Euler(double dt, double leg[LEG_NUM],double phi[LEG_NUM]);
    void Read_Param(double param_set[]);
    void Jacovian(double angle1, double angle2, double dangles1, double dangles2);
    // Calculate Vertual Leg  
    //  sensed angles and angular velocity of each leg
    double sns_angles[LEG_NUM][2];
    double sns_dangles[LEG_NUM][2];

    //  sensed virtual leg length and velocity converted from sensed angles
    double sns_slength[LEG_NUM];
    double sns_dslength[LEG_NUM];

    double sns_slength1[LEG_NUM];
    double sns_slength2[LEG_NUM];
    double sns_dslength1[LEG_NUM];
    double sns_dslength2[LEG_NUM];

    //  sensed virtual leg angle and angular vel converted from sensed angles        
    double sns_sangle[LEG_NUM];
    double sns_dsangle[LEG_NUM];

    //  forces and torques that the virtual leg should realize
    double spring_force[LEG_NUM];
    double spring_torque[LEG_NUM];        
    double spring_force1[LEG_NUM];
    double spring_force2[LEG_NUM];

    double denominator_th1[LEG_NUM];
    double denominator_th2[LEG_NUM];
    // double Jacobian[LEG_NUM][2];

    double natural_length[LEG_NUM];
    double natural_angle[LEG_NUM];
    
    // float cmd_Tmotor[LEG_NUM][2][5];
    double jacov[2][2];
    double new_jacov[2][2];

};

void Custom::RobotControl(double dt, vector<int> motor_ids, std::map<int, motor_driver::motorState> motor_state_map, double cmd_array[][2][5]){
    limit limit;

    for (int j=0; j<LEG_NUM; j++){
        // Leg j 
        double sns_angle_check[2] = {0,0};

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////// rewrite here
        // data sensing 
        for(int i=0; i<2; i++){
            sns_angles[j][i] = motor_state_map[motor_ids[2*j+i]].position;
            sns_dangles[j][i] = motor_state_map[motor_ids[2*j+i]].velocity;
            sns_angle_check[i] = sns_angles[j][i];
        }
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        if(!sns_error_check){
            sns_error_check = limit.angle_limit(sns_angle_check);
        }
        if(sns_error_check){
            // std::cout << " torq off " << std::endl;
            // Torque offset for joint 1,2 
            spring_torque_th1[j] = 0.;
            spring_torque_th2[j] = 0.;
            // std::cout << spring_torque_th1[j] << "," << spring_torque_th2[j] << std::endl;
        }

        // // for real pantagraph
        // denominator_th1[j] = sqrt(LEG_LENGTH*LEG_LENGTH-(PANTA_WIDTH/2. + LEG_LENGTH*sin(sns_angles[j][0]))*(PANTA_WIDTH/2. + LEG_LENGTH*sin(sns_angles[j][0])));
        // denominator_th2[j] = sqrt(LEG_LENGTH*LEG_LENGTH-(PANTA_WIDTH/2. - LEG_LENGTH*sin(sns_angles[j][1]))*(PANTA_WIDTH/2. - LEG_LENGTH*sin(sns_angles[j][1])));
        // Jacobian[j][0] = (- sin(sns_angles[j][0])* LEG_LENGTH - LEG_LENGTH*( LEG_LENGTH*sin(sns_angles[j][0]) + PANTA_WIDTH/2.)*cos(sns_angles[j][0]) / denominator_th1[j]);
        // Jacobian[j][1] = (- sin(sns_angles[j][1])* LEG_LENGTH + LEG_LENGTH*(-LEG_LENGTH*sin(sns_angles[j][1]) + PANTA_WIDTH/2.)*cos(sns_angles[j][1]) / denominator_th2[j]);
        // // Jacobian[j][1] = (- sin(sns_angles[j][1])* LEG_LENGTH - LEG_LENGTH*( LEG_LENGTH*sin(sns_angles[j][1] + PANTA_WIDTH/2.)*cos(sns_angles[j][1])) / denominator_th2[j]);
        // // Jacobian[j][1] = (sin(-sns_angles[j][1])* LEG_LENGTH + LEG_LENGTH*( LEG_LENGTH*sin(-sns_angles[j][1]) + PANTA_WIDTH/2.)*cos(sns_angles[j][1])) / denominator_th2[j];

        // sns_slength[j] = LEG_LENGTH * cos(sns_angles[j][0]) + sqrt(pow(LEG_LENGTH,2)-pow(PANTA_WIDTH/2. + LEG_LENGTH*sin(sns_angles[j][0]),2));
        // // sns_slength[j] = LEG_LENGTH * cos(sns_angles[j][1]) + sqrt(pow(LEG_LENGTH,2)-pow(PANTA_WIDTH/2. - LEG_LENGTH*sin(sns_angles[j][1]),2));
        // sns_slength1[j] = LEG_LENGTH * cos(sns_angles[j][0]) + sqrt(pow(LEG_LENGTH,2)-pow(PANTA_WIDTH/2. + LEG_LENGTH*sin(sns_angles[j][0]),2));
        // sns_slength2[j] = LEG_LENGTH * cos(sns_angles[j][1]) + sqrt(pow(LEG_LENGTH,2)-pow(PANTA_WIDTH/2. - LEG_LENGTH*sin(sns_angles[j][1]),2));
        Jacovian(sns_angles[j][0],sns_angles[j][1],sns_dangles[j][0],sns_dangles[j][1]);

        // sns_dslength[j] = sns_dangles[j][0]*(Jacobian[j][0]);
        // sns_dslength1[j] = sns_dangles[j][0]*(Jacobian[j][0]);
        // sns_dslength2[j] = sns_dangles[j][1]*(Jacobian[j][1]);  
        // sns_sangle[j] = (sns_angles[j][0] + sns_angles[j][1])/2.;
        // sns_dsangle[j] = (sns_dangles[j][0] + sns_dangles[j][1])/2.;
        // std::cout << sns_slength[j] << " , "<< sns_dslength[j] << " , "<<sns_sangle[j] <<" , "<< sns_dsangle[j] <<" , "<< std::endl;
    }

    if(!sns_error_check){
    // //  calculation of phi 
        Euler(dt,sns_slength,phi); 
        for(int j=0;j<LEG_NUM;j++){   
            if(phi[j]>2*M_PI){
                phi[j]-=2*M_PI;
            }

            // if( phi[j] >=0 && phi[j] < M_PI){
            //     natural_length[j] = l_0 + amp_sw * (cos(2*phi[j])-1);
            // }else{
            //     natural_length[j] = l_0 - amp_st * (cos(2*phi[j])-1);
            // }
            natural_length[j] = l_0 + amp_sw * sin(phi[j]);
            // natural_length[j] = l_0;
            natural_angle[j] = 0;

            spring_force[j] = spring_coeff * ( natural_length[j] - sns_slength[j] ) - damping_coeff * sns_dslength[j]; 
            spring_torque[j] = spring_rot_coeff * ( natural_angle[j] - sns_sangle[j] ) - damping_rot_coeff * sns_dsangle[j];
 
            // spring_force1[j] = spring_coeff * ( natural_length[j] - sns_slength1[j] ) - damping_coeff * sns_dslength1[j]; 
            // spring_force2[j] = spring_coeff * ( natural_length[j] - sns_slength2[j] ) - damping_coeff * sns_dslength2[j];            
            // Torque offset for joint 1,2 
            // spring_torque_th1[j] = -LEG_LENGTH * spring_force[j] * sin((sns_angles[j][0] - sns_angles[j][1] )/2. )+ spring_torque[j] /2.;
            // spring_torque_th2[j] =  LEG_LENGTH * spring_force[j] * sin((sns_angles[j][0] - sns_angles[j][1] )/2. )+ spring_torque[j] /2.;

            // Torque for real pantagraph
            // spring_torque_th1[j] = spring_force[j] * Jacobian[j][0]/2.;
            // spring_torque_th2[j] = spring_force[j] * Jacobian[j][1]/2.;
            // spring_torque_th1[j] = spring_force1[j] * Jacobian[j][0]/2.;
            // spring_torque_th2[j] = spring_force2[j] * Jacobian[j][1]/2.;


            spring_torque_th1[j] = spring_force[j] * new_jacov[0][0] + spring_torque[j]*new_jacov[1][0];
            spring_torque_th2[j] = spring_force[j] * new_jacov[0][1] + spring_torque[j]*new_jacov[1][1];

            // std::cout << spring_force[j] << "," << spring_torque[j] << std::endl;
            std::cout << spring_torque_th1[j] << "," << spring_torque_th2[j] << std::endl;
                
        }
    }

    // for(int j=0;j<LEG_NUM;j++){ 
    //     cmd_Tmotor[j][0][0] = 0;//desired position
    //     cmd_Tmotor[j][0][1] = 0;//desired velcity
    //     cmd_Tmotor[j][0][2] = 0;//kp
    //     cmd_Tmotor[j][0][3] = 0;//kd
    //     cmd_Tmotor[j][0][4] = spring_torque_th1[j];//torq
    
    //     cmd_Tmotor[j][1][0] = 0;//desired position
    //     cmd_Tmotor[j][1][1] = 0;//desired velcity
    //     cmd_Tmotor[j][1][2] = 0;//kp
    //     cmd_Tmotor[j][1][3] = 0;//kd
    //     cmd_Tmotor[j][1][4] = spring_torque_th2[j];//torq
    // }
    for(int j=0;j<LEG_NUM;j++){
        cmd_array[j][0][0] = 0;//desired position
        cmd_array[j][0][1] = 0;//desired velcity
        cmd_array[j][0][2] = 0;//kp
        cmd_array[j][0][3] = 0;//kd
        cmd_array[j][0][4] = spring_torque_th1[j];//torq
        
        cmd_array[j][1][0] = 0;//desired position
        cmd_array[j][1][1] = 0;//desired velcity
        cmd_array[j][1][2] = 0;//kp
        cmd_array[j][1][3] = 0;//kd
        cmd_array[j][1][4] = spring_torque_th2[j];//torq
    }
    // std::cout<< sns_error_check <<std::endl;
}

void Custom::Euler(double dt,double leg[LEG_NUM],double phi[LEG_NUM]){
    double ka[LEG_NUM];
    double g1[LEG_NUM] = {0};
    double g2[LEG_NUM] = {0};

    for(int i=0;i<LEG_NUM;i++){
        ka[i]= omega * dt;// phase shift by omega
        // if(phi[i]>=M_PI && phi[i]<2*M_PI){
        //     g2[i] = -2*kappa*amp_st*(leg[i]-l_0+amp_st*(cos(2*phi[i])-1))*sin(2*phi[i]);
        //     ka[i]+=g2[i]*dt;// phase shift by g2(local feedback)
        // }
    }
    // for(int i=0;i<LEG_NUM;i++){// phase shift by g1(limb coordination)
    //     g1[0]-=K[0][i]*sin(phi[0]-phi[i]-deruta[0][i])*dt;
    //     g1[1]-=K[1][i]*sin(phi[1]-phi[i]-deruta[1][i])*dt;
    //     g1[2]-=K[2][i]*sin(phi[2]-phi[i]-deruta[2][i])*dt;
    //     g1[3]-=K[3][i]*sin(phi[3]-phi[i]-deruta[3][i])*dt;
    // }
    // for(int i=0;i<LEG_NUM;i++){// phase shift by g1(limb coordination)
    //     ka[i] += g1[i];
    // }
    // std::cout<<"g1  ::  "<<g1[0]<<","<<g1[1]<<","<<g1[2]<<","<<g1[3]<<std::endl;
    // std::cout<<"g2  ::  "<<g2[0]<<","<<g2[1]<<","<<g2[2]<<","<<g2[3]<<std::endl;
    // std::cout << std::endl;
    
    for(int i=0;i<LEG_NUM;i++){
        phi[i] +=ka[i];
    }
}

void Custom::Read_Param(double param_set[]){
    param_set[0] = amp_sw;
    param_set[1] = amp_st;
    param_set[2] = l_0;
    param_set[3] = spring_coeff;
    param_set[4] = damping_coeff;
    param_set[5] = spring_rot_coeff;
    param_set[6] = damping_rot_coeff;
    param_set[7] = omega;
}


void Custom::Jacovian(double angle1, double angle2, double dangles1, double dangles2){
    double theta = (angle1 - angle2)/2.;
    double psi = (angle1 + angle2)/2.;
    double dtheta = (dangles1 - dangles2)/2.;
    double dpsi = (dangles1 + dangles2)/2.;

    double length = LEG_LENGTH*cos(theta) + sqrt(pow(LEG_LENGTH,2) - pow((PANTA_WIDTH+2*LEG_LENGTH*sin(theta)),2)/4. );
    // double rot_angle = (PANTA_WIDTH*LEG_LENGTH*cos(theta)+2*length*sin(theta))/(length*(PANTA_WIDTH+2*LEG_LENGTH*sin(theta)))*psi; //keisan miss 1022
    double rot_angle = (PANTA_WIDTH*LEG_LENGTH*cos(theta)+2*length*sin(theta)+LEG_LENGTH*(LEG_LENGTH-1)*sin(2*theta))/(length*(PANTA_WIDTH+2*LEG_LENGTH*sin(theta)))*psi;

    double deno = sqrt(pow(LEG_LENGTH,2) - pow((PANTA_WIDTH+2*LEG_LENGTH*sin(theta)),2)/4.);
    jacov[0][0] = -  LEG_LENGTH*sin(theta) - LEG_LENGTH*( LEG_LENGTH*sin(theta) + PANTA_WIDTH/2.)*cos(theta) / deno;
    jacov[0][1] = 0;

    // jacov[1][0] = -2*PANTA_WIDTH*psi*(LEG_LENGTH*(jacov[0][0]*LEG_LENGTH*cos(theta) + PANTA_WIDTH*LEG_LENGTH/2.)*sin(theta) + (jacov[0][0]*LEG_LENGTH*PANTA_WIDTH/2. - length*length)*cos(theta)+LEG_LENGTH*LEG_LENGTH*length)
    //             /(pow(length,2)*pow((PANTA_WIDTH+2*LEG_LENGTH*sin(theta)),2));
    // jacov[1][1] = 2*(LEG_LENGTH*sin(theta) + PANTA_WIDTH/2.)*(PANTA_WIDTH*LEG_LENGTH*cos(theta)+2*length*sin(theta)) 
    //             /(length*pow((PANTA_WIDTH+2*LEG_LENGTH*sin(theta)),2));
    jacov[1][0] = (-2*PANTA_WIDTH*(LEG_LENGTH*(jacov[0][0]*LEG_LENGTH*cos(theta) + PANTA_WIDTH*length/2.)*sin(theta) + (jacov[0][0]*LEG_LENGTH*PANTA_WIDTH/2. - length*length)*cos(theta)+LEG_LENGTH*LEG_LENGTH*length - length*LEG_LENGTH*(LEG_LENGTH-1)*pow(cos(theta),2)) - 2*LEG_LENGTH*(LEG_LENGTH-1)*sin(theta)*(PANTA_WIDTH+2*LEG_LENGTH*sin(theta))*(length*sin(theta)+jacov[0][0]*cos(theta)))*psi
                /(pow(length,2)*pow((PANTA_WIDTH+2*LEG_LENGTH*sin(theta)),2));
    jacov[1][1] = (PANTA_WIDTH*LEG_LENGTH*cos(theta)+2*length*sin(theta)+LEG_LENGTH*(LEG_LENGTH-1)*sin(2*theta))
                /(length*(PANTA_WIDTH+2*LEG_LENGTH*sin(theta)));


    new_jacov[0][0] =   jacov[0][0]/2.;
    new_jacov[0][1] = - jacov[0][0]/2.;
    new_jacov[1][0] =   jacov[1][0]/2. + jacov[1][1]/2.;
    new_jacov[1][1] = - jacov[1][0]/2. + jacov[1][1]/2.;

    sns_slength[0] = length;
    sns_sangle[0] = rot_angle;
    sns_dslength[0] = jacov[0][0]*dtheta;
    sns_dsangle[0] = jacov[1][0]*dtheta + jacov[1][1]*dpsi;
}

#endif
