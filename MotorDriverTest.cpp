#include <assert.h>
#include <stdio.h>
#include <string.h>

#include <chrono>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <thread>
#include <vector>
#include <chrono>

#include "motor_driver/MotorDriver.hpp"
#include "pantagraph_ctr.h"

using namespace std;

using namespace std::chrono;

// Get time stamp in microseconds.
uint64_t get_time_in_microseconds() {
  uint64_t us =
      std::chrono::duration_cast<std::chrono::microseconds>(
          std::chrono::high_resolution_clock::now().time_since_epoch())
          .count();
  return us;
}

// Print Motor State
void printMotorStates(std::map<int, motor_driver::motorState> motor_states) {
  for (auto const& x : motor_states) {
    std::cout << "Motor ID: " << x.first << " Position: " << x.second.position
              << " Velocity: " << x.second.velocity
              << " Torque: " << x.second.torque << std::endl;
  }
}

int main(int argc, char** argv) {
  if (argc < 3) {
    cout
        << "Example Usage: ./MotorDriverTest can0 <motor_id_1> <motor_id_2> ..."
        << endl;
    return 0;
  }
  vector<int> motor_ids;

  char* can_interface(argv[1]);

  assert(strncmp(can_interface, "can", 3) == 0 &&
         "First argument must be a can interface starting with 'can'. Example: can0 or can1");

  for (int i = 2; i < argc; i++) {
    motor_ids.push_back(atoi(argv[i]));
  }
  for (auto ids : motor_ids) {
    cout << "IDS: " << ids << endl;
  }

  motor_driver::MotorDriver motor_controller(
      motor_ids, can_interface, motor_driver::MotorType::AK60_6_V1);

  cout << "Enabling Motor..." << endl;
  auto start_state = motor_controller.enableMotor(motor_ids);
  printMotorStates(start_state);
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));

  cout << "Setting Zero Position..." << endl;
  auto stateZero = motor_controller.setZeroPosition(motor_ids);
  printMotorStates(stateZero);

  std::this_thread::sleep_for(std::chrono::milliseconds(1000));

  // initialize the position to zero 
  // double pos1 = stateZero[motor_ids[0]].position;
  // double pos2 = stateZero[motor_ids[1]].position;
  double pos1 =0.;
  double pos2 = 0.;
  // double d_pos1 = 3.14/4;
  // double d_pos2 = -3.14/4;
  // double d_pos1 = 0.8;
  // double d_pos2 = -0.8;
  double d_pos1 = 3.141592653/12.;
  double d_pos2 = -3.141592653/12.;
  std::cout << "Initializing Position ..." << endl;
  std::map<int, motor_driver::motorCommand> commandMap;
  for (int i=0; i<1000; i++){
    // motor_driver::motorCommand moveCommandStruct = {(1000-1-i)/1000. * pos1, 0, 5, 1, 0};
    // motor_driver::motorCommand moveCommandStruct1 = {((1000-1-i)* pos1+i*d_pos1)/1000. , 0, 10, 2, 0};
    motor_driver::motorCommand moveCommandStruct1 = {((1000-1-i)* pos1+i*d_pos1)/1000. , 0, 20, 2, 0};
    commandMap[motor_ids[0]] = moveCommandStruct1;
    // moveCommandStruct = {(1000-1-i)/1000. * pos2, 0, 10, 2, 0};
    // motor_driver::motorCommand moveCommandStruct2 = {((1000-1-i)* pos2+i*d_pos2)/1000. , 0, 10, 2, 0};
    motor_driver::motorCommand moveCommandStruct2 = {((1000-1-i)* pos2+i*d_pos2)/1000. , 0, 20, 2, 0};
    commandMap[motor_ids[1]] = moveCommandStruct2;
    auto istate = motor_controller.sendRadCommand(commandMap); 
    std::this_thread::sleep_for(std::chrono::milliseconds(3));
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  
  std::cout << "After Initializing Position ..." << endl;
  motor_driver::motorCommand stopCommandStruct = {0, 0, 0, 0, 0};
  std::map<int, motor_driver::motorCommand> stopCommandMap;
  for (int i = 0; i < motor_ids.size(); i++) {
    stopCommandMap[motor_ids[i]] = stopCommandStruct;
  }
  auto commandState = motor_controller.sendRadCommand(stopCommandMap);
  printMotorStates(commandState);

  int i;
  cout << "Enter any number to move motors 180 deg..." << endl;
  cin >> i;

  //  Iterate over motor ids to create command map for 90 deg rotation and send  
  auto startT = get_time_in_microseconds();
  cout << "Moving Motor..." << endl;
  int duration = 30000;
  int freq = 1000;
  u_int64_t index = 0;
  double torque_limit = 9;
  bool time_flag = false;
  // u_int64_t dts[duration];
  double poss[2][duration];
  double vels[2][duration];
  double taus[2][duration];
  double nat_angle[duration];
  double nat_length[duration];
  double sns_angle[duration];
  double sns_length[duration];
  double sns_dlength[duration];
  double sns_dangle[duration];
  double spring_force[duration];
  double spring_torque[duration];
  double cmd_torque[2][duration];
  double time[duration];
  double phase[duration];
  double cmd_array[1][2][5];
  double param_set[8];


  double sns_length1[duration];
  double sns_length2[duration];

  Custom custom;
  std::ofstream ofs("freq_.dat");
  
  for (int i=0; i<duration; i++){
    index++;
    // Calculate command 
    // double omega = 0.5;
    // float pos = 3.14 * sin(omega/freq * i);

    // motor_driver::motorCommand tmpCommandStruct1 = {pos, 0, 10, 2, 0};
    // motor_driver::motorCommand tmpCommandStruct1 = {0, 0, 0, 0, 0.5};
    // commandMap[motor_ids[0]] = tmpCommandStruct1;
    
    // motor_driver::motorCommand tmpCommandStruct2 = {pos, 0, 10, 2, 0};
    // motor_driver::motorCommand tmpCommandStruct2 = {0, 0, 0, 0, 0.5};
    // commandMap[motor_ids[1]] = tmpCommandStruct2;
    
    // auto state = motor_controller.sendRadCommand(commandMap); 
    // if (i %10 ==1){
    //    printMotorStates(state);
    // }
    // poss[i] = state[motor_ids[0]].position;
    // vels[i] = state[motor_ids[0]].velocity;
    // taus[i] = state[motor_ids[0]].torque;
    
  // }
/////////////////////////////////////////////////////////////////////////////////////////////////////////
  

  //loop
  // while(1){
    // custom.RobotControl(1./freq, motor_ids, commandState);
    custom.RobotControl(1./freq, motor_ids, commandState, cmd_array);

    // // motor_driver::motorCommand moveCommandStruct11 = {custom.cmd_Tmotor[0][0][0],custom.cmd_Tmotor[0][0][1] ,custom.cmd_Tmotor[0][0][2] ,custom.cmd_Tmotor[0][0][3] ,custom.cmd_Tmotor[0][0][4] };
    // // motor_driver::motorCommand moveCommandStruct12 = {custom.cmd_Tmotor[0][1][0],custom.cmd_Tmotor[0][1][1] ,custom.cmd_Tmotor[0][1][2] ,custom.cmd_Tmotor[0][1][3] ,custom.cmd_Tmotor[0][1][4] };
    motor_driver::motorCommand moveCommandStruct11 = {cmd_array[0][0][0],cmd_array[0][0][1] ,cmd_array[0][0][2] ,cmd_array[0][0][3] ,cmd_array[0][0][4] };
    motor_driver::motorCommand moveCommandStruct12 = {cmd_array[0][1][0],cmd_array[0][1][1] ,cmd_array[0][1][2] ,cmd_array[0][1][3] ,cmd_array[0][1][4] };
    std::map<int, motor_driver::motorCommand> commandMap;
    
    commandMap[motor_ids[0]] = moveCommandStruct11;
    commandMap[motor_ids[1]] = moveCommandStruct12;
    
    commandState = motor_controller.sendRadCommand(commandMap);

    // cout << "Moving Motor..." << endl;
    // auto startT = get_time_in_microseconds();
    // std::ofstream ofs.open("test.dat");
    // ofs << "1:ID, 4* (2:cmdsTheta , 3:cmdsLength, 4:sTheta, 5:dsTheta, 6:sLength, 7:dsLength, 8:sForce, 9:cmdTorq1, 10:cmdTorq2, " << "12-13: joint0-1, " << "14-15: djoint0-1, " << "16-17: estTorq0-1, " << "22: phase), "<< std::endl;
    // ofs << index << "," <<  (get_time_in_microseconds() - startT)/1000. << "," << custom.natural_angle[0] << "," << custom.natural_length[0] << "," << custom.sns_sangle[0] << "," << custom.sns_slength[0] << "," << custom.sns_dslength[0] << "," << custom.spring_force[0] << "," << custom.spring_torque_th1[0] << "," << custom.spring_torque_th2[0] << std::endl;
    
    poss[0][i] = commandState[motor_ids[0]].position;
    vels[0][i] = commandState[motor_ids[0]].velocity;
    taus[0][i] = commandState[motor_ids[0]].torque;
    poss[1][i] = commandState[motor_ids[1]].position;
    vels[1][i] = commandState[motor_ids[1]].velocity;
    taus[1][i] = commandState[motor_ids[1]].torque;
    nat_angle[i] = custom.natural_angle[0];
    nat_length[i] = custom.natural_length[0];
    sns_angle[i] = custom.sns_sangle[0];
    sns_length[i] = custom.sns_slength[0];
    sns_dlength[i] = custom.sns_dslength[0];
    sns_dangle[i] = custom.sns_dsangle[0];
    spring_force[i] = custom.spring_force[0];
    spring_torque[i] = custom.spring_torque[0];
    cmd_torque[0][i] = cmd_array[0][0][4];
    cmd_torque[1][i] = cmd_array[0][1][4];
    time[i] = (get_time_in_microseconds() - startT)/1000.;
    phase[i] = custom.phi[0];
    sns_length1[i] = custom.sns_slength1[0];
    sns_length2[i] = custom.sns_slength2[0];

    // Torque limit check    
    if( fabs(commandState[motor_ids[0]].torque) >  torque_limit || fabs(commandState[motor_ids[1]].torque) >  torque_limit){
      std::cout << "ERROR :: TORQUE exceeds " << torque_limit << " Nm" << std::endl;
      commandState = motor_controller.sendRadCommand(stopCommandMap);
      printMotorStates(commandState);
      break;
    }

    auto now = std::chrono::system_clock::now();
    double millis = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
    while( (get_time_in_microseconds() - startT) < index * 1000000 / freq){
      // if(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count() - millis > 3000){
      //   time_flag = true;
      //   break;
      // }
    }
    // if(time_flag){
    //   break;
    // }
    // dts[i] = get_time_in_microseconds() - startT;
  }
//////////////////////////////////////////////////////////////////////////////////////////////////////////////

  auto endT = get_time_in_microseconds();
  auto dt = (endT - startT);
  std::cout << "Time Taken for 1 Command: " << (dt / (double)duration) << " us" << std::endl;
  // auto startT = get_time_in_microseconds();
  //auto commandState = motor_controller.sendRadCommand(commandMap);
  commandState = motor_controller.sendRadCommand(stopCommandMap);

  // std::ofstream file;
  // file.open("test.dat");
  // file << "# ID, dt, pos, vel, tau"  <<  std::endl;
  custom.Read_Param(param_set);
  ofs << "% amp_sw : " << param_set[0] << ", amp_st : " << param_set[1] << ", l_0 : " << param_set[2] << ", omega : " << param_set[7] << std::endl;
  ofs << "% spring K : " << param_set[3] << ", damping : " << param_set[4] << ", rot spring K : " << param_set[5]  << ", rot damping : " << param_set[6] << std::endl;
  ofs << "% 1:ID, 2:time, 3: phase,  4:cmdsTheta , 5:cmdsLength, 6:sTheta, 7:dsTheta, 8:sLength, 9:dsLength, 10:spring_Force, 11: rot_spring_force, 12:cmdTorq1, 13:cmdTorq2, " << "14-15: joint0-1, " << "16-17: djoint0-1, 18-19: est_torq0-1" << std::endl;
  ofs << "% 20-21:sLength1,2" << std::endl; 
  for (int i=0; i<duration; i++){
    // file << i << ", " <<  dts[i] - dts[i-1] << ", "<< poss[i] << ", " << vels[i] << ", "  << taus[i] <<  std::endl;
    ofs << i+1 << ", " <<  time[i] << ", " <<  phase[i] << ", "<< nat_angle[i] << ", " << nat_length[i] << ", " << sns_angle[i] << ", " << sns_dangle[i] << ", " << sns_length[i] << ", " << sns_dlength[i] << ", " << spring_force[i] << ", " << spring_torque[i] << ", " << cmd_torque[0][i] << ", " << cmd_torque[1][i] << ", ";
    ofs << poss[0][i] << ", " << poss[1][i] << ", "<< vels[0][i] << ", " << vels[1][i] << ", " << taus[0][i] << ", " << taus[1][i] <<", ";
    ofs << sns_length1[i] << ", " <<sns_length2[i] << std::endl;
  }
  // file.close();
  ofs.close();

  //std::this_thread::sleep_for(std::chrono::milliseconds(3000));
  // commandState = motor_controller.sendRadCommand(stopCommandMap);

  // auto endT = get_time_in_microseconds();
  // auto dt = (endT - startT);
  // std::cout << "Time Taken for Command: " << double(dt / 1e6) << std::endl;

  std::this_thread::sleep_for(std::chrono::milliseconds(1000));

  cout << "Disabling Motor..." << endl;
  auto end_state = motor_controller.disableMotor(motor_ids);
  printMotorStates(end_state);
  return 0;
}
