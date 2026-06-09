#pragma once

#include <string>
#include <vector>
#include <iostream>
#include  <assert.h> 

enum PlatoonType {
   NONE  = 0,  //
   BUILD = 1,    //构建
   JOIN  = 2,     //入队
   LEAVE = 3,    //出队
   DISSOLVE = 4, //解散
   COLUMN = 5,   //纵队
   DIAMOND = 6,  //菱形
   TRIANGLE = 7, //三角形
   REVESE = 8,   //编队倒车
   CANCEL_REVESE = 9, //取消编队倒车
   MASS = 10,          //集结
   DISTRIBUTE = 11,    //分散
   RUNNING = 12
};

enum PlatoonRole {
  FOLLOWER , //跟随车
  LEADER     //引导车  
};


enum PlatoonPolicy {
  TIMING_INTERVAL ,  //定时距
  SPACE_INTERVAL     //定间距离 
};


enum PlatoonDrivingMode {
  MANUAL,  //人工
  AUTO     //自动 
};


int     platoonGetNumIndex(const std::vector<unsigned char> &vehicle_num_list,int self_num);

double  platoonGetLoffset(const std::vector<unsigned char> &vehicle_num_list,int self_num,const double &config_l_offset);

bool    platoonEraseNum( std::vector<unsigned char> &vehicle_num_list,int num);

bool    platoonCheckNum(const std::vector<unsigned char> &vehicle_num_list,int num);

int     platoonFronterIndex(PlatoonType type,int selfIndex);



