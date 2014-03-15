#include "armstate.h"
#include <robot/arm_joint.h>

ArmState::ArmState()
{


}

bool ArmState::ArmInit(){
    //step 0 初始化接口设备
        arm_caninit();
    //step 1 检查机器人的状态

    //step 1.1读取机器人当前状态后可以设置当前关节角度
    for(int i=0; i<ARM_DOF; i++)
    {
        arm_current_state[i]= arm_init_joint_state[i];
        //ArmState::arm_current_state
    }

    ArmIkCheckFk(&arm_pose.position, &arm_pose.orientation,arm_current_state);
    PrintfPose(arm_pose);
    if(!ArmIk(arm_pose.position, arm_pose.orientation, arm_current_state, arm_solve_state, NULL))
        SetArmJoState(arm_current_state);
    //step 2 读取机器人状态，设置相应的按钮参数
    return true;
}

//设置机械手臂状态
/*
 *parm pose     arm_temp_pose   @  表示arm解算得到的暂时姿态
 *     double   arm_state       @  表示需要设置的手臂的角度
 *
 *return bool   true            @  表示CAN总线连接成功
 *              false           @  表示CAN总线失败
 *
 */
bool ArmState::SetArmJoState(pose arm_temp_pose, double arm_state[ARM_DOF]){
    //step1 设置关节角度
    for(int i=0x01; i <= ARM_DOF; i++){
         arm_setpos_joint_noth(i,arm_state[i-1],JOINT_RADIAN);
    }

    //TODO 错误处理

    //step2 设置变量arm_current_state
     for(int i=0; i < ARM_DOF; i++){
         arm_current_state[i] = arm_state[i];
     }

    //step3 更新arm_pose
     arm_pose = arm_temp_pose;
     return true;
}

//设置机械手臂状态
/*
 *parm pose     arm_temp_pose   @  表示arm解算得到的暂时姿态
 *     double   arm_state       @  表示需要设置的手臂的角度
 *
 *return bool   true            @  表示CAN总线连接成功
 *              false           @  表示CAN总线失败
 *
 */
bool ArmState::SetArmJoState(pose arm_temp_pose, double arm_state[ARM_DOF], int joint_index){
    //step1 设置关节角度
    //TODO 错误处理
    if(joint_index == SET_ALL_JOINT){
        for(int i=0x01; i <= ARM_DOF; i++){
             arm_setpos_joint_noth(i,arm_state[i-1],JOINT_RADIAN);
        }
    } else{
         arm_setpos_joint_noth(joint_index,arm_state[joint_index-1],JOINT_RADIAN);
    }

    //TODO 错误处理

    //step2 设置变量arm_current_state
     for(int i=0; i < ARM_DOF; i++){
         arm_current_state[i] = arm_state[i];
     }

    //step3 更新arm_pose
     arm_pose = arm_temp_pose;
     return true;
}

bool ArmState::SetArmJoState(double arm_state[ARM_DOF]){
   //step1 设置关节角度
     // arm_setpos_joint_noth(0x06,0,JOINT_RADIAN);
    for(int i=0x01; i <= ARM_DOF; i++){
         arm_setpos_joint_noth(i,arm_state[i-1],JOINT_RADIAN);
    }

    //TODO 错误处理

   //step2 设置变量arm_current_state
    for(int i=0; i < ARM_DOF; i++){
        arm_current_state[i] = arm_state[i];
    }
   //step3 更新arm_pose
    if(!ArmFk(&arm_pose.position,&arm_pose.orientation,arm_current_state)){
        //TODO: 如何处理错误？
    }
    return true;
}

//读取机械手臂各个关节的状态
bool ArmState::ReadArmState(double arm_state[]){
    return true;
}

bool ArmState::SetArmInitState(){

    //step 1.1读取机器人当前状态后可以设置当前关节角度
    for(int i=0; i<ARM_DOF; i++)
    {
        arm_current_state[i]= arm_init_joint_state[i];
        //ArmState::arm_current_state
    }

    ArmIkCheckFk(&arm_pose.position, &arm_pose.orientation,arm_current_state);
    PrintfPose(arm_pose);
    if(!ArmIk(arm_pose.position, arm_pose.orientation, arm_current_state, arm_solve_state, NULL))
        SetArmJoState(arm_current_state);
    //step 2 读取机器人状态，设置相应的按钮参数

    return true;
}

double ArmState::arm_current_state[ARM_DOF] = {0,0,0,0,0,0};

pose ArmState::arm_pose = {{0,0,0},{0,0,0,0}};



