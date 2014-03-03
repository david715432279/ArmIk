#ifndef ARMSTATE_H
#define ARMSTATE_H
#include <ikfunc.h>

#define POS_PRECISION 4
#define ORI_PRECISION 4
#define JOINT_PRECISION 4

class ArmState
{
public:
    ArmState();
    static double arm_current_state[ARM_DOF];
    static pose arm_pose;

    //初始化机械臂各个关节
    bool ArmInit();
    //读取机械臂各个关节的状态信息
    bool ReadArmState(double arm_state[ARM_DOF]);
    bool SetArmJoState(double arm_state[ARM_DOF]);
    bool SetArmJoState(pose arm_temp_pose, double arm_state[ARM_DOF]);

private:
    double arm_solve_state[ARM_DOF];

};

#endif // ARMSTATE_H
