#ifndef IKFUNC_H
#define IKFUNC_H


#define IKFAST_HAS_LIBRARY // Build IKFast with API functions
#define IKFAST_NO_MAIN // Don't include main() from IKFast
#define IK_VERSION 61


#include "ikfast.h"
#include <ikarmdesc.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h> // for clock_gettime()

#include <cmath>
#include <Eigen/Eigen>

using namespace Eigen;

#define IKREAL_TYPE IkReal // for IKFast 56,61
using namespace ikfast;

double SIGN(double x);
double NORM(double a, double b, double c, double d);

//截断小数，保留后8位
double SectionNum(double num);

/*
 * fuction solve the joint
 * parm arm_pos @      arm postion for destination  x,y,z
 *      arm_ori @      arm orientation for destination x,y,z,w
 *      arm_state @    arm state now
 *      joint_solve @  return the joint if ik is ok
 *      flag @         for other function
 */
/*
 * fuction solve the joint
 * parm arm_pos @      机械臂末端目的坐标
 *      arm_ori @      机械末端姿态（四元素表示法）
 *      arm_state @    机械臂末端当前的姿态
 *      joint_solve @  解算结果
 *      flag @         for other function
 *      return         解算结果是否成功
 */
int ArmIk(pos arm_pos, ori arm_ori, double arm_state[6], double joint_solve[6], int flag = NULL);

/*
 * fuction solve the joint
 * parm arm_pos @      机械臂末端目的坐标
 *      arm_ori @      机械末端姿态（四元素表示法）
 *      joint_solve @  机械臂角度
 *      return         解算结果是否成功
 */
int ArmFk(pos* arm_pos, ori* arm_ori, double joint_solve[6]);

/*
 * fuction solve the joint
 * parm arm_pos @      机械臂末端目的坐标
 *      arm_ori @      机械末端姿态（四元素表示法）
 *      joint_solve @  机械臂角度
 *      return         解算结果是否成功
 */
int ArmIkCheckFk(pos* arm_pos, ori* arm_ori, double joint_solve[6]);

/*
 *function EndMoveXYZ
 * parm  end_ori @ 表示机械臂当前末端的姿态（四元素）
 *       mov_dst @ 表示机械臂沿着末端坐标系x，y，z移动的距离
 *
 * return pos    @ 表示机械臂末端沿着世界坐标系移动的距离
 */
pos EndMoveXYZ(ori end_ori, pos mov_dst);
pos posMoveori(ori endOri, pos posB);

/*
 * funciton EndRotXYZ
 * parm  end_ori @ 表示机械臂当前末端的姿态（四元素）
 *       rel_ori @ 表示机械臂沿着末端坐标系x，y，z轴旋转的角度
 *
 * return ori    @ 表示机械臂旋转后的末端的姿态（四元素）
 */
ori EndRotXYZ(ori end_ori, roz  rel_ori);
ori rozMove(ori endOri, roz rel_ori);

/*
 * function printfpose
 * print the pose data
 */
void PrintfPose(pose arm_pose);

/*
 * function printfpos
 * print the pos data
 */
void PrintfPos(pos arm_pos);

/*
 * function printfori
 * print the ori data
 */
void PrintfOri(ori arm_ori);

/*
 * function printfroz
 * print the roz data
 */
void PrintfRoz(roz arm_roz);


/*
 * function printfroz
 * print the joint data
 */
void PrintfJoint(double arm_joint[ARM_DOF]);

#endif // IKFUNC_H
