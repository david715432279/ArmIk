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
#define STEP_PRECISION 0.001

using namespace ikfast;

double SIGN(double x);
double NORM(double a, double b, double c, double d);

//得到两点间的坐标插值
pos PosDis(pos begin,pos end);

//截断小数，保留后8位
double SectionNum(double num);

/*
 * fuction test the angle between point to point
 * 测试解算出的点与点之间机械臂各个关节角度是否存在翻转
 * parm a @      第一个点
 *      b @      第二个点
 *      angle @    最大角度（弧度表示）
 *      return     是否超过最大角度
 */
bool PointAngTest(double a[ARM_DOF], double b[ARM_DOF], double angle);

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
 *      0              成功
 *      -1             失败
 */
int ArmIk(pos arm_pos, ori arm_ori, const double arm_state[ARM_DOF], double joint_solve[ARM_DOF], int flag = NULL);

//int ArmIkF(pos arm_pos, ori arm_ori, const float arm_state[ARM_DOF], float joint_solve[ARM_DOF], int flag = NULL);

/*
 * fuction solve the joint
 * parm arm_pos @      机械臂末端目的坐标
 *      arm_ori @      机械末端姿态（四元素表示法）
 *      joint_solve @  机械臂角度
 *      return         解算结果是否成功
 */
int ArmIkCheckFk(pos* arm_pos, ori* arm_ori,const double joint_solve[6]);

/*
 * fuction solve the joint
 * parm arm_pos @      机械臂末端目的坐标
 *      arm_ori @      机械末端姿态（四元素表示法）
 *      joint_solve @  机械臂角度
 *      return         解算结果是否成功
 *      0              成功
 *      -1             失败
 */
int ArmFk(pos* arm_pos, ori* arm_ori, double joint_solve[6]);



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
 *function RelatMove
 * parm  end_pos @ 表示机械臂当前末端的位置和姿态（四元素）
 *       mov_dst @ 表示机械臂沿着末端坐标系x，y，z移动的距离
 *
 * return pose    @ 表示机械臂末端沿着相对坐标系移动mov——dst的最后坐标
 */
pose RelatMove(pose end_pos, pos mov_dst);

/*
 *function RelatRot
 * parm  end_pos @ 表示机械臂当前末端的位置和姿态（四元素）
 *       rot_dst @ 表示机械臂沿着末端坐标系x，y，z轴旋转的角度
 *
 * return pose    @ 表示机械臂末端沿着相对坐标系旋转的最后坐标
 */
pose RelatRot(pose end_pos, roz rot_dst);

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
 *function ConunLine
 *检测直线是否成立 计算机械臂末端直线规划的规模点数
 *parm  s_point @      起始世界坐标
 *      e_point @      终点世界坐标
 *      arm_ori @      末端姿态
 *      precis  @      步长（默认 0.01*precis）一般为1
 *      angle   @      点之间角度检测误差（弧度表示）
 *    arm_state @    机械臂末端当前的姿态
 *
 *      return     是否有满足条件的解
 *             -1     无解
 *             》2    解中有多少个点
 */
int CountLine(pos s_point, pos e_point, ori arm_ori, double precis, double angle, const double arm_state[6]);

/*
 *function ConunLine
 * 计算机械臂末端直线规划
 *parm  s_point @      起始世界坐标
 *      e_point @      终点世界坐标
 *      arm_ori @      末端姿态,
 *      precis  @      步长（默认 0.01*precis）
 *      angle   @      点之间角度检测误差（弧度表示）
 *    arm_state @    机械臂末端当前的姿态
 *
 *      return -1    没有解
 *             >2    表示解arm_slov_array的大小
 */
int CountLineToBuffer(pos s_point, pos e_point, ori arm_ori, double precis, double angle, const double arm_state[6], double * slov_array);

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
