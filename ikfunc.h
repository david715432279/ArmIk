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

float SIGN(float x);
float NORM(float a, float b, float c, float d);
int test();
/*
 * fuction solve the joint
 * parm arm_pos @      arm postion for destination  x,y,z
 *      arm_ori @      arm orientation for destination x,y,z,w
 *      arm_state @    arm state now
 *      joint_solve @  return the joint if ik is ok
 *      flag @         for other function
 */
int arm_ik(pos arm_pos, ori arm_ori, double arm_state[6], double joint_solve[6], int flag = NULL);

/*
 * fuction solve the joint
 * parm arm_pos @      arm postion for destination  x,y,z
 *      arm_ori @      arm orientation for destination x,y,z,w
 *      arm_state @    arm state now
 *      joint_solve @  return the joint if ik is ok
 *      flag @         for other function
 */
//move the end point alone the axis x,y,z related to the world
//mov_dst表示相对末端点x,y,z坐标轴移动的距离
//end_ori表示末端的姿态四元素
//return pos表示沿着世界坐标系x,y,z轴移动的距离
pos EndMoveXYZ(ori end_ori, pos mov_dst);
pos posMoveori(ori endOri, pos posB);

ori EndRotXYZ(ori end_ori, roz  rel_ori);
ori rozMove(ori endOri, roz rel_ori);





#endif // IKFUNC_H
