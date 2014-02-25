#ifndef IKARMDESC_H
#define IKARMDESC_H

#define ARM_DOF 6
//limit the joint for the arm
//const float arm_limit[12] = {-2.96,2.96,-1.91,1.91,-1.91,1.91,-2.96,2.96,-1.91,1.91,-2.96,2.96};
const float arm_limit[12] = {-3.0,3.0,-2.96,2.96,-2.96,2.96,-2.96,2.96,-2.96,2.96,-2.96,2.96};

//record the joint for now

//pos表示当前末端的世界坐标，z向上
typedef struct {
    double x;
    double y;
    double z;
}pos;

//roz表示目前末端对应自身x,y,z坐标系的姿态旋转角度，同一时间只能朝着一个方向进行旋转
typedef struct {
    double x;
    double y;
    double z;
}roz;

//ori表示当前末端的四元素姿态
typedef struct {
    double w;
    double x;
    double y;
    double z;
}ori;

typedef struct {
    pos position;
    ori orientation;
}pose;
#endif // IKARMDESC_H
