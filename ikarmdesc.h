#ifndef IKARMDESC_H
#define IKARMDESC_H

#define ARM_DOF 6
//limit the joint for the arm
const float arm_limit[12] = {-2.96,2.96,-1.91,1.91,-1.91,1.91,-2.96,2.96,-1.91,1.91,-2.96,2.96};

//record the joint for now


typedef struct {
    double x;
    double y;
    double z;
}pos;

typedef struct {
    double x;
    double y;
    double z;
}roz;

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
