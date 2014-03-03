#include "ikfunc.h"
#include "stdlib.h"

bool PointAngTest(double a[ARM_DOF], double b[ARM_DOF], double angle){
    double temp;

    for(int i=0; i<ARM_DOF; i++){
        temp = fabs(a[i]-b[i]);
        if(temp > angle)
            return false;
     //   printf("the %d angle bet is %.6f\n",i,temp);z
    }
    return true;
}

bool CountLine(pos s_point, pos e_point, ori arm_ori, double precis, double angle, double arm_state[6]){
    double step = 0.01*precis;
    int step_count = 0;
    pos dst,temp_pose;
    double joint_sol_temp[ARM_DOF];
    double sol_old[ARM_DOF];

    bool bSuccess;

    for(int i=0;i<ARM_DOF;i++){
        joint_sol_temp[i] = arm_state[i];
        sol_old[i] = arm_state[i];
    }


   // step 1 count the dis between the point
    dst = PosDis(s_point,e_point);
        PrintfPos(dst);
    // step 2 count the length
    double length = NORM(dst.x, dst.y, dst.z, 0);
        printf("the length is %f\n",length);
    step_count = (int)(length/step);
    printf("the count is %d\n",step_count);

    for(int i=0;i<step_count-1;i++){
        temp_pose.x = s_point.x + i*dst.x/step_count;
        temp_pose.y = s_point.y + i*dst.y/step_count;
        temp_pose.z = s_point.z + i*dst.z/step_count;

        bSuccess = ArmIk(temp_pose, arm_ori, sol_old, joint_sol_temp, 0);
        if(bSuccess  == -1){
            printf("ik error at %d",i);
            PrintfPos(temp_pose);
            return false;
        }
        if(i>0 && !PointAngTest(sol_old, joint_sol_temp, angle)){
            printf("angle test error at %d",i);
            PrintfPos(temp_pose);
            return false;
        }
        for(int j=0;j<ARM_DOF;j++){
            sol_old[j] = joint_sol_temp[j];
        }
    }

    bSuccess = ArmIk(e_point, arm_ori, sol_old, joint_sol_temp, 0);
    if(bSuccess  == -1){
        printf("the end point is error!\n");
        PrintfPos(temp_pose);
        return false;
    }
     return true;
}


int ArmIkCheckFk(pos* arm_pos, ori* arm_ori,const double joint_solve[6]){

     IKREAL_TYPE eerot[9],eetrans[3];
     IkSolutionList<IKREAL_TYPE> solutions;
     IKREAL_TYPE joint_temp[6];
     float joint_miss = 0.007;

     ComputeFk(joint_solve, eetrans, eerot);
     bool bSuccess = ComputeIk(eetrans, eerot, NULL, solutions);

     if(!bSuccess){
        for(int j=-1 ; j<2; j++)
            for(int k=-1; k<2 ;k++)
                for(int l=-1; l < 2; l++)
                    for(int m=-1; m<2; m++)
                        for(int n=-1; n<2; n++){
                            joint_temp[0] = joint_solve[0] + j*joint_miss;
                            joint_temp[1] = joint_solve[1] + k*joint_miss;
                            joint_temp[2] = joint_solve[2] + l*joint_miss;
                            joint_temp[3] = joint_solve[3] + m*joint_miss;
                            joint_temp[4] = joint_solve[4] + n*joint_miss;
                            joint_temp[5] = joint_solve[5];
                            ComputeFk(joint_temp, eetrans, eerot);
                            for(int p=0;p<3;p++)
                                eetrans[p] = SectionNum(eetrans[p]);

                            for(int p=0;p<9;p++)
                                eerot[p] = SectionNum(eerot[p]);

                            bSuccess = ComputeIk(eetrans, eerot, NULL, solutions);
                            if(bSuccess){
                                goto iksuccess;
                            }
                        }

        fprintf(stderr,"Failed to get ik check fk  solution\n");
        return  -1;
     }else     //bSuccess is true
     {
iksuccess:
         // Convert rotation matrix to quaternion (Daisuke Miyazaki)
         double q0 = ( eerot[0] + eerot[4] + eerot[8] + 1.0f) / 4.0f;
         double q1 = ( eerot[0] - eerot[4] - eerot[8] + 1.0f) / 4.0f;
         double q2 = (-eerot[0] + eerot[4] - eerot[8] + 1.0f) / 4.0f;
         double q3 = (-eerot[0] - eerot[4] + eerot[8] + 1.0f) / 4.0f;
         if(q0 < 0.0f) q0 = 0.0f;
         if(q1 < 0.0f) q1 = 0.0f;
         if(q2 < 0.0f) q2 = 0.0f;
         if(q3 < 0.0f) q3 = 0.0f;
         q0 = sqrt(q0);
         q1 = sqrt(q1);
         q2 = sqrt(q2);
         q3 = sqrt(q3);
         if(q0 >= q1 && q0 >= q2 && q0 >= q3) {
             q0 *= +1.0f;
             q1 *= SIGN(eerot[7] - eerot[5]);
             q2 *= SIGN(eerot[2] - eerot[6]);
             q3 *= SIGN(eerot[3] - eerot[1]);
         } else if(q1 >= q0 && q1 >= q2 && q1 >= q3) {
             q0 *= SIGN(eerot[7] - eerot[5]);
             q1 *= +1.0f;
             q2 *= SIGN(eerot[3] + eerot[1]);
             q3 *= SIGN(eerot[2] + eerot[6]);
         } else if(q2 >= q0 && q2 >= q1 && q2 >= q3) {
             q0 *= SIGN(eerot[2] - eerot[6]);
             q1 *= SIGN(eerot[3] + eerot[1]);
             q2 *= +1.0f;
             q3 *= SIGN(eerot[7] + eerot[5]);
         } else if(q3 >= q0 && q3 >= q1 && q3 >= q2) {
             q0 *= SIGN(eerot[3] - eerot[1]);
             q1 *= SIGN(eerot[6] + eerot[2]);
             q2 *= SIGN(eerot[7] + eerot[5]);
             q3 *= +1.0f;
         } else {
             printf("Error while converting to quaternion! \n");
             return -1;
         }
         double r = NORM(q0, q1, q2, q3);
         q0 /= r;
         q1 /= r;
         q2 /= r;
         q3 /= r;

         arm_pos->x = eetrans[0];
         arm_pos->y = eetrans[1];
         arm_pos->z = eetrans[2];

         arm_ori->w = q0;
         arm_ori->x = q1;
         arm_ori->y = q2;
         arm_ori->z = q3;
         return 0;
     }
}

int ArmFk(pos* arm_pos, ori* arm_ori, double joint_solve[6]){
    IKREAL_TYPE eerot[9],eetrans[3];

  /*  IKREAL_TYPE joints[ARM_DOF];

    for (unsigned int i=0; i<ARM_DOF; i++){
        joints[i] = joint_solve[i];
    }
*/
     ComputeFk(joint_solve, eetrans, eerot);

     for(int p=0;p<3;p++)
         eetrans[p] = SectionNum(eetrans[p]);

     for(int p=0;p<9;p++)
         eerot[p] = SectionNum(eerot[p]);

     // Convert rotation matrix to quaternion (Daisuke Miyazaki)
     double q0 = ( eerot[0] + eerot[4] + eerot[8] + 1.0f) / 4.0f;
     double q1 = ( eerot[0] - eerot[4] - eerot[8] + 1.0f) / 4.0f;
     double q2 = (-eerot[0] + eerot[4] - eerot[8] + 1.0f) / 4.0f;
     double q3 = (-eerot[0] - eerot[4] + eerot[8] + 1.0f) / 4.0f;
     if(q0 < 0.0f) q0 = 0.0f;
     if(q1 < 0.0f) q1 = 0.0f;
     if(q2 < 0.0f) q2 = 0.0f;
     if(q3 < 0.0f) q3 = 0.0f;
     q0 = sqrt(q0);
     q1 = sqrt(q1);
     q2 = sqrt(q2);
     q3 = sqrt(q3);
     if(q0 >= q1 && q0 >= q2 && q0 >= q3) {
         q0 *= +1.0f;
         q1 *= SIGN(eerot[7] - eerot[5]);
         q2 *= SIGN(eerot[2] - eerot[6]);
         q3 *= SIGN(eerot[3] - eerot[1]);
     } else if(q1 >= q0 && q1 >= q2 && q1 >= q3) {
         q0 *= SIGN(eerot[7] - eerot[5]);
         q1 *= +1.0f;
         q2 *= SIGN(eerot[3] + eerot[1]);
         q3 *= SIGN(eerot[2] + eerot[6]);
     } else if(q2 >= q0 && q2 >= q1 && q2 >= q3) {
         q0 *= SIGN(eerot[2] - eerot[6]);
         q1 *= SIGN(eerot[3] + eerot[1]);
         q2 *= +1.0f;
         q3 *= SIGN(eerot[7] + eerot[5]);
     } else if(q3 >= q0 && q3 >= q1 && q3 >= q2) {
         q0 *= SIGN(eerot[3] - eerot[1]);
         q1 *= SIGN(eerot[6] + eerot[2]);
         q2 *= SIGN(eerot[7] + eerot[5]);
         q3 *= +1.0f;
     } else {
         printf("Error while converting to quaternion! \n");
         return -1;
     }
     double r = NORM(q0, q1, q2, q3);
     q0 /= r;
     q1 /= r;
     q2 /= r;
     q3 /= r;
   /*  printf("  Translation:  x: %f  y: %f  z: %f  \n", eetrans[0], eetrans[1], eetrans[2] );
     printf("  Quaternion:  %f   %f   %f   %f   \n", q0, q1, q2, q3 );
     printf("               ");
     // print quaternion with convention and +/- signs such that it can be copy-pasted into WolframAlpha.com
     printf("%f ", q0);
     if (q1 > 0) printf("+ %fi ", q1); else if (q1 < 0) printf("- %fi ", -q1); else printf("+ 0.00000i ");
     if (q2 > 0) printf("+ %fj ", q2); else if (q2 < 0) printf("- %fj ", -q2); else printf("+ 0.00000j ");
     if (q3 > 0) printf("+ %fk ", q3); else if (q3 < 0) printf("- %fk ", -q3); else printf("+ 0.00000k ");
     printf("  (alternate convention) \n");
     printf("\n\n");*/

     arm_pos->x = eetrans[0];
     arm_pos->y = eetrans[1];
     arm_pos->z = eetrans[2];

     arm_ori->w = q0;
     arm_ori->x = q1;
     arm_ori->y = q2;
     arm_ori->z = q3;
     return 0;
}



//move the end point alone the axis x,y,z related to the world
//mov_dst表示相对末端点x,y,z坐标轴移动的距离
//end_ori表示末端的姿态四元素
//return pos表示沿着世界坐标系x,y,z轴移动的距离
pos EndMoveXYZ(ori end_ori, pos mov_dst){

    pos world_frame;

    double RPY[3][3];
    double q0,q1,q2,q3;
    double rela_dst[3]={mov_dst.x,mov_dst.y,mov_dst.z};
    double world_dst[3] = {0,0,0};

    q0 = end_ori.w;
    q1 = end_ori.x;
    q2 = end_ori.y;
    q3 = end_ori.z;

    RPY[0][0] = q0*q0+q1*q1-q2*q2-q3*q3;
    RPY[0][1] = 2*(q1*q2-q0*q3);
    RPY[0][2] = 2*(q1*q3+q0*q2);
    RPY[1][0] = 2*(q1*q2+q0*q3);
    RPY[1][1]= q0*q0-q1*q1+q2*q2-q3*q3;
    RPY[1][2] = 2*(q2*q3-q0*q1);
    RPY[2][0] = 2*(q1*q3-q0*q2);
    RPY[2][1] = 2*(q2*q3+q0*q1);
    RPY[2][2] = q0*q0-q1*q1-q2*q2+q3*q3;

    for(int i=0; i < 3; i++){
        for(int j=0; j < 3; j++)
            world_dst[i]+=RPY[i][j]*rela_dst[j];
    }

    world_frame.x = world_dst[0];
    world_frame.y = world_dst[1];
    world_frame.z = world_dst[2];

    return world_frame;
}

ori EndRotXYZ(ori end_ori, roz rel_ori){
    ori roztemp = {0,0,0,0};

    if(rel_ori.x){
        roztemp.w = cos(rel_ori.x/2);
        roztemp.x = sin(rel_ori.x/2);
    }else if(rel_ori.y){
        roztemp.w = cos(rel_ori.y/2);
        roztemp.y = sin(rel_ori.y/2);
    }else if(rel_ori.z){
        roztemp.w = cos(rel_ori.z/2);
        roztemp.z = sin(rel_ori.z/2);
    }

    double q0,q1,q2,q3;
    double q[4][4];
    double RozBC[4] = {roztemp.w,roztemp.x,roztemp.y, roztemp.z};
    double RozCA[4] = {0,0,0,0};
    ori result;

    q0 = end_ori.w;
    q1 = end_ori.x;
    q2 = end_ori.y;
    q3 = end_ori.z;

    q[0][0] = q0;
    q[0][1] = -q1;
    q[0][2] = -q2;
    q[0][3] = -q3;

    q[1][0] = q1;
    q[1][1] = q0;
    q[1][2] = -q3;
    q[1][3] = q2;

    q[2][0] = q2;
    q[2][1] = q3;
    q[2][2] = q0;
    q[2][3] = -q1;

    q[3][0] = q3;
    q[3][1] = -q2;
    q[3][2] = q1;
    q[3][3] = q0;

    for(int i=0; i < 4; i++){
        for(int j=0; j < 4; j++)
            RozCA[i]+=q[i][j]*RozBC[j];
    }

    result.w = RozCA[0];
    result.x = RozCA[1];
    result.y = RozCA[2];
    result.z = RozCA[3];

    return result;
}

ori rozMove(ori endOri, roz rel_ori){
    ori roztemp = {0,0,0,0};

    if(rel_ori.x){
        roztemp.w = cos(rel_ori.x/2);
        roztemp.x = sin(rel_ori.x/2);
    }else if(rel_ori.y){
        roztemp.w = cos(rel_ori.y/2);
        roztemp.y = sin(rel_ori.y/2);
    }else if(rel_ori.z){
        roztemp.w = cos(rel_ori.z/2);
        roztemp.z = sin(rel_ori.z/2);
    }

    double q0,q1,q2,q3;
    Eigen::Matrix4d q =  Eigen::Matrix4d::Identity();
    Eigen::Vector4d PosB_C(roztemp.w, roztemp.x, roztemp.y, roztemp.z);
    Eigen::Vector4d PosC_A;
    ori result;

    q0 = endOri.w;
    q1 = endOri.x;
    q2 = endOri.y;
    q3 = endOri.z;

    q(0,0) = q0;
    q(0,1) = -q1;
    q(0,2) = -q2;
    q(0,3) = -q3;

    q(1,0) = q1;
    q(1,1) = q0;
    q(1,2) = -q3;
    q(1,3) = q2;

    q(2,0) = q2;
    q(2,1) = q3;
    q(2,2) = q0;
    q(2,3) = -q1;

    q(3,0) = q3;
    q(3,1) = -q2;
    q(3,2) = q1;
    q(3,3) = q0;

    PosC_A = q*PosB_C;

    result.w = PosC_A[0];
    result.x = PosC_A[1];
    result.y = PosC_A[2];
    result.z = PosC_A[3];

    return result;
}

pos posMoveori(ori endOri, pos posB){
    pos PosA;
    double q0,q1,q2,q3;
    Eigen::Matrix3d RPY =  Eigen::Matrix3d::Identity();
    Eigen::Vector3d PosA_B(posB.x, posB.y, posB.z);
    Eigen::Vector3d PosB_A;

    q0 = endOri.w;
    q1 = endOri.x;
    q2 = endOri.y;
    q3 = endOri.z;

    RPY(0,0) = q0*q0+q1*q1-q2*q2-q3*q3;
    RPY(0,1) = 2*(q1*q2-q0*q3);
    RPY(0,2) = 2*(q1*q3+q0*q2);
    RPY(1,0) = 2*(q1*q2+q0*q3);
    RPY(1,1) = q0*q0-q1*q1+q2*q2-q3*q3;
    RPY(1,2) = 2*(q2*q3-q0*q1);
    RPY(2,0) = 2*(q1*q3-q0*q2);
    RPY(2,1) = 2*(q2*q3+q0*q1);
    RPY(2,2) = q0*q0-q1*q1-q2*q2+q3*q3;

    PosB_A = RPY*PosA_B;
    PosA.x = PosB_A[0];
    PosA.y = PosB_A[1];
    PosA.z = PosB_A[2];

    return PosA;
}

/*int ArmIkF(pos arm_pos, ori arm_ori, const float arm_state[ARM_DOF], float joint_solve[ARM_DOF], int flag = NULL){
    return 0;
}
*/
//do the solve about the end pose
int ArmIk(pos arm_pos, ori arm_ori,const double arm_state[ARM_DOF], double joint_solve[ARM_DOF], int flag){

    IKREAL_TYPE eerot[9],eetrans[3];

    unsigned int num_of_joints = 6;
    int best_solve_id = -1;
    double arm_route = 0;
    double temp_route = 0;
    // for IKFast 56,61
    IkSolutionList<IKREAL_TYPE> solutions;

    eetrans[0] = arm_pos.x;
    eetrans[1] = arm_pos.y;
    eetrans[2] = arm_pos.z;
    // Convert input effector pose, in w x y z quaternion notation, to rotation matrix.
    // Must use doubles, else lose precision compared to directly inputting the rotation matrix.
    double qw = arm_ori.w;
    double qx = arm_ori.x;
    double qy = arm_ori.y;
    double qz = arm_ori.z;
    const double n = 1.0f/sqrt(qx*qx+qy*qy+qz*qz+qw*qw);
    qw *= n;
    qx *= n;
    qy *= n;
    qz *= n;
    eerot[0] = 1.0f - 2.0f*qy*qy - 2.0f*qz*qz;  eerot[1] = 2.0f*qx*qy - 2.0f*qz*qw;         eerot[2] = 2.0f*qx*qz + 2.0f*qy*qw;
    eerot[3] = 2.0f*qx*qy + 2.0f*qz*qw;         eerot[4] = 1.0f - 2.0f*qx*qx - 2.0f*qz*qz;  eerot[5] = 2.0f*qy*qz - 2.0f*qx*qw;
    eerot[6] = 2.0f*qx*qz - 2.0f*qy*qw;         eerot[7] = 2.0f*qy*qz + 2.0f*qx*qw;         eerot[8] = 1.0f - 2.0f*qx*qx - 2.0f*qy*qy;

    // for IKFast 56,61
    bool bSuccess = ComputeIk(eetrans, eerot, NULL, solutions);

    if( !bSuccess){
        for(int i=-1 ; i < 2; i++)
            for(int j = -1 ; j < 2; j++)
                for(int k = -1; k < 2; k++){
                    eetrans[0] = arm_pos.x + i*0.01;
                    eetrans[1] = arm_pos.y + j*0.01;
                    eetrans[2] = arm_pos.z + k*0.01;
                    bSuccess = ComputeIk(eetrans, eerot, NULL, solutions);
                    if(bSuccess){
                        i=2;
                        j=2;
                        k=2;
                    }
                }
        if(!bSuccess){
            fprintf(stderr,"Failed to get  ik solution\n");
            return  -1;
        }
    }

    // for IKFast 56,61
    unsigned int num_of_solutions = (int)solutions.GetNumSolutions();
    std::vector<IKREAL_TYPE> solvalues(num_of_joints);
    //std::vector<bool> solutions_flag(num_of_solutions);
 /*  for(std::size_t i = 0; i < num_of_solutions; ++i) {

        const IkSolutionBase<IKREAL_TYPE>& sol = solutions.GetSolution(i);
        int this_sol_free_params = (int)sol.GetFree().size();

        printf("sol%d (free=%d): ", (int)i, this_sol_free_params );
        sol.GetSolution(&solvalues[0],NULL);

        for( std::size_t j = 0; j < solvalues.size(); ++j)
            printf("%.15f, ", solvalues[j]);
            printf("\n");
     }

*/
    for(std::size_t i = 0; i < num_of_solutions; ++i) {

       const IkSolutionBase<IKREAL_TYPE>& sol = solutions.GetSolution(i);
       temp_route = 0;
       // printf("sol%d (free=%d): ", (int)i, this_sol_free_params );

       sol.GetSolution(&solvalues[0],NULL);
        //count the best solver
        for( std::size_t j = 0; j < solvalues.size(); ++j){
            if(solvalues[j] < arm_limit[j*2] || solvalues[j] > arm_limit[j*2+1]){
                //printf("the err num is %d\n",j);
                break;
            }
            temp_route += fabs(solvalues[j]-arm_state[j]);    //count the angle arm move
            if(j == solvalues.size()-1){
                if(arm_route == 0.0f || (temp_route - arm_route) < 0.0f){
                     // printf("the %d best sol id the temp_route is %f arm_route is %f",i,temp_route,arm_route);
                    best_solve_id = (int)i;
                    arm_route = temp_route;
                }
            }
        }
    }
    //count the best solver

    if(best_solve_id>=0){
        const IkSolutionBase<IKREAL_TYPE>& sol = solutions.GetSolution(best_solve_id);
        sol.GetSolution(&solvalues[0],NULL);
        printf("sol%d: ", best_solve_id );
        for( std::size_t j = 0; j < solvalues.size(); ++j){
            joint_solve[j] = solvalues[j];
            printf("%.15f, ", solvalues[j]);
        }
          printf("\n");
          return 0;
    }
    fprintf(stderr,"Failed to get  ik solution\n");
    return -1;
}

double SIGN(double x) {
    return (x >= 0.0f) ? +1.0f : -1.0f;
}

double NORM(double a, double b, double c, double d) {
    return sqrt(a * a + b * b + c * c + d * d);
}

/*
 * function printfpose
 * print the pose data
 */
void PrintfPose(pose arm_pose){
    printf("the arm pose is :\n");
    PrintfPos(arm_pose.position);
    PrintfOri(arm_pose.orientation);
    printf("the ./compute ik %.6f %.6f %.6f %.6f %.6f %.6f %.6f\n",arm_pose.position.x, arm_pose.position.y, arm_pose.position.z,arm_pose.orientation.w,arm_pose.orientation.x,arm_pose.orientation.y,arm_pose.orientation.z);
}

/*
 * function printfpos
 * print the pos data
 */
void PrintfPos(pos arm_pos){
    printf("the pose is x =%.6f y = %.6f z = %.6f\n", arm_pos.x, arm_pos.y, arm_pos.z);
}

/*
 * function printfori
 * print the ori data
 */
void PrintfOri(ori arm_ori){
    printf("the ori is w = %.6f,i =%.6f, j = %.6f, k =%.6f\n", arm_ori.w,arm_ori.x,arm_ori.y,arm_ori.z);
}

/*
 * function printfroz
 * print the roz data
 */
void PrintfRoz(roz arm_roz){
    printf("the roz is x =%.6f y = %.6f z = %.6f\n", arm_roz.x, arm_roz.y, arm_roz.z);
}


/*
 * function printfroz
 * print the roz data
 */
void PrintfJoint(double arm_joint[ARM_DOF]){
    printf("the joint is ");
    for(int i=0; i < ARM_DOF; i++){
        printf(" %.6f ",arm_joint[i]);
    }
      printf("\n");
}

double SectionNum(double num){
     return int(num*100000000+0.5f)*0.00000001f;
}

pos PosDis(pos begin,pos end){
    pos temp;
    temp.x = end.x - begin.x;
    temp.y = end.y - begin.y;
    temp.z = end.z - begin.z;
    return temp;
}
