//int main(int argc, char** argv)

#include "ikfunc.h"
#include "stdlib.h"



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

//do the solve about the end pose
int arm_ik(pos arm_pos, ori arm_ori, double arm_state[ARM_DOF], double joint_solve[ARM_DOF], int flag){

    IKREAL_TYPE eerot[9],eetrans[3];

    unsigned int num_of_joints = 6;
    unsigned int best_solve_id = 0;
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
        fprintf(stderr,"Failed to get  ik solution\n");
        return  -1;
    }

    // for IKFast 56,61
    unsigned int num_of_solutions = (int)solutions.GetNumSolutions();

   // printf("Found %d ik solutions:\n", num_of_solutions );

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
  //  printf("found the best ik!\n");
    for(std::size_t i = 0; i < num_of_solutions; ++i) {

       const IkSolutionBase<IKREAL_TYPE>& sol = solutions.GetSolution(i);
       temp_route = 0;
       // printf("sol%d (free=%d): ", (int)i, this_sol_free_params );

        sol.GetSolution(&solvalues[0],NULL);
         //count the best solver
        for( std::size_t j = 0; j < solvalues.size(); ++j){
            if(solvalues[j] < arm_limit[j*2] || solvalues[j] > arm_limit[j*2+1]){
                break;
            }
            temp_route += abs(solvalues[j]-arm_state[j]);    //count the angle arm move
            if(j == solvalues.size()-1){
                if(arm_route == 0 || temp_route < arm_route){
                    best_solve_id = (int)i;
                    arm_route = temp_route;
                }
            }
        }
    }
    //count the best solver
    if(best_solve_id){
        const IkSolutionBase<IKREAL_TYPE>& sol = solutions.GetSolution(best_solve_id);
        sol.GetSolution(&solvalues[0],NULL);
      //  printf("sol%d: ", best_solve_id );
        for( std::size_t j = 0; j < solvalues.size(); ++j){
            joint_solve[j] = solvalues[j];
            printf("%.15f, ", solvalues[j]);
        }
          printf("\n");
          return 0;
    }
    return -1;
}

int test()
{
    IKREAL_TYPE eerot[9],eetrans[3];

#if IK_VERSION > 54
    // for IKFast 56,61
    unsigned int num_of_joints = GetNumJoints();
    unsigned int num_free_parameters = GetNumFreeParameters();
#else
    // for IKFast 54
    unsigned int num_of_joints = getNumJoints();
    unsigned int num_free_parameters = getNumFreeParameters();
#endif

#if IK_VERSION > 54
        // for IKFast 56,61
        IkSolutionList<IKREAL_TYPE> solutions;
#else
        // for IKFast 54
        std::vector<IKSolution> vsolutions;
#endif
        std::vector<IKREAL_TYPE> vfree(num_free_parameters);

        //for(std::size_t i = 0; i < vfree.size(); ++i)
        //    vfree[i] = atof(argv[13+i]);

        srand( (unsigned)time(0) ); // seed random number generator
        float min = -3.14;
        float max = 3.14;

        IKREAL_TYPE joints[num_of_joints];

        timespec start_time, end_time;
        unsigned int elapsed_time = 0;
        unsigned int sum_time = 0;

#if IK_VERSION > 54
        // for IKFast 56,61
      //  unsigned int num_of_tests = 1000000;
        unsigned int num_of_tests = 1000;
#else
        // for IKFast 54
        unsigned int num_of_tests = 100000;
#endif

       int count = 0;
        for (unsigned int i=0; i < num_of_tests; i++)
        {
            // Measure avg time for whole process
            //clock_gettime(CLOCK_REALTIME, &start_time);

            // Put random joint values into array
            for (unsigned int i=0; i<num_of_joints; i++)
            {
                float rnd = (float)rand() / (float)RAND_MAX;
                joints[i] = min + rnd * (max - min);
            }
           /*
            printf("Joint angles:  ");
            for (unsigned int i=0; i<num_of_joints; i++)
            {
                printf("%f  ", joints[i] );
            }
            printf("\n");*/

#if IK_VERSION > 54
            // for IKFast 56,61
            ComputeFk(joints, eetrans, eerot); // void return
#else
            // for IKFast 54
            fk(joints, eetrans, eerot); // void return
#endif

            // Measure avg time for IK
            clock_gettime(CLOCK_REALTIME, &start_time);
#if IK_VERSION > 54
            // for IKFast 56,61
            bool bSucces = ComputeIk(eetrans, eerot, vfree.size() > 0 ? &vfree[0] : NULL, solutions);
            if(bSucces)
               count++;
#else
            // for IKFast 54
            ik(eetrans, eerot, vfree.size() > 0 ? &vfree[0] : NULL, vsolutions);
#endif

            /*
#if IK_VERSION > 54
            // for IKFast 56,61
            unsigned int num_of_solutions = (int)solutions.GetNumSolutions();
#else
            // for IKFast 54
            unsigned int num_of_solutions = (int)vsolutions.size();
#endif
            printf("Found %d ik solutions:\n", num_of_solutions );
            */

            clock_gettime(CLOCK_REALTIME, &end_time);
            elapsed_time = (unsigned int)(end_time.tv_nsec - start_time.tv_nsec);
            sum_time += elapsed_time;
        } // endfor

        unsigned int avg_time = (unsigned int)sum_time / (unsigned int)num_of_tests;
        printf("the success is %d \n",count);
        printf("avg time: %f ms   over %d tests \n", (float)avg_time/1000.0, num_of_tests );

    return 0;
}

float SIGN(float x) {
    return (x >= 0.0f) ? +1.0f : -1.0f;
}

float NORM(float a, float b, float c, float d) {
    return sqrt(a * a + b * b + c * c + d * d);
}
