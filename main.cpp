#include "widget.h"
#include <QApplication>
//#include <ikfunc.h>
//#include "stdio.h"
#include <robot/arm_joint.h>

int main(int argc, char *argv[])
{



    QApplication a(argc, argv);
    Widget w;
    w.show();
//double arm_slov_joint[6];
//double arm_joint_state[6] = {0,0,0,0,0,0};
   // test();

    //test_pos.position = end_pos;
  //  pos resulta,resultb;
  //  resulta = EndMoveXYZ(test_pos.orientation, test_pos.position);
  //  resultb = posMoveori(test_pos.orientation, test_pos.position);

   // printf("the resulata is x %f, y %f, z %f\n",resulta.x, resulta.y, resulta.z);
   // printf("the resulatb is x %f, y %f, z %f\n",resultb.x, resultb.y, resultb.z);

    //resultc = EndRotXYZ(test_pos.orientation, test_roz);
   // resultd = rozMove(test_pos.orientation, test_roz);

   // printf("the resulatc is w %f,x %f, y %f, z %f\n",resultc.w,resultc.x, resultc.y, resultc.z);
   // printf("the resulatd is w %f,x %f, y %f, z %f\n",resultd.w,resultd.x, resultd.y, resultd.z);


  //  arm_ik(test_pos.position, test_pos.orientation, arm_joint_state, arm_slov_joint, NULL);
  //  ArmIkCheckFk(&test_pos.position,&test_pos.orientation,arm_joint_state);
  // ArmFk(&test_pos.position,&test_pos.orientation,arm_joint_state);
   // PrintfPose(test_pos);
   // ArmIk(test_pos.position, test_pos.orientation, arm_joint_state, arm_slov_joint, NULL);
  //  PrintfJoint(arm_slov_joint);

   // printfpose(test_pos);

    return a.exec();
}
