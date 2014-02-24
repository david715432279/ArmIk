#include "widget.h"
#include <QApplication>
#include "ikfunc.h"
#include "stdio.h"
int main(int argc, char *argv[])
{
   // QApplication a(argc, argv);
  //  Widget w;
   // w.show();
double arm_slov_joint[6];
double arm_joint_state[6] = {0,0,0,0,0,0};
   // test();
    pose test_pos;
    roz test_roz;
    test_roz.x = 0;
    test_roz.y = 0.8;
    test_roz.z = 0;
    ori resultc,resultd;

    test_pos.position.x = -0.001;
    test_pos.position.y = -0.78;
    test_pos.position.z = -0.23;
    test_pos.orientation.w = -0.45;
    test_pos.orientation.x = -0.699;
    test_pos.orientation.y = -0.334;
    test_pos.orientation.z = -0.87;
    pos resulta,resultb;
    resulta = EndMoveXYZ(test_pos.orientation, test_pos.position);
    resultb = posMoveori(test_pos.orientation, test_pos.position);

    printf("the resulata is x %f, y %f, z %f\n",resulta.x, resulta.y, resulta.z);
    printf("the resulatb is x %f, y %f, z %f\n",resultb.x, resultb.y, resultb.z);

    resultc = EndRotXYZ(test_pos.orientation, test_roz);
    resultd = rozMove(test_pos.orientation, test_roz);

    printf("the resulatc is w %f,x %f, y %f, z %f\n",resultc.w,resultc.x, resultc.y, resultc.z);
    printf("the resulatd is w %f,x %f, y %f, z %f\n",resultd.w,resultd.x, resultd.y, resultd.z);

 //   arm_ik(test_pos.position, test_pos.orientation, arm_joint_state, arm_slov_joint, NULL);

   // return a.exec();
}
