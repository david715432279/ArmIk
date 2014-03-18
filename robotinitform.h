#ifndef ROBOTINITFORM_H
#define ROBOTINITFORM_H

#include <QWidget>
#include <ikfunc.h>
#include <armstate.h>
#include <QTimer>

#define PBQTIMER_STEP  150       //the qtimer step is 150ms
#define SEND_POINT_STEP 100      //the send cmd to robot step is 500ms

namespace Ui {
class RobotInitForm;
}

class RobotInitForm : public QWidget
{
    Q_OBJECT

public:
    explicit RobotInitForm(QWidget *parent = 0);
    ~RobotInitForm();

public slots:
    void CheckRobotState();
    void PressButton();
    void ResetRobot();
    void LinePathPlan();
    void SendPathCmd();
   // void PressButtonTimeout();

private:
    Ui::RobotInitForm *ui;
    ArmState *arm_ob;
    QTimer *PButtonTimer;
    QTimer *SendCmdTimer;
    double arm_temp_joint[ARM_DOF];
    pose   arm_temp_pose;
    int PButtonID;    //记录当前按下按钮的ID号
    int slov_size;    //记录解的数量
    int slov_count;   //记录以发送的数量
    double * slov_line_ary;//记录直线的所有解的指针
    //显示坐标信息
    void UpdateArmState();
};

#endif // ROBOTINITFORM_H
