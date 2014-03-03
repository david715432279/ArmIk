#ifndef ROBOTINITFORM_H
#define ROBOTINITFORM_H

#include <QWidget>
#include <ikfunc.h>
#include <armstate.h>
#include <QTimer>

#define PBQTIMER_STEP  150       //the qtimer step is 100ms

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
   // void PressButtonTimeout();

private:
    Ui::RobotInitForm *ui;
    ArmState *arm_ob;
    QTimer *PButtonTimer;
    double arm_temp_joint[ARM_DOF];
    pose   arm_temp_pose;
    int PButtonID;    //记录当前按下按钮的ID号
    //显示坐标信息
    void UpdateArmState();
};

#endif // ROBOTINITFORM_H
