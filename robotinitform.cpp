#include "robotinitform.h"
#include "ui_robotinitform.h"
#include  "QString"
#include  "QDebug"
#include  "math.h"

#define ANGLE_PRECISION 1*M_PI/180.0
#define POS_MOVESTEP_PERCISION 0.001

RobotInitForm::RobotInitForm(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::RobotInitForm)
{
    ui->setupUi(this);
    //初始化定时器
    PButtonTimer = new QTimer(this);
    PButtonTimer->setInterval(PBQTIMER_STEP);
    connect(PButtonTimer, SIGNAL(timeout()), this, SLOT(PressButton()));
    connect(ui->Ax1DButton,SIGNAL(pressed()), this,SLOT(PressButton()));
    connect(ui->Ax1UButton,SIGNAL(pressed()),this, SLOT(PressButton()));
    connect(ui->Ax2DButton,SIGNAL(pressed()), this,SLOT(PressButton()));
    connect(ui->Ax2UButton,SIGNAL(pressed()),this, SLOT(PressButton()));
    this->hide();

}

void RobotInitForm::CheckRobotState(){
    //step 0 打开界面
    this->show();

 //   connect(timer, SIGNAL(timeout()), this, SLOT(update()));
  //      timer->start(1000);
    //初始化机械手臂
    arm_ob = new ArmState();
    if(!arm_ob->ArmInit()){
        //机械手臂初始化失败
        return;
    }
    //显示机械手臂关节状态
    this->UpdateArmState();


}

void RobotInitForm::UpdateArmState(){
    //update the temp data
    for(int i = 0; i < ARM_DOF; i++)
        arm_temp_joint[i] = ArmState::arm_current_state[i];
    arm_temp_pose = ArmState::arm_pose;
    //show the Arm Data
    ui->WOriLineEdit->setText(QString::number(ArmState::arm_pose.orientation.w,'f',ORI_PRECISION));
    ui->XOriLineEdit->setText(QString::number(ArmState::arm_pose.orientation.x,'f',ORI_PRECISION));
    ui->YOriLineEdit->setText(QString::number(ArmState::arm_pose.orientation.y,'f',ORI_PRECISION));
    ui->ZOriLineEdit->setText(QString::number(ArmState::arm_pose.orientation.z,'f',ORI_PRECISION));
    ui->XPosLineEdit->setText(QString::number(ArmState::arm_pose.position.x,'f',POS_PRECISION));
    ui->YPosLineEdit->setText(QString::number(ArmState::arm_pose.position.y,'f',POS_PRECISION));
    ui->ZPosLineEdit->setText(QString::number(ArmState::arm_pose.position.z,'f',POS_PRECISION));
    ui->Ax1LineEdit->setText(QString::number(ArmState::arm_current_state[0]*180/M_PI,'f',JOINT_PRECISION));
    ui->Ax2LineEdit->setText(QString::number(ArmState::arm_current_state[1]*180/M_PI,'f',JOINT_PRECISION));
    ui->Ax3LineEdit->setText(QString::number(ArmState::arm_current_state[2]*180/M_PI,'f',JOINT_PRECISION));
    ui->Ax4LineEdit->setText(QString::number(ArmState::arm_current_state[3]*180/M_PI,'f',JOINT_PRECISION));
    ui->Ax5LineEdit->setText(QString::number(ArmState::arm_current_state[4]*180/M_PI,'f',JOINT_PRECISION));
    ui->Ax6LineEdit->setText(QString::number(ArmState::arm_current_state[5]*180/M_PI,'f',JOINT_PRECISION));
   //Check the Button State
   //检查是否所以按钮都可以使用，只留下可以进行操作的按钮

   //  ArmIk(arm_pose.position, arm_pose.orientation, arm_current_state, arm_solve_state, NULL);
    //step 2 读取机器人状态，设置相应的按钮参数
}

/*     PB                        ID
 * Ax1DButton                     1
 * Ax1UButton                     2
 * Ax2DButton                     3
 * Ax2UButton                     4
 * Ax3DButton                     5
 * Ax3UButton                     6
 * Ax4DButton                     7
 * Ax4UButton                     8
 * Ax5DButton                     9
 * Ax5UButton                    10
 * Ax6DButton                    11
 * Ax6UButton                    12
 * FPosButton                    13
 * BPosButton                    14
 * LPosButton                    15
 * RPosButton                    16
 * UPosButton                    17
 * DPosButton                    18
 */
void RobotInitForm::PressButton(){
    if(ui->Ax1DButton->isDown()){
        arm_temp_joint[0] -= ANGLE_PRECISION;
        //检查是否超过控制范围
        if(arm_temp_joint[0] < arm_limit[0] || arm_temp_joint[0] > arm_limit[1]){
            arm_temp_joint[0] += ANGLE_PRECISION;
            return;
        }
        //进行运动学正解
        if(ArmFk(&arm_temp_pose.position,&arm_temp_pose.orientation,arm_temp_joint)==0){
            if(arm_ob->SetArmJoState(arm_temp_pose,arm_temp_joint)){
                //TODO: 错误处理
            }
        }
        if(!PButtonTimer->isActive()){
            PButtonTimer->start();
        }
    }else if(ui->Ax1UButton->isDown()){
        arm_temp_joint[0] += ANGLE_PRECISION;
        //检查是否超过控制范围
        if(arm_temp_joint[0] < arm_limit[0] || arm_temp_joint[0] > arm_limit[1]){
            arm_temp_joint[0] -= ANGLE_PRECISION;
            return;
        }
        //进行运动学正解
        if(ArmFk(&arm_temp_pose.position,&arm_temp_pose.orientation,arm_temp_joint)==0){
            if(arm_ob->SetArmJoState(arm_temp_pose,arm_temp_joint)){
                //TODO: 错误处理
            }
        }
        if(!PButtonTimer->isActive()){
            PButtonTimer->start();
        }
    }else if(ui->Ax2DButton->isDown()){
        arm_temp_joint[1] -= ANGLE_PRECISION;
        //检查是否超过控制范围
        if(arm_temp_joint[1] < arm_limit[2] || arm_temp_joint[1] > arm_limit[3]){
            arm_temp_joint[1] += ANGLE_PRECISION;
            return;
        }
        //进行运动学正解
        if(ArmFk(&arm_temp_pose.position,&arm_temp_pose.orientation,arm_temp_joint)==0){
            if(arm_ob->SetArmJoState(arm_temp_pose,arm_temp_joint)){
                //TODO: 错误处理
            }
        }
        if(!PButtonTimer->isActive()){
            PButtonTimer->start();
        }
    }else if(ui->Ax2UButton->isDown()){
        arm_temp_joint[1] += ANGLE_PRECISION;
        //检查是否超过控制范围
        if(arm_temp_joint[1] < arm_limit[2] || arm_temp_joint[1] > arm_limit[3]){
            arm_temp_joint[1] -= ANGLE_PRECISION;
            return;
        }
        //进行运动学正解
        if(ArmFk(&arm_temp_pose.position,&arm_temp_pose.orientation,arm_temp_joint)==0){
            if(arm_ob->SetArmJoState(arm_temp_pose,arm_temp_joint)){
                //TODO: 错误处理
            }
        }
        if(!PButtonTimer->isActive()){
            PButtonTimer->start();
        }
    }
    else{
             PButtonTimer->stop();
    }


    this->UpdateArmState();

}
/*
void RobotInitForm::PressButtonTimeout(){
    if(ui->Ax1DButton->isDown()){
        arm_temp_joint[0] -= STEP_PRECISION;
        //检查是否超过控制范围
        if(arm_temp_joint[0] < arm_limit[0] || arm_temp_joint[1] > arm_limit[1]){
            return;
        }
        //进行运动学正解
        if(ArmFk(&arm_temp_pose.position,&arm_temp_pose.orientation,arm_temp_joint)==0){
            if(arm_ob->SetArmJoState(arm_temp_pose,arm_temp_joint)){
                //TODO: 错误处理
            }
        }
    }
}*/

RobotInitForm::~RobotInitForm()
{
    delete ui;
}
