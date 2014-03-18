#include "robotinitform.h"
#include "ui_robotinitform.h"
#include  "QString"
#include  "QDebug"
#include  "math.h"

#define ANGLE_PRECISION 1*M_PI/180.8
#define POS_MOVESTEP_PERCISION 0.005

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
    connect(ui->Ax3DButton,SIGNAL(pressed()), this,SLOT(PressButton()));
    connect(ui->Ax3UButton,SIGNAL(pressed()),this, SLOT(PressButton()));
    connect(ui->Ax4DButton,SIGNAL(pressed()), this,SLOT(PressButton()));
    connect(ui->Ax4UButton,SIGNAL(pressed()),this, SLOT(PressButton()));
    connect(ui->Ax5DButton,SIGNAL(pressed()), this,SLOT(PressButton()));
    connect(ui->Ax5UButton,SIGNAL(pressed()),this, SLOT(PressButton()));
    connect(ui->Ax6DButton,SIGNAL(pressed()), this,SLOT(PressButton()));
    connect(ui->Ax6UButton,SIGNAL(pressed()),this, SLOT(PressButton()));
    connect(ui->DPosButton,SIGNAL(pressed()),this, SLOT(PressButton()));
    connect(ui->UPosButton,SIGNAL(pressed()),this, SLOT(PressButton()));
    connect(ui->FPosButton,SIGNAL(pressed()),this, SLOT(PressButton()));
    connect(ui->BPosButton,SIGNAL(pressed()),this, SLOT(PressButton()));
    connect(ui->RPosButton,SIGNAL(pressed()),this, SLOT(PressButton()));
    connect(ui->LPosButton,SIGNAL(pressed()),this, SLOT(PressButton()));
    connect(ui->XOriUButton,SIGNAL(pressed()),this, SLOT(PressButton()));
    connect(ui->XOriDButton,SIGNAL(pressed()),this, SLOT(PressButton()));
    connect(ui->YOriDButton,SIGNAL(pressed()),this, SLOT(PressButton()));
    connect(ui->YOriUButton,SIGNAL(pressed()),this, SLOT(PressButton()));
    connect(ui->ZOriDButton,SIGNAL(pressed()),this, SLOT(PressButton()));
    connect(ui->ZOriUButton,SIGNAL(pressed()),this, SLOT(PressButton()));
    connect(ui->TurnZeroButton,SIGNAL(pressed()),this,SLOT(ResetRobot()));
    connect(ui->LinePathShow,SIGNAL(clicked()),this,SLOT(LinePathPlan()));
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
   // printf("asdfasdf\n");
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
            if(!arm_ob->SetArmJoState(arm_temp_pose,arm_temp_joint,0x01)){
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
            if(!arm_ob->SetArmJoState(arm_temp_pose,arm_temp_joint,0x01)){
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
            if(!arm_ob->SetArmJoState(arm_temp_pose,arm_temp_joint,0x02)){
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
            if(!arm_ob->SetArmJoState(arm_temp_pose,arm_temp_joint,0x02)){
                //TODO: 错误处理
            }
        }
        if(!PButtonTimer->isActive()){
            PButtonTimer->start();
        }
    }else if(ui->Ax3DButton->isDown()){
        arm_temp_joint[2] -= ANGLE_PRECISION;
        //检查是否超过控制范围
        if(arm_temp_joint[2] < arm_limit[4] || arm_temp_joint[2] > arm_limit[5]){
            arm_temp_joint[2] += ANGLE_PRECISION;
            return;
        }
        //进行运动学正解
        if(ArmFk(&arm_temp_pose.position,&arm_temp_pose.orientation,arm_temp_joint)==0){
            if(!arm_ob->SetArmJoState(arm_temp_pose,arm_temp_joint,0x03)){
                //TODO: 错误处理
            }
        }
        if(!PButtonTimer->isActive()){
            PButtonTimer->start();
        }
    }else if(ui->Ax3UButton->isDown()){
        arm_temp_joint[2] += ANGLE_PRECISION;
        //检查是否超过控制范围
        if(arm_temp_joint[2] < arm_limit[4] || arm_temp_joint[2] > arm_limit[5]){
            arm_temp_joint[2] -= ANGLE_PRECISION;
            return;
        }
        //进行运动学正解
        if(ArmFk(&arm_temp_pose.position,&arm_temp_pose.orientation,arm_temp_joint)==0){
            if(!arm_ob->SetArmJoState(arm_temp_pose,arm_temp_joint,0x03)){
                //TODO: 错误处理
            }
        }
        if(!PButtonTimer->isActive()){
            PButtonTimer->start();
        }
    }else if(ui->Ax4DButton->isDown()){
        arm_temp_joint[3] -= ANGLE_PRECISION;
        //检查是否超过控制范围
        if(arm_temp_joint[3] < arm_limit[6] || arm_temp_joint[3] > arm_limit[7]){
            arm_temp_joint[3] += ANGLE_PRECISION;
            return;
        }
        //进行运动学正解
        if(ArmFk(&arm_temp_pose.position,&arm_temp_pose.orientation,arm_temp_joint)==0){
            if(!arm_ob->SetArmJoState(arm_temp_pose,arm_temp_joint,0x04)){
                //TODO: 错误处理
            }
        }
        if(!PButtonTimer->isActive()){
            PButtonTimer->start();
        }
    }else if(ui->Ax4UButton->isDown()){
        arm_temp_joint[3] += ANGLE_PRECISION;
        //检查是否超过控制范围
        if(arm_temp_joint[3] < arm_limit[6] || arm_temp_joint[3] > arm_limit[7]){
            arm_temp_joint[3] -= ANGLE_PRECISION;
            return;
        }
        //进行运动学正解
        if(ArmFk(&arm_temp_pose.position,&arm_temp_pose.orientation,arm_temp_joint)==0){
            if(!arm_ob->SetArmJoState(arm_temp_pose,arm_temp_joint,0x04)){
                //TODO: 错误处理
            }
        }
        if(!PButtonTimer->isActive()){
            PButtonTimer->start();
        }
    }else if(ui->Ax5DButton->isDown()){
        arm_temp_joint[4] -= ANGLE_PRECISION;
        //检查是否超过控制范围
        if(arm_temp_joint[4] < arm_limit[8] || arm_temp_joint[4] > arm_limit[9]){
            arm_temp_joint[4] += ANGLE_PRECISION;
            return;
        }
        //进行运动学正解
        if(ArmFk(&arm_temp_pose.position,&arm_temp_pose.orientation,arm_temp_joint)==0){
            if(!arm_ob->SetArmJoState(arm_temp_pose,arm_temp_joint,0x05)){
                //TODO: 错误处理
            }
        }
        if(!PButtonTimer->isActive()){
            PButtonTimer->start();
        }
    }else if(ui->Ax5UButton->isDown()){
        arm_temp_joint[4] += ANGLE_PRECISION;
        //检查是否超过控制范围
        if(arm_temp_joint[4] < arm_limit[8] || arm_temp_joint[4] > arm_limit[9]){
            arm_temp_joint[4] -= ANGLE_PRECISION;
            return;
        }
        //进行运动学正解
        if(ArmFk(&arm_temp_pose.position,&arm_temp_pose.orientation,arm_temp_joint)==0){
            if(!arm_ob->SetArmJoState(arm_temp_pose,arm_temp_joint,0x05)){
                //TODO: 错误处理
            }
        }
        if(!PButtonTimer->isActive()){
            PButtonTimer->start();
        }
    }else if(ui->Ax6DButton->isDown() || ui->XOriUButton->isDown()){
        arm_temp_joint[5] -= ANGLE_PRECISION;
        //检查是否超过控制范围
        if(arm_temp_joint[5] < arm_limit[10] || arm_temp_joint[5] > arm_limit[11]){
            arm_temp_joint[5] += ANGLE_PRECISION;
            return;
        }
        //进行运动学正解
        if(ArmFk(&arm_temp_pose.position,&arm_temp_pose.orientation,arm_temp_joint)==0){
            if(!arm_ob->SetArmJoState(arm_temp_pose,arm_temp_joint,0x06)){
                //TODO: 错误处理
            }
        }
        if(!PButtonTimer->isActive()){
            PButtonTimer->start();
        }
    }else if(ui->Ax6UButton->isDown() || ui->XOriDButton->isDown()){
        arm_temp_joint[5] += ANGLE_PRECISION;
        //检查是否超过控制范围
        if(arm_temp_joint[5] < arm_limit[10] || arm_temp_joint[5] > arm_limit[11]){
            arm_temp_joint[5] -= ANGLE_PRECISION;
            return;
        }
        //进行运动学正解
        if(ArmFk(&arm_temp_pose.position,&arm_temp_pose.orientation,arm_temp_joint)==0){
            if(!arm_ob->SetArmJoState(arm_temp_pose,arm_temp_joint,0x06)){
                //TODO: 错误处理
            }
        }
        if(!PButtonTimer->isActive()){
            PButtonTimer->start();
        }
    }else if(ui->DPosButton->isDown()){
        double arm_solve_temp[ARM_DOF];
         bool flag = true;
        //step 1 测试按下的是相对坐标变换还是绝对坐标变换
        if(ui->EndPosRadioButton->isChecked()){
            //相对坐标移动
            pose arm_old_pose = arm_temp_pose;              //旧的位置姿态
            pose move_pos = {{0,0,-POS_MOVESTEP_PERCISION},{0,0,0,0}}; //定义一个相对变量

            arm_temp_pose = RelatMove(arm_temp_pose,move_pos.position);

            //在初始位置需要重复解算一次
            for(int i = 0; i < ARM_DOF; i++){
                if(arm_temp_joint[i]!=0){
                    flag = false;
                    break;
                }
            }

            if(flag){
                arm_temp_pose = arm_old_pose;
                arm_temp_pose.position.z = POSE_Z_MAX;
            }

            if(!ArmIk(arm_temp_pose.position,arm_temp_pose.orientation, ArmState::arm_current_state,arm_solve_temp,NULL)){
                //设置如果解算出的结果需要轴进行翻转则停止前进，默认无法解算，为了保持相对运动时轨迹的连续
                if(!PointAngTest(arm_solve_temp,ArmState::arm_current_state, 1)){
                       printf("the PointAngTest is error!\n");
                       //保持原来姿态
                       arm_temp_pose = arm_old_pose;
                       return;
                }

                //设置机械手臂状态
                if(!arm_ob->SetArmJoState(arm_temp_pose,arm_solve_temp,SET_ALL_JOINT)){
                    //TODO: 错误处理
                }

                //更新arm_temp_pose
                for(int i=0; i < ARM_DOF; i++){
                    arm_temp_joint[i] = arm_solve_temp[i];
                }
            }else{
                //保持原来姿态
                arm_temp_pose = arm_old_pose;
                return;
            }
        }else{
           //绝对坐标移动
           arm_temp_pose.position.z -= POS_MOVESTEP_PERCISION;

           //在初始位置需要重复解算一次
           for(int i = 0; i < ARM_DOF; i++){
               if(arm_temp_joint[i]!=0){
                   flag = false;
                   break;
               }
           }

           if(flag){
               arm_temp_pose.position.z = POSE_Z_MAX;
           }

           if(!ArmIk(arm_temp_pose.position,arm_temp_pose.orientation, ArmState::arm_current_state,arm_solve_temp,NULL)){
               //设置如果解算出的结果需要轴进行翻转则停止前进，默认无法解算，为了保持相对运动时轨迹的连续
               if(!PointAngTest(arm_solve_temp,ArmState::arm_current_state, 1)){
                      printf("the PointAngTest is error!\n");
                      //保持原来姿态
                      arm_temp_pose.position.z += POS_MOVESTEP_PERCISION;
                      return;
               }

               if(!arm_ob->SetArmJoState(arm_temp_pose,arm_solve_temp,SET_ALL_JOINT)){
                   //TODO: 错误处理
               }
               //更新arm_temp_pose
               for(int i=0; i < ARM_DOF; i++){
                   arm_temp_joint[i] = arm_solve_temp[i];
               }
           }else{
               //保持原来姿态
               arm_temp_pose.position.z += POS_MOVESTEP_PERCISION;
           }
        }

        if(!PButtonTimer->isActive()){
            PButtonTimer->start();
        }
    }else if(ui->UPosButton->isDown()){
        double arm_solve_temp[ARM_DOF];
        //step 1 测试按下的是相对坐标变换还是绝对坐标变换
        if(ui->EndPosRadioButton->isChecked()){

            if(arm_temp_pose.position.z > POSE_Z_MAX)      //检查是否超过量程
                return;

            //相对坐标移动
            pose arm_old_pose = arm_temp_pose;              //旧的位置姿态
            pose move_pos = {{0,0,POS_MOVESTEP_PERCISION},{0,0,0,0}}; //定义一个相对变量

            arm_temp_pose = RelatMove(arm_temp_pose,move_pos.position);

            if(arm_temp_pose.position.z > POSE_Z_MAX){
                return;
            }else  if(arm_temp_pose.position.z == POSE_Z_MAX){
                for(int i=0; i < ARM_DOF-1; i++){
                    arm_solve_temp[i] = 0;
                }
                if(!arm_ob->SetArmJoState(arm_temp_pose,arm_solve_temp,SET_ALL_JOINT)){
                    //TODO: 错误处理
                }
                //更新arm_temp_pose
                for(int i=0; i < ARM_DOF-1; i++){
                    arm_temp_joint[i] = arm_solve_temp[i];
                }
            }else if(!ArmIk(arm_temp_pose.position,arm_temp_pose.orientation, ArmState::arm_current_state,arm_solve_temp,NULL)){
                //设置如果解算出的结果需要轴进行翻转则停止前进，默认无法解算，为了保持相对运动时轨迹的连续
                if(!PointAngTest(arm_solve_temp,ArmState::arm_current_state, 1)){
                       printf("the PointAngTest is error!\n");
                       //保持原来姿态
                       arm_temp_pose = arm_old_pose;
                       return;
                }

                if(!arm_ob->SetArmJoState(arm_temp_pose,arm_solve_temp,SET_ALL_JOINT)){
                    //TODO: 错误处理
                }
                //更新arm_temp_pose
                for(int i=0; i < ARM_DOF; i++){
                    arm_temp_joint[i] = arm_solve_temp[i];
                }
            }else{
                //保持原来姿态
                arm_temp_pose = arm_old_pose;
            }

        }else{
             //绝对坐标移动
            if(arm_temp_pose.position.z <= POSE_Z_MAX){
                arm_temp_pose.position.z += POS_MOVESTEP_PERCISION;
            }else{
                return;
            }

           // 如果位置高于最高点，需要调整到最高点，解算出所有点都为0
            if(arm_temp_pose.position.z > POSE_Z_MAX){
                return;
            }else if(arm_temp_pose.position.z == POSE_Z_MAX){
                for(int i=0; i < ARM_DOF-1; i++){
                    arm_solve_temp[i] = 0;
                }
                if(!arm_ob->SetArmJoState(arm_temp_pose,arm_solve_temp,SET_ALL_JOINT)){
                    //TODO: 错误处理
                }
                //更新arm_temp_pose
                for(int i=0; i < ARM_DOF-1; i++){
                    arm_temp_joint[i] = arm_solve_temp[i];
                }
            }else if(!ArmIk(arm_temp_pose.position,arm_temp_pose.orientation, ArmState::arm_current_state,arm_solve_temp,NULL)){
                //设置如果解算出的结果需要轴进行翻转则停止前进，默认无法解算，为了保持相对运动时轨迹的连续
                if(!PointAngTest(arm_solve_temp,ArmState::arm_current_state, 1)){
                       printf("the PointAngTest is error!\n");
                       //保持原来姿态
                       arm_temp_pose.position.z -= POS_MOVESTEP_PERCISION;
                       return;
                }

                if(!arm_ob->SetArmJoState(arm_temp_pose,arm_solve_temp,SET_ALL_JOINT)){
                    //TODO: 错误处理
                }
                //更新arm_temp_pose
                for(int i=0; i < ARM_DOF; i++){
                    arm_temp_joint[i] = arm_solve_temp[i];
                }
            }else{
                //保持原来姿态
                arm_temp_pose.position.z -= POS_MOVESTEP_PERCISION;
            }
        }

        if(!PButtonTimer->isActive()){
            PButtonTimer->start();
        }
    }else if(ui->FPosButton->isDown()){
        double arm_solve_temp[ARM_DOF];
        //step 1 测试按下的是相对坐标变换还是绝对坐标变换
        if(ui->EndPosRadioButton->isChecked()){
            //相对坐标移动
            pose arm_old_pose = arm_temp_pose;              //旧的位置姿态
            pose move_pos = {{-POS_MOVESTEP_PERCISION,0,0},{0,0,0,0}}; //定义一个相对变量

            arm_temp_pose = RelatMove(arm_temp_pose,move_pos.position);

            if(!ArmIk(arm_temp_pose.position,arm_temp_pose.orientation, ArmState::arm_current_state,arm_solve_temp,NULL)){
                //设置如果解算出的结果需要轴进行翻转则停止前进，默认无法解算，为了保持相对运动时轨迹的连续
                if(!PointAngTest(arm_solve_temp,ArmState::arm_current_state, 1)){
                       printf("the PointAngTest is error!\n");
                       //保持原来姿态
                       arm_temp_pose = arm_old_pose;
                       return;
                }

                if(!arm_ob->SetArmJoState(arm_temp_pose,arm_solve_temp,SET_ALL_JOINT)){
                    //TODO: 错误处理
                }
                //更新arm_temp_pose
                for(int i=0; i < ARM_DOF; i++){
                    arm_temp_joint[i] = arm_solve_temp[i];
                }
            }else{
                //保持原来姿态
                arm_temp_pose = arm_old_pose;
            }
        }else{
            //绝对坐标移动 Y+
            arm_temp_pose.position.y -= POS_MOVESTEP_PERCISION;

            if(!ArmIk(arm_temp_pose.position,arm_temp_pose.orientation, ArmState::arm_current_state,arm_solve_temp,NULL)){
                //设置如果解算出的结果需要轴进行翻转则停止前进，默认无法解算，为了保持相对运动时轨迹的连续
                if(!PointAngTest(arm_solve_temp,ArmState::arm_current_state, 1)){
                       printf("the PointAngTest is error!\n");
                       //保持原来姿态
                       arm_temp_pose.position.y += POS_MOVESTEP_PERCISION;
                       return;
                }

                if(!arm_ob->SetArmJoState(arm_temp_pose,arm_solve_temp,SET_ALL_JOINT)){
                    //TODO: 错误处理
                }
                //更新arm_temp_pose
                for(int i=0; i < ARM_DOF; i++){
                    arm_temp_joint[i] = arm_solve_temp[i];
                }
            }else{
                //保持原来姿态
                arm_temp_pose.position.y += POS_MOVESTEP_PERCISION;
            }
        }

        if(!PButtonTimer->isActive()){
            PButtonTimer->start();
        }
    }else if(ui->BPosButton->isDown()){
        double arm_solve_temp[ARM_DOF];
        //step 1 测试按下的是相对坐标变换还是绝对坐标变换
        if(ui->EndPosRadioButton->isChecked()){
            //相对坐标移动
            pose arm_old_pose = arm_temp_pose;              //旧的位置姿态
            pose move_pos = {{POS_MOVESTEP_PERCISION,0,0},{0,0,0,0}}; //定义一个相对变量

            arm_temp_pose = RelatMove(arm_temp_pose,move_pos.position);

            if(!ArmIk(arm_temp_pose.position,arm_temp_pose.orientation, ArmState::arm_current_state,arm_solve_temp,NULL)){
                //设置如果解算出的结果需要轴进行翻转则停止前进，默认无法解算，为了保持相对运动时轨迹的连续
                if(!PointAngTest(arm_solve_temp,ArmState::arm_current_state, 1)){
                       printf("the PointAngTest is error!\n");
                       //保持原来姿态
                       arm_temp_pose = arm_old_pose;
                       return;
                }

                if(!arm_ob->SetArmJoState(arm_temp_pose,arm_solve_temp,SET_ALL_JOINT)){
                    //TODO: 错误处理
                }
                //更新arm_temp_pose
                for(int i=0; i < ARM_DOF; i++){
                    arm_temp_joint[i] = arm_solve_temp[i];
                }
            }else{
                //保持原来姿态
                arm_temp_pose = arm_old_pose;
            }
        }else{
            //绝对坐标移动 Y-
            arm_temp_pose.position.y += POS_MOVESTEP_PERCISION;

            if(!ArmIk(arm_temp_pose.position,arm_temp_pose.orientation, ArmState::arm_current_state,arm_solve_temp,NULL)){
                //设置如果解算出的结果需要轴进行翻转则停止前进，默认无法解算，为了保持相对运动时轨迹的连续
                if(!PointAngTest(arm_solve_temp,ArmState::arm_current_state, 1)){
                       printf("the PointAngTest is error!\n");
                       //保持原来姿态
                       arm_temp_pose.position.y -= POS_MOVESTEP_PERCISION;
                       return;
                }

                if(!arm_ob->SetArmJoState(arm_temp_pose,arm_solve_temp,SET_ALL_JOINT)){
                    //TODO: 错误处理
                }
                //更新arm_temp_pose
                for(int i=0; i < ARM_DOF; i++){
                    arm_temp_joint[i] = arm_solve_temp[i];
                }
            }else{
                //保持原来姿态
                arm_temp_pose.position.y -= POS_MOVESTEP_PERCISION;
            }
        }

        if(!PButtonTimer->isActive()){
            PButtonTimer->start();
        }
    }else if(ui->RPosButton->isDown()){
        double arm_solve_temp[ARM_DOF];
        //step 1 测试按下的是相对坐标变换还是绝对坐标变换
        if(ui->EndPosRadioButton->isChecked()){
            //相对坐标移动
            pose arm_old_pose = arm_temp_pose;              //旧的位置姿态
            pose move_pos = {{0,-POS_MOVESTEP_PERCISION,0},{0,0,0,0}}; //定义一个相对变量

            arm_temp_pose = RelatMove(arm_temp_pose,move_pos.position);

            if(!ArmIk(arm_temp_pose.position,arm_temp_pose.orientation, ArmState::arm_current_state,arm_solve_temp,NULL)){
                //设置如果解算出的结果需要轴进行翻转则停止前进，默认无法解算，为了保持相对运动时轨迹的连续
                if(!PointAngTest(arm_solve_temp,ArmState::arm_current_state, 1)){
                       printf("the PointAngTest is error!\n");
                       //保持原来姿态
                       arm_temp_pose = arm_old_pose;
                       return;
                }

                if(!arm_ob->SetArmJoState(arm_temp_pose,arm_solve_temp,SET_ALL_JOINT)){
                    //TODO: 错误处理
                }
                //更新arm_temp_pose
                for(int i=0; i < ARM_DOF; i++){
                    arm_temp_joint[i] = arm_solve_temp[i];
                }
            }else{
                //保持原来姿态
                arm_temp_pose = arm_old_pose;
            }
        }else{
            //绝对坐标移动 X+
            arm_temp_pose.position.x += POS_MOVESTEP_PERCISION;

            if(!ArmIk(arm_temp_pose.position,arm_temp_pose.orientation, ArmState::arm_current_state,arm_solve_temp,NULL)){
                //设置如果解算出的结果需要轴进行翻转则停止前进，默认无法解算，为了保持相对运动时轨迹的连续
                if(!PointAngTest(arm_solve_temp,ArmState::arm_current_state, 1)){
                       printf("the PointAngTest is error!\n");
                       //保持原来姿态
                       arm_temp_pose.position.x -= POS_MOVESTEP_PERCISION;
                       return;
                }

                if(!arm_ob->SetArmJoState(arm_temp_pose,arm_solve_temp,SET_ALL_JOINT)){
                    //TODO: 错误处理
                }
                //更新arm_temp_pose
                for(int i=0; i < ARM_DOF; i++){
                    arm_temp_joint[i] = arm_solve_temp[i];
                }
            }else{
                //保持原来姿态
                arm_temp_pose.position.x -= POS_MOVESTEP_PERCISION;
            }
        }

        if(!PButtonTimer->isActive()){
            PButtonTimer->start();
        }
    }else if(ui->LPosButton->isDown()){
        double arm_solve_temp[ARM_DOF];
        //step 1 测试按下的是相对坐标变换还是绝对坐标变换
        if(ui->EndPosRadioButton->isChecked()){
            //相对坐标移动
            pose arm_old_pose = arm_temp_pose;              //旧的位置姿态
            pose move_pos = {{0,POS_MOVESTEP_PERCISION,0},{0,0,0,0}}; //定义一个相对变量

            arm_temp_pose = RelatMove(arm_temp_pose,move_pos.position);

            if(!ArmIk(arm_temp_pose.position,arm_temp_pose.orientation, ArmState::arm_current_state,arm_solve_temp,NULL)){
                //设置如果解算出的结果需要轴进行翻转则停止前进，默认无法解算，为了保持相对运动时轨迹的连续
                if(!PointAngTest(arm_solve_temp,ArmState::arm_current_state, 1)){
                       printf("the PointAngTest is error!\n");
                       //保持原来姿态
                       arm_temp_pose = arm_old_pose;
                       return;
                }

                if(!arm_ob->SetArmJoState(arm_temp_pose,arm_solve_temp,SET_ALL_JOINT)){
                    //TODO: 错误处理
                }
                //更新arm_temp_pose
                for(int i=0; i < ARM_DOF; i++){
                    arm_temp_joint[i] = arm_solve_temp[i];
                }
            }else{
                //保持原来姿态
                arm_temp_pose = arm_old_pose;
            }
        }else{
            //绝对坐标移动 X-

            arm_temp_pose.position.x -= POS_MOVESTEP_PERCISION;

            if(!ArmIk(arm_temp_pose.position,arm_temp_pose.orientation, ArmState::arm_current_state,arm_solve_temp,NULL)){
                //设置如果解算出的结果需要轴进行翻转则停止前进，默认无法解算，为了保持相对运动时轨迹的连续
                if(!PointAngTest(arm_solve_temp,ArmState::arm_current_state, 1)){
                       printf("the PointAngTest is error!\n");
                       //保持原来姿态
                       arm_temp_pose.position.x += POS_MOVESTEP_PERCISION;
                       return;
                }

                if(!arm_ob->SetArmJoState(arm_temp_pose,arm_solve_temp,SET_ALL_JOINT)){
                    //TODO: 错误处理
                }
                //更新arm_temp_pose
                for(int i=0; i < ARM_DOF; i++){
                    arm_temp_joint[i] = arm_solve_temp[i];
                }
            }else{
                //保持原来姿态
                arm_temp_pose.position.x += POS_MOVESTEP_PERCISION;
            }
        }

        if(!PButtonTimer->isActive()){
            PButtonTimer->start();
        }
    }else if(ui->YOriDButton->isDown()){//前倾为Y轴正转
        double arm_solve_temp[ARM_DOF];
        //step 1 测试按下的是相对坐标变换还是绝对坐标变换
            //相对坐标移动
            pose arm_old_pose = arm_temp_pose;              //旧的位置姿态
            roz rot_pos = {0,ANGLE_PRECISION,0};
            arm_temp_pose = RelatRot(arm_temp_pose,rot_pos);

            if(!ArmIk(arm_temp_pose.position,arm_temp_pose.orientation, ArmState::arm_current_state,arm_solve_temp,NULL)){
                //设置如果解算出的结果需要轴进行翻转则停止前进，默认无法解算，为了保持相对运动时轨迹的连续
                if(!PointAngTest(arm_solve_temp,ArmState::arm_current_state, 1)){
                       printf("the PointAngTest is error!\n");
                       //保持原来姿态
                       arm_temp_pose = arm_old_pose;
                       return;
                }

            //    arm_solve_temp[5] = arm_temp_joint[5];
            //    for(int i=0x01; i < ARM_DOF; i++){ //关节转动与末端无关
                    if(!arm_ob->SetArmJoState(arm_temp_pose,arm_solve_temp,SET_ALL_JOINT)){
                        //TODO: 错误处理
                    }
             //   }
                //更新arm_temp_pose
                for(int i=0; i < ARM_DOF; i++){    //关节转动与末端无关
                    arm_temp_joint[i] = arm_solve_temp[i];
                }
            }else{
                //保持原来姿态
                arm_temp_pose = arm_old_pose;
            }

        if(!PButtonTimer->isActive()){
            PButtonTimer->start();
        }
    }else if(ui->YOriUButton->isDown()){//前倾为Y轴正转
        double arm_solve_temp[ARM_DOF];
        //step 1 测试按下的是相对坐标变换还是绝对坐标变换
            //相对坐标移动
            pose arm_old_pose = arm_temp_pose;              //旧的位置姿态
            roz rot_pos = {0,-ANGLE_PRECISION,0};
            arm_temp_pose = RelatRot(arm_temp_pose,rot_pos);

            if(!ArmIk(arm_temp_pose.position,arm_temp_pose.orientation, ArmState::arm_current_state,arm_solve_temp,NULL)){
                //设置如果解算出的结果需要轴进行翻转则停止前进，默认无法解算，为了保持相对运动时轨迹的连续
                if(!PointAngTest(arm_solve_temp,ArmState::arm_current_state, 1)){
                       printf("the PointAngTest is error!\n");
                       //保持原来姿态
                       arm_temp_pose = arm_old_pose;
                       return;
                }
            //           arm_solve_temp[5] = arm_temp_joint[5];
                if(!arm_ob->SetArmJoState(arm_temp_pose,arm_solve_temp,SET_ALL_JOINT)){
                    //TODO: 错误处理
                }

                //更新arm_temp_pose
                for(int i=0; i < ARM_DOF; i++){    //关节转动与末端无关
                    arm_temp_joint[i] = arm_solve_temp[i];
                }
            }else{
                //保持原来姿态
                arm_temp_pose = arm_old_pose;
            }

        if(!PButtonTimer->isActive()){
            PButtonTimer->start();
        }
    }else if(ui->ZOriDButton->isDown()){//左转为绕Z轴顺时针转
        double arm_solve_temp[ARM_DOF];
        //step 1 测试按下的是相对坐标变换还是绝对坐标变换
            //相对坐标移动
            pose arm_old_pose = arm_temp_pose;              //旧的位置姿态
            roz rot_pos = {0,0,ANGLE_PRECISION};
            arm_temp_pose = RelatRot(arm_temp_pose,rot_pos);

            if(!ArmIk(arm_temp_pose.position,arm_temp_pose.orientation, ArmState::arm_current_state,arm_solve_temp,NULL)){
                //设置如果解算出的结果需要轴进行翻转则停止前进，默认无法解算，为了保持相对运动时轨迹的连续
                if(!PointAngTest(arm_solve_temp,ArmState::arm_current_state, 1)){
                       printf("the PointAngTest is error!\n");
                       //保持原来姿态
                       arm_temp_pose = arm_old_pose;
                       return;
                }

                if(!arm_ob->SetArmJoState(arm_temp_pose,arm_solve_temp,SET_ALL_JOINT)){
                    //TODO: 错误处理
                }
                //更新arm_temp_pose
                for(int i=0; i < ARM_DOF; i++){    //关节转动与末端无关
                    arm_temp_joint[i] = arm_solve_temp[i];
                }
            }else{
                //保持原来姿态
                arm_temp_pose = arm_old_pose;
            }

        if(!PButtonTimer->isActive()){
            PButtonTimer->start();
        }
    }else if(ui->ZOriUButton->isDown()){//右转为绕Z轴逆时针转
        double arm_solve_temp[ARM_DOF];
        //step 1 测试按下的是相对坐标变换还是绝对坐标变换
            //相对坐标移动
            pose arm_old_pose = arm_temp_pose;              //旧的位置姿态
            roz rot_pos = {0,0,-ANGLE_PRECISION};
            arm_temp_pose = RelatRot(arm_temp_pose,rot_pos);

            if(!ArmIk(arm_temp_pose.position,arm_temp_pose.orientation, ArmState::arm_current_state,arm_solve_temp,NULL)){
                //设置如果解算出的结果需要轴进行翻转则停止前进，默认无法解算，为了保持相对运动时轨迹的连续
                if(!PointAngTest(arm_solve_temp,ArmState::arm_current_state, 1)){
                       printf("the PointAngTest is error!\n");
                       //保持原来姿态
                       arm_temp_pose = arm_old_pose;
                       return;
                }

                if(!arm_ob->SetArmJoState(arm_temp_pose,arm_solve_temp,SET_ALL_JOINT)){
                    //TODO: 错误处理
                }

                //更新arm_temp_pose
                for(int i=0; i < ARM_DOF; i++){    //关节转动与末端无关
                    arm_temp_joint[i] = arm_solve_temp[i];
                }
            }else{
                //保持原来姿态
                arm_temp_pose = arm_old_pose;
            }

        if(!PButtonTimer->isActive()){
            PButtonTimer->start();
        }
    }else{
             PButtonTimer->stop();
    }
    this->UpdateArmState();
}

void RobotInitForm::ResetRobot(){
    //step 1 设置机器人到初始状态
    arm_ob->ArmState::SetArmInitState();

    //step 2 清空对象全局变量
     this->UpdateArmState();

    //step 3 停止所有定时器
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

void RobotInitForm::LinePathPlan(){
    SendCmdTimer = new QTimer(this);
    SendCmdTimer->setInterval(SEND_POINT_STEP);

    pose test_pos;
    pose  end_pos;

    test_pos.position.x = -0.736397445202;
    test_pos.position.y = -0.0129018928856;
    test_pos.position.z = -0.0753068029881;
    test_pos.orientation.w = 0.999771595001;
    test_pos.orientation.x = -0.000220606132643;
    test_pos.orientation.y = -0.0169492401183;
    test_pos.orientation.z = 0.0130159296095;

    end_pos.position.x = -0.430622845888;
    end_pos.position.y = -0.00417962437496;
    end_pos.position.z = 0.790055513382;

    slov_size = CountLine(test_pos.position, end_pos.position, test_pos.orientation, 10, 1.0, arm_init_joint_state);
    if(slov_size == -1){
         printf("slove error!\n");
        return;
    }
    // step 3 申请空间存放解
    slov_line_ary = (double*)malloc(sizeof(double)*ARM_DOF*slov_size);
    CountLineToBuffer(test_pos.position, end_pos.position, test_pos.orientation, 10, 1.0, arm_init_joint_state, slov_line_ary);

    connect(SendCmdTimer,SIGNAL(timeout()),this,SLOT(SendPathCmd()));
    slov_count = 0;
  //  SendCmdTimer->start();
    SendCmdTimer->start();
}

void RobotInitForm::SendPathCmd(){
    if(!arm_ob->SetArmJoState(arm_temp_pose,slov_line_ary+slov_count*ARM_DOF)){
        //TODO: 错误处理
        SendCmdTimer->stop();
        return;
    }
    slov_count++;
    if(slov_count==slov_size)
        SendCmdTimer->stop();

}

RobotInitForm::~RobotInitForm()
{
    delete ui;
}
