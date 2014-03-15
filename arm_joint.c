/****************************************************************************
 * user/can_joint.c
 *
 *   Copyright (C) 2013 BUAA robot label. All rights reserved.
 *   Author: David Huang <davidhuang715432279@gmail.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/


#include <sys/types.h>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include <math.h>

#include <libpcan.h>
#include <common.h>
#include <robot/arm_joint.h>
#include <robot/arm_cmd.h>

//****************************************************************************
// DEFINES
#define DEFAULT_NODE "/dev/pcan1"
#define CURRENT_RELEASE "Release_20140121_n"

//****************************************************************************
// GLOBALS
HANDLE h = NULL;
const char *current_release;

/****************************************************************************
 * Name: arm_caninit(void)
 ****************************************************************************/
int arm_caninit(){

  /* Initialization of the CAN hardware is performed by logic external to
   * this test.
   */
    const char *szDeviceNode = DEFAULT_NODE;

    errno = 0;
  /* Open the CAN device for reading */
   // open the CAN port
   // please use what is appropriate
   // HW_DONGLE_SJA
   // HW_DONGLE_SJA_EPP
   // HW_ISA_SJA
   // HW_PCI
   h = LINUX_CAN_Open(szDeviceNode, O_RDWR);
   if (!h)
   {
     errno = nGetLastError();
     perror("bitratetest: CAN_Open()");
     return 0;
   }

   /*set the bitrate*/
    errno = CAN_Init(h,CAN_BAUD_1M,CAN_INIT_TYPE_ST);
    if(errno)
    {
        perror("set can speed error");
        CAN_Close(h);
        return 0;
    }

  /* Now loop the appropriate number of times, performing one loopback test
   * on each pass.
   */
       printf("can init is ok!\n");
   //return 1 is ok;
    return 1;
}

/****************************************************************************
 * Name: arm_recvack() 
 * 
 * Description:
 *   Recv the msg from the arm 
 * param:
 *   int joint (01-06) 
 *   uint8_t cmd
 *   uint8_t index
 * return: 0  OK
 *         <0 ERROR
 ****************************************************************************/

int arm_recvack(uint16_t joint, uint8_t cmd, uint8_t index){

    TPCANMsg rxmsg;

	//TODO: becareful the thread 
        memset(&rxmsg,0,sizeof(TPCANMsg));
    /* Read the RX message */
        if(CAN_Read(h,&rxmsg))
      	{
            printf("ERROR: read ack error returned 1\n");
			return -1;
      	}
        if(rxmsg.ID != (u_int32_t)(joint + (uint16_t)0x100)){
        	printf("ERROR: read ack ID error\n");
			return -1;
		}
        if(rxmsg.LEN != 0x03){
        	printf("ERROR: read ack DLC error\n");
			return -1;
		}
        if(rxmsg.DATA[0] != cmd){
        	printf("ERROR: read ack cmd error\n");
			return -1;
		}
        if(rxmsg.DATA[1] != index){
        	printf("ERROR: read ack index error\n");
			return -1;
		}
		return 0;
}

/****************************************************************************
 * Name: arm_sendmsg() 
 * 
 * Description:
 *   Send the msg to the arm 
 * param:
 *   int joint (01-06) 
 *   uint8_t cmd @the command you want send if you send CMDTYPE_RD ,the
 *   			  data_length and data must be NULL
 *   uint8_t index 
 *   uint8_t *data 
 *   int data_length
 * return: 0  OK
 *         <0 ERROR
 ****************************************************************************/

int arm_sendmsg(uint16_t joint, uint8_t cmd, uint8_t index, uint8_t *data, int data_length){

    TPCANMsg txmsg;

	if(cmd == CMDTYPE_WR){
		if(data_length > 6){
  	 		printf("arm_sendmsg error,the data_length must <= 6!\n");
        }
		// clear the txmsg
        memset(&txmsg,0,sizeof(TPCANMsg));
		// Construct the txmsg
		// 1 the STID
            txmsg.ID= joint;
            txmsg.MSGTYPE = 0;
		// 2 the DLC/4 is the data_length + 2
            txmsg.LEN   = data_length + 2;
		// 3 the data 1 Byte is the cmd
        txmsg.DATA[0] = cmd;
        txmsg.DATA[1] = index;
        memcpy(&txmsg.DATA[2], data, data_length);

        //Send the TX message 
        if (CAN_Write(h,&txmsg))
      	{
            printf("ERROR: write 1 returned 1\n");
			return -1;
      	}
		//TODO : receive the ack message
		if(arm_recvack(joint, cmd, index)==0)
			return 0;
		else{
			printf("recvack is error\n");
			return -1;
		}
	}else if(cmd == CMDTYPE_WR_NR){
		if(data_length > 6){
  	 		printf("arm_sendmsg error,the data_length must <= 6!\n");
        }
		// clear the txmsg
        memset(&txmsg,0,sizeof(TPCANMsg));
		// Construct the txmsg
		// 1 the STID
            txmsg.ID    = joint;
            txmsg.MSGTYPE = 0;
		// 2 the DLC/4 is the data_length + 2
            txmsg.LEN   = data_length + 2;
		// 3 the data 1 Byte is the cmd
        txmsg.DATA[0] = cmd;
        txmsg.DATA[1] = index;
        memcpy(&txmsg.DATA[2], data, data_length);

        //Send the TX message 
        if (CAN_Write(h,&txmsg))
      	{
            printf("ERROR: write returned 1\n");
			return -1;
      	}
		return 0;
	
	}else if(cmd == CMDTYPE_RD){
		// clear the txmsg
        memset(&txmsg,0,sizeof(TPCANMsg));
		// Construct the txmsg
		// 1 the STID
            txmsg.ID    = joint;
            txmsg.MSGTYPE = 0;
		// 2 the DLC/4 is the data_length + 2
		// 2 the DLC/4 is 03 when read cmd
            txmsg.LEN   = 0x03;
		// 3 the data 1 Byte is the cmd
        txmsg.DATA[0] = cmd;
        txmsg.DATA[1] = index;
        txmsg.DATA[2] = (uint8_t)data_length;

        //Send the TX message 
        if (CAN_Write(h,&txmsg))
      	{
            printf("ERROR: sendmsg returned 1\n");
			return -1;
      	}
		return 0;

	}else if(cmd == CMDTYPE_WR_REG){

	}else if(cmd == CMDTYPE_SCP){

	}else if(cmd == CMDTYPE_RST){

	}else{
      		printf("cmd error,the joint must in 00-05!\n");
			return -1;
	}
        	printf("unknown error\n");
			return -1;
		
}

/****************************************************************************
 * Name: arm_readmsg() 
 * 
 * Description:
 *   Read the msg from the arm 
 * param:
 *   int 		joint (the joint ID) 
 *   uint8_t 	addr
 *   int8_t		length the length want read
 *   uint8_t    return the data
 *   return: 0  OK
 *         <0 ERROR
 ****************************************************************************/

int arm_readmsg(int joint, uint8_t addr, int8_t length, uint8_t *data){

    TPCANMsg rxmsg;
    ssize_t nbytes;
	uint8_t *temp = data;
	int 	i;

    if(arm_sendmsg(joint, CMDTYPE_RD, addr, NULL, length)!=0){
		printf("send read cmd is error!\n");
		return -1;
	}

	for(; length>0; ){
	//TODO: becareful the thread 
        memset(&rxmsg,0,sizeof(TPCANMsg));
    /* Read the RX message */
        nbytes = CAN_Read(h,&rxmsg);
        if (nbytes)
      	{
            printf("ERROR: read msg error returned %d\n", nbytes);
			return -1;
      	}
    //check the CAN-ID and cmd and index is OK? if no ,return -1 ;		
        if(rxmsg.ID != (u_int32_t)(joint + (uint16_t)0x100)){
        	printf("ERROR: read msg ID error\n");
			return -1;
		}
        if(rxmsg.DATA[0] != CMDTYPE_RD){
        	printf("ERROR: read msg cmd error\n");
			return -1;
		}
        if(nbytes == 0){
        	printf("read Data :");
            for (i = 0; i < rxmsg.LEN; i++)
                printf("%x ",rxmsg.DATA[i]);
        	printf("\n");
            memcpy(temp, rxmsg.DATA+2, rxmsg.LEN-2);
            temp += (rxmsg.LEN-2);
		}
        length = length - (rxmsg.LEN - 2);
	}
	return 0;
}

/****************************************************************************
 * Name: arm_read_worddata() 
 * 
 * Description:
 *   Read the ratio of the modle
 * param:
 *   int 		joint (01-06) 
 *   uint8_t    cmd
 *
 * return: >=0 the int data you read
 *         <=0 error
 *
 ****************************************************************************/

int arm_read_worddata(int joint, uint8_t addr){
	uint8_t data[2];   
	int data_num;

	if(arm_readmsg(joint, addr, 0x02, data) != 0){
    	printf("arm_read_REDU_RADIO error\n");
		return -1;	
	}
	data_num = data[0] + data[1]*256;
	return data_num;
}

/****************************************************************************
 * Name: arm_write_worddata() 
 * 
 * Description:
 *   Read the ratio of the modle
 * param:
 *   int 		joint (01-06) 
 *   uint8_t    addr   @the addr your want to read
 *   int        num   @the int data your want to write
 *
 * return: >=0 OK
 *         <=0 error
 *
 ****************************************************************************/

int arm_write_worddata(int joint, uint8_t addr, int num){
	uint8_t data[2];   

	data[1]= (uint8_t)((num&0xff00)>>8);
	data[0]= (uint8_t)(num&0xff);

	if(arm_sendmsg(joint, CMDTYPE_WR, addr, data, 2) != 0){
    	printf("arm_read_REDU_RADIO error\n");
		return -1;	
	}
	return 0;
}

/****************************************************************************
 * Name: arm_read_REDU_RATIO() 
 * 
 * Description:
 *   Read the ratio of the modle
 * param:
 *   int 		joint (01-06) 
 * return: >=0 OK
 *         <0 ERROR
 ****************************************************************************/

int arm_read_REDU_RATIO(int joint){
	uint8_t data[2];   
	int REDU_NUM;

	if(arm_readmsg(joint, SYS_REDU_RATIO, 0x02, data) != 0){
    	printf("arm_read_REDU_RADIO error\n");
		return -1;	
	}
	REDU_NUM = data[0] + data[1]*256;
	return REDU_NUM;
}

/****************************************************************************
 * Name: arm_readpos_joint(int joint,int cmd) 
 * 
 * Description:
 *   Set the arm in to the position
 * param:
 *   int 		joint (01-06) 
 *   int        cmd   (1: the angle is angle 2:the angle is radian)
 *   uint8_t    opt   (add to let the arm_readmsg can read the limit position)
 * return: the angle or the radian for the joint
 *
 ****************************************************************************/

float arm_readpos_joint(int joint, int cmd, uint8_t opt){

	uint8_t data[4];   
	int		temp;
	float ang;
	
	if(arm_readmsg(joint, opt, 0x04, data) != 0){
    	printf("arm_read_REDU_RADIO error\n");
		return -1;	
	}

	temp = (int)(data[0] + data[1]*256 + data[2]*65536 + data[3]*16777216);
	if((joint >= 4 && joint <=6) || (joint >= 0x14 && joint <= 0x16)){
		ang = (float)((int)temp*360.00/(4*GEAR_RATIO_J60*MOT_ENC_LINES));
	}else{
		ang = (float)temp*360.00/(4*GEAR_RATIO_J80*MOT_ENC_LINES);
	}
    
	if(cmd == JOINT_RADIAN){
		ang = ang*M_PI/180.00;	
	}

	printf("the read pos is %x %x %x %x\n",data[0],data[1],data[2],data[3]);
   	printf("read the joint %d angle is %.6f \n",joint, ang);
	return ang;

}

/****************************************************************************
 * Name: arm_setpos_joint(int joint, int angle, int cmd) 
 * 
 * Description:
 *   Set the arm in to the position
 * param:
 *   int 		joint (01-06) 
 *   float 		angle (radian or angle) 
 *   int        cmd   (1: the angle is angle 2:the angle is radian)
 * return: >=0  OK
 *         <0 ERROR
 ****************************************************************************/

int arm_setpos_joint(int joint, float angle, int cmd){

	uint8_t pos[4];   
	int		units;
	float 	ang;
	
	if(cmd == JOINT_ANGLE){
		ang = angle;
	}else if(cmd == JOINT_RADIAN){
		ang = angle*180/M_PI;
	}else{
		return -1;
	}

		if((joint >= 4 && joint <=6) || (joint >= 0x14 && joint <= 0x16)){
	   		units = (int)(ang*4*GEAR_RATIO_J60*MOT_ENC_LINES/360.00);
		    pos[3]=	(uint8_t)(units>>24);
			pos[2]=	(uint8_t)((units>>16)&0xff);
			pos[1]= (uint8_t)((units&0xff00)>>8);
			pos[0]= (uint8_t)(units&0xff);
		}
		else{
	   		units = (int)(ang*4*GEAR_RATIO_J80*MOT_ENC_LINES/360.00);
		    pos[3]=	(uint8_t)(units>>24);
			pos[2]=	(uint8_t)((units>>16)&0xff);
			pos[1]= (uint8_t)((units&0xff00)>>8);
			pos[0]= (uint8_t)(units&0xff);
		}

		printf("the set pos is %x %x %x %x\n",pos[0],pos[1],pos[2],pos[3]);
  	    arm_sendmsg(joint, CMDTYPE_WR, TAG_POSITION_L, pos, 4);
		//set the tar_pos
       //	tar_pos[joint] = units;
	return 0;
}

int arm_setpos_joint_noth(int joint, float angle, int cmd){

	uint8_t pos[4];   
	int		units;
	float 	ang;
	
	if(cmd == JOINT_ANGLE){
		ang = angle;
	}else if(cmd == JOINT_RADIAN){
		ang = angle*180/M_PI;
	}else{
		return -1;
	}

		if((joint >= 4 && joint <=6) || (joint >= 0x14 && joint <= 0x16)){
	   		units = (int)(ang*4*GEAR_RATIO_J60*MOT_ENC_LINES/360.00);
		//	message("the units is %ld the num is %d",units, units>>24);
		    pos[3]=	(uint8_t)(units>>24);
			pos[2]=	(uint8_t)((units>>16)&0xff);
			pos[1]= (uint8_t)((units&0xff00)>>8);
			pos[0]= (uint8_t)(units&0xff);
		}
		else{
	   		units = (int)(ang*4*GEAR_RATIO_J80*MOT_ENC_LINES/360.00);
		//	message("the units is %ld the num is %d",units, units>>24);
		    pos[3]=	(uint8_t)(units>>24);
			pos[2]=	(uint8_t)((units>>16)&0xff);
			pos[1]= (uint8_t)((units&0xff00)>>8);
			pos[0]= (uint8_t)(units&0xff);
		}
  	    arm_sendmsg(joint, CMDTYPE_WR_NR, TAG_POSITION_L, pos, 4);
	return 0;
}

/****************************************************************************
 * Name: arm_readerror_joint(int joint) 
 * 
 * Description:
 *   read the arm error
 * param:
 *   int 		joint (01-06) 
 * return: 0 is OK
 *
 ****************************************************************************/

int arm_readerror_joint(int joint){

	uint8_t data[2];   

	if(arm_readmsg(joint, SYS_ERROR, 0x02, data) != 0){
    	printf("arm_read_SYS_ERROR error\n");
		return -1;	
	}

	printf("the arm error 0 is %x 1 is %x\n",data[0],data[1]);
	return 0;
}

/****************************************************************************
 * Name: arm_readpos_thread() 
 * 
 * Description:
 *   read the pos_thread about the pos for each joint in this moment  
 * param:
 * return: 0 is OK
 *
 ****************************************************************************/
/*
 void *arm_readcurpos_thread(void *parameter){

	uint8_t data[4];   
	int i;

	for(;;){
		pthread_mutex_lock(&rdwr_mutex);
		printf("thead is run!\n");
		for( i=1; i<6; i++){
			if(arm_readmsg(i, SYS_POSITION_L, 0x04, data) != 0){
    			message("arm_read_REDU_RADIO error\n");
				continue;	
			}
			cur_pos[i] = (int)(data[0] + data[1]*256 + data[2]*65536 + data[3]*16777216);
		}
		pthread_mutex_unlock(&rdwr_mutex);
		usleep(JOINT_READ_TIMEDLY);
	}

	//message("the read pos is %x %x %x %x\n",data[0],data[1],data[2],data[3]);
   	//message("read the joint %d angle is %.6f \n",joint, ang);
	return NULL;
}
*/
/****************************************************************************
 * Name: arm_init() 
 * 
 * Description:
 * 		init the arm device start the read thread
 * param:
 * return: 0 is OK
 *         <0 is error
 *
 ****************************************************************************/

 int arm_init(){

	 //  int status; 
	//   int i;
	     
/*
	   //step 1 init the thread
	   printf("Initializing the device\n");
	   status = pthread_mutex_init(&rdwr_mutex, NULL);
	   if (status != 0){
		   printf("ERROR pthread_mutex_init failed, status=%d\n",status);
		   return -1;
	   }
	   
	   status = pthread_attr_init(&attr);
	   if (status != 0){
		   printf("ERROR pthread_attr_init failed, status=%d\n",status);
		   return -1;
	   }

	   status = pthread_create(&readpos_th, &attr, arm_readcurpos_thread, NULL);
	   if (status != 0){
	   	   printf("ERROR pthread_create failed, status=%d\n",status);
		   return -1;
	   }

	   //step 2 set down the LIT_MAX_SPEED to each joint
		for(i=1; i<=6 ; i++){
			if(i<=3){
				arm_write_worddata(i, LIT_MAX_SPEED, 3520);
				printf("the 2-J80 LIT_MIN_SPEED is %d \n", arm_read_worddata(i, LIT_MAX_SPEED));
			}else{
				arm_write_worddata(i, LIT_MAX_SPEED, 1600);
				printf("the 2-J60 LIT_MIN_SPEED is %d \n", arm_read_worddata(i, LIT_MAX_SPEED));
			}
		}
       	//arm_action();*/
	   	return 0;
}

/****************************************************************************
 * Name: arm_action() 
 * 
 * Description:
 * 			define the arm action in the realtime_OS
 * param:
 * return: none 
 *
 ****************************************************************************/

 void arm_action(void){
	   int i;
	   int checkopt = 1;

	   while(1){
	   for(i=1;i<=6;i++){
			arm_setpos_joint(i, 0, JOINT_ANGLE);
	   }

	   //init the arm_device every joint position
    //   arm_checkposition(&checkopt);
	   for(i=1;i<=6;i++){
			arm_setpos_joint(i, 30, JOINT_ANGLE);
	   }

    //   arm_checkposition(&checkopt);
	   for(i=1;i<=6;i++){
			arm_setpos_joint(i, 0, JOINT_ANGLE);
	   }

    //   arm_checkposition(&checkopt);
	   for(i=1;i<=6;i++){
			arm_setpos_joint(i, -50, JOINT_ANGLE);
	   }
      // arm_checkposition(&checkopt);
       }
 }

/****************************************************************************
 * Name: arm_checkpositon() 
 * 
 * Description:
 * 		 check if the current and the target position is equal
 * param:
 *       bool *opt   the check is block or not
 * return: true is OK
 *         false is error
 *
 ****************************************************************************/
/*
int arm_checkposition(int *opt){

	 int result = 0;
	 int i;
	 while(result == 0 && *opt){
		result = 1;  
		//TODO: we mush fix the i<=4 to i<=6 
		for (i=1; i<=4; i++){
			if(abs(cur_pos[i]-tar_pos[i]) > 2){
				result = 0;
				break;
			}
			    
		}
		usleep(JOINT_READ_TIMEDLY);
	 }
	 return result;
 }
*/
