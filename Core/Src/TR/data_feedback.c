#include "data_feedback.h"
	/*
float vcan[10];//山外上位机
int data_counter =0;                                ///altitude_PID
void ANO_SendData()                                 ///speed_z_PID 
{  																							    ///accel_z_PID 
	if(UAV_System.UAV_Send_data_flag==1)
	{
		if(FS_remote.remote_vr_left>0)
	 {
		 vcan[0]=  UAV_control.Height_Target ;
		 vcan[1]=  UAV_Height.Height_now    ;
		 vcan[2]=  UAV_control.V_z_Target  ; 
		 vcan[3]=  Altitude_KF.X_state[1]  ;    
		 vcan[4]=  UAV_Height.VeticalSpeed_fil2 ; 
		 vcan[5]=  Speed_z_PID.PID_OUT ; 
		 vcan[6]=  UAV_Height.SinsAcc_Z ;  
		 vcan[7]=  u1 ; 
		 vcan_sendware((uint8_t *)&vcan, 32)     ;       
		data_counter++;                                   
	 }
	 
		if((FS_remote.remote_vr_left==0)&&(FS_remote.remote_vr_right>=1))
	 {	
		vcan[0]= 	tunnal_time_buff[0];       //
		vcan[1]= 	tunnal_time_buff[1];       //遥控信号
		vcan[2]= 	tunnal_time_buff[2];       //
		vcan[3]= 	tunnal_time_buff[3];       //
		vcan[4]= 	tunnal_time_buff[4];       //
		vcan[5]= 	tunnal_time_buff[5];       //
		vcan[6]= 	tunnal_time_buff[6];       //
		vcan[7]= 	tunnal_time_buff[7];       //
		 vcan_sendware((uint8_t *)&vcan, 32);//山外虚拟示波器
	 }
	////	if((FS_remote.remote_vr_left==0)&&(FS_remote.remote_vr_right==2))
	//// {
	////	vcan[0]= my_sensor.Angle.Pit;        //
	////	vcan[1]= my_sensor.Angle.Rol;        //姿态
	////	vcan[2]= my_sensor.Angle.Yaw;        //
	////	vcan[3]= my_sensor.Angle_gyro.Pit;   //
	////	vcan[4]= my_sensor.Angle_gyro.Rol;   //
	////	vcan[5]= my_sensor.Angle_gyro.Yaw;   //
	////	vcan[6]= 0;                          //
	////	vcan[7]= 0;                          //
	////	vcan_sendware((uint8_t *)&vcan, 32);//山外虚拟示波器
	//// }
	}
	/////UAV_System.UAV_Send_data_flag=0;
}*/


