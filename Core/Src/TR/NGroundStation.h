#ifndef __NGroundStation_H
#define __NGroundStation_H

#include "../base_head.h"

#define BYTE0(dwTemp)  (*((char *)(&dwTemp)))
#define BYTE1(dwTemp)  (*((char *)(&dwTemp)+1))
#define BYTE2(dwTemp)  (*((char *)(&dwTemp)+2))
#define BYTE3(dwTemp)  (*((char *)(&dwTemp)+3))
	
#define NCLINK_STATUS         0x01
#define NCLINK_SENSER         0x02
#define NCLINK_RCDATA         0x03
#define NCLINK_GPS 		      0x04
#define NCLINK_OBS_NE         0x05
#define NCLINK_OBS_UOP        0x06
#define NCLINK_FUS_U          0x07
#define NCLINK_FUS_NE         0x08
#define NCLINK_USER           0x09
#define NCLINK_SEND_CAL_RAW1  0x11
#define NCLINK_SEND_CAL_RAW2  0x12
#define NCLINK_SEND_CAL_PARA1 0x13
#define NCLINK_SEND_CAL_PARA2 0x14
#define NCLINK_SEND_CAL_PARA3 0x15


#define NCLINK_SEND_PID1_3    0x0A
#define NCLINK_SEND_PID4_6    0x0B
#define NCLINK_SEND_PID7_9    0x0C
#define NCLINK_SEND_PID10_12  0x0D
#define NCLINK_SEND_PID13_15  0x0E
#define NCLINK_SEND_PID16_18  0x0F
#define NCLINK_SEND_PARA      0x10
#define NCLINK_SEND_RC        0x11
#define NCLINK_SEND_DIS       0x12
#define NCLINK_SEND_CAL       0x13
#define NCLINK_SEND_CAL_READ  0x14

#define NCLINK_SEND_CHECK     0xF0

void Float2Byte(float *FloatValue, unsigned char *Byte, unsigned char Subscript);
void Byte2Float(unsigned char *Byte, unsigned char Subscript, float *FloatValue);

void NCLink_Send_Status(float roll,float pitch,float yaw,
											  float roll_gyro,float pitch_gyro,float yaw_gyro,
												float imu_temp,float vbat,uint8_t fly_model,uint8_t armed);
												
void NCLink_Send_Senser(int16_t a_x,int16_t a_y,int16_t a_z,
																 int16_t g_x,int16_t g_y,int16_t g_z,
															   int16_t m_x,int16_t m_y,int16_t m_z);

void NCLink_Send_Userdata(float userdata1	 ,float userdata2,
											    float userdata3  ,float userdata4,
											    float userdata5  ,float userdata6);


void NCLink_Send_RCData(uint16_t ch1,uint16_t ch2,uint16_t ch3,uint16_t ch4,
	                      uint16_t ch5,uint16_t ch6,uint16_t ch7,uint16_t ch8);
												
void NCLink_Data_Prase_Prepare(uint8_t data);//����վ���ݽ���
void NCLink_Data_Prase_Process(uint8_t *data_buf,uint8_t num);//�ɿ����ݽ�������	

uint8_t NCLink_Send_Check_Status_Parameter(void);


void NCLink_Send_Check(uint8_t response);//����վӦ��У��


void NGS_send_parama(void);

#endif
