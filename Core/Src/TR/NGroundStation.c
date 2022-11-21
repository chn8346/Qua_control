#include "NGroundStation.h"
#include "usart_plus.h"

extern unsigned char LCD_Message_flag;
static uint8_t NCLink_Head[2]={0xFF,0xFC};//����֡ͷ
static uint8_t NCLink_End[2] ={0xA1,0xA2};//����֡β
uint8_t NCLink_Send_Ask_Flag[10]={0};//�ɿؽ��ջ�ȡ�����������󣬸�����վ���ͱ�־λ
uint8_t NCLink_Send_Check_Flag[20]={0};//���ݽ����ɹ����ɿظ�����վ���ͱ�־λ
uint8_t nclink_databuf[100];//���������ݻ�����
unsigned char Sort_PID_Flag;
unsigned char cal_flag,cal_step,cal_cmd ;
extern float horizon_roll,horizon_pitch;
extern unsigned char	horizon_calibration_start,horizon_calibration_finish;


#if(using_DMA_UART1==1)
void Serial_Data_Send(unsigned char  *str,uint32_t strlen )
{
	//Usart1_DMA_Write_buf( str,strlen);
}
#else
void Serial_Data_Send_(unsigned char  *str,uint32_t strlen)
{
	//Usart_SendNByte( USART1	, str,  strlen );
}
#endif

void Pilot_Status_Tick()
{


}


/***************************************************************************************
@��������void NCLink_Send_Status(float roll,float pitch,float yaw,
											           float roll_gyro,float pitch_gyro,float yaw_gyro,
												         float imu_temp,float vbat,uint8_t fly_model,uint8_t armed)
@��ڲ�����roll:�����
			     pitch:������
           yaw:ƫ����
					 roll_gyro:roll���ٶ�
					 pitch_gyro:pitch���ٶ�
					 yaw_gyro:ƫ�����ٶ�
					 imu_temp:IMU�¶�
					 vbat:ƫ�����ٶ�
					 fly_model:����ģʽ
					 armed:����״̬
@���ڲ�������
����������������̬���¶ȡ��ɿ�״̬������վ
@���ߣ�����С��
@���ڣ�2020��01��17��
****************************************************************************************/
void NCLink_Send_Status(float roll,float pitch,float yaw,
											  float roll_gyro,float pitch_gyro,float yaw_gyro,
												float imu_temp,float vbat,uint8_t fly_model,uint8_t armed)
{
  uint8_t _cnt=0;
  int16_t _temp;
	int32_t _temp1;
  uint8_t sum = 0;
  uint8_t i;
  nclink_databuf[_cnt++]=NCLink_Head[0];
  nclink_databuf[_cnt++]=NCLink_Head[1];
  nclink_databuf[_cnt++]=NCLINK_STATUS;
  nclink_databuf[_cnt++]=0;
  
  _temp = (int)(roll*100);
  nclink_databuf[_cnt++]=BYTE1(_temp);
  nclink_databuf[_cnt++]=BYTE0(_temp);
  _temp = (int)(pitch*100);
  nclink_databuf[_cnt++]=BYTE1(_temp);
  nclink_databuf[_cnt++]=BYTE0(_temp);
  _temp = (int)(yaw*100);
  nclink_databuf[_cnt++]=BYTE1(_temp);
  nclink_databuf[_cnt++]=BYTE0(_temp);  
  _temp1=100*roll_gyro;
	nclink_databuf[_cnt++]=BYTE3(_temp1);
  nclink_databuf[_cnt++]=BYTE2(_temp1);
  nclink_databuf[_cnt++]=BYTE1(_temp1);
  nclink_databuf[_cnt++]=BYTE0(_temp1);
	_temp1=100*pitch_gyro;
	nclink_databuf[_cnt++]=BYTE3(_temp1);
  nclink_databuf[_cnt++]=BYTE2(_temp1);
  nclink_databuf[_cnt++]=BYTE1(_temp1);
  nclink_databuf[_cnt++]=BYTE0(_temp1);
	
	_temp1=100*yaw_gyro;
	nclink_databuf[_cnt++]=BYTE3(_temp1);
  nclink_databuf[_cnt++]=BYTE2(_temp1);
  nclink_databuf[_cnt++]=BYTE1(_temp1);
  nclink_databuf[_cnt++]=BYTE0(_temp1);
	
  _temp = (int16_t)(100*imu_temp);//��λ��
  nclink_databuf[_cnt++]=BYTE1(_temp);
  nclink_databuf[_cnt++]=BYTE0(_temp);	

  _temp = (int16_t)(100*vbat);//��λV
  nclink_databuf[_cnt++]=BYTE1(_temp);
  nclink_databuf[_cnt++]=BYTE0(_temp);
	
	
  nclink_databuf[_cnt++]=fly_model;//����ģʽ
  nclink_databuf[_cnt++]=armed;//����0������1
  nclink_databuf[3] = _cnt-4;
  for(i=0;i<_cnt;i++) sum ^= nclink_databuf[i];
  nclink_databuf[_cnt++]=sum;
	nclink_databuf[_cnt++]=NCLink_End[0];
	nclink_databuf[_cnt++]=NCLink_End[1];
  Serial_Data_Send(nclink_databuf, _cnt);
}


union
{
	unsigned char floatByte[4];
	float floatValue;
}FloatUnion;

/***************************************************************************************
@��������void Float2Byte(float *FloatValue, unsigned char *Byte, unsigned char Subscript)
@��ڲ�����FloatValue:floatֵ
			     Byte:����
		       Subscript:ָ��������ڼ���Ԫ�ؿ�ʼд��
@���ڲ�������
������������float����ת��4�ֽ����ݲ�����ָ����ַ
@���ߣ�����С��
@���ڣ�2020��01��17��
****************************************************************************************/
void Float2Byte(float *FloatValue, unsigned char *Byte, unsigned char Subscript)
{
	FloatUnion.floatValue = (float)2;
	if(FloatUnion.floatByte[0] == 0)//С��ģʽ
	{
		FloatUnion.floatValue = *FloatValue;
		Byte[Subscript]     = FloatUnion.floatByte[0];
		Byte[Subscript + 1] = FloatUnion.floatByte[1];
		Byte[Subscript + 2] = FloatUnion.floatByte[2];
		Byte[Subscript + 3] = FloatUnion.floatByte[3];
	}
	else//���ģʽ
	{
		FloatUnion.floatValue = *FloatValue;
		Byte[Subscript]     = FloatUnion.floatByte[3];
		Byte[Subscript + 1] = FloatUnion.floatByte[2];
		Byte[Subscript + 2] = FloatUnion.floatByte[1];
		Byte[Subscript + 3] = FloatUnion.floatByte[0];
	}
}


/***************************************************************************************
@��������void Byte2Float(unsigned char *Byte, unsigned char Subscript, float *FloatValue)
@��ڲ�����Byte:����
			     Subscript:ָ��������ڼ���Ԫ�ؿ�ʼд��
		       FloatValue:floatֵ
@���ڲ�������
������������ָ����ַ��4�ֽ�����ת��float����
@���ߣ�����С��
@���ڣ�2020��01��17��
****************************************************************************************/
void Byte2Float(unsigned char *Byte, unsigned char Subscript, float *FloatValue)
{
	FloatUnion.floatByte[0]=Byte[Subscript];
	FloatUnion.floatByte[1]=Byte[Subscript + 1];
	FloatUnion.floatByte[2]=Byte[Subscript + 2];
	FloatUnion.floatByte[3]=Byte[Subscript + 3];
	*FloatValue=FloatUnion.floatValue;
}
/***************************************************************************************
@��������void NCLink_Send_Senser(int16_t a_x,int16_t a_y,int16_t a_z,
																 int16_t g_x,int16_t g_y,int16_t g_z,
															   int16_t m_x,int16_t m_y,int16_t m_z)
@��ڲ�����a_x:���ٶȼ�X��ԭʼ������
			     a_y:���ٶȼ�Y��ԭʼ������
           a_z:���ٶȼ�Z��ԭʼ������
					 g_x:������X��ԭʼ������
					 g_y:������Y��ԭʼ������
					 g_z:������Z��ԭʼ������
					 m_x:������X��ԭʼ������
					 m_y:������Y��ԭʼ������
					 m_z:������Z��ԭʼ������
@���ڲ�������
�������������ʹ�����ԭʼ���ݸ�����վ
@���ߣ�����С��
@���ڣ�2020��01��17��
****************************************************************************************/
void NCLink_Send_Senser(int16_t a_x,int16_t a_y,int16_t a_z,
												int16_t g_x,int16_t g_y,int16_t g_z,
												int16_t m_x,int16_t m_y,int16_t m_z)
{
  uint8_t _cnt=0;
  int16_t _temp;
  uint8_t sum = 0;
  uint8_t i=0;
  nclink_databuf[_cnt++]=NCLink_Head[0];
  nclink_databuf[_cnt++]=NCLink_Head[1];
  nclink_databuf[_cnt++]=NCLINK_SENSER;
  nclink_databuf[_cnt++]=0;
  
  _temp = a_x;
  nclink_databuf[_cnt++]=BYTE1(_temp);
  nclink_databuf[_cnt++]=BYTE0(_temp);
  _temp = a_y;
  nclink_databuf[_cnt++]=BYTE1(_temp);
  nclink_databuf[_cnt++]=BYTE0(_temp);
  _temp = a_z;
  nclink_databuf[_cnt++]=BYTE1(_temp);
  nclink_databuf[_cnt++]=BYTE0(_temp);
	
  _temp = g_x;
  nclink_databuf[_cnt++]=BYTE1(_temp);
  nclink_databuf[_cnt++]=BYTE0(_temp);
  _temp = g_y;
  nclink_databuf[_cnt++]=BYTE1(_temp);
  nclink_databuf[_cnt++]=BYTE0(_temp);
  _temp = g_z;
  nclink_databuf[_cnt++]=BYTE1(_temp);
  nclink_databuf[_cnt++]=BYTE0(_temp);
  
  _temp = m_x;
  nclink_databuf[_cnt++]=BYTE1(_temp);
  nclink_databuf[_cnt++]=BYTE0(_temp);
  _temp = m_y;
  nclink_databuf[_cnt++]=BYTE1(_temp);
  nclink_databuf[_cnt++]=BYTE0(_temp);
  _temp = m_z;
  nclink_databuf[_cnt++]=BYTE1(_temp);
  nclink_databuf[_cnt++]=BYTE0(_temp);
  
  nclink_databuf[3] = _cnt-4;
  
  for(i=0;i<_cnt;i++)sum ^= nclink_databuf[i];
  nclink_databuf[_cnt++] = sum;
	nclink_databuf[_cnt++]=NCLink_End[0];
	nclink_databuf[_cnt++]=NCLink_End[1];
  Serial_Data_Send(nclink_databuf, _cnt);
}


/***************************************************************************************
@��������NCLink_Send_Userdata(float userdata1	 ,float userdata2,
											        float userdata3  ,float userdata4,
											        float userdata5  ,float userdata6)
@��ڲ�����userdata1:�û�����1
					 userdata2:�û�����2
					 userdata3:�û�����3
					 userdata4:�û�����4
					 userdata5:�û�����5
					 userdata6:�û�����6
@���ڲ�������
���������������û����ݸ�����վ
@���ߣ�����С��
@���ڣ�2020��01��17��
****************************************************************************************/
void NCLink_Send_Userdata(float userdata1	 ,float userdata2,
											    float userdata3  ,float userdata4,
											    float userdata5  ,float userdata6)
{
  uint8_t sum=0,_cnt=0,i=0;
  nclink_databuf[_cnt++]=NCLink_Head[0];
  nclink_databuf[_cnt++]=NCLink_Head[1];
  nclink_databuf[_cnt++]=NCLINK_USER;
  nclink_databuf[_cnt++]=0;
		
	Float2Byte(&userdata1,nclink_databuf,_cnt);//4
	_cnt+=4;
	Float2Byte(&userdata2,nclink_databuf,_cnt);//8
	_cnt+=4;
	Float2Byte(&userdata3,nclink_databuf,_cnt);//12
	_cnt+=4;
	Float2Byte(&userdata4,nclink_databuf,_cnt);//16
	_cnt+=4;
	Float2Byte(&userdata5,nclink_databuf,_cnt);//20
	_cnt+=4;
	Float2Byte(&userdata6,nclink_databuf,_cnt);//24
	_cnt+=4;//28

  nclink_databuf[3] = _cnt-4;
	
  for(i=0;i<_cnt;i++) sum ^= nclink_databuf[i]; 
  nclink_databuf[_cnt++]=sum;
	
	nclink_databuf[_cnt++]=NCLink_End[0];
	nclink_databuf[_cnt++]=NCLink_End[1];
  Serial_Data_Send(nclink_databuf,_cnt);
}



/***************************************************************************************
@��������void NCLink_Send_RCData(uint16_t ch1,uint16_t ch2,uint16_t ch3,uint16_t ch4,
	                               uint16_t ch5,uint16_t ch6,uint16_t ch7,uint16_t ch8)
@��ڲ�����ch1:ң������1ͨ������
			     ch2:ң������2ͨ������
           ch3:ң������3ͨ������
					 ch4:ң������4ͨ������
					 ch5:ң������5ͨ������
					 ch6:ң������6ͨ������
					 ch7:ң������7ͨ������
					 ch8:ң������8ͨ������
@���ڲ�������
��������������ң������ͨ�����ݸ�����վ
@���ߣ�����С��
@���ڣ�2020��01��17��
****************************************************************************************/
void NCLink_Send_RCData(uint16_t ch1,uint16_t ch2,uint16_t ch3,uint16_t ch4,
	                      uint16_t ch5,uint16_t ch6,uint16_t ch7,uint16_t ch8)
{
  uint8_t _cnt=0,i=0,sum = 0;
  nclink_databuf[_cnt++]=NCLink_Head[0];
  nclink_databuf[_cnt++]=NCLink_Head[1];
  nclink_databuf[_cnt++]=NCLINK_RCDATA;
  nclink_databuf[_cnt++]=0;
  nclink_databuf[_cnt++]=BYTE1(ch1);
  nclink_databuf[_cnt++]=BYTE0(ch1);
  nclink_databuf[_cnt++]=BYTE1(ch2);
  nclink_databuf[_cnt++]=BYTE0(ch2);
  nclink_databuf[_cnt++]=BYTE1(ch3);
  nclink_databuf[_cnt++]=BYTE0(ch3);
  nclink_databuf[_cnt++]=BYTE1(ch4);
  nclink_databuf[_cnt++]=BYTE0(ch4);
  nclink_databuf[_cnt++]=BYTE1(ch5);
  nclink_databuf[_cnt++]=BYTE0(ch5);
  nclink_databuf[_cnt++]=BYTE1(ch6);
  nclink_databuf[_cnt++]=BYTE0(ch6);
  nclink_databuf[_cnt++]=BYTE1(ch7);
  nclink_databuf[_cnt++]=BYTE0(ch7);
  nclink_databuf[_cnt++]=BYTE1(ch8);
  nclink_databuf[_cnt++]=BYTE0(ch8);
  nclink_databuf[3] = _cnt-4;
  for(i=0;i<_cnt;i++) sum ^= nclink_databuf[i];
  nclink_databuf[_cnt++]=sum;
  nclink_databuf[_cnt++]=NCLink_End[0];
	nclink_databuf[_cnt++]=NCLink_End[1];
  Serial_Data_Send(nclink_databuf, _cnt);
}



