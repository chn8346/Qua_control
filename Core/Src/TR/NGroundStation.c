#include "NGroundStation.h"
#include "usart_plus.h"

extern unsigned char LCD_Message_flag;
static uint8_t NCLink_Head[2]={0xFF,0xFC};//数据帧头
static uint8_t NCLink_End[2] ={0xA1,0xA2};//数据帧尾
uint8_t NCLink_Send_Ask_Flag[10]={0};//飞控接收获取参数命令请求，给地面站发送标志位
uint8_t NCLink_Send_Check_Flag[20]={0};//数据解析成功，飞控给地面站发送标志位
uint8_t nclink_databuf[100];//待发送数据缓冲区
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
@函数名：void NCLink_Send_Status(float roll,float pitch,float yaw,
											           float roll_gyro,float pitch_gyro,float yaw_gyro,
												         float imu_temp,float vbat,uint8_t fly_model,uint8_t armed)
@入口参数：roll:横滚角
			     pitch:俯仰角
           yaw:偏航角
					 roll_gyro:roll角速度
					 pitch_gyro:pitch角速度
					 yaw_gyro:偏航角速度
					 imu_temp:IMU温度
					 vbat:偏航角速度
					 fly_model:飞行模式
					 armed:解锁状态
@出口参数：无
功能描述：发送姿态、温度、飞控状态给地面站
@作者：无名小哥
@日期：2020年01月17日
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
	
  _temp = (int16_t)(100*imu_temp);//单位℃
  nclink_databuf[_cnt++]=BYTE1(_temp);
  nclink_databuf[_cnt++]=BYTE0(_temp);	

  _temp = (int16_t)(100*vbat);//单位V
  nclink_databuf[_cnt++]=BYTE1(_temp);
  nclink_databuf[_cnt++]=BYTE0(_temp);
	
	
  nclink_databuf[_cnt++]=fly_model;//飞行模式
  nclink_databuf[_cnt++]=armed;//上锁0、解锁1
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
@函数名：void Float2Byte(float *FloatValue, unsigned char *Byte, unsigned char Subscript)
@入口参数：FloatValue:float值
			     Byte:数组
		       Subscript:指定从数组第几个元素开始写入
@出口参数：无
功能描述：将float数据转成4字节数据并存入指定地址
@作者：无名小哥
@日期：2020年01月17日
****************************************************************************************/
void Float2Byte(float *FloatValue, unsigned char *Byte, unsigned char Subscript)
{
	FloatUnion.floatValue = (float)2;
	if(FloatUnion.floatByte[0] == 0)//小端模式
	{
		FloatUnion.floatValue = *FloatValue;
		Byte[Subscript]     = FloatUnion.floatByte[0];
		Byte[Subscript + 1] = FloatUnion.floatByte[1];
		Byte[Subscript + 2] = FloatUnion.floatByte[2];
		Byte[Subscript + 3] = FloatUnion.floatByte[3];
	}
	else//大端模式
	{
		FloatUnion.floatValue = *FloatValue;
		Byte[Subscript]     = FloatUnion.floatByte[3];
		Byte[Subscript + 1] = FloatUnion.floatByte[2];
		Byte[Subscript + 2] = FloatUnion.floatByte[1];
		Byte[Subscript + 3] = FloatUnion.floatByte[0];
	}
}


/***************************************************************************************
@函数名：void Byte2Float(unsigned char *Byte, unsigned char Subscript, float *FloatValue)
@入口参数：Byte:数组
			     Subscript:指定从数组第几个元素开始写入
		       FloatValue:float值
@出口参数：无
功能描述：从指定地址将4字节数据转成float数据
@作者：无名小哥
@日期：2020年01月17日
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
@函数名：void NCLink_Send_Senser(int16_t a_x,int16_t a_y,int16_t a_z,
																 int16_t g_x,int16_t g_y,int16_t g_z,
															   int16_t m_x,int16_t m_y,int16_t m_z)
@入口参数：a_x:加速度计X轴原始数字量
			     a_y:加速度计Y轴原始数字量
           a_z:加速度计Z轴原始数字量
					 g_x:陀螺仪X轴原始数字量
					 g_y:陀螺仪Y轴原始数字量
					 g_z:陀螺仪Z轴原始数字量
					 m_x:磁力计X轴原始数字量
					 m_y:磁力计Y轴原始数字量
					 m_z:磁力计Z轴原始数字量
@出口参数：无
功能描述：发送传感器原始数据给地面站
@作者：无名小哥
@日期：2020年01月17日
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
@函数名：NCLink_Send_Userdata(float userdata1	 ,float userdata2,
											        float userdata3  ,float userdata4,
											        float userdata5  ,float userdata6)
@入口参数：userdata1:用户数据1
					 userdata2:用户数据2
					 userdata3:用户数据3
					 userdata4:用户数据4
					 userdata5:用户数据5
					 userdata6:用户数据6
@出口参数：无
功能描述：发送用户数据给地面站
@作者：无名小哥
@日期：2020年01月17日
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
@函数名：void NCLink_Send_RCData(uint16_t ch1,uint16_t ch2,uint16_t ch3,uint16_t ch4,
	                               uint16_t ch5,uint16_t ch6,uint16_t ch7,uint16_t ch8)
@入口参数：ch1:遥控器第1通道数据
			     ch2:遥控器第2通道数据
           ch3:遥控器第3通道数据
					 ch4:遥控器第4通道数据
					 ch5:遥控器第5通道数据
					 ch6:遥控器第6通道数据
					 ch7:遥控器第7通道数据
					 ch8:遥控器第8通道数据
@出口参数：无
功能描述：发送遥控器各通道数据给地面站
@作者：无名小哥
@日期：2020年01月17日
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



