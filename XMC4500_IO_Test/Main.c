/******************** (C) COPYRIGHT 2014 MiaoW Labs ***************************
**	   		| PB8 - I2C1_SCL  |
**			| PB9 - I2C1_SDA  |
**			 -----------------
**********************************************************************************/
#include <DAVE3.h>
//#include "I2C.h"
#define Control_P5_0(Mode, DriveStrength)       PORT5->IOCR0 = (PORT5->IOCR0 & ~0x000000F8) | (Mode << 3); PORT5->PDR0 = (PORT5->PDR0 & ~0x00000007) | (DriveStrength)
#define Control_P5_1(Mode, DriveStrength)       PORT5->IOCR0 = (PORT5->IOCR0 & ~0x0000F800) | (Mode << 11); PORT5->PDR0 = (PORT5->PDR0 & ~0x00000070) | (DriveStrength << 4)
#define Control_P5_2(Mode, DriveStrength)       PORT5->IOCR0 = (PORT5->IOCR0 & ~0x00F80000) | (Mode << 19); PORT5->PDR0 = (PORT5->PDR0 & ~0x00000700) | (DriveStrength << 8)
// Mode Input 0x00
#define   uchar unsigned char

#define   uint unsigned int



// 定义MPU6050内部地址

//****************************************

#define	SMPLRT_DIV		0x19	//陀螺仪采样率，典型值：0x07(125Hz)

#define	CONFIG			0x1A	//低通滤波频率，典型值：0x06(5Hz)

#define	GYRO_CONFIG		0x1B	//陀螺仪自检及测量范围，典型值：0x18(不自检，2000deg/s)

#define	ACCEL_CONFIG	0x1C	//加速计自检、测量范围及高通滤波频率，典型值：0x01(不自检，2G，5Hz)

#define	ACCEL_XOUT_H	0x3B

#define	ACCEL_XOUT_L	0x3C

#define	ACCEL_YOUT_H	0x3D

#define	ACCEL_YOUT_L	0x3E

#define	ACCEL_ZOUT_H	0x3F

#define	ACCEL_ZOUT_L	0x40

#define	TEMP_OUT_H		0x41

#define	TEMP_OUT_L		0x42



#define	GYRO_XOUT_H		0x43

#define	GYRO_XOUT_L		0x44

#define	GYRO_YOUT_H		0x45

#define	GYRO_YOUT_L		0x46

#define	GYRO_ZOUT_H		0x47

#define	GYRO_ZOUT_L		0x48



#define	PWR_MGMT_1		0x6B	//电源管理，典型值：0x00(正常启用)

#define	WHO_AM_I		0x75	//IIC地址寄存器(默认数值0x68，只读)





//****************************



#define	MPU6050_Addr   0xD0	  //定义器件在IIC总线中的从地址,根据ALT  ADDRESS地址引脚不同修改



unsigned char TX_DATA[4];  	 //显示据缓存区

unsigned char BUF[10];       //接收数据缓存区

char  test=0; 				 //IIC用到

short T_X,T_Y,T_Z,T_T;		 //X,Y,Z轴，温度


typedef enum INPUT_Type
{
  INPUT          = 0x00,
  INPUT_PD       = 0x01, // Input Pull Down Device
  INPUT_PU       = 0x02, // Input Pull Up Device
  INPUT_PPS      = 0x03,
  INPUT_INV      = 0x04,
  INPUT_INV_PD   = 0x05,
  INPUT_INV_PU   = 0x06,
  INPUT_INV_PPS  = 0x07
}INPUT_Type;
typedef enum OUTPUT_Type
{
  OUTPUT_PP_GP   = 0x10, // Push Pull, GP: General Purpose
  OUTPUT_PP_AF1  = 0x11, // Alternate Function 1
  OUTPUT_PP_AF2  = 0x12,
  OUTPUT_PP_AF3  = 0x13,
  OUTPUT_PP_AF4  = 0x14,
  OUTPUT_OD_GP   = 0x18, // Open Drain, GP: General Purpose
  OUTPUT_OD_AF1  = 0x19,
  OUTPUT_OD_AF2  = 0x1A,
  OUTPUT_OD_AF3  = 0x1B,
  OUTPUT_OD_AF4  = 0X1C
}OUTPUT_Type;
typedef enum PIN_Strength
{
  WEAK           = 0x7,
  MEDIUM         = 0x4,
  STRONG         = 0x2,
  VERYSTRONG     = 0x0
}PIN_Strength;

/*************************************/
#define  I2C_Direction_Transmitter      ((uint8_t)0x00)
#define  I2C_Direction_Receiver         ((uint8_t)0x01)

#define  XMC_Soft_IIC
static volatile bool I2C_Received = FALSE;
volatile uint16_t DataReceived = 0x0000;
volatile uint32_t Value;
static volatile uint32_t SDA_IN;
static volatile uint32_t TEMP = 0;
static uint32_t I2C_Status = (uint32_t)0;

handle_t TimerId;
handle_t TimerId_1;
volatile uint32_t TimerId_2_cnt = (uint32_t)0;
volatile uint32_t soft_cnt      = (uint32_t)0;
bool TimerExpired = FALSE;
/*************************************/
#ifdef XMC_Soft_IIC

/*
 * IO004_Handle4 : SCL
 * IO004_Handle5 : SDA
 * IO004_SetOutputValue(IO004_Handle4,1);
 * IO004_SetOutputValue(IO004_Handle5,1);
 * // End of Soft I2C Pin Initialization
 * IO004_ReadPin(IO004_Handle0);
 * */

#define SCL_H         IO004_SetOutputValue(IO004_Handle0,1) /* GPIO_SetBits(GPIOB , GPIO_Pin_10)   */
#define SCL_L         IO004_SetOutputValue(IO004_Handle0,0) /* GPIO_ResetBits(GPIOB , GPIO_Pin_10) */

#define SDA_H         IO004_SetOutputValue(IO004_Handle1,1) /* GPIO_SetBits(GPIOB , GPIO_Pin_11)   */
#define SDA_L         IO004_SetOutputValue(IO004_Handle1,0) /* GPIO_ResetBits(GPIOB , GPIO_Pin_11) */

#define SCL_read      IO004_ReadPin(IO004_Handle0) /* GPIO_ReadInputDataBit(GPIOB , GPIO_Pin_10) */
#define SDA_read      ReadSDA()//IO004_ReadPin(IO004_Handle1) /* GPIO_ReadInputDataBit(GPIOB , GPIO_Pin_11) */


//IO004_EnableOutputDriver(&IO004_Handle0,IO004_OPENDRAIN);

void delay5ms(void);

void DATA_printf(uchar *s,short temp_data)
{
    if(temp_data < 0)
    {
        temp_data = -temp_data;
        *s        = '-';
    }
    else
    {
        *s        = ' ';
    }
    *++s      = temp_data/100 + 0x30; //  取百位 加上0的ascii码
    temp_data = temp_data     % 100;      //  去除百位
    *++s      = temp_data/10  + 0x30; //  取十位 加上0的ascii码
    temp_data = temp_data     % 10;       //  去除十位
    *++s      = temp_data     + 0x30; //  取个位 加上0的ascii码
}
//void USART1_SendData(char *s)
void USART1_SendData(char *s) // 这里需要为char* 否则打印出来的数据乱码
{
	char *p;
	p=s;
	while(!UART001_WriteDataBytes(&UART001_Handle0, p, 1));
/*	while(*p != '\0')
	{
		 USART1_Send_Byte(*p);

        UART001_WriteDataBytes(&UART001_Handle0, p, 1);
		p++;
	}*/
}

static void I2C_delay(void)
{
    volatile int i = 14; // TODO： 改成12试试 原来值为7
    while (i)
        i--;
}
static uint32_t ReadSDA(void)
{
//  IO口的Strength是在IO004_Init()里面设置的
//	IO004_EnableOutputDriver(&IO004_Handle1,INPUT);
/*	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");*/
	SDA_IN = IO004_ReadPin(IO004_Handle1);
	//IO004_EnableOutputDriver(&IO004_Handle0,IO004_PUSHPULL);
	return SDA_IN;
}
static bool I2C_Start(void)
{
    SDA_H;
    SCL_H;
    I2C_delay();
    I2C_delay();
    if (!SDA_read)
        return FALSE; // 判断SDA是否被成功拉高
  //  IO004_EnableOutputDriver(&IO004_Handle1,OUTPUT_OD_GP);
    SDA_L;            // 拉低SDA，用于产生START信号
    I2C_delay();      // 让SDA保持低

    if (SDA_read)     // 读取SDA确保SDA被成功拉低
         return FALSE;
   // IO004_EnableOutputDriver(&IO004_Handle1,OUTPUT_OD_GP);
    SDA_L;            // 读取SDA后把SDA从输入变为输出,需要重新配置SDA的状态：低
    I2C_delay();      // 延时一段时间
    I2C_delay();
    return TRUE;
}

static void I2C_Stop(void)
{
    SCL_L;
    I2C_delay();
    SDA_L;
    I2C_delay();
    SCL_H;
    I2C_delay();
    SDA_H;
    I2C_delay();
}

static void I2C_Ack(void)
{
    SCL_L;
    I2C_delay();
    SDA_L;
    I2C_delay();
    SCL_H;
    I2C_delay();
    SCL_L;
    I2C_delay();
}

static void I2C_NoAck(void)
{
    SCL_L;
    I2C_delay();
    SDA_H;
    I2C_delay();
    SCL_H;
    I2C_delay();
    SCL_L;
    I2C_delay();
}

static bool I2C_WaitAck(void)
{
    SCL_L;
    I2C_delay();
    SDA_H;
    I2C_delay();
    SCL_H;
    I2C_delay();
    I2C_delay();
    I2C_delay();
    I2C_delay();
    if (SDA_read) {
        SCL_L;
        I2C_delay();
        TEMP  = SDA_IN;
       return FALSE;
    }
   // IO004_EnableOutputDriver(&IO004_Handle1,OUTPUT_OD_GP);
    SCL_L;
    //
    I2C_delay();
    return TRUE;
}

static void I2C_SendByte(uint8_t byte)
{
    uint8_t i = 8;
    while (i--) {
        SCL_L;           // 拉低SCL准备给SLAVE传送数据
        I2C_delay();
        if (byte & 0x80)
            SDA_H;
        else
            SDA_L;
        byte <<= 1;
        I2C_delay();
        SCL_H;
        I2C_delay();
    }
    SCL_L;
}

static uint8_t I2C_ReceiveByte(void)
{
    uint8_t i = 8;
    uint8_t byte = 0;

    SDA_H;
    while (i--) {
        byte <<= 1;
        SCL_L;
        I2C_delay();
        SCL_H;
        I2C_delay();
        if (SDA_read) {
            byte |= 0x01;
        }
    //    IO004_EnableOutputDriver(&IO004_Handle1,OUTPUT_OD_GP);
    }
    SCL_L;
    return byte;
}

#if 0
void i2cInit(void)
{
  GPIO_InitTypeDef gpio;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB , ENABLE);
    gpio.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
    gpio.GPIO_Speed = GPIO_Speed_2MHz;
    gpio.GPIO_Mode = GPIO_Mode_Out_OD;
    GPIO_Init(GPIOB, &gpio);
	/*
    I2C_InitTypeDef  I2C_InitStructure;
    GPIO_InitTypeDef  GPIO_InitStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1,ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);

    //PB8,9 SCL and SDA
    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_8 | GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    // I2C1_SCL on PB08, I2C1_SDA on PB09
    GPIO_PinRemapConfig(GPIO_Remap_I2C1, ENABLE);   //

    //I2C_DeInit(I2C1);
    I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
    //I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
    //I2C_InitStructure.I2C_OwnAddress1 = 0x30;
    I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
    I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    I2C_InitStructure.I2C_ClockSpeed = 100000;//100K

    I2C_Cmd(I2C1, ENABLE);
    I2C_Init(I2C1, &I2C_InitStructure);
    //
    I2C_AcknowledgeConfig(I2C1, ENABLE);   */
}
#endif
bool i2cWriteBuffer(uint8_t addr, uint8_t reg, uint8_t len, uint8_t * data)
{
    int i;
    if (!I2C_Start())
    {
        return FALSE;
    }
    I2C_SendByte(addr << 1 | I2C_Direction_Transmitter);//I2C_Direction_Transmitter:д
    if (!I2C_WaitAck()) {
        I2C_Stop();
        return FALSE;
    }
    I2C_SendByte(reg);
    I2C_WaitAck();
    for (i = 0; i < len; i++) {
        I2C_SendByte(data[i]);
        if (!I2C_WaitAck()) {
            I2C_Stop();
            return FALSE;
        }
    }
    I2C_Stop();
    return TRUE;
}
//单字节写入*******************************************



bool Single_Write(unsigned char SlaveAddress,unsigned char REG_Address,unsigned char REG_data)		     //void

{

  	if(!I2C_Start())return FALSE;

    I2C_SendByte(SlaveAddress);   //发送设备地址+写信号//I2C_SendByte(((REG_Address & 0x0700) >>7) | SlaveAddress & 0xFFFE);//设置高起始地址+器件地址

    if(!I2C_WaitAck()){I2C_Stop(); return FALSE;}

    I2C_SendByte(REG_Address );   //设置低起始地址

    I2C_WaitAck();

    I2C_SendByte(REG_data);

    I2C_WaitAck();

    I2C_Stop();

 //   delay5ms();// TODO:   需要实现

    return TRUE;

}



//单字节读取*****************************************

unsigned char Single_Read(unsigned char SlaveAddress,unsigned char REG_Address)

{   unsigned char REG_data;

	if(!I2C_Start())return FALSE;

    I2C_SendByte(SlaveAddress); //I2C_SendByte(((REG_Address & 0x0700) >>7) | REG_Address & 0xFFFE);//设置高起始地址+器件地址

    if(!I2C_WaitAck()){I2C_Stop();test=1; return FALSE;}

    I2C_SendByte((u8) REG_Address);   //设置低起始地址

    I2C_WaitAck();

    I2C_Start();

    I2C_SendByte(SlaveAddress+1);

    I2C_WaitAck();



	REG_data= I2C_ReceiveByte();

    I2C_NoAck();

    I2C_Stop();

    //return TRUE;

	return REG_data;



}
void Init_MPU6050(void)

{

/*

   Single_Write(MPU6050_Addr,PWR_M, 0x80);   //

   Single_Write(MPU6050_Addr,SMPL, 0x07);    //

   Single_Write(MPU6050_Addr,DLPF, 0x1E);    //±2000°

   Single_Write(MPU6050_Addr,INT_C, 0x00 );  //

   Single_Write(MPU6050_Addr,PWR_M, 0x00);   //

*/

   	Single_Write(MPU6050_Addr,PWR_MGMT_1, 0x00);	//解除休眠状态

	Single_Write(MPU6050_Addr,SMPLRT_DIV, 0x07);

	Single_Write(MPU6050_Addr,CONFIG, 0x06);

	Single_Write(MPU6050_Addr,GYRO_CONFIG, 0x18);

	Single_Write(MPU6050_Addr,ACCEL_CONFIG, 0x01);

}
//******读取MPU6050数据****************************************

void READ_MPU6050(void)

{

   BUF[0]=Single_Read(MPU6050_Addr,GYRO_XOUT_L);

   BUF[1]=Single_Read(MPU6050_Addr,GYRO_XOUT_H);

   T_X=	(BUF[1]<<8)|BUF[0];

   T_X/=16.4; 						   //读取计算X轴数据



   BUF[2]=Single_Read(MPU6050_Addr,GYRO_YOUT_L);

   BUF[3]=Single_Read(MPU6050_Addr,GYRO_YOUT_H);

   T_Y=	(BUF[3]<<8)|BUF[2];

   T_Y/=16.4; 						   //读取计算Y轴数据

   BUF[4]=Single_Read(MPU6050_Addr,GYRO_ZOUT_L);

   BUF[5]=Single_Read(MPU6050_Addr,GYRO_ZOUT_H);

   T_Z=	(BUF[5]<<8)|BUF[4];

   T_Z/=16.4; 					       //读取计算Z轴数据



  // BUF[6]=Single_Read(MPU6050_Addr,TEMP_OUT_L);

  // BUF[7]=Single_Read(MPU6050_Addr,TEMP_OUT_H);

  // T_T=(BUF[7]<<8)|BUF[6];

  // T_T = 35+ ((double) (T_T + 13200)) / 280;// 读取计算出温度

}

void myputchar(uchar axis)
{
	USART1_SendData(&axis);
}
 //********串口发送数据***************************************

 void Send_data(uchar axis)

 {uchar i;

  USART1_SendData(&axis);
  i = ':';
  //USART1_SendData('\:');
  USART1_SendData(&i);
  for(i=0;i<4;i++)USART1_SendData(&TX_DATA[i]);
  i = ' ';
 // USART1_SendData('\ ');
  USART1_SendData(&i);
 // USART1_SendData('\ ');
  USART1_SendData(&i);
 }



#endif

#ifdef stm32

#define SCL_H         GPIOB->BSRR = GPIO_Pin_8 /* GPIO_SetBits(GPIOB , GPIO_Pin_10)   */
#define SCL_L         GPIOB->BRR  = GPIO_Pin_8 /* GPIO_ResetBits(GPIOB , GPIO_Pin_10) */

#define SDA_H         GPIOB->BSRR = GPIO_Pin_9 /* GPIO_SetBits(GPIOB , GPIO_Pin_11)   */
#define SDA_L         GPIOB->BRR  = GPIO_Pin_9 /* GPIO_ResetBits(GPIOB , GPIO_Pin_11) */

#define SCL_read      GPIOB->IDR  & GPIO_Pin_8 /* GPIO_ReadInputDataBit(GPIOB , GPIO_Pin_10) */
#define SDA_read      GPIOB->IDR  & GPIO_Pin_9 /* GPIO_ReadInputDataBit(GPIOB , GPIO_Pin_11) */


static void I2C_delay(void)
{
    volatile int i = 7;
    while (i)
        i--;
}

static bool I2C_Start(void)
{
    SDA_H;
    SCL_H;
    I2C_delay();
    if (!SDA_read)
        return FALSE;
    SDA_L;
    I2C_delay();
    if (SDA_read)
        return FALSE;
    SDA_L;
    I2C_delay();
    return TRUE;
}

static void I2C_Stop(void)
{
    SCL_L;
    I2C_delay();
    SDA_L;
    I2C_delay();
    SCL_H;
    I2C_delay();
    SDA_H;
    I2C_delay();
}

static void I2C_Ack(void)
{
    SCL_L;
    I2C_delay();
    SDA_L;
    I2C_delay();
    SCL_H;
    I2C_delay();
    SCL_L;
    I2C_delay();
}

static void I2C_NoAck(void)
{
    SCL_L;
    I2C_delay();
    SDA_H;
    I2C_delay();
    SCL_H;
    I2C_delay();
    SCL_L;
    I2C_delay();
}

static bool I2C_WaitAck(void)
{
    SCL_L;
    I2C_delay();
    SDA_H;
    I2C_delay();
    SCL_H;
    I2C_delay();
    if (SDA_read) {
        SCL_L;
        return FALSE;
    }
    SCL_L;
    return TRUE;
}

static void I2C_SendByte(uint8_t byte)
{
    uint8_t i = 8;
    while (i--) {
        SCL_L;
        I2C_delay();
        if (byte & 0x80)
            SDA_H;
        else
            SDA_L;
        byte <<= 1;
        I2C_delay();
        SCL_H;
        I2C_delay();
    }
    SCL_L;
}

static uint8_t I2C_ReceiveByte(void)
{
    uint8_t i = 8;
    uint8_t byte = 0;

    SDA_H;
    while (i--) {
        byte <<= 1;
        SCL_L;
        I2C_delay();
        SCL_H;
        I2C_delay();
        if (SDA_read) {
            byte |= 0x01;
        }
    }
    SCL_L;
    return byte;
}

void i2cInit(void)
{
  GPIO_InitTypeDef gpio;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB , ENABLE);
    gpio.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
    gpio.GPIO_Speed = GPIO_Speed_2MHz;
    gpio.GPIO_Mode = GPIO_Mode_Out_OD;
    GPIO_Init(GPIOB, &gpio);
	/*
    I2C_InitTypeDef  I2C_InitStructure;
    GPIO_InitTypeDef  GPIO_InitStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1,ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);

    //PB8,9 SCL and SDA
    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_8 | GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    // I2C1_SCL on PB08, I2C1_SDA on PB09
    GPIO_PinRemapConfig(GPIO_Remap_I2C1, ENABLE);   //

    //I2C_DeInit(I2C1);
    I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
    //I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
    //I2C_InitStructure.I2C_OwnAddress1 = 0x30;
    I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
    I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    I2C_InitStructure.I2C_ClockSpeed = 100000;//100K

    I2C_Cmd(I2C1, ENABLE);
    I2C_Init(I2C1, &I2C_InitStructure);
    //
    I2C_AcknowledgeConfig(I2C1, ENABLE);   */
}

bool i2cWriteBuffer(uint8_t addr, uint8_t reg, uint8_t len, uint8_t * data)
{
    int i;
    if (!I2C_Start())
        return FALSE;
    I2C_SendByte(addr << 1 | I2C_Direction_Transmitter);//I2C_Direction_Transmitter:д
    if (!I2C_WaitAck()) {
        I2C_Stop();
        return FALSE;
    }
    I2C_SendByte(reg);
    I2C_WaitAck();
    for (i = 0; i < len; i++) {
        I2C_SendByte(data[i]);
        if (!I2C_WaitAck()) {
            I2C_Stop();
            return FALSE;
        }
    }
    I2C_Stop();
    return TRUE;
}
#endif
/////////////////////////////////////////////////////////////////////////////////
#ifdef XMC_Soft_IIC
bool i2cRead(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf)
{
    if (!I2C_Start())
        return FALSE;
    I2C_SendByte(addr << 1 | I2C_Direction_Transmitter);
    if (!I2C_WaitAck()) {
        I2C_Stop();
         return FALSE;
    }
    I2C_SendByte(reg);
    I2C_WaitAck();
    I2C_Start();
    I2C_SendByte(addr << 1 | I2C_Direction_Receiver);
    I2C_WaitAck();
    while (len) {
        *buf = I2C_ReceiveByte();
        if (len == 1)
            I2C_NoAck();
        else
            I2C_Ack();
        buf++;
        len--;
    }
    I2C_Stop();
    return TRUE;
}
int8_t i2cwrite(uint8_t addr, uint8_t reg, uint8_t len, uint8_t * data)
{
	 if(i2cWriteBuffer(addr,reg,len,data))
	/*if(XMCi2cWriteBuffer(addr,reg,len,data))*/
	{
		return TRUE;
	}
	else
	{
		return FALSE;
	}
	//return FALSE;
}
int8_t i2cread(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf)
{
	 if(i2cRead(addr,reg,len,buf))
	/*if(XMCi2cRead(addr,reg,len,buf))*/
	{
		return TRUE;
	}
	else
	{
		return FALSE;
	}
	//return FALSE;
}
#else
int8_t i2cwrite(uint8_t addr, uint8_t reg, uint8_t len, uint8_t * data)
{
	/* if(i2cWriteBuffer(addr,reg,len,data)) */
	if(XMCi2cWriteBuffer(addr,reg,len,data))
	{
		return TRUE;
	}
	else
	{
		return FALSE;
	}
	//return FALSE;
}
int8_t i2cread(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf)
{
	/* if(i2cRead(addr,reg,len,buf)) */
	if(XMCi2cRead(addr,reg,len,buf))
	{
		return TRUE;
	}
	else
	{
		return FALSE;
	}
	//return FALSE;
}
#endif
#ifdef stm32
//////////////////////////////////////////////////////////////////////////////////
bool i2cWrite(uint8_t addr, uint8_t reg, uint8_t data)
{
    if (!I2C_Start())
        return FALSE;
    I2C_SendByte(addr << 1 | I2C_Direction_Transmitter);
    if (!I2C_WaitAck()) {
        I2C_Stop();
        return FALSE;
    }
    I2C_SendByte(reg);
    I2C_WaitAck();
    I2C_SendByte(data);
    I2C_WaitAck();
    I2C_Stop();
    return TRUE;
}

bool i2cRead(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf)
{
    if (!I2C_Start())
        return FALSE;
    I2C_SendByte(addr << 1 | I2C_Direction_Transmitter);
    if (!I2C_WaitAck()) {
        I2C_Stop();
        return FALSE;
    }
    I2C_SendByte(reg);
    I2C_WaitAck();
    I2C_Start();
    I2C_SendByte(addr << 1 | I2C_Direction_Receiver);
    I2C_WaitAck();
    while (len) {
        *buf = I2C_ReceiveByte();
        if (len == 1)
            I2C_NoAck();
        else
            I2C_Ack();
        buf++;
        len--;
    }
    I2C_Stop();
    return TRUE;
}
#endif

uint16_t i2cGetErrorCounter(void)
{
    // TODO maybe fix this, but since this is test code, doesn't matter.
    return 0;
}


void my_func_a(void* Temp);
void millisecond_handler(void* Temp);

int main()
{

	IO004_EnableOutputDriver(&IO004_Handle0,OUTPUT_OD_GP);
	IO004_EnableOutputDriver(&IO004_Handle1,OUTPUT_OD_GP);
	uint8_t test[2] = {0x80,0x45};
	DAVE_Init();
	TimerId   = SYSTM001_CreateTimer(10,SYSTM001_PERIODIC,my_func_a,NULL);
	TimerId_1 = SYSTM001_CreateTimer(1,SYSTM001_PERIODIC,millisecond_handler,NULL);
  //  delay10ms();

    TimerId_2_cnt  = 10;
    SYSTM001_StartTimer(TimerId_1);
    while(TimerId_2_cnt != 0)
    {}
    SYSTM001_StopTimer(TimerId_1);

    Init_MPU6050();
	while(1)
	{
//		I2C_Status = i2cwrite(0x68,0x6B,1,&test[0]);
//		Value = SCL_read;
//    /*	SCL_H;
//		SCL_L;*/
    READ_MPU6050();	         //读取MPU6050数据

    DATA_printf(TX_DATA,T_X);//转换X轴数据到数组

	Send_data('X');			 //发送X轴数

	DATA_printf(TX_DATA,T_Y);//转换Y轴数据到数组

	Send_data('Y');			 //发送Y轴数

	DATA_printf(TX_DATA,T_Z);//转换Z轴数据到数组

	Send_data('Z');			 //发送Z轴数

	DATA_printf(TX_DATA,T_T);//转换温度数据到数组

	Send_data('T');			 //发送温度数据
/*

	USART1_SendData(0X0D);	 //换行

	USART1_SendData(0X0A);	 //回车
*/
	myputchar(0X0D);
	myputchar(0X0A);
//	myputchar('\n');
//	myputchar('\r');
/*
	Send_data(0X0D);	 //换行

	Send_data(0X0A);	 //回车
*/
	//Delayms(5);				 //延时
 //   delay5ms();

	}
	return 0;
}

void my_func_a(void* Temp)
{
	soft_cnt ++;
	if(soft_cnt == 1)
	{
		soft_cnt = 0;
		TimerExpired = TRUE;
	}

}
void millisecond_handler(void* Temp)
{
    TimerId_2_cnt = TimerId_2_cnt - 1;
}
void delay5ms(void)
{
    TimerId_2_cnt  = 5;
    SYSTM001_StartTimer(TimerId_1);
    while(TimerId_2_cnt != 0)
    {}
    SYSTM001_StopTimer(TimerId_1);

}
