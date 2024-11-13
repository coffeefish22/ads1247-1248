#if 1
#include <rthw.h>
#include <rtthread.h>
#include "myconfig.h"
#include "globalval.h"


#define ADS1247_CMD_RREG 0x20
#define ADS1247_CMD_WREG 0x40

#define ADS1247_CMD_NOP  0xFF
#define ADS1247_CMD_RDATA 0x12
#define ADS1247_CMD_RESET 0x06
#define ADS1247_CMD_SYNC  0x04

#define ADS1247_REG_MUX0  0x00
#define ADS1247_REG_VBIAS 0X01
#define ADS1247_REG_MUX1  0x02
#define ADS1247_REG_SYS0  0x03
#define ADS1247_REG_IDAC0 0X0A 
#define ADS1247_REG_IDAC1 0X0B 

#define	MS_PORT		GPIOD
#define	MS_CLK_PIN		3
#define	MS_DOUT_PIN		5
#define	MS_DIN_PIN		4
#define	MS_CS_PIN		7

#define	MS_PORT_RESET		GPIOD
#define MS_RESET_PIN     0  //PD0
#define MS_START_PIN     3  //PB3

#define	S_SPI_CSL	MS_PORT->BRR = (1 << MS_CS_PIN)
#define	S_SPI_CSH	MS_PORT->BSRR = (1 << MS_CS_PIN)
	
#define	S_SPI_CLKL	MS_PORT->BRR = (1 << MS_CLK_PIN)
#define	S_SPI_CLKH	MS_PORT->BSRR = (1 << MS_CLK_PIN)
	
#define	S_SPI_MOSIL	MS_PORT->BRR = (1 << MS_DIN_PIN)
#define	S_SPI_MOSIH	MS_PORT->BSRR = (1 << MS_DIN_PIN)
	
#define	S_SPI_MISOH	MS_PORT->BSRR = (1 << MS_DOUT_PIN)
	
#define	S_SPI_MISO	((MS_PORT->IDR & (1 << MS_DOUT_PIN)) != 0)


#define ADS_CS_0  S_SPI_CSL
#define ADS_CS_1  S_SPI_CSH

#define ADS_RESET_1  GPIOD->BSRR = (1 << MS_RESET_PIN)
#define ADS_RESET_0  GPIOD->BRR = (1 << MS_RESET_PIN)

#define ADS_START_1  GPIOB->BSRR = (1 << MS_START_PIN)
#define ADS_START_0  GPIOB->BRR = (1 << MS_START_PIN)


	
#define	S_SPI_DELAY 10//15//10//对于72M时钟，时钟分频值设置为1098（对于36M使用549）

	char q;
	u8 ADH,ADM,ADL;
	u32 k1,k2,k3;
	 
	/******************ads1247管脚与MCU对应关系*******************
	SCK----------PD3
	MISO---------PD4
	MOSI---------PD5							(SPI)
	CS-----------PD7
	RESET--------PD0
	START--------PB3
	DRDY---------PD6
	******************ads1247管脚与MCU对应关系*******************/
void ADS1247_SPI_Configuration()
{
 
//  	SPI_InitTypeDef SPI_InitStruct;  
  	GPIO_InitTypeDef GPIO_InitStructure;
	 
  	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB|RCC_APB2Periph_GPIOD|RCC_APB2Periph_AFIO,ENABLE); 	 //SPI的SCK、MISO、MOSI都是PB
//	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable,ENABLE); 
   
   //CLK
	   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;    
	   GPIO_Init(GPIOD, &GPIO_InitStructure);
	   
   //drdy
	   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
	   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;    
	   GPIO_Init(GPIOD, &GPIO_InitStructure);
   //DOUT
	   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
	   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;  
	   GPIO_Init(GPIOD, &GPIO_InitStructure);
   //CS DIN reset
	   GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_4|GPIO_Pin_7|GPIO_Pin_0;
	   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;  
	   GPIO_Init(GPIOD, &GPIO_InitStructure);

		GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_3;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;  
		GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable,ENABLE);
		GPIO_Init(GPIOB, &GPIO_InitStructure);

	   
}
u8 SPI_ADS1247_SendByte(unsigned char in)
{
	int bi = 8;
	int dli = 0;

	if (1)
	{
		S_SPI_CLKH;
		S_SPI_CSL;
		
		while (bi--)
		{
			S_SPI_CLKH;//时钟上升沿，传感器读取数据
			dli = S_SPI_DELAY;
			
			while (dli--);
			if (in & 0x80)
			{
				S_SPI_MOSIH;
			}
			else
			{
				S_SPI_MOSIL;
			}
			in <<= 1;
			dli = S_SPI_DELAY;
			while (dli--);
			S_SPI_CLKL;//时钟下降沿，MCU改变数据
			dli = S_SPI_DELAY;
			while (dli--);
		}
		S_SPI_MOSIH;
	}
	return 0;
}

u8 SPI_ADS1247_ReadByte()
{
   int bi = 8;
   int dli = 0;
   int out = -1;
   out = 0;
   if (1)
   {
	   for (bi = 0; bi < 8; bi++)
	   {
		   S_SPI_CLKH;//时钟上升沿，传感器改变数据
		   
		   out <<= 1;
		   dli = S_SPI_DELAY;
		   while (dli--);
		   
		   S_SPI_CLKL;//时钟下降沿，MCU读取数据
	   
		   dli = S_SPI_DELAY;
		   while (dli--);
		   
		   if (S_SPI_MISO)
		   {
			   out |= 1;
		   }
		   dli = S_SPI_DELAY;
		   while (dli--);
		   
	   }
   }
   return (out);
}

	   /*******************************************************************************  
	* Function Name  : SPI_ADS1247_SendByte  
	* Description	 : 写ADS1247的寄存器
	* Input 		 :	 
	* Output		 :	
	*******************************************************************************/
	 void ADS1247WREG(unsigned char regaddr,unsigned char databyte)
	 {
	   
		SPI_ADS1247_SendByte(ADS1247_CMD_WREG+(regaddr & 0xF)); 
	 
		SPI_ADS1247_SendByte(0); 
	 
		SPI_ADS1247_SendByte(databyte);
	  
	 }
	  
	   /*******************************************************************************  
	* Function Name  : SPI_ADS1247_SendByte  
	* Description	 : 读ADS1247的寄存器
	* Input 		 :	 
	* Output		 :	
	*******************************************************************************/
	 unsigned char ADS1247RREG(unsigned char regaddr)
	 { 
	 
		unsigned char r=0; 
	 
		SPI_ADS1247_SendByte(ADS1247_CMD_RREG+(regaddr & 0xF)); 
	 
		SPI_ADS1247_SendByte(0); 
	 
		r=SPI_ADS1247_ReadByte(); 
	 
		return r;
	 }
	 
	/*******************************************************************************  
	* Function Name  : ADS1247_ReadData  
	* Description	 : 读 ADS1247 转换值
	* Input 		 : addr:Regiter address 		 
	* Output		 : None  
	* Return		 : Register data
	*******************************************************************************/ 
	u32 ADS1247_ReadData()
	{
			u32 value = 0;
		SPI_ADS1247_SendByte(ADS1247_CMD_RDATA);
		while(GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_6));
		 rt_thread_delay(1);
		value = SPI_ADS1247_ReadByte()<<16;
		value += SPI_ADS1247_ReadByte()<<8;
		value += SPI_ADS1247_ReadByte();
		return value;
		
	}
	 
	 
	/*******************************************************************************  
	* Function Name  : ADS1247_Init 	 (通道和电流输出端口没选择)
	* Description	 : ADS1247 init
	* Input 		 : None 		 
	* Output		 : None  
	* Return		 : None
	*******************************************************************************/
	void ADS1247_init_start(void)
	{
		ADS1247_SPI_Configuration();
		ADS_CS_0;    //CS=0
//		ADS_START_0;
//		rt_thread_delay(1);
//		ADS_RESET_0;   
//		rt_thread_delay(1);
		ADS_RESET_1;   
//		rt_thread_delay(5);
		ADS_START_1;
		rt_thread_delay(1);
		while(GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_6) ==1);
		SPI_ADS1247_SendByte(ADS1247_CMD_RESET);	 //RESET
		rt_thread_delay(1);
		ADS1247WREG(ADS1247_REG_SYS0,0x0); 	//增益0,5sps    //增益5,160sps 
		rt_thread_delay(1);
		ADS1247WREG(ADS1247_REG_VBIAS,0);   //偏置电压关闭（默认）
		rt_thread_delay(1);
		ADS1247WREG(ADS1247_REG_MUX1,0x30);   //内部电压基准一直开选择内部校准
		rt_thread_delay(1);
		
		q=ADS1247RREG(ADS1247_REG_MUX1);
		
		rt_kprintf("测试:%x\r\n",q);
		ADS_CS_1;
	}
	 
	 
int ADS1247_Readchannel_A()
{	
	u32 data11;
	ADS_CS_0;
	ADS1247WREG(ADS1247_REG_MUX0,0x04);    //AIN0+	   AIN4-
	data11=ADS1247_ReadData();
	ADS_CS_1;
	return data11;
}
int ADS1247_Readchannel_B()
{  
	u32 data22;
	ADS_CS_0;	
	ADS1247WREG(ADS1247_REG_MUX0,0x0C);   //AIN1+	  AIN4-
	data22=ADS1247_ReadData();
	ADS_CS_1;
	return data22;

}
	 
	 
	 
int ADS1247_Readchannel_C()
{  
	u32 data22;
	ADS_CS_0;	
	ADS1247WREG(ADS1247_REG_MUX0,0x1D);   //AIN3+	  AIN5-
	data22=ADS1247_ReadData();
	ADS_CS_1;
	return data22;

}
int ADS1247_Readchannel_D()
{  
	u32 data22;
	ADS_CS_0;	
	ADS1247WREG(ADS1247_REG_MUX0,0x15);   //AIN2+	  AIN5-
	data22=ADS1247_ReadData();
	ADS_CS_1;
	return data22;

}

int pd_v;
int V_1248[4]={0};
int ret00=0;
long yun_suan_v[4]={0};
int yuan_1247v[4][10]={0};
static void ADS1247_process_plus(void* parameter)
{
//	int i=0;
//	float sum=0;	//端口初始化
	int channel_select=0;
	ADS1247_init_start();

	while(1)
	{

		switch(channel_select)
		{
			case 0:
				ret00=ADS1247_Readchannel_B();
			break;
			case 1:
				ret00=ADS1247_Readchannel_C();
			break;
			case 2:
				ret00=ADS1247_Readchannel_D();
			break;
			default:
				ret00=ADS1247_Readchannel_A();
			break;
		}
		channel_select++;
		if(channel_select>3)
			channel_select=0;
		if(ret00>8388608)
		{
			ret00=ret00-16777216;
		}
		V_1248[channel_select]=(int)(ret00*125/512); //mv
		yun_suan_v[channel_select]=V_1248[channel_select];
		modifyro(V_A0+channel_select,yun_suan_v[channel_select]);
//		rt_kprintf("channel %d电压=%d mv\n",channel_select,(int)V_1248[channel_select]);
		rt_thread_delay(220);	
	}
	
}

void ADS1247_init()
{
		rt_thread_t thread;

	thread = rt_thread_create("1247",		  
							  ADS1247_process_plus, RT_NULL,
							  1024,
							  THREAD_1247_PRIORITY, 50);
	if (thread != RT_NULL)
		rt_thread_startup(thread);
	
}

#endif
