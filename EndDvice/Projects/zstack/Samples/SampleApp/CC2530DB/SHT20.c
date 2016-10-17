#include "ioCC2530.h"
#include "OnBoard.h"

/******************************************************************************
引脚分配
*******************************************************************************/
#define DATA_P P0_4
#define SCK_P P0_5

/******************************************************************************
函数原型
*******************************************************************************/
void SHT_Start(void);
uint8 SHT_WritCOM(uint8 com);
uint16 SHT_ReadData(void);
void SHT_DataRevise(void);
void Delay_10us(void);
void Delay_ms(uint16 Time);
/******************************************************************************
变量申请
*******************************************************************************/
#define ADR_W 128
#define ADR_R 129
double temper,humid;
uint8 SHTerror=0;
/******************************************************************************
启动时序
*******************************************************************************/
void SHT_Start(void)
{
  P0DIR|=0x30;
  MicroWait(1000);
  SCK_P=1;
  DATA_P=1;
  MicroWait(10);
  DATA_P=0;
  MicroWait(10);
  SCK_P=0;
  MicroWait(10);
}
/******************************************************************************
写入命令，并检测ASK位
*******************************************************************************/
uint8 SHT_WritCOM(uint8 com)
{
  uint8 i,ADRR;
  ADRR=ADR_W;
//发送地址*************************************
  for(i=0;i<8;i++)
  {
    if(ADRR & 0x80)
      DATA_P=1;
    else
      DATA_P=0;
    MicroWait(5);
    SCK_P=1;
    MicroWait(10);
    SCK_P=0;
    MicroWait(10);
    ADRR<<=1;
  }
  DATA_P=1;
  //检测ASK信号
  P0DIR &=~0x10;
  MicroWait(1000);
  SCK_P=1;
  MicroWait(5);
  if(DATA_P)    //检测应答  错误返回error=1
    SHTerror=1;
  MicroWait(5);
  SCK_P=0;
  MicroWait(20);
//发送命令******************************************  
  P0DIR |=0x10;
  MicroWait(1000);
  for(i=0;i<8;i++)
  {
    if(com & 0x80)
      DATA_P=1;
    else
      DATA_P=0;
    MicroWait(5);
    SCK_P=1;
    MicroWait(10);
    SCK_P=0;
    MicroWait(10);
    com<<=1;
  }
  DATA_P=1;
  //检测ASK信号
  P0DIR &=~0x10;
  MicroWait(1000);
  SCK_P=1;
  MicroWait(5);
  if(DATA_P)    //检测应答  错误返回error=1
    SHTerror=1;
  MicroWait(5);
  SCK_P=0;
  MicroWait(20);
  
  return 0; 
}
/******************************************************************************
读出数据并存储
*******************************************************************************/
uint16 SHT_ReadData(void)
{
  uint16 data;
  uint16 i;
  uint8 ADRR;  
  ADRR=ADR_R;
//发送地址*************************************
  for(i=0;i<8;i++)
  {
    if(ADRR & 0x80)
      DATA_P=1;
    else
      DATA_P=0;
    MicroWait(5);
    SCK_P=1;
    MicroWait(10);
    SCK_P=0;
    MicroWait(10);
    ADRR<<=1;
  }
  DATA_P=1;
  //检测ASK信号
  P0DIR &=~0x10;
  MicroWait(1000);
  SCK_P=1;
  MicroWait(5);
  if(DATA_P)    //检测应答  错误返回error=1
    SHTerror=1;
  MicroWait(5);
  SCK_P=0;
  MicroWait(10);
  
  SCK_P=1;
  P0DIR &=~0x20;
  MicroWait(1000);
  
  for(i=0;i<150;i++) //等待150ms
  {
    MicroWait(1000);
    if(SCK_P==1)
      break;
  }
  if(SCK_P==0)
    SHTerror=1;          //检测应答  错误返回error=1
//读取高字节****************************************************************  
  P0DIR|=0x20;
  MicroWait(1000);

  DATA_P=1;
  for(i=0;i<8;i++)
  {
    data<<=1;
    SCK_P=1;
    MicroWait(5);
    data|=DATA_P;
    MicroWait(5);
    SCK_P=0;
    MicroWait(10);
  }
//回复ASK********************************************************************
  P0DIR |=0x10;
  MicroWait(1000);
  DATA_P=0;
  MicroWait(5);
  SCK_P=1;
  MicroWait(10);
  SCK_P=0;
  MicroWait(10);
//读取低字节*****************************************************************
  P0DIR &=~0x10;
  MicroWait(1000);
  for(i=0;i<8;i++)
  {
    data<<=1;
    SCK_P=1;
    MicroWait(5);
    data|=DATA_P;
    MicroWait(5);
    SCK_P=0;
    MicroWait(10);
  }
//回应ASK*****************************************************************
  P0DIR |=0x10;
  MicroWait(1000);
  DATA_P=1;
  MicroWait(5);
  SCK_P=1;
  MicroWait(10);
  SCK_P=0;
  MicroWait(10);
//结束信号****************************************************************
  DATA_P=0;
  SCK_P=0;
  MicroWait(5);
  SCK_P=1;
  MicroWait(10);
  DATA_P=1;
  MicroWait(10);
  
  return data;
}

/******************************************************************************
数据修正
*******************************************************************************/
void SHT_DataRevise(void)
{
  uint16 t,h; 
  temper=0;
  humid=0;
  SHT_Start();
  SHT_WritCOM(0xE3);
  SHT_Start();
  t=SHT_ReadData();
  t&=~0x0003;
  SHT_Start();
  SHT_WritCOM(0xE5);
  SHT_Start();
  h=SHT_ReadData();
  h&=~0x0003;
  //修正
  temper=(((double)t)/65536)*175.72-46.85;
  humid=(((double)h)/65536)*125.0-6.0;
}
/******************************************************************************
延时
*******************************************************************************/
void Delay_10us(void) //10 us延时
{
   MicroWait(10);
}

void Delay_ms(uint16 Time)//n ms延时
{
  uint8 i;
  while(Time--)
  {
    for(i=0;i<100;i++)
     Delay_10us();
  }
}