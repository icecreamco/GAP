/*****************************************************************************
换温湿度传感器SHT10

2014.10.25
******************************************************************************/
#include "ioCC2530.h"
#include "OnBoard.h"

#define unsigned char uint8
#define unsigned int uint16

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

/******************************************************************************
变量申请
*******************************************************************************/
//uint8 SHT_Value[5]={0};    //依次代表温度整数。温度小数。湿度整数。湿度小数
double temper,humid;
uint8 SHTerror=0;
/******************************************************************************
启动时序
*******************************************************************************/
void SHT_Start(void)
{
  P0DIR|=0x30;
  MicroWait(500);
  DATA_P=1;
  SCK_P=0;
  MicroWait(10);
  SCK_P=1;
  MicroWait(5);
  DATA_P=0;
  MicroWait(5);
  SCK_P=0;
  MicroWait(10);
  SCK_P=1;
  MicroWait(5);
  DATA_P=1;
  MicroWait(5);
  SCK_P=0;
  MicroWait(10);
}

/******************************************************************************
写入命令，并检测ASK位
*******************************************************************************/
uint8 SHT_WritCOM(uint8 com)
{
  uint8 i;
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
  MicroWait(500);
  SCK_P=1;
  MicroWait(5);
  if(DATA_P)    //检测应答  错误返回error=1
    SHTerror=1;
  MicroWait(5);
  SCK_P=0;
  MicroWait(10);
  return 1; 
}

/******************************************************************************
读出数据并存储
*******************************************************************************/
uint16 SHT_ReadData(void)
{
  uint16 data;
  uint16 i;
  P0DIR |=0x10;
  MicroWait(500);
  DATA_P=1;
  for(i=0;i<320;i++)
  {
    MicroWait(1000);
  }
  P0DIR &=~0x10;
  MicroWait(500);
  if(DATA_P)               //检测数据，错误返回error=1
    SHTerror=1;
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
  P0DIR |=0x10;
  MicroWait(500);
  DATA_P=0;
  SCK_P=1;
  MicroWait(10);
  SCK_P=0;
  MicroWait(10);
  P0DIR &=~0x10;
  MicroWait(500);
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
  P0DIR |=0x10;
  MicroWait(500);
  DATA_P=1;
  SCK_P=1;
  MicroWait(10);
  SCK_P=0;
  MicroWait(10);
  return data;
}

/******************************************************************************
数据修正
*******************************************************************************/
void SHT_DataRevise(void)
{
  double hline;
  uint16 t,h; 
  temper=0;
  hline=0;
  humid=0;
  SHT_Start();
  SHT_WritCOM(0x03);
  t=SHT_ReadData();
  t&=~0xc000;
  SHT_Start();
  SHT_WritCOM(0x05);
  h=SHT_ReadData();
  h&=~0xf000;
  //修正
  temper=((double)t)*0.01-39.66;
  hline=0.0405*((double)h)-0.0000028*((double)h)*((double)h)-4.0;
  humid=(temper-25)*(0.01+0.00008*((double)h))+hline;
/*  if(temper<0)
  {
    SHT_Value[0]='-';
    temper=-temper;
  }
  else
  {
    SHT_Value[0]=' ';
  }
  SHT_Value[1]=(uint8)temper;
  temper=(temper-SHT_Value[1])*100;
  SHT_Value[2]=(uint8)temper;
  SHT_Value[3]=(uint8)humid;
  SHT_Value[4]=(uint8)((humid-SHT_Value[3])*100);*/
}