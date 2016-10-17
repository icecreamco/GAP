#include "ioCC2530.h"
#include "OnBoard.h"

/******************************************************************************
���ŷ���
*******************************************************************************/
#define DATA_P P0_4
#define SCK_P P0_5

/******************************************************************************
����ԭ��
*******************************************************************************/
void SHT_Start(void);
uint8 SHT_WritCOM(uint8 com);
uint16 SHT_ReadData(void);
void SHT_DataRevise(void);
void Delay_10us(void);
void Delay_ms(uint16 Time);
/******************************************************************************
��������
*******************************************************************************/
#define ADR_W 128
#define ADR_R 129
double temper,humid;
uint8 SHTerror=0;
/******************************************************************************
����ʱ��
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
д����������ASKλ
*******************************************************************************/
uint8 SHT_WritCOM(uint8 com)
{
  uint8 i,ADRR;
  ADRR=ADR_W;
//���͵�ַ*************************************
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
  //���ASK�ź�
  P0DIR &=~0x10;
  MicroWait(1000);
  SCK_P=1;
  MicroWait(5);
  if(DATA_P)    //���Ӧ��  ���󷵻�error=1
    SHTerror=1;
  MicroWait(5);
  SCK_P=0;
  MicroWait(20);
//��������******************************************  
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
  //���ASK�ź�
  P0DIR &=~0x10;
  MicroWait(1000);
  SCK_P=1;
  MicroWait(5);
  if(DATA_P)    //���Ӧ��  ���󷵻�error=1
    SHTerror=1;
  MicroWait(5);
  SCK_P=0;
  MicroWait(20);
  
  return 0; 
}
/******************************************************************************
�������ݲ��洢
*******************************************************************************/
uint16 SHT_ReadData(void)
{
  uint16 data;
  uint16 i;
  uint8 ADRR;  
  ADRR=ADR_R;
//���͵�ַ*************************************
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
  //���ASK�ź�
  P0DIR &=~0x10;
  MicroWait(1000);
  SCK_P=1;
  MicroWait(5);
  if(DATA_P)    //���Ӧ��  ���󷵻�error=1
    SHTerror=1;
  MicroWait(5);
  SCK_P=0;
  MicroWait(10);
  
  SCK_P=1;
  P0DIR &=~0x20;
  MicroWait(1000);
  
  for(i=0;i<150;i++) //�ȴ�150ms
  {
    MicroWait(1000);
    if(SCK_P==1)
      break;
  }
  if(SCK_P==0)
    SHTerror=1;          //���Ӧ��  ���󷵻�error=1
//��ȡ���ֽ�****************************************************************  
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
//�ظ�ASK********************************************************************
  P0DIR |=0x10;
  MicroWait(1000);
  DATA_P=0;
  MicroWait(5);
  SCK_P=1;
  MicroWait(10);
  SCK_P=0;
  MicroWait(10);
//��ȡ���ֽ�*****************************************************************
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
//��ӦASK*****************************************************************
  P0DIR |=0x10;
  MicroWait(1000);
  DATA_P=1;
  MicroWait(5);
  SCK_P=1;
  MicroWait(10);
  SCK_P=0;
  MicroWait(10);
//�����ź�****************************************************************
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
��������
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
  //����
  temper=(((double)t)/65536)*175.72-46.85;
  humid=(((double)h)/65536)*125.0-6.0;
}
/******************************************************************************
��ʱ
*******************************************************************************/
void Delay_10us(void) //10 us��ʱ
{
   MicroWait(10);
}

void Delay_ms(uint16 Time)//n ms��ʱ
{
  uint8 i;
  while(Time--)
  {
    for(i=0;i<100;i++)
     Delay_10us();
  }
}