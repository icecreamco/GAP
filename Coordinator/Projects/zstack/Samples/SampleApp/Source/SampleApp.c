/**************************************************************************************************
  Filename:       SampleApp.c
  Revised:        $Date: 2009-03-18 15:56:27 -0700 (Wed, 18 Mar 2009) $
  Revision:       $Revision: 19453 $

  Description:    Sample Application (no Profile).


  Copyright 2007 Texas Instruments Incorporated. All rights reserved.

  IMPORTANT: Your use of this Software is limited to those specific rights
  granted under the terms of a software license agreement between the user
  who downloaded the software, his/her employer (which must be your employer)
  and Texas Instruments Incorporated (the "License").  You may not use this
  Software unless you agree to abide by the terms of the License. The License
  limits your use, and you acknowledge, that the Software may not be modified,
  copied or distributed unless embedded on a Texas Instruments microcontroller
  or used solely and exclusively in conjunction with a Texas Instruments radio
  frequency transceiver, which is integrated into your product.  Other than for
  the foregoing purpose, you may not use, reproduce, copy, prepare derivative
  works of, modify, distribute, perform, display or sell this Software and/or
  its documentation for any purpose.

  YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
  PROVIDED 揂S IS?WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
  INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE,
  NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL
  TEXAS INSTRUMENTS OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER CONTRACT,
  NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER
  LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
  INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE
  OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT
  OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
  (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.

  Should you have any questions regarding your right to use this Software,
  contact Texas Instruments Incorporated at www.TI.com.
**************************************************************************************************/

/*********************************************************************
  This application isn't intended to do anything useful, it is
  intended to be a simple example of an application's structure.

  This application sends it's messages either as broadcast or
  broadcast filtered group messages.  The other (more normal)
  message addressing is unicast.  Most of the other sample
  applications are written to support the unicast message model.

  Key control:
    SW1:  Sends a flash command to all devices in Group 1.
    SW2:  Adds/Removes (toggles) this device in and out
          of Group 1.  This will enable and disable the
          reception of the flash command.
*********************************************************************/

/*********************************************************************
 * INCLUDES
 */
#include "OSAL.h"
#include "OSAL_Clock.h"
#include "OSAL_Timers.h"
#include "ZGlobals.h"
#include "mac_radio.h"
#include "AF.h"
#include "aps_groups.h"
#include "ZDApp.h"

#include "SampleApp.h"
#include "SampleAppHw.h"

#include "OnBoard.h"

/* HAL */
#include "hal_lcd.h"
#include "hal_led.h"
#include "hal_key.h"

#include "MT_UART.h"
#include "hal_uart.h"
#include "Mac_low_level.h"

#include "mac_rx.h"
/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */

// This list should be filled with Application specific Cluster IDs.
const cId_t SampleApp_ClusterList[SAMPLEAPP_MAX_CLUSTERS] =
{
  SAMPLEAPP_PERIODIC_CLUSTERID,
  SAMPLEAPP_FLASH_CLUSTERID
};

const SimpleDescriptionFormat_t SampleApp_SimpleDesc =
{
  SAMPLEAPP_ENDPOINT,              //  int Endpoint;
  SAMPLEAPP_PROFID,                //  uint16 AppProfId[2];
  SAMPLEAPP_DEVICEID,              //  uint16 AppDeviceId[2];
  SAMPLEAPP_DEVICE_VERSION,        //  int   AppDevVer:4;
  SAMPLEAPP_FLAGS,                 //  int   AppFlags:4;
  SAMPLEAPP_MAX_CLUSTERS,          //  uint8  AppNumInClusters;
  (cId_t *)SampleApp_ClusterList,  //  uint8 *pAppInClusterList;
  SAMPLEAPP_MAX_CLUSTERS,          //  uint8  AppNumInClusters;
  (cId_t *)SampleApp_ClusterList   //  uint8 *pAppInClusterList;
};

// This is the Endpoint/Interface description.  It is defined here, but
// filled-in in SampleApp_Init().  Another way to go would be to fill
// in the structure here and make it a "const" (in code space).  The
// way it's defined in this sample app it is define in RAM.
endPointDesc_t SampleApp_epDesc;

/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */
uint8 SampleApp_TaskID;   // Task ID for internal task/event processing
                          // This variable will be received when
                          // SampleApp_Init() is called.
devStates_t SampleApp_NwkState;

uint8 SampleApp_TransID;  // This is the unique message ID (counter)

afAddrType_t SampleApp_Periodic_DstAddr;
afAddrType_t SampleApp_Flash_DstAddr;
afAddrType_t SampleApp_P2P_DstAddr;

aps_Group_t SampleApp_Group;

uint8 SampleAppPeriodicCounter = 0;
uint8 SampleAppFlashCounter = 0;

//*****************自定义变量*************************************************//
uint8 Count=0;                    //节点个数计数
#define ABLen 10                  //地址分类的个数
uint32 sleeptime=3600000;         //数据间隔
#define Maxtime 64000             //每次最大定时时间
uint8 DataFlag[30]={0};           //节点在线标志位
uint8 OnlineDEV[100]={0};         //节点在线表
uint8 Timer_Flag=0;               //计算总定时时间标志位
uint8 T_Online_Flag=0;            //发送在线表标志位
uint32 Timeout=0;                 //总定时时间
uint8 TimeCount=0;                //最大定时时间的个数
uint16 Time_ms=0;                 //每次定时的时间

uint8 Mode='C';                   //工作模式 'C':正常 'M':调试
uint8 WaitFlag=1;                 //串口数据等待标志
uint8 TX_2=0;                     //第二次在线表发送标志
uint8 LinkWait=0;                 //建网等待标志
uint8 LinkCount=0;                //建网等待计数

uint8 res16[5];
uint8 res8[3];
/*********************************************************************
 * LOCAL FUNCTIONS
 */
void SampleApp_HandleKeys( uint8 shift, uint8 keys );
void SampleApp_MessageMSGCB( afIncomingMSGPacket_t *pckt );
void SampleApp_SendPeriodicMessage(uint8 ADR,uint8 select);
void SampleApp_SendFlashMessage( uint16 flashTime );
void SampleApp_Send_P2P_Message(uint8 dat1,uint8 dat2);

void Delayus(void);                //10us延时
void Delayms(uint16 Time);         //n ms延时
int isDigit(uint8 *x,uint8 n);     //判断字符串是否为纯数字 是返回1 否则返回0

void convert16(uint16 data);
void convert8(uint8 data);
/*********************************************************************
 * NETWORK LAYER CALLBACKS
 */

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      SampleApp_Init
 *
 * @brief   Initialization function for the Generic App Task.
 *          This is called during initialization and should contain
 *          any application specific initialization (ie. hardware
 *          initialization/setup, table initialization, power up
 *          notificaiton ... ).
 *
 * @param   task_id - the ID assigned by OSAL.  This ID should be
 *                    used to send messages and set timers.
 *
 * @return  none
 */
void SampleApp_Init( uint8 task_id )
{
  SampleApp_TaskID = task_id;
  SampleApp_NwkState = DEV_INIT;
  SampleApp_TransID = 0;

//*******************设置无线功率*********************************************//
//  macRadioSetTxPower(0XF5);  
  
//*******************初始化并开启串口*****************************************//
  MT_UartInit ();
  MT_UartRegisterTaskID( task_id);

//*******************参数初始化***********************************************//  
  Timer_Flag=0;                     //计算总定时时间标志位清零 不启动
  T_Online_Flag=0;                  //发送在线表标志位清零 不发送
  WaitFlag=1;                       //串口数据等待标志置1 等待
  TX_2=0;                           //第二次发送在线表标志位清零 不发送
  LinkWait=0;                       //建网等待标志位清零 不等待
  LinkCount=0;                      //建网等待计数清零
  
  // Device hardware initialization can be added here or in main() (Zmain.c).
  // If the hardware is application specific - add it here.
  // If the hardware is other parts of the device add it in main().

 #if defined ( BUILD_ALL_DEVICES )
  // The "Demo" target is setup to have BUILD_ALL_DEVICES and HOLD_AUTO_START
  // We are looking at a jumper (defined in SampleAppHw.c) to be jumpered
  // together - if they are - we will start up a coordinator. Otherwise,
  // the device will start as a router.
  if ( readCoordinatorJumper() )
    zgDeviceLogicalType = ZG_DEVICETYPE_COORDINATOR;
  else
    zgDeviceLogicalType = ZG_DEVICETYPE_ROUTER;
#endif // BUILD_ALL_DEVICES

#if defined ( HOLD_AUTO_START )
  // HOLD_AUTO_START is a compile option that will surpress ZDApp
  //  from starting the device and wait for the application to
  //  start the device.
  ZDOInitDevice(0);
#endif

  // Setup for the periodic message's destination address
  // Broadcast to everyone
  SampleApp_Periodic_DstAddr.addrMode = (afAddrMode_t)AddrBroadcast;
  SampleApp_Periodic_DstAddr.endPoint = SAMPLEAPP_ENDPOINT;
  SampleApp_Periodic_DstAddr.addr.shortAddr = 0xFFFF;

  // Setup for the flash command's destination address - Group 1
  SampleApp_Flash_DstAddr.addrMode = (afAddrMode_t)afAddrGroup;
  SampleApp_Flash_DstAddr.endPoint = SAMPLEAPP_ENDPOINT;
  SampleApp_Flash_DstAddr.addr.shortAddr = SAMPLEAPP_FLASH_GROUP;

  SampleApp_P2P_DstAddr.addrMode = (afAddrMode_t)Addr16Bit;
  SampleApp_P2P_DstAddr.endPoint = SAMPLEAPP_ENDPOINT;
  SampleApp_P2P_DstAddr.addr.shortAddr = 0x0000;
  
  // Fill out the endpoint description.
  SampleApp_epDesc.endPoint = SAMPLEAPP_ENDPOINT;
  SampleApp_epDesc.task_id = &SampleApp_TaskID;
  SampleApp_epDesc.simpleDesc
            = (SimpleDescriptionFormat_t *)&SampleApp_SimpleDesc;
  SampleApp_epDesc.latencyReq = noLatencyReqs;

  // Register the endpoint description with the AF
  afRegister( &SampleApp_epDesc );

  // Register for all key events - This app will handle all key events
  RegisterForKeys( SampleApp_TaskID );

  // By default, all devices start out in Group 1
  SampleApp_Group.ID = 0x0001;
  osal_memcpy( SampleApp_Group.name, "Group 1", 7  );
  aps_AddGroup( SAMPLEAPP_ENDPOINT, &SampleApp_Group );

#if defined ( LCD_SUPPORTED )
  HalLcdWriteString( "SampleApp", HAL_LCD_LINE_1 );
#endif
  osal_start_timerEx( SampleApp_TaskID, SAMPLEAPP_SEND_PERIODIC_MSG_EVT,
        2000);
}

/*********************************************************************
 * @fn      SampleApp_ProcessEvent
 *
 * @brief   Generic Application Task event processor.  This function
 *          is called to process all events for the task.  Events
 *          include timers, messages and any other user defined events.
 *
 * @param   task_id  - The OSAL assigned task ID.
 * @param   events - events to process.  This is a bit map and can
 *                   contain more than one event.
 *
 * @return  none
 */
uint16 SampleApp_ProcessEvent( uint8 task_id, uint16 events )
{
  uint8 i;
  afIncomingMSGPacket_t *MSGpkt;
  (void)task_id;  // Intentionally unreferenced parameter

  if ( events & SYS_EVENT_MSG )
  {
    MSGpkt = (afIncomingMSGPacket_t *)osal_msg_receive( SampleApp_TaskID );
    while ( MSGpkt )
    {
      switch ( MSGpkt->hdr.event )
      {
        // Received when a key is pressed
        case KEY_CHANGE:
          SampleApp_HandleKeys( ((keyChange_t *)MSGpkt)->state, ((keyChange_t *)MSGpkt)->keys );
          break;

        // Received when a messages is received (OTA) for this endpoint
        case AF_INCOMING_MSG_CMD:
          SampleApp_MessageMSGCB( MSGpkt );
          break;

        // Received whenever the device changes state in the network
        case ZDO_STATE_CHANGE:
          SampleApp_NwkState = (devStates_t)(MSGpkt->hdr.status);
          if ( (SampleApp_NwkState == DEV_ZB_COORD)
              || (SampleApp_NwkState == DEV_ROUTER)
                )
          { HalUARTWrite(0,"online",6);
            // Start sending the periodic message in a regular interval.
//**********macRadioEnergyDetectStart();**************建网完毕后开始检测信号强度
            osal_start_timerEx( SampleApp_TaskID,
                             SAMPLEAPP_SEND_PERIODIC_MSG_EVT,
                              5000);
          }
          else
          {
            // Device is no longer in the network
          }
          break;

        default:
          break;
      }

      // Release the memory
      osal_msg_deallocate( (uint8 *)MSGpkt );

      // Next - if one is available
      MSGpkt = (afIncomingMSGPacket_t *)osal_msg_receive( SampleApp_TaskID );
    }

    // return unprocessed events
    return (events ^ SYS_EVENT_MSG);
  }

  // Send a message out - This event is generated by a timer
  //  (setup in SampleApp_Init()).
  if ( events & SAMPLEAPP_SEND_PERIODIC_MSG_EVT )
  {
/******************************************************************************************
*功能：建网延时
*参数：LinkCount-----循环次数
*      Time_ms-------延时时间
******************************************************************************************/
    if(LinkWait==1)                                   //若建网延时标志位为1 则执行建网延时
    {
      LinkCount++;
      if(Mode=='M') 
      {
        if(LinkCount == 1)                               //定时个数为0个
        {
          LinkCount=0;                                  //延时个数清零
          LinkWait=0;                                   //延时标志位清零
          ZDOInitDevice( 0 );                           //开启设备建网
        }
      }
      else if(Mode=='C')
      {
        if(LinkCount == 66)                               //定时个数为65个
        {
          LinkCount=0;                                  //延时个数清零
          LinkWait=0;                                   //延时标志位清零
          ZDOInitDevice( 0 );                           //开启设备建网
        }
        else
        {
          Time_ms=60000;                                //每次定时为65S
          osal_start_timerEx( SampleApp_TaskID, SAMPLEAPP_SEND_PERIODIC_MSG_EVT,
          Time_ms);                                     //开启定时器
        }
      }  
    }
/******************************************************************************************
*功能：定时发送在线表
*参数：TX_2----------------发送次数
*      T_Online_Flag-------发送标志位
******************************************************************************************/
    else                                              //不许建网延时则执行接收-上传数据工作
    {
      if(TX_2==2)                                     //若第二次发送在线表标志位为2 第二次发送
      {
/*****************第二次发送**************************************************************/
        TX_2=0;                                       //标志位清零
        Timer_Flag=1;                                 //计算总定时时间标志位置1
        HalUARTWrite(0,OnlineDEV,Count*3+2);          //串口发送在线表
        osal_memset(DataFlag,0,sizeof(DataFlag));     //将在线标志数组清零
        osal_memset(OnlineDEV,0,sizeof(OnlineDEV));   //将在线表清零
      }
      if(T_Online_Flag==1)                            //发送在线表标志为1 第一次发送
      {
/*****************第一次发送**************************************************************/
        T_Online_Flag=0;                              //发送在线表标志清零
        Count=0;                                      //节点个数先清零
        OnlineDEV[0]='L';                             //在线表首字母为'L'
        for(i=0;i<sizeof(DataFlag);i++)               //循环 填写在线表并计算节点个数
        {
          if(DataFlag[i]>0)                           //如果节点在线标志位大于等于1 则在线
          {
            Count++;                                  //节点个数加1
            OnlineDEV[(Count-1)*3+1]=i/ABLen+0x41;    //填在线表,i+1代表节点标号
            OnlineDEV[(Count-1)*3+2]=((i+1)-(OnlineDEV[(Count-1)*3+1]-0X41)*ABLen)/10+0x30;
            OnlineDEV[(Count-1)*3+3]=((i+1)-(OnlineDEV[(Count-1)*3+1]-0X41)*ABLen)%10+0x30;
          }
        }
       OnlineDEV[Count*3+1]='@';                      //在线表末字母为'@'
       HalUARTWrite(0,OnlineDEV,Count*3+2);           //发送在线表
       TX_2=1;                                        //将第二次发送标志位置1
      }
/*****************计算定时时间************************************************************/
     if(Timer_Flag==1)                                //若计算总定时时间标志位为1 计算总时间                     
     {
       Timer_Flag=0;                                  //计算总定时时间标志位清零
       
       if(Mode=='M')                                  //调试模式 数据间隔为两分钟
       {
         sleeptime=120000;
         Timeout=sleeptime-osal_systemClock%sleeptime+110000; //等待节点数据接收完毕 110s
       }
       else if(Mode=='C')                             //正常模式 数据间隔为1小时
       {
         sleeptime=3600000;
         Timeout=sleeptime-osal_systemClock%sleeptime+180000; //等待节点数据接收完毕 180s
       }
       TimeCount=Timeout/Maxtime;                     //需要最大定时时间的个数
       Timeout=Timeout%Maxtime;                       //剩余的定时时间
     }
/*****************等待网关串口命令******************************************************************/
     if(WaitFlag==1)                                  //串口数据等待标志为1
     {
       Time_ms=5000;                                  //等待5S
     }
/******************正常计算每次睡眠的时间*************************************************************/
     else
     {
       if(TimeCount>0)                                //如果最大定时时间的次数大于等于1
       {
         TimeCount--;                                 //次数减1
         Time_ms=Maxtime;                             //此次定时时间为最大定时时间
       }
       else                                           //若最大定时时间次数为0
       {
         Time_ms=(uint16)Timeout;                     //此次定时时间为剩余定时时间
         T_Online_Flag=1;                             //在线表发送标志位置1
       }
     }
/*****************第二次发送在线表延时**************************************************************/
      if(TX_2==1)                                     //若第一次在线表发送成功
      {
       TX_2=2;                                        //第二次发送标志位置2
       T_Online_Flag=0;                               //发送标志位清零                            
       Time_ms=3000;                                  //间隔3S
      }
      
//********************开启定时器**********************************************//
    osal_start_timerEx( SampleApp_TaskID, SAMPLEAPP_SEND_PERIODIC_MSG_EVT,
        Time_ms);
    }
    // return unprocessed events
    return (events ^ SAMPLEAPP_SEND_PERIODIC_MSG_EVT);
  }

  // Discard unknown events
  return 0;
}

/*********************************************************************
 * Event Generation Functions
 */
/*********************************************************************
 * @fn      SampleApp_HandleKeys
 *
 * @brief   Handles all key events for this device.
 *
 * @param   shift - true if in shift/alt.
 * @param   keys - bit field for key events. Valid entries:
 *                 HAL_KEY_SW_2
 *                 HAL_KEY_SW_1
 *
 * @return  none
 */
void SampleApp_HandleKeys( uint8 shift, uint8 keys )
{
  (void)shift;  // Intentionally unreferenced parameter
  
  if ( keys & HAL_KEY_SW_1 )
  {
    /* This key sends the Flash Command is sent to Group 1.
     * This device will not receive the Flash Command from this
     * device (even if it belongs to group 1).
     */
    SampleApp_SendFlashMessage( SAMPLEAPP_FLASH_DURATION );
  }

  if ( keys & HAL_KEY_SW_2 )
  {
    /* The Flashr Command is sent to Group 1.
     * This key toggles this device in and out of group 1.
     * If this device doesn't belong to group 1, this application
     * will not receive the Flash command sent to group 1.
     */
    aps_Group_t *grp;
    grp = aps_FindGroup( SAMPLEAPP_ENDPOINT, SAMPLEAPP_FLASH_GROUP );
    if ( grp )
    {
      // Remove from the group
      aps_RemoveGroup( SAMPLEAPP_ENDPOINT, SAMPLEAPP_FLASH_GROUP );
    }
    else
    {
      // Add to the flash group
      aps_AddGroup( SAMPLEAPP_ENDPOINT, &SampleApp_Group );
    }
  }
}
/******************************************************************************
//判断字符串函数
//返回1：纯数字或数字前面为空格  返回0：包含字母
******************************************************************************/
int isDigit(uint8 *x,uint8 n)
{
  //检测数字字符串
  //如果字符串只含数字或者数字前面是空格，则返回1
  //否则返回0
  uint8 i,flag=0;
  for(i=0;i<n;i++)
    if(x[i]==' ' && flag==0)
      flag=0;
    else if(x[i]>='0' && x[i]<='9')
      flag=1;
    else
      break;
    return (n==i);
}
/*********************************************************************
 * LOCAL FUNCTIONS
 */

/*********************************************************************
 * @fn      SampleApp_MessageMSGCB
 *
 * @brief   Data message processor callback.  This function processes
 *          any incoming data - probably from other devices.  So, based
 *          on cluster ID, perform the intended action.
 *
 * @param   none
 *
 * @return  none
 */
void SampleApp_MessageMSGCB( afIncomingMSGPacket_t *pkt )
{
  uint16 flashTime;
  uint8 data[25]={0};                                        //接收数据缓存数组
  uint8 addr_i;                                              //**************rssilevel;测强度
  int Hvalue;                                                //湿度数值
  uint16 sourceAddr;
  uint16 routerAddr;
  uint8 quality;
  uint8 info[] = "Source ADDR :      ; Router ADDR :      ; Link Quality :      ";
  uint8 i;
  switch ( pkt->clusterId )
  {
    case SAMPLEAPP_PERIODIC_CLUSTERID:

      break;
      
    case SAMPLEAPP_P2P_CLUSTERID:
       osal_memcpy(data,pkt->cmd.Data,pkt->cmd.DataLength);  //接收数据放入缓存数组
		
	    sourceAddr = pkt->srcAddr.addr.shortAddr;
		convert16(sourceAddr);
		for(i = 0; i < 5; i++) {
			info[14 + i] = res16[i];
		}
		routerAddr = lastDeviceShortAddr;
		if(routerAddr == sourceAddr) {
			for(i = 0; i < 5; i++) {
				info[35 + i] = ' ';
			}
		} else {
			convert16(routerAddr);
			for(i = 0; i < 5; i++) {
				info[35 + i] = res16[i];
			}
		}
		quality = pkt->LinkQuality;
		convert8(quality);
		for(i = 0; i < 3; i++) {
				info[57 + i] = res8[i];
		}		
       HalUARTWrite(0, info, 62);  
//**************************湿度爆表处理***********************************
       if(data[12]==' ')
       {
         Hvalue=data[13]-0x30;
       }
       else
       {
         Hvalue=(data[12]-0x30)*10+(data[13]-0x30);         
       }
       if(Hvalue<0)                                         //湿度值小于0 置0
       {
         data[12]=' ';
         data[13]='0';
       }
       if(Hvalue>99)                                        //湿度值大于99 置99
       {
         data[12]='9';
         data[13]='9';
       }
//****************************温度范围规范化********************************       
//data:
       //5:' ' or '-'
       //6: 温度十位
       //7:温度各位
       //8:'.'
       //9:温度十分位
       //10:温度百分位
       if((data[5]!=' ' && data[5]!='-') || data[8]!='.' || 
          !isDigit(&data[6],2) || !isDigit(&data[9],2))
         osal_memcpy(&data[4],"T-99.88",7);
//****************************湿度范围规范化********************************       
//data:
       //12: 湿度十位
       //13：湿度各位
       if(!isDigit(&data[12],2))
         osal_memcpy(&data[12],"-8",2);
//****************************光照范围规范化********************************       
//data:
       //15: 光照万位
       //16：光照千位
       //17: 光照百位
       //18：光照十位
       //19：光照个位
       if(!isDigit(&data[15],5))
         osal_memcpy(&data[15],"65528",5);
//********************错误格式转换*****************************************
       if(data[4]=='E')
       {
         osal_memcpy(&data[4],"T-99.77H-7",10);
       }
       if(data[14]=='E')
       {
         osal_memcpy(&data[14],"I65527",6);
       }
       
//****************接收数据处理************************************************//
       if((data[0]=='D') && (pkt->cmd.DataLength==21))                 //若数据首字符为'D' 长度为21 为采集数据
       {
         HalUARTWrite(0, data, pkt->cmd.DataLength);                   //串口上传接收数据
		 HalUARTWrite(0, "\n", 1);
         addr_i=(data[1]-0x41)*ABLen+(data[2]-0x30)*10+(data[3]-0x30); //计算节点地址标号
         DataFlag[addr_i-1]++;                                         //相应地址标号的在线标志位加1
       }
       else if(data[0]=='T')                                           //若数据首字符为'T' 为时钟同步请求
       {
		 HalUARTWrite(0, "clock synchronize!", 18);
		 HalUARTWrite(0, "\n", 1);
         SampleApp_SendPeriodicMessage((data[1]-0x41)*ABLen+(data[2]-0x30)*10+(data[3]-0x30),0x01);
       }
      break;  
      
    case SAMPLEAPP_FLASH_CLUSTERID:
      flashTime = BUILD_UINT16(pkt->cmd.Data[1], pkt->cmd.Data[2] );
      HalLedBlink( HAL_LED_4, 4, 50, (flashTime / 4) );
      break;
  }
}

void convert16(uint16 data) {
	res16[0] = data / 10000 + 0x30;
	data %= 10000;
	res16[1] = data / 1000 + 0x30;
	data %= 1000;
	res16[2] = data / 100 + 0x30;
	data %= 100;
	res16[3] = data / 10 + 0x30;
	data %= 10;
	res16[4] = data + 0x30;
}
void convert8(uint8 data) {
	res8[0] = data / 100 + 0x30;
	data %= 100;
	res8[1] = data / 10 + 0x30;
	data %= 10;
	res8[2] = data + 0x30;
}

/*********************************************************************
 * @fn      SampleApp_SendPeriodicMessage
 *
 * @brief   Send the periodic message.
 *
 * @param   none
 *
 * @return  none
 */
void SampleApp_SendPeriodicMessage(uint8 ADR,uint8 select)
{
  UTCTime utime;                              //32位时钟数据
  uint8 data[20]="D";                         //数据首字母为'D'
  data[1]=(ADR-1)/ABLen+0x41;                 //计算第一位地址 为字母A B C
  data[2]=(ADR-(data[1]-0X41)*ABLen)/10+0x30; //计算第二位地址 
  data[3]=(ADR-(data[1]-0X41)*ABLen)%10+0x30; //计算第三位地址 1-10

  if(select==0x01)                            //发送选择为1
  {
    utime=osal_GetSystemClock();              //获得定时器数据
    data[4]=utime>>24;                        //32-25
    data[5]=utime>>16;                        //24-17
    data[6]=utime>>8;                         //16-9
    data[7]=utime;                            //8-1 
    data[8]=Mode;                             //工作模式
    if ( AF_DataRequest( &SampleApp_Periodic_DstAddr, &SampleApp_epDesc,
                         SAMPLEAPP_PERIODIC_CLUSTERID,
                         9,
                         data,
                         &SampleApp_TransID,
                         AF_DISCV_ROUTE,
                         AF_DEFAULT_RADIUS ) == afStatus_SUCCESS )
    {
    }
    else
    {
        // Error occurred in request to send.
    }
  }
/*  else if(select==0x02)
  {
    osal_memcpy(&data[4],"ASK",3);
    if ( AF_DataRequest( &SampleApp_Periodic_DstAddr, &SampleApp_epDesc,
                         SAMPLEAPP_PERIODIC_CLUSTERID,
                         7,
                         data,
                         &SampleApp_TransID,
                         AF_DISCV_ROUTE,
                         AF_DEFAULT_RADIUS ) == afStatus_SUCCESS )
    {
    }
    else
    {
      // Error occurred in request to send.
    }
  }*/
}

/*********************************************************************
 * @fn      SampleApp_SendFlashMessage
 *
 * @brief   Send the flash message to group 1.
 *
 * @param   flashTime - in milliseconds
 *
 * @return  none
 */
void SampleApp_SendFlashMessage( uint16 flashTime )
{
  uint8 buffer[3];
  buffer[0] = (uint8)(SampleAppFlashCounter++);
  buffer[1] = LO_UINT16( flashTime );
  buffer[2] = HI_UINT16( flashTime );

  if ( AF_DataRequest( &SampleApp_Flash_DstAddr, &SampleApp_epDesc,
                       SAMPLEAPP_FLASH_CLUSTERID,
                       3,
                       buffer,
                       &SampleApp_TransID,
                       AF_DISCV_ROUTE,
                       AF_DEFAULT_RADIUS ) == afStatus_SUCCESS )
  {
  }
  else
  {
    // Error occurred in request to send.
  }
}

void SampleApp_Send_P2P_Message(uint8 dat1,uint8 dat2) 
{ 
  
    uint8 ans[3]="A00";
    ans[1]=dat1;
    ans[2]=dat2;
    
  if ( AF_DataRequest( &SampleApp_P2P_DstAddr, &SampleApp_epDesc, 
                        SAMPLEAPP_P2P_CLUSTERID, 
                        3, 
                        ans, 
                        &SampleApp_TransID, 
                        AF_DISCV_ROUTE, 
                        AF_DEFAULT_RADIUS ) == afStatus_SUCCESS ) 
  { 
  }


  else 
  { 
      // Error occurred in request to send. 
  } 
} 
/*********************************************************************
*********************************************************************/
void Delayus(void)                      //10 us延时
{
   MicroWait(10);
}

void Delayms(uint16 Time)               //n ms延时
{
  unsigned char i;
  while(Time--)
  {
    for(i=0;i<100;i++)
     Delayus();
  }
}