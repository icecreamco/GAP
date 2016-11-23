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
  PROVIDED �AS IS?WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
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

//*****************�Զ������*************************************************//
uint8 Count=0;                    //�ڵ��������
#define ABLen 10                  //��ַ����ĸ���
uint32 sleeptime=3600000;         //���ݼ��
#define Maxtime 64000             //ÿ�����ʱʱ��
uint8 DataFlag[30]={0};           //�ڵ����߱�־λ
uint8 OnlineDEV[100]={0};         //�ڵ����߱�
uint8 Timer_Flag=0;               //�����ܶ�ʱʱ���־λ
uint8 T_Online_Flag=0;            //�������߱��־λ
uint32 Timeout=0;                 //�ܶ�ʱʱ��
uint8 TimeCount=0;                //���ʱʱ��ĸ���
uint16 Time_ms=0;                 //ÿ�ζ�ʱ��ʱ��

uint8 Mode='C';                   //����ģʽ 'C':���� 'M':����
uint8 WaitFlag=1;                 //�������ݵȴ���־
uint8 TX_2=0;                     //�ڶ������߱��ͱ�־
uint8 LinkWait=0;                 //�����ȴ���־
uint8 LinkCount=0;                //�����ȴ�����

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

void Delayus(void);                //10us��ʱ
void Delayms(uint16 Time);         //n ms��ʱ
int isDigit(uint8 *x,uint8 n);     //�ж��ַ����Ƿ�Ϊ������ �Ƿ���1 ���򷵻�0

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

//*******************�������߹���*********************************************//
//  macRadioSetTxPower(0XF5);  
  
//*******************��ʼ������������*****************************************//
  MT_UartInit ();
  MT_UartRegisterTaskID( task_id);

//*******************������ʼ��***********************************************//  
  Timer_Flag=0;                     //�����ܶ�ʱʱ���־λ���� ������
  T_Online_Flag=0;                  //�������߱��־λ���� ������
  WaitFlag=1;                       //�������ݵȴ���־��1 �ȴ�
  TX_2=0;                           //�ڶ��η������߱��־λ���� ������
  LinkWait=0;                       //�����ȴ���־λ���� ���ȴ�
  LinkCount=0;                      //�����ȴ���������
  
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
//**********macRadioEnergyDetectStart();**************������Ϻ�ʼ����ź�ǿ��
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
*���ܣ�������ʱ
*������LinkCount-----ѭ������
*      Time_ms-------��ʱʱ��
******************************************************************************************/
    if(LinkWait==1)                                   //��������ʱ��־λΪ1 ��ִ�н�����ʱ
    {
      LinkCount++;
      if(Mode=='M') 
      {
        if(LinkCount == 1)                               //��ʱ����Ϊ0��
        {
          LinkCount=0;                                  //��ʱ��������
          LinkWait=0;                                   //��ʱ��־λ����
          ZDOInitDevice( 0 );                           //�����豸����
        }
      }
      else if(Mode=='C')
      {
        if(LinkCount == 66)                               //��ʱ����Ϊ65��
        {
          LinkCount=0;                                  //��ʱ��������
          LinkWait=0;                                   //��ʱ��־λ����
          ZDOInitDevice( 0 );                           //�����豸����
        }
        else
        {
          Time_ms=60000;                                //ÿ�ζ�ʱΪ65S
          osal_start_timerEx( SampleApp_TaskID, SAMPLEAPP_SEND_PERIODIC_MSG_EVT,
          Time_ms);                                     //������ʱ��
        }
      }  
    }
/******************************************************************************************
*���ܣ���ʱ�������߱�
*������TX_2----------------���ʹ���
*      T_Online_Flag-------���ͱ�־λ
******************************************************************************************/
    else                                              //��������ʱ��ִ�н���-�ϴ����ݹ���
    {
      if(TX_2==2)                                     //���ڶ��η������߱��־λΪ2 �ڶ��η���
      {
/*****************�ڶ��η���**************************************************************/
        TX_2=0;                                       //��־λ����
        Timer_Flag=1;                                 //�����ܶ�ʱʱ���־λ��1
        HalUARTWrite(0,OnlineDEV,Count*3+2);          //���ڷ������߱�
        osal_memset(DataFlag,0,sizeof(DataFlag));     //�����߱�־��������
        osal_memset(OnlineDEV,0,sizeof(OnlineDEV));   //�����߱�����
      }
      if(T_Online_Flag==1)                            //�������߱��־Ϊ1 ��һ�η���
      {
/*****************��һ�η���**************************************************************/
        T_Online_Flag=0;                              //�������߱��־����
        Count=0;                                      //�ڵ����������
        OnlineDEV[0]='L';                             //���߱�����ĸΪ'L'
        for(i=0;i<sizeof(DataFlag);i++)               //ѭ�� ��д���߱�����ڵ����
        {
          if(DataFlag[i]>0)                           //����ڵ����߱�־λ���ڵ���1 ������
          {
            Count++;                                  //�ڵ������1
            OnlineDEV[(Count-1)*3+1]=i/ABLen+0x41;    //�����߱�,i+1����ڵ���
            OnlineDEV[(Count-1)*3+2]=((i+1)-(OnlineDEV[(Count-1)*3+1]-0X41)*ABLen)/10+0x30;
            OnlineDEV[(Count-1)*3+3]=((i+1)-(OnlineDEV[(Count-1)*3+1]-0X41)*ABLen)%10+0x30;
          }
        }
       OnlineDEV[Count*3+1]='@';                      //���߱�ĩ��ĸΪ'@'
       HalUARTWrite(0,OnlineDEV,Count*3+2);           //�������߱�
       TX_2=1;                                        //���ڶ��η��ͱ�־λ��1
      }
/*****************���㶨ʱʱ��************************************************************/
     if(Timer_Flag==1)                                //�������ܶ�ʱʱ���־λΪ1 ������ʱ��                     
     {
       Timer_Flag=0;                                  //�����ܶ�ʱʱ���־λ����
       
       if(Mode=='M')                                  //����ģʽ ���ݼ��Ϊ������
       {
         sleeptime=120000;
         Timeout=sleeptime-osal_systemClock%sleeptime+110000; //�ȴ��ڵ����ݽ������ 110s
       }
       else if(Mode=='C')                             //����ģʽ ���ݼ��Ϊ1Сʱ
       {
         sleeptime=3600000;
         Timeout=sleeptime-osal_systemClock%sleeptime+180000; //�ȴ��ڵ����ݽ������ 180s
       }
       TimeCount=Timeout/Maxtime;                     //��Ҫ���ʱʱ��ĸ���
       Timeout=Timeout%Maxtime;                       //ʣ��Ķ�ʱʱ��
     }
/*****************�ȴ����ش�������******************************************************************/
     if(WaitFlag==1)                                  //�������ݵȴ���־Ϊ1
     {
       Time_ms=5000;                                  //�ȴ�5S
     }
/******************��������ÿ��˯�ߵ�ʱ��*************************************************************/
     else
     {
       if(TimeCount>0)                                //������ʱʱ��Ĵ������ڵ���1
       {
         TimeCount--;                                 //������1
         Time_ms=Maxtime;                             //�˴ζ�ʱʱ��Ϊ���ʱʱ��
       }
       else                                           //�����ʱʱ�����Ϊ0
       {
         Time_ms=(uint16)Timeout;                     //�˴ζ�ʱʱ��Ϊʣ�ඨʱʱ��
         T_Online_Flag=1;                             //���߱��ͱ�־λ��1
       }
     }
/*****************�ڶ��η������߱���ʱ**************************************************************/
      if(TX_2==1)                                     //����һ�����߱��ͳɹ�
      {
       TX_2=2;                                        //�ڶ��η��ͱ�־λ��2
       T_Online_Flag=0;                               //���ͱ�־λ����                            
       Time_ms=3000;                                  //���3S
      }
      
//********************������ʱ��**********************************************//
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
//�ж��ַ�������
//����1�������ֻ�����ǰ��Ϊ�ո�  ����0��������ĸ
******************************************************************************/
int isDigit(uint8 *x,uint8 n)
{
  //��������ַ���
  //����ַ���ֻ�����ֻ�������ǰ���ǿո��򷵻�1
  //���򷵻�0
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
  uint8 data[25]={0};                                        //�������ݻ�������
  uint8 addr_i;                                              //**************rssilevel;��ǿ��
  int Hvalue;                                                //ʪ����ֵ
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
       osal_memcpy(data,pkt->cmd.Data,pkt->cmd.DataLength);  //�������ݷ��뻺������
		
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
//**************************ʪ�ȱ�����***********************************
       if(data[12]==' ')
       {
         Hvalue=data[13]-0x30;
       }
       else
       {
         Hvalue=(data[12]-0x30)*10+(data[13]-0x30);         
       }
       if(Hvalue<0)                                         //ʪ��ֵС��0 ��0
       {
         data[12]=' ';
         data[13]='0';
       }
       if(Hvalue>99)                                        //ʪ��ֵ����99 ��99
       {
         data[12]='9';
         data[13]='9';
       }
//****************************�¶ȷ�Χ�淶��********************************       
//data:
       //5:' ' or '-'
       //6: �¶�ʮλ
       //7:�¶ȸ�λ
       //8:'.'
       //9:�¶�ʮ��λ
       //10:�¶Ȱٷ�λ
       if((data[5]!=' ' && data[5]!='-') || data[8]!='.' || 
          !isDigit(&data[6],2) || !isDigit(&data[9],2))
         osal_memcpy(&data[4],"T-99.88",7);
//****************************ʪ�ȷ�Χ�淶��********************************       
//data:
       //12: ʪ��ʮλ
       //13��ʪ�ȸ�λ
       if(!isDigit(&data[12],2))
         osal_memcpy(&data[12],"-8",2);
//****************************���շ�Χ�淶��********************************       
//data:
       //15: ������λ
       //16������ǧλ
       //17: ���հ�λ
       //18������ʮλ
       //19�����ո�λ
       if(!isDigit(&data[15],5))
         osal_memcpy(&data[15],"65528",5);
//********************�����ʽת��*****************************************
       if(data[4]=='E')
       {
         osal_memcpy(&data[4],"T-99.77H-7",10);
       }
       if(data[14]=='E')
       {
         osal_memcpy(&data[14],"I65527",6);
       }
       
//****************�������ݴ���************************************************//
       if((data[0]=='D') && (pkt->cmd.DataLength==21))                 //���������ַ�Ϊ'D' ����Ϊ21 Ϊ�ɼ�����
       {
         HalUARTWrite(0, data, pkt->cmd.DataLength);                   //�����ϴ���������
		 HalUARTWrite(0, "\n", 1);
         addr_i=(data[1]-0x41)*ABLen+(data[2]-0x30)*10+(data[3]-0x30); //����ڵ��ַ���
         DataFlag[addr_i-1]++;                                         //��Ӧ��ַ��ŵ����߱�־λ��1
       }
       else if(data[0]=='T')                                           //���������ַ�Ϊ'T' Ϊʱ��ͬ������
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
  UTCTime utime;                              //32λʱ������
  uint8 data[20]="D";                         //��������ĸΪ'D'
  data[1]=(ADR-1)/ABLen+0x41;                 //�����һλ��ַ Ϊ��ĸA B C
  data[2]=(ADR-(data[1]-0X41)*ABLen)/10+0x30; //����ڶ�λ��ַ 
  data[3]=(ADR-(data[1]-0X41)*ABLen)%10+0x30; //�������λ��ַ 1-10

  if(select==0x01)                            //����ѡ��Ϊ1
  {
    utime=osal_GetSystemClock();              //��ö�ʱ������
    data[4]=utime>>24;                        //32-25
    data[5]=utime>>16;                        //24-17
    data[6]=utime>>8;                         //16-9
    data[7]=utime;                            //8-1 
    data[8]=Mode;                             //����ģʽ
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
void Delayus(void)                      //10 us��ʱ
{
   MicroWait(10);
}

void Delayms(uint16 Time)               //n ms��ʱ
{
  unsigned char i;
  while(Time--)
  {
    for(i=0;i<100;i++)
     Delayus();
  }
}