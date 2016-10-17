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
#include "OSAL_PwrMgr.h"
#include "OSAL_Timers.h"
#include "ZGlobals.h"
#include "AF.h"
#include "aps_groups.h"
#include "ZDApp.h"

#include "SampleApp.h"
#include "SampleAppHw.h"

#include "OnBoard.h"
#include "ioCC2530.h"
/* HAL */
#include "hal_lcd.h"
#include "hal_led.h"
#include "hal_key.h"

#include "MT_UART.h"
#include "hal_uart.h"
#include "SHT20.h"
#include "lux.h"
#include "string.h"
#include "math.h"
#include "Mac_low_level.h"
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

#define	POWER P1_0           //���ص�Դ����λ
uint8 pf;                    //ʱ���¼���־
uint8 powf;                  //����˯�߱�־
#define ABLen 10             //ÿ����ĸ�����Ľڵ����
uint32 SleepTime_MS = 0;     //ÿ��˯�ߵ�ʱ��
uint32 SleepTime = 0;        //��˯��ʱ��      
uint8 TX_Flag = 0;           //�������ݱ�־λ
uint8 TCount = 0;            //���˯��ʱ��Ĵ���
uint8 CountF = 0;            //����˯��ʱ���־λ
uint8 Mode = 'C';            //�ڵ㹤��ģʽ  C������  M������  
uint8 ADDR,ADDR1,ADDR2,ADDR3;     //��ַ
uint16 delayt;               //������ʱ
/*********************************************************************
 * LOCAL FUNCTIONS
 */
void SampleApp_HandleKeys( uint8 shift, uint8 keys );
void SampleApp_MessageMSGCB( afIncomingMSGPacket_t *pckt );
void SampleApp_SendPeriodicMessage(uint8 ADR);
void SampleApp_SendFlashMessage( uint16 flashTime );
void SampleApp_Send_P2P_Message(uint8 select);

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
  
//*****************���÷��͹���***********************************************//  
  macRadioSetTxPower( 0XF5 ); 
  
  
//*****************������ʼ��*************************************************//
  TX_Flag = 0;             //�������ݱ�־λ���㣬��������
  TCount = 0;              //���˯��ʱ��Ĵ�������
  CountF = 0;              //����˯��ʱ���־λ����
  pf = 0;                  //ʱ��ʱ����־����
  powf = 0;                //����˯�߱�־λ����  ������˯��
  SleepTime_MS = 0;        //ÿ��˯�ߵ�ʱ������
  SleepTime = 0;           //��˯��ʱ������           

//*****************���ų�ʼ��*************************************************//  
  P1SEL = 0X00;
  P1DIR |= 0X03;           //P1_0 P1_1����Ϊ���
  Delay_ms( 50 );          //��ʱ50MS
  SWLED = 1;               //LED��
  POWER = 0;               //�������ر�
  P1DIR &= ~0X01;          //P1_0����Ϊ����
  P1INP |= 0X01;          //P1_0����Ϊ����̬

  P1DIR &= ~0X30;          //����P1_4 P1_5Ϊ����
  P1DIR &= ~0X0C;          //����P1_3 P1_2Ϊ����
  P1INP |= 0X30;           //����P1_4 P1_5Ϊ����̬
  P1INP |= 0X0C;           //����P1_3 P1_2Ϊ����̬
 
  //��ȡ�ڵ��ַ//
  P0SEL = 0X00;
  P0DIR = 0X00;            //����P0Ϊ����
  P0INP = 0X00;
  Delay_ms( 1000 );          //��ʱ1000MS
  ADDR = P0;
  if(ADDR == 0)
    ADDR = 1;
  delayt = Delaybet * ADDR;
  ADDR1 = ((ADDR - 1) / ABLen) + 0x41;
  ADDR %= ABLen;
  if(ADDR == 0)
    ADDR = ABLen;
  if(ADDR >= 100)
    ADDR = 99;
  ADDR2 = ADDR / 10 + 0x30;
  ADDR3 = ADDR % 10 + 0X30;
  P0INP = 0Xff;
  
  P2DIR &= ~0X04;
  P2INP |= 0X04;
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
//*******************�������ݵ����ݴ�����***********************************//
        case AF_INCOMING_MSG_CMD:
          SampleApp_MessageMSGCB( MSGpkt );
         break;

//*******************�ڵ�״̬�ı�Ĵ�����***********************************//
        // Received whenever the device changes state in the network
        case ZDO_STATE_CHANGE:
          SampleApp_NwkState = (devStates_t)(MSGpkt->hdr.status);
          if (   (SampleApp_NwkState == DEV_ROUTER)
              || (SampleApp_NwkState == DEV_END_DEVICE) 
                )
          {
//*******************��ʱ������***********************************************//
        // Start sending the periodic message in a regular interval.
            pf = 0;                             //��ʱ���¼���־λ����
            SWLED = 0;                          //LED�ر�
            CountF = 1;                         //����˯��ʱ���־λ����
            osal_start_timerEx( SampleApp_TaskID,
                                SAMPLEAPP_SEND_PERIODIC_MSG_EVT,
                                500 );          //��ʱ500MS
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
//*********************��ʱ�¼�������***************************************//
  if ( events & SAMPLEAPP_SEND_PERIODIC_MSG_EVT )
  { 
    if( pf == 0 )                            //pfΪ0  ����ʱ��ͬ������
    {
      SWLED = 1;                             //LED����
      SampleApp_Send_P2P_Message( 0x02 );    //����ʱ��ͬ������
      osal_start_timerEx( SampleApp_TaskID, SAMPLEAPP_SEND_PERIODIC_MSG_EVT,
                          1100);             //1.1s�����ͬ��  roll_rateΪ1s
    }
    else if( pf == 1 )                       //pfΪ1 ͬ���ɹ� �������ݲ�����͹���ģʽ     
    {
      SWLED = 0;                             //LED�ر�
      
//**************������ֱ�ӷ�һ������******************************************//
      P1INP &= ~0X30;                        //ȡ��P1_5 P1_4�ĸ���̬ģʽ
      P1INP &= ~0X0C;                        //ȡ��P1_3 P1_2�ĸ���̬ģʽ
      P1DIR |= 0X01;                         //����P1_0Ϊ���
      Delay_ms( 1 );
      POWER = 1;                             //�򿪸��ص�Դ
      Delay_ms( 1000 );                      //�ȴ����ص�ѹ�ȶ�
      SampleApp_Send_P2P_Message( 0x01 );    //���Ͳɼ�����  ��һ��
      SampleApp_Send_P2P_Message( 0x01 );    //���Ͳɼ�����  �ڶ���
      POWER = 0;                             //�ص����ص�Դ
      P1DIR &= ~0X01;                        //����P1_0Ϊ����
      P1INP |= 0X01;                       //����P1_0Ϊ����̬
       
      P1DIR &= ~0X30;                        //����P1_4 P1_5Ϊ����
      P1DIR &= ~0X0C;                        //����P1_3 P1_2Ϊ����
      P1INP |= 0X30;                         //����P1_4 P1_5Ϊ����̬
      P1INP |= 0X0C;                         //����P1_3 P1_2Ϊ����̬          

//***************����rate   ����͹���****************************************//
      NLME_SetPollRate( 0 );
      NLME_SetQueuedPollRate( 0 );
      NLME_SetResponseRate( 0 );       
      
      powf = 1;                               //��˯�߱�־                                      
      pf = 2;                                 //pf��2
    }
    if( pf == 2 )                             //pfΪ2 ���������ķ���-˯�߹���ģʽ
    {
      if( TX_Flag == 1 )                      //������ͱ�־λΪ1 ���Ͳɼ�����
      {
        TX_Flag = 0;                          //���ͱ�־λ����
//*****************���Ͳɼ�����***********************************************//        
        P1INP &= ~0X30;           
        P1INP &= ~0X0C;
        P1DIR |= 0X01;
        Delay_ms( 1 );
        POWER = 1;             
        Delay_ms( 1000 );     
        SampleApp_Send_P2P_Message( 0x01 );
        SampleApp_Send_P2P_Message( 0x01 );
        POWER = 0;
        P1DIR &= ~0X01;
        P1INP |= 0X01;
         
        P1DIR &= ~0X30;                    
        P1DIR &= ~0X0C;
        P1INP |= 0X30;
        P1INP |= 0X0C;
      }
      osal_start_timerEx( SampleApp_TaskID, SAMPLEAPP_SEND_PERIODIC_MSG_EVT,
         (SAMPLEAPP_SEND_PERIODIC_MSG_TIMEOUT + (osal_rand() & 0x00FF)) );
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
//******************��ʼ������************************************************//  
  uint8 data[10]={0};                                            //�������ݻ�������
  uint8 datacmp[10]={0};                                         //���ݱȽ�����
  osal_memcpy(datacmp, "D", 1);                                  //������λΪ'D'
  datacmp[1]=ADDR1;                                              //����2λΪ��ַADDR1
  datacmp[2]=ADDR2;                                              //����3λΪ��ַADDR2
  datacmp[3]=ADDR3;                                              //����4λΪ��ַADDR3
  
  switch ( pkt->clusterId )
  {
//*************���ݴ�����***************************************************//
    case SAMPLEAPP_PERIODIC_CLUSTERID:                            
      osal_memcpy(data,pkt->cmd.Data,pkt->cmd.DataLength);       //�����ݻ���������data   
      if(osal_memcmp(datacmp,data,4) && (pkt->cmd.DataLength==9))//ʱ��ͬ����ģʽ���� 9λ
      { 
         osal_systemClock=(((uint32)data[4])<<24)+(((uint32)data[5])<<16)+(((uint32)data[6])<<8)+(data[7]);
         Mode=data[8];                                           //ǰ8λΪʱ������ ��9λΪ����ģʽ
         pf = 1;                                                 //ʱ��ͬ����� pf��1
      }
      break;
      
    case SAMPLEAPP_P2P_CLUSTERID:

      break;  
      
    case SAMPLEAPP_FLASH_CLUSTERID:
      flashTime = BUILD_UINT16(pkt->cmd.Data[1], pkt->cmd.Data[2] );
      HalLedBlink( HAL_LED_4, 4, 50, (flashTime / 4) );
      break;
  }
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
void SampleApp_SendPeriodicMessage(uint8 ADR)
{
  uint8 data[]="D1";
  data[1]=ADR+0X30;
  if ( AF_DataRequest( &SampleApp_Periodic_DstAddr, &SampleApp_epDesc,
                       SAMPLEAPP_PERIODIC_CLUSTERID,
                       2,
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
/*************************************************************
���Ͷ˵�������
select��������Ϣ����  1������  2ͬ������
**************************************************************/
void SampleApp_Send_P2P_Message( uint8 select ) 
{ 
  uint8 i;                                        //ѭ������
  double TIMER[3],HUM[3],temp;                    //�¶ȡ�ʪ�Ȼ���������м����
  uint16 LIGHT[3],Temp;                           //���ջ���������м����
  uint8  strTemp[21]={0};                         //�ɼ����ݻ�������
  uint8  timerq[10]={0};                          //ͬ�����󻺴�����
  
//****************���ݼ���͸�ֵ**********************************************//
//* ���ݸ�ʽ��DA01T 24.32H64I  347@ --21λ
//* ÿ�����ݲɼ����Σ�ȡ�м�ֵ
//* ͬ���������ݸ�ʽ��TA01 --4λ
  osal_memcpy(strTemp,"D", 1);                   
  strTemp[1]=ADDR1;
  strTemp[2]=ADDR2;
  strTemp[3]=ADDR3;
  osal_memcpy(timerq,"T", 1);
  osal_memcpy(&timerq[1],&strTemp[1],3);
  if(select==0x01)
  {
//*******************��ʪ��****************************************
    for(i=0;i<3;i++)
    {
      SHT_DataRevise();
      TIMER[i]=temper;
      HUM[i]=humid;
      Delay_ms(10);
    }
    for(i=0;i<2;i++)
    {
      if(TIMER[i]>=TIMER[i+1])
      {
        temp=TIMER[i+1];
        TIMER[i+1]=TIMER[i];
        TIMER[i]=temp;
      }
      if(HUM[i]>=HUM[i+1])
      {
        temp=HUM[i+1];
        HUM[i+1]=HUM[i];
        HUM[i]=temp;
      }
    }
//***********************����****************************************
    for(i=0;i<3;i++)
    {
      lux();
      LIGHT[i]=Light;
      Delay_ms(200);
    }
    for(i=0;i<2;i++)
    {
      if(LIGHT[i]>=LIGHT[i+1])
      {
        Temp=LIGHT[i+1];
        LIGHT[i+1]=LIGHT[i];
        LIGHT[i]=Temp;
      }
    }
//*****************����ת��*******************************************   
//***************�¶�***************************
    if(TIMER[1]<0)
    {
      strTemp[5]='-';
      TIMER[1]=-TIMER[1];
    }
    else
    {
      strTemp[5]=' ';
    }
    Temp=(uint16)(TIMER[1]*100);
    Temp%=10000;
    strTemp[6]=Temp/1000+0x30;
    Temp%=1000;
    strTemp[7]=Temp/100+0x30;
    Temp%=100;
    strTemp[8]='.';
    strTemp[9]=Temp/10+0x30; 
    strTemp[10]=Temp%10+0x30;  
    
//***************ʪ��***************************    
    Temp=(uint16)HUM[1];
    if(Temp>=100)
    {
        Temp=99;
    }
    strTemp[12]=Temp/10+0x30; 
    strTemp[13]=Temp%10+0x30;

//********��ʪ�ȼ��***************************    
    if(SHTerror)
    {
        osal_memcpy(&strTemp[4],"T-99.99", 7);  //�¶ȴ�����ʾ
        osal_memcpy(&strTemp[11],"H-1", 3);     //ʪ�ȴ�����ʾ
    }
    else
    {
        osal_memcpy(&strTemp[4],"T", 1);
        osal_memcpy(&strTemp[11],"H", 1);
    }
    SHTerror=0;
    
//***************����**************************   
    Temp=LIGHT[1];
    strTemp[15]=Temp/10000+0x30;
    Temp%=10000;
    strTemp[16]=Temp/1000+0x30;
    Temp%=1000;
    strTemp[17]=Temp/100+0x30;
    Temp%=100;
    strTemp[18]=Temp/10+0x30;  
    strTemp[19]=Temp%10+0x30; 
    
//***********���ռ��*************************** 
    if(LUXerror)
    {
        osal_memcpy(&strTemp[14],"I65535", 6);
    }                                        
    else
    {
      osal_memcpy(&strTemp[14],"I", 1);
    }
    LUXerror=0;
    strTemp[20]='@';
//************��ȥ��������***********************
     if(strTemp[6]=='0')
     {
        strTemp[6]=' ';
     }
     if(strTemp[12]=='0')
     {
        strTemp[12]=' ';
     }
     for(i=0;i<4;i++)
     {
        if(strTemp[15+i]=='0')
        {
          strTemp[15+i]=' ';
        }
        else
        {
          break;
        }
     } 
//**************�������**************************
     osal_memset(TIMER,0,sizeof(TIMER));
     osal_memset(HUM,0,sizeof(HUM));
     osal_memset(LIGHT,0,sizeof(LIGHT));

//**************����******************************    
    if ( AF_DataRequest( &SampleApp_P2P_DstAddr, &SampleApp_epDesc, 
                          SAMPLEAPP_P2P_CLUSTERID, 
                          21, 
                          strTemp, 
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
//*****************����ʱ��ͬ������*******************************************//
  else if(select==0x02)
  {  
    if ( AF_DataRequest( &SampleApp_P2P_DstAddr, &SampleApp_epDesc, 
                          SAMPLEAPP_P2P_CLUSTERID, 
                          4, 
                          timerq, 
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
} 