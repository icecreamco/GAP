/**************************************************************************************************
  Filename:       SampleApp.h
  Revised:        $Date: 2007-10-27 17:22:23 -0700 (Sat, 27 Oct 2007) $
  Revision:       $Revision: 15795 $

  Description:    This file contains the Sample Application definitions.


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

#ifndef SAMPLEAPP_H
#define SAMPLEAPP_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */
#include "ZComDef.h"

/*********************************************************************
 * CONSTANTS
 */
extern uint8 powf;                  //进入睡眠标志--外部
extern uint32 SleepTime_MS;         //每次睡眠的时间--外部
extern uint8 TX_Flag;               //发送数据标志位--外部
extern uint8 TCount;                //最大睡眠时间的次数--外部
extern uint32 SleepTime;            //总睡眠时间--外部   
extern uint8 CountF;                //计算睡眠时间标志位--外部
extern uint8 Mode;                  //节点工作模式  C：正常  M：调试--外部

// These constants are only for example and should be changed to the
// device's needs
#define SAMPLEAPP_ENDPOINT           20

#define SAMPLEAPP_PROFID             0x0F08
#define SAMPLEAPP_DEVICEID           0x0001
#define SAMPLEAPP_DEVICE_VERSION     0
#define SAMPLEAPP_FLAGS              0

#define SAMPLEAPP_MAX_CLUSTERS       2
#define SAMPLEAPP_PERIODIC_CLUSTERID 1
#define SAMPLEAPP_FLASH_CLUSTERID     2
#define SAMPLEAPP_P2P_CLUSTERID       4

#define SWLED P1_1                   //定义P1_1为LED开关
#define Delaybet 3300                //定义基础延时时间为3.3S

//************************地址和延时定义**************************************//
#if defined endA01
    #define delayt Delaybet*1
    #define ADDR1 'A'
    #define ADDR2 '0'
    #define ADDR3 '1'
#elif defined endA02
    #define delayt Delaybet*2
    #define ADDR1 'A'
    #define ADDR2 '0'
    #define ADDR3 '2'
#elif defined endA03
    #define delayt Delaybet*3
    #define ADDR1 'A'
    #define ADDR2 '0'
    #define ADDR3 '3'
#elif defined endA04
    #define delayt Delaybet*4
    #define ADDR1 'A'
    #define ADDR2 '0'
    #define ADDR3 '4'
#elif defined endA05
    #define delayt Delaybet*5
    #define ADDR1 'A'
    #define ADDR2 '0'
    #define ADDR3 '5'
#elif defined endA06
    #define delayt Delaybet*6
    #define ADDR1 'A'
    #define ADDR2 '0'
    #define ADDR3 '6'
#elif defined endA07
    #define delayt Delaybet*7
    #define ADDR1 'A'
    #define ADDR2 '0'
    #define ADDR3 '7'
#elif defined endA08
    #define delayt Delaybet*8
    #define ADDR1 'A'
    #define ADDR2 '0'
    #define ADDR3 '8'
#elif defined endA09
    #define delayt Delaybet*9
    #define ADDR1 'A'
    #define ADDR2 '0'
    #define ADDR3 '9'
#elif defined endA10
    #define delayt Delaybet*10
    #define ADDR1 'A'
    #define ADDR2 '1'
    #define ADDR3 '0'
#elif defined endB01
    #define delayt Delaybet*11
    #define ADDR1 'B'
    #define ADDR2 '0'
    #define ADDR3 '1'
#elif defined endB02
    #define delayt Delaybet*12
    #define ADDR1 'B'
    #define ADDR2 '0'
    #define ADDR3 '2'
#elif defined endB03
    #define delayt Delaybet*13
    #define ADDR1 'B'
    #define ADDR2 '0'
    #define ADDR3 '3'
#elif defined endB04
    #define delayt Delaybet*14
    #define ADDR1 'B'
    #define ADDR2 '0'
    #define ADDR3 '4'
#elif defined endB05
    #define delayt Delaybet*15
    #define ADDR1 'B'
    #define ADDR2 '0'
    #define ADDR3 '5'
#elif defined endB06
    #define delayt Delaybet*16
    #define ADDR1 'B'
    #define ADDR2 '0'
    #define ADDR3 '6'
#elif defined endB07
    #define delayt Delaybet*17
    #define ADDR1 'B'
    #define ADDR2 '0'
    #define ADDR3 '7'
#elif defined endB08
    #define delayt Delaybet*18
    #define ADDR1 'B'
    #define ADDR2 '0'
    #define ADDR3 '8'
#elif defined endB09
    #define delayt Delaybet*19
    #define ADDR1 'B'
    #define ADDR2 '0'
    #define ADDR3 '9'
#elif defined endB10
    #define delayt Delaybet*20
    #define ADDR1 'B'
    #define ADDR2 '1'
    #define ADDR3 '0'
#elif defined endC01
    #define delayt Delaybet*21
    #define ADDR1 'C'
    #define ADDR2 '0'
    #define ADDR3 '1'
#elif defined endC02
    #define delayt Delaybet*22
    #define ADDR1 'C'
    #define ADDR2 '0'
    #define ADDR3 '2'
#elif defined endC03
    #define delayt Delaybet*23
    #define ADDR1 'C'
    #define ADDR2 '0'
    #define ADDR3 '3'
#elif defined endC04
    #define delayt Delaybet*24
    #define ADDR1 'C'
    #define ADDR2 '0'
    #define ADDR3 '4'
#elif defined endC05
    #define delayt Delaybet*25
    #define ADDR1 'C'
    #define ADDR2 '0'
    #define ADDR3 '5'
#elif defined endC06
    #define delayt Delaybet*26
    #define ADDR1 'C'
    #define ADDR2 '0'
    #define ADDR3 '6'
#elif defined endC07
    #define delayt Delaybet*27
    #define ADDR1 'C'
    #define ADDR2 '0'
    #define ADDR3 '7'
#elif defined endC08
    #define delayt Delaybet*28
    #define ADDR1 'C'
    #define ADDR2 '0'
    #define ADDR3 '8'
#elif defined endC09
    #define delayt Delaybet*29
    #define ADDR1 'C'
    #define ADDR2 '0'
    #define ADDR3 '9'
#elif defined endC10
    #define delayt Delaybet*30
    #define ADDR1 'C'
    #define ADDR2 '1'
    #define ADDR3 '0'
#elif defined endC13
    #define delayt Delaybet*33
    #define ADDR1 'C'
    #define ADDR2 '1'
    #define ADDR3 '3'
#endif 

// Send Message Timeout
#define SAMPLEAPP_SEND_PERIODIC_MSG_TIMEOUT  5000     // Every 5 seconds

// Application Events (OSAL) - These are bit weighted definitions.
#define SAMPLEAPP_SEND_PERIODIC_MSG_EVT       0x0001
  
// Group ID for Flash Command
#define SAMPLEAPP_FLASH_GROUP                  0x0001
  
// Flash Command Duration - in milliseconds
#define SAMPLEAPP_FLASH_DURATION               1000

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * FUNCTIONS
 */

/*
 * Task Initialization for the Generic Application
 */
extern void SampleApp_Init( uint8 task_id );

/*
 * Task Event Processor for the Generic Application
 */
extern UINT16 SampleApp_ProcessEvent( uint8 task_id, uint16 events );

/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* SAMPLEAPP_H */
