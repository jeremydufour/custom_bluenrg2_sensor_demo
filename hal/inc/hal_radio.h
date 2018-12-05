/**
  ******************************************************************************
  * @file    hal_radio.h
  * @author  AMS - VMA RF Application team
  * @version V1.0.0
  * @date    17-July-2017
  * @brief   BlueNRG-1,2 HAL radio APIs 
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2015 STMicroelectronics</center></h2>
  ******************************************************************************
  */
#ifndef __HAL_RADIO_H_
#define __HAL_RADIO_H_

#include "BlueNRG1_radio.h"

/* Default state machine number */
#define STATE_MACHINE_NO        (uint8_t)(0)

uint8_t HAL_RADIO_SendPacket( uint8_t channel, 
                    uint32_t time, 
                    uint8_t time_type, 
                    uint8_t* txBuffer, 
                    uint8_t (*Callback)(ActionPacket*, ActionPacket*) );
                          
uint8_t HAL_RADIO_SendPacketWithAck(uint8_t channel, 
                                 uint32_t time, 
                                 uint8_t time_type, 
                                 uint8_t* txBuffer, 
                                 uint8_t* rxBuffer, 
                                 uint32_t receive_timeout,
                                 uint8_t (*Callback)(ActionPacket*, ActionPacket*) 
                                );
                                
uint8_t HAL_RADIO_ReceivePacket(uint8_t channel, 
                      uint32_t time, 
                      uint8_t time_type,
                      uint8_t* rxBuffer,                       
                      uint32_t receive_timeout, 
                      uint8_t timestamp_type,
                      uint8_t (*Callback)(ActionPacket*, ActionPacket*) );

uint8_t HAL_RADIO_ReceivePacketWithAck(uint8_t channel, 
                             uint32_t time,
                             uint8_t time_type, 
                             uint8_t* rxBuffer, 
                             uint8_t* txBuffer,
                             uint32_t receive_timeout, 
                             uint8_t timestamp_type,
                             uint8_t (*Callback)(ActionPacket*, ActionPacket*)
                             );
                             
uint8_t HAL_RADIO_SetNetworkID(uint32_t ID);


#endif /* __HAL_RADIO_H_ */
