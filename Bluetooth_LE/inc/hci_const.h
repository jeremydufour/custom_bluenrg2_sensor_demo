/**
  ******************************************************************************
  * @file    hci_const.h
  * @author  AMS - VMA RF Application team
  * @version V1.0.0
  * @date    21-Sept-2015
  * @brief   This file defines constants and functions for HCI layer.
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
#ifndef __HCI_INTERNAL_H_
#define __HCI_INTERNAL_H_

/*** Data types ***/
/**
 * @defgroup HCI_Error_codes HCI Error codes
 * @{
 */
#define HCI_UNKNOWN_COMMAND				0x01
#define HCI_NO_CONNECTION				0x02
#define HCI_HARDWARE_FAILURE				0x03
#define HCI_PAGE_TIMEOUT				0x04
#define HCI_AUTHENTICATION_FAILURE			0x05
#define HCI_PIN_OR_KEY_MISSING				0x06
#define HCI_MEMORY_FULL					0x07
#define HCI_CONNECTION_TIMEOUT				0x08
#define HCI_MAX_NUMBER_OF_CONNECTIONS	         	0x09
#define HCI_MAX_NUMBER_OF_SCO_CONNECTIONS         	0x0a
#define HCI_ACL_CONNECTION_EXISTS			0x0b
#define HCI_COMMAND_DISALLOWED				0x0c
#define HCI_REJECTED_LIMITED_RESOURCES	        	0x0d
#define HCI_REJECTED_SECURITY				0x0e
#define HCI_REJECTED_PERSONAL				0x0f
#define HCI_HOST_TIMEOUT			       	0x10
#define HCI_UNSUPPORTED_FEATURE				0x11
#define HCI_INVALID_PARAMETERS				0x12
#define HCI_OE_USER_ENDED_CONNECTION	        	0x13
#define HCI_OE_LOW_RESOURCES				0x14
#define HCI_OE_POWER_OFF			       	0x15
#define HCI_CONNECTION_TERMINATED			0x16
#define HCI_REPEATED_ATTEMPTS				0x17
#define HCI_PAIRING_NOT_ALLOWED				0x18
#define HCI_UNKNOWN_LMP_PDU			       	0x19
#define HCI_UNSUPPORTED_REMOTE_FEATURE	        	0x1a
#define HCI_SCO_OFFSET_REJECTED				0x1b
#define HCI_SCO_INTERVAL_REJECTED			0x1c
#define HCI_AIR_MODE_REJECTED				0x1d
#define HCI_INVALID_LMP_PARAMETERS			0x1e
#define HCI_UNSPECIFIED_ERROR				0x1f
#define HCI_UNSUPPORTED_LMP_PARAMETER_VALUE     	0x20
#define HCI_ROLE_CHANGE_NOT_ALLOWED			0x21
#define HCI_LMP_RESPONSE_TIMEOUT			0x22
#define HCI_LMP_ERROR_TRANSACTION_COLLISION	        0x23
#define HCI_LMP_PDU_NOT_ALLOWED				0x24
#define HCI_ENCRYPTION_MODE_NOT_ACCEPTED         	0x25
#define HCI_UNIT_LINK_KEY_USED				0x26
#define HCI_QOS_NOT_SUPPORTED				0x27
#define HCI_INSTANT_PASSED			        0x28
#define HCI_PAIRING_NOT_SUPPORTED			0x29
#define HCI_TRANSACTION_COLLISION			0x2a
#define HCI_QOS_UNACCEPTABLE_PARAMETER	         	0x2c
#define HCI_QOS_REJECTED			       	0x2d
#define HCI_CLASSIFICATION_NOT_SUPPORTED        	0x2e
#define HCI_INSUFFICIENT_SECURITY			0x2f
#define HCI_PARAMETER_OUT_OF_RANGE			0x30
#define HCI_ROLE_SWITCH_PENDING				0x32
#define HCI_SLOT_VIOLATION			       	0x34
#define HCI_ROLE_SWITCH_FAILED				0x35
#define HCI_EIR_TOO_LARGE			       	0x36
#define HCI_SIMPLE_PAIRING_NOT_SUPPORTED        	0x37
#define HCI_HOST_BUSY_PAIRING				0x38
#define HCI_CONN_REJ_NO_CH_FOUND			0x39
#define HCI_CONTROLLER_BUSY			       	0x3A
#define HCI_UNACCEPTABLE_CONN_INTERV	        	0x3B
#define HCI_DIRECTED_ADV_TIMEOUT			0x3C
#define HCI_CONN_TERM_MIC_FAIL				0x3D
#define HCI_CONN_FAIL_TO_BE_ESTABL			0x3E
#define HCI_MAC_CONN_FAILED			       	0x3F
/**
 * @}
 */

#endif /* __HCI_INTERNAL_H_ */
