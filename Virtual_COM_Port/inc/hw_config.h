/**
  ******************************************************************************
  * @file    hw_config.h
  * @author  MCD Application Team
  * @version V4.0.0
  * @date    21-January-2013
  * @brief   Hardware Configuration & Setup
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2013 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */


/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __HW_CONFIG_H
#define __HW_CONFIG_H

/* Includes ------------------------------------------------------------------*/
#include "platform_config.h"
#include "usb_type.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
#define  JETDRIVER_STM32_REV			(__DATE__ "  "  __TIME__)
/* Exported macro ------------------------------------------------------------*/
/* Exported define -----------------------------------------------------------*/
#define MASS_MEMORY_START     0x04002000
#define BULK_MAX_PACKET_SIZE  0x00000040
#define LED_ON                0xF0
#define LED_OFF               0xFF

#define USART_RX_DATA_SIZE   2048
/* Exported functions ------------------------------------------------------- */
void Set_System(void);
void Set_USBClock(void);
void Enter_LowPowerMode(void);
void Leave_LowPowerMode(void);
void USB_Interrupts_Config(void);
void USB_Cable_Config (FunctionalState NewState);
void USART_Config_Default(void);
bool USART_Config(void);
void USB_To_USART_Send_Data(uint8_t* data_buffer, uint8_t Nb_bytes);
void USART_To_USB_Send_Data(void);
void Handle_USBAsynchXfer (void);
void Get_SerialNum(void);


/* External variables --------------------------------------------------------*/
void TM_DelayMicros(uint32_t micros) ;
void swathCounterHandle(void);
void VHPup(int j);
void VHPdown(int j);
void showHelp(void);
void showHelpRPI(void);
void youCouldShow(void);
void abortShow(void);
void showPrintingStatus(void);

//uint16_t ParsedID(uint8_t *pbuf, uint8_t *errorFlag);
uint16_t ParsedID(uint8_t *pbuf, uint8_t len);
void   VCOMprint(const  char  *pbuf, int whichmessage) ;

struct dropletRec {
//  boolean         on_off;
  int             FireFrequency;         // use the formula: 16MHz/FireFrequency to get the timeConst
  int             Nozzle;                // selected which nozzle.
  int             PulseWidth;
  int             Delay4LED;
  int   					Nozzle2Fire;
	int						 quite;									// do not send swath-data up to Host.
//  unsigned char   ModeOperation;         // under Mode_Firing: DownCount, Free-run
  long            dropCounts;              		//  elapse counting it  
} ;  

struct PrintHead {                           /* structure of the PrintHead information record     */
  //int             	        firePeriod;         	// use the formula: 
  int											VH_Volt;						// VH volt digits  0 ~ 31
	int											pulseWidth;				//  10(0.75us)  ~    28 ( 2.1us)   ~ 37 (2.995 us)
	int											dwell;								// timed dewell (delay time for non-jetting)
  uint16_t							ID;
	uint16_t             encoder;
  int											swathCounter	;
	int											VHon;
	int										 quite;									// do not send swath-data up to Host.
};
#define  dwell_MIN								26
// minima is 26us.
#define  dwell_MAX							0xFFF
// maxima is  4095us
#define  pulseWidth_MIN		   0x0A
#define  pulseWidth_MAX     0x25
// 28 (decimal) is around 2.1us
//  ==> calculated as 20.8ns *4 * (28-1) ~= 2.246us
// guess 37 is enough for around 
//  ==> calculated as 20.8ns *4 * (37-1) ~=  2.995us
// so  0x0A = 10 (decimal).
// ==> calculated as 20.8ns *4 * (10-1) ~=  0.75us

#define SDOA_PB12_Pin GPIO_PIN_12
#define SDOA_PB12_GPIO_Port GPIOB
#define SCKA_PB13_Pin GPIO_PIN_13
#define SCKA_PB13_GPIO_Port GPIOB
#define LOADA_PB15_Pin GPIO_PIN_15
#define LOADA_PB15_GPIO_Port GPIOB
#define LOADP_PC12_Pin GPIO_PIN_12
#define LOADP_PC12_GPIO_Port GPIOC
#define SCKP_PB3_Pin GPIO_PIN_3
#define SCKP_PB3_GPIO_Port GPIOB
#define SDOP_PB5_Pin GPIO_PIN_5
#define SDOP_PB5_GPIO_Port GPIOB

//  Host to STM32
#define ESC_KEY1			'\t'		// CTRL-I
#define ESC_KEY2     '!'
#define ESC_KEY3     '@'

#define FSM_DEFault               0
#define FSM_KEYCheck					1
#define FSM_STM32store					2
#define FSM_STM32fetch				  3

//  Slave to STM32
#define PAT_KEY1			'\v'			// CTRL-K
#define PAT_KEY2			'!'
#define PAT_KEY3      '@'

#define RFSM_DEFault							0
#define RFSM_KEYCheck			  1
#define RFSM_STM32store	    2
#define RFSM_STM32fetch      3

#define     VHP_ON_PC5		GPIO_Pin_5		// High enable.
#define		VHP_ON          	 GPIOC->BSRR = VHP_ON_PC5;
#define     VHP_OFF				   GPIOC->BRR = VHP_ON_PC5;

#define    PARSER_DATA_SIZE    (2*22*2+16)
 
#define	   FIREn_PA5					GPIO_PIN_5		// Low enable.
#define     FIRE_OFF						GPIOA->BSRR = FIREn_PA5	// set to High
#define     FIRE_ON       				GPIOA->BRR = FIREn_PA5


#define	   TSRC_PC2						GPIO_PIN_2			// High enable.
#define	   TSRC_ON						HAL_GPIO_WritePin(GPIOC, TSRC_PC2, GPIO_PIN_SET)
#define	   TSRC_OFF						HAL_GPIO_WritePin(GPIOC, TSRC_PC2, GPIO_PIN_RESET)

#define      ID_Fusing_PC3					GPIO_PIN_3		// High enable
#define	   ID_Fusing_ON						HAL_GPIO_WritePin(GPIOC, ID_Fusing_PC3, GPIO_PIN_SET)
#define	   ID_Fusing_OFF					HAL_GPIO_WritePin(GPIOC, ID_Fusing_PC3, GPIO_PIN_RESET)

#define      UD_PC6									GPIO_PIN_6		//  UD=L => Down, UD=H => Up
#define	   UD_UP											 GPIOC->BSRR = UD_PC6
#define	   UD_DOWN							 GPIOC->BRR  =  UD_PC6


#define      INC_PC7									GPIO_PIN_7	//  A pulse here to increase
#define	   INC_H										  GPIOC->BSRR = INC_PC7 
#define	   INC_L        							GPIOC->BRR   = INC_PC7 

#define      CS0_PA8									GPIO_PIN_8		// Low enable
#define	   CS0_ON										GPIOA->BRR = CS0_PA8 
#define	   CS0_OFF     							GPIOA->BSRR=CS0_PA8 


#define     ENC_ENA_PA3				GPIO_PIN_3		// Low enable.
#define     ENC_OFF                 GPIOA->BSRR = ENC_ENA_PA3 
#define     ENC_ON                  GPIOA->BRR   = ENC_ENA_PA3

// =======================  ADDRESS PINS
#define	   LOADA_PB15          GPIO_PIN_15
#define	   LOADA_H								GPIOB->BSRR =  LOADA_PB15 
#define      LOADA_L								GPIOB -> BRR =  LOADA_PB15 

#define	    SCKA_PB13            GPIO_PIN_13
#define	    SCKA_H								  GPIOB-> BSRR = SCKA_PB13 
#define      SCKA_L								  GPIOB->   BRR = SCKA_PB13 

#define	   SDOA_PB12           GPIO_PIN_12
#define	   SDOA_H								 GPIOB-> BSRR = SDOA_PB12 
#define      SDOA_L								 GPIOB-> BRR = SDOA_PB12 

// =======================  POWER PINS
#define	   LOADP_PC12           GPIO_PIN_12
#define	   LOADP_H								 GPIOC-> BSRR = LOADP_PC12 
#define      LOADP_L								 GPIOC-> BRR  = LOADP_PC12 

#define	    SCKP_PB3              GPIO_PIN_3
#define	    SCKP_H								  GPIOB-> BSRR = SCKP_PB3 
#define      SCKP_L								   GPIOB-> BRR = SCKP_PB3 

#define	   SDOP_PB5              GPIO_PIN_5
#define	   SDOP_H								   GPIOB-> BSRR = SDOP_PB5 
#define      SDOP_L								 GPIOB-> BRR = SDOP_PB5 

#define			PH_POWER_PINS  			14
#define		   PH_ADDRESS_PINS  	22

#define bufferWrite_MAX			(44*100*3 )
// the  real-max:
// #define bufferWrite_MAX			(44*100*3 +1024*3)
	
extern  int	fsm_parsed;
extern  int  rfsm_parsed;
extern  int  notINfsm;
extern  int  notINrfsm;
extern  uint8_t accumulated_string[];
extern uint8_t acc_string[];
extern  int  rcv_index;
extern  int  rrcv_index;
extern 	int  stopFire;
extern  struct PrintHead printHead;
extern  const char  ToHostmenu[] ;
extern  const char ToRPImenu[] ;
extern  const char ToRPImessage[] ;
extern  const char ToRPIabort[] ;
extern  const char  ToHostStatus[] ;


extern	int bufferFull;
extern	int bufferRead_idx;
extern	int bufferWrite_idx;
extern  int  insideSwath;
extern  int nibbleToggle;		// for hex digit nibbles for receiving.
extern  int  start2print;	// when bufferWrite_idx is Okay.
extern  uint8_t  myPATTERN  [];
extern  int map001go;
extern  int map002go;

#endif  /*__HW_CONFIG_H*/
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
