/**
  ******************************************************************************
  * @file    main.c
  * @author  MCD Application Team
  * @version V4.0.0
  * @date    21-January-2013
  * @brief   Virtual Com Port Demo main file
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


/* Includes ------------------------------------------------------------------*/
#include "hw_config.h"
#include "usb_lib.h"
#include "usb_desc.h"
#include "usb_pwr.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
int notINfsm=0;			// FSM flag for host->slave (PC/Host -> Rpi)
int notINrfsm=0;		// FSM flag for slave->host (Rpi-> PC/host)
 
 int	fsm_parsed =FSM_DEFault;
 int  rfsm_parsed= RFSM_DEFault;

 	uint8_t accumulated_string[PARSER_DATA_SIZE];
  int  rcv_index;
 	uint8_t acc_string[PARSER_DATA_SIZE];
	int  rrcv_index;		// incoming index for Rpi
	
	#define magicNum				97			
	// it is the distance of EVEN/ODD nozzles.
	// when the buffer filled-up (97 records/swath).
	// printing could go.
	

	uint8_t  myPATTERN  [bufferWrite_MAX+1];
	// it(host) sends 44 bytes per-swath, fill-up 97 buffers then start print.
	//
	int bufferFull=0;
	int bufferRead_idx=0;
	int bufferWrite_idx=0;
	int nibbleToggle=0;
	int insideSwath=0;
	
	int map001go=0;

  int map002go=0;
	
	// if bufferWrite_idx> magicNum, then bufferFull=1.
	// starts to print.
	// when printing, it reads/consumes bufferRead_idx
 // when bufferWrite_idx> bufferMAX, it wraps. (assign bufferWrite_idx =0)
 //  when bufferRead_idx = bufferWrite_idx.  it stops printing.

	int  start2print=0;	// when bufferWrite_idx is Okay.
	
	int  stopFire=1;	// initially not to fire.
  struct PrintHead  printHead;

/* Extern variables ----------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void VHPup(int j) ;
void VHPdown(int j) ;
void TM_Delay_Init(void) ;
void TM_DelayMicros(uint32_t micros) ;
void resetAddress(void);
void  allsetAddress(void);
void  set1A(void);

void  shift1A(void);
void change2nextAddress(void);
void change2nextPower(void);

void  resetPower(void);
void  allsetPower(void);
void  set1P(void);

/* USER CODE END PFP */


void showHelp(void){
	//int k= sizeof(ToHostmenu);
	VCOMprint(&ToHostmenu[0] , 0);
}
void showHelpRPI(void){
	VCOMprint(&ToRPImenu[0], 1);	//sizeof(ToRPImenu));
}
void youCouldShow(void){
	VCOMprint(&ToRPImessage[0], 2);	
}
void abortShow(void){
	VCOMprint(&ToRPIabort[0], 3);	
}
void showPrintingStatus(void){
	VCOMprint(&ToHostStatus[0], 4);		
}


void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  RCC->APB2ENR |= RCC_APB2Periph_GPIOA;	
	RCC->APB2ENR |= RCC_APB2Periph_GPIOB;	
	RCC->APB2ENR |= RCC_APB2Periph_GPIOC;	
	RCC->APB2ENR |= RCC_APB2Periph_GPIOD;	
	
  /*Configure GPIO pins : PC13 PC14 PC15 PC0 
                           PC1 PC8 PC9 PC10 
                           PC11  */
  GPIO_InitStruct.GPIO_Pin = GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_0 
                          |GPIO_PIN_1|GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10 
                          |GPIO_PIN_11;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AIN;	
  GPIO_Init(GPIOC, &GPIO_InitStruct);

	
  /*Configure GPIO pins : PC2 PC3 PC4 PC5 
                           PC6 PC7 */
  GPIO_InitStruct.GPIO_Pin = GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5 
                          |GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA1 PA4 PA15 */
  GPIO_InitStruct.GPIO_Pin = GPIO_PIN_1|GPIO_PIN_4|GPIO_PIN_15;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AIN;
  GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA2 PA3 PA5 PA8 */
  GPIO_InitStruct.GPIO_Pin = GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_5|GPIO_PIN_8;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB2 PB10 
                           PB11 PB14 PB4 */
  GPIO_InitStruct.GPIO_Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_10 
                          |GPIO_PIN_11|GPIO_PIN_14|GPIO_PIN_4;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AIN;
  GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : SDOA_PB12_Pin SCKA_PB13_Pin LOADA_PB15_Pin SCKP_PB3_Pin 
                           SDOP_PB5_Pin */
  GPIO_InitStruct.GPIO_Pin = SDOA_PB12_Pin|SCKA_PB13_Pin|LOADA_PB15_Pin|SCKP_PB3_Pin 
                          |SDOP_PB5_Pin;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : LOADP_PC12_Pin */
  GPIO_InitStruct.GPIO_Pin = LOADP_PC12_Pin;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
   GPIO_Init(LOADP_PC12_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PD2 */
  GPIO_InitStruct.GPIO_Pin = GPIO_PIN_2;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AIN;
   GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin Output Level */
  GPIO_ResetBits(GPIOC, GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5 
                          |GPIO_PIN_6|GPIO_PIN_7|LOADP_PC12_Pin );

  /*Configure GPIO pin Output Level */
  GPIO_ResetBits(GPIOA, GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_5|GPIO_PIN_8);

  /*Configure GPIO pin Output Level */
  GPIO_ResetBits(GPIOB, SDOA_PB12_Pin|SCKA_PB13_Pin|LOADA_PB15_Pin|SCKP_PB3_Pin 
                          |SDOP_PB5_Pin); 

}

/*******************************************************************************
* Function Name  : printHead Init.
* Description    :  Initialization subroutine.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void printHead_Init(void){
	
	  FIRE_OFF;	// Has to. if following VHP_ON.
   VHP_ON;
	 
	 printHead.pulseWidth = 28;	// my initial value for #626 Ink. Aquo version. (non-solvent.)
	 printHead.VH_Volt = 14; 		// about 10.5 Volt.
	 printHead.dwell = 500;				// 500 us dewell
   printHead.ID= 0xFFF;								// not programmed yet
	 printHead.encoder = 0x0000; 	// not programmed yet
	 printHead.swathCounter =0;			// so you could enter command from Rpi also.
	printHead.quite=0;	// not in quite mode (initially.)
	
	// normally Rpi send data-chunk to MCU (STM32) to DemoPrinting.
	// but later after DemoPrining (time), the Rpi will directly drive the whole driving circuit to generate printing.


  VHPdown(32);
	TM_DelayMicros(500);
	VHPup(printHead.VH_Volt );
	TM_DelayMicros(500000);		// 500ms, 0.5 second for VHP raising
}
/*******************************************************************************
* Function Name  : main.
* Description    : Main routine.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
int main(void)
{
  /* USER CODE BEGIN 1 */
 unsigned int bitmap_value ;
 //unsigned int  *thatPtr = 0x000C004;
 //bitmap_value = *thatPtr;
	
	int 	tick4address=0;
	int  tick4power=0;
//	int  tick2fire=0;
	
  uint16_t *loadPword =0x0000C004;
	
	int k;
  /* USER CODE END 1 */
//	 int i;

	
  Set_System();
	MX_GPIO_Init();
	
	RCC->APB2ENR |= RCC_APB2Periph_GPIOA;	
	/*-- GPIO Mode Configuration速度，輸入或輸出 -----------------------*/
	/*-- GPIO CRL Configuration 設置IO端口低8位的模式（輸入還是輸出）---*/
	/*-- GPIO CRH Configuration 設置IO端口高8位的模式（輸入還是輸出）---*/
	GPIOA->CRL &=  0xFFFF00FF;
	GPIOA->CRL  |=   0x00003300;

  Set_USBClock();
  USB_Interrupts_Config();
  USB_Init();
  
	 GPIOA->BSRR = GPIO_Pin_2 ;		// PA2 high, to ensure VBUS pull-H
	 GPIOA->BRR = GPIO_Pin_3 ;

   TM_Delay_Init();

  //VHP_OFF;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
		ENC_OFF;
		TM_DelayMicros(1);
		ENC_ON;
		//TM_DelayMicros(500);		// 500 us.. aka 2KHz.   if firing
		TM_DelayMicros(100000);
		
	printHead_Init();	// it has to
	
  resetAddress();	// clear the shift-register.  also it has <mreset> hardware reset. Even FIRE_ON can't change anything.
	 set1A(); // turn on 1A
	 set1P();	// turn on 1 P

  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

/*  Horse running
  if startprint==1, then load buffer to P,  provide a narrow-pulse.
	
*/
 if (1==start2print)      // swath enough for print.
 {
	 tick4address=22;
	 resetAddress();
	 
	while(tick4address--) {
			shift1A();
// loadp()
	LOADP_L;
		//   original myPATTERN  (RAM)
   //		loadPword = (uint16_t *) myPATTERN[bufferRead_idx];
		
		loadPword = (uint16_t *) myPATTERN[bufferRead_idx];
		
		  for (tick4power=0; tick4power<14; tick4power++){
				if  (0==(*loadPword && (1<<tick4power)))  
							SDOP_H;			
				else SDOP_L;
			SCKP_H;
		 //k=0x05; while(k--) ;
			SCKP_L;
			}
	LOADP_H; // strobe in.
			bufferRead_idx+=2;	// increament to next pointer.
			if (bufferRead_idx > bufferWrite_MAX) 
				bufferRead_idx =0;

	 // now, provide the Fire pulse
		  if (0==stopFire){
				k= printHead.pulseWidth;			///## pulseWidth  is HERE !!
					__disable_irq();
					FIRE_ON;
					while(k--);
					FIRE_OFF;
				__enable_irq();
			}
	 // loadP();
	 // narrow-pulse-POWER
	 // if (1==stopFire) {
	 // something has to be reset.
		// also break the loop
	 // else
	 //   nextAddress ?
	  //change2nextAddress();
   }
		if (0!=printHead.swathCounter) {
			   printHead.swathCounter--;
		} else {
			start2print=0;	// so. I am free again.
		}
	 // after 22 address fired.
	 // then dwell time delayed.
		if (printHead.dwell > (dwell_MIN-1))
				TM_DelayMicros(printHead.dwell);		//## dwell  is HERE !!
		else
				TM_DelayMicros(1000);		// in-case  HOST issue wrong parameter. PH may burn-out.
 } else {
	  if  (map001go>0) {
				TM_DelayMicros(1000);		// in-case  HOST issue wrong parameter. PH may burn-out.
		 		//showPrintingStatus();
	 tick4address=22;
	 resetAddress();
	 set1A();
		 // loadp()  // all P  turns on (0 = turn on)

		LOADP_L;
		  for (tick4power=0; tick4power<14; tick4power++){
							SDOP_L;			
					k=0x05; while(k--) ;
		SCKP_H;
					k=0x05; while(k--) ;
				SCKP_L;
			}
	LOADP_H; // strobe in.
	do {
		/* signal for oberservation
		ENC_OFF;
		TM_DelayMicros(1);
		ENC_ON;
			*/
			if (0==stopFire){
				k= printHead.pulseWidth;			///## pulseWidth  is HERE !!
					__disable_irq();
					FIRE_ON;
					while(k--);
					FIRE_OFF;
				__enable_irq();
			}		
			//k=0x05; while(k--) ;	// prevent the latch-up issue
			change2nextAddress();
		} while(--tick4address) ;
	
		
			// MILAGE  check.  segment.start  if fin then END
			map001go--;
			if (0==map001go) {
				//showPrintingStatus();
				stopFire=1;
				resetAddress();
				//VHP_OFF;	// job done indicator
			}
			if (0==(map001go%128)){
				//showPrintingStatus();
			}
			// MILAGE check. segment.end
		} 
   	 if (map002go>0) {
			 
		if (printHead.dwell > (dwell_MIN-1))
				TM_DelayMicros(printHead.dwell);		//## dwell  is HERE !!
		else
				TM_DelayMicros(1000);		// in-case  HOST issue wrong parameter. PH may burn-out.
	 tick4address=22;
	 resetAddress();
	 set1A();
												
	do {
			
		LOADP_L;
		  loadPword = (uint16_t *) myPATTERN[ 2*(22-tick4address)];
			for (tick4power=0; tick4power<14; tick4power++){
						if  (0==(*loadPword && (1<<tick4power))) 
								SDOP_H;
						else
							 SDOP_L;	// 1 => turn-on,  L to turn on.
						
			 		k=0x05; while(k--) ;
				SCKP_H;
					k=0x05; while(k--) ;
				SCKP_L;
				}
	LOADP_H; // strobe in.
	
			if (0==stopFire){
				k= printHead.pulseWidth;			///## pulseWidth  is HERE !!
					__disable_irq();
					FIRE_ON;
					while(k--);
					FIRE_OFF;
				__enable_irq();
			}		
			change2nextAddress();
		} while(--tick4address) ;
	
		
			// MILAGE  check.  segment.start  if fin then END
			if (0==map002go--) {
				stopFire=1;
				resetAddress();
			}
			// MILAGE check. segment.end
		} 
  }
}
	
  /* USER CODE END 3 */
}
/* USER CODE BEGIN 4 */

#define HAL_Delay(x)		TM_DelayMicros(x)


void VHPup(int j){
UD_UP;   	INC_H;		// initial condition
CS0_ON;
	 while(j--){
		 INC_L;
		 HAL_Delay(1);	// better to have
		 INC_H;
		 HAL_Delay(1);
	 }
	CS0_OFF;	// disable
}
void VHPdown(int j){
UD_DOWN;   	INC_H;		// initial condition
CS0_ON;
	 while(j--){
		 INC_L;
		 HAL_Delay(1);	// better to have
		 INC_H;
		 HAL_Delay(1);
	 }
	CS0_OFF;	// disable	
}
uint32_t multiplier;
 
void TM_Delay_Init(void) {
    ///RCC_ClocksTypeDef RCC_Clocks;
    
    /* Get system clocks */
    ///RCC_GetClocksFreq(&RCC_Clocks);
    
    /* While loop takes 4 cycles */
    /* For 1 us delay, we need to divide with 4M */
    multiplier = (SystemCoreClock / 4000000 *2/3);	// program in flash. only up to 48MHz.
}
 
void TM_DelayMicros(uint32_t micros) {
    /* Multiply micros with multipler */
    /* Substract 10 */
    micros = micros * multiplier - 10;
    /* 4 cycles for one loop */
    while (micros--);
}

void change2nextAddress(void){
	uint32_t k;
	LOADA_L;  //initial condition.// 20160831(Sakura, found this forgotten
  SDOA_L;		 // prepared data =0
		 k=0x03; while(k--) ;	// 20160831(Sakura, found this forgotte.  from LOADA_L, it takes a little
	SCKA_H;
	//	 k=0x05; while(k--) ;	// 20160831 (Sakura, removed. because L>H triggered to VHC595.
	SCKA_L;
  LOADA_H;	// strobe in.
}
void resetAddress(void){
	uint32_t i=PH_ADDRESS_PINS;
	uint32_t k;
	FIRE_OFF;		// make sure it is.
	LOADA_L;		// initial condition
	SCKA_L;				// initial condition
  SDOA_L;		 // prepared data =0
while	(i--){	
		 k=0x05; while(k--) ;
	SCKA_H;
		 k=0x05; while(k--) ;
	SCKA_L;
}
  LOADA_H;	// strobe in.
}
void  allsetAddress(void){
	uint32_t i=PH_ADDRESS_PINS;
	uint32_t k;
	FIRE_OFF;		// make sure it is.
	LOADA_L;		// initial condition
	SCKA_L;				// initial condition
  SDOA_H;		 // prepared data =1
while	(i--){
		 k=0x05; while(k--) ;
	SCKA_H;
		 k=0x05; while(k--) ;
	SCKA_L;
}
  LOADA_H;	// strobe in.
}
void  set1A(void){
	uint32_t k;
	FIRE_OFF;		// make sure it is.
	resetAddress();	// first of all, clear all
	LOADA_L;		// initial condition
	SCKA_L;				// initial condition
  SDOA_H;		 // prepared data =1
		 k=0x06; while(k--) ;
	SCKA_H;
		 k=0x04; while(k--) ;
	SCKA_L;
  LOADA_H;	// strobe in.
}
void  shift1A(void){
	uint32_t k;
	//FIRE_OFF;		// make sure it is.
	LOADA_L;		// initial condition
	SCKA_L;				// initial condition
  SDOA_L;		 // prepared data =0		
			k=0x06; while(k--) ;
	SCKA_H;
		 k=0x04; while(k--) ;
	SCKA_L;
  LOADA_H;	// strobe in.
}

void change2nextPower(void){
	uint32_t k;
	LOADP_L;
	SDOP_H;			//  prepared data=1, only shit 1 bit
		 k=0x05; while(k--) ;
	SCKP_H;
		 k=0x04; while(k--) ;
	SCKP_L;
	LOADP_H; // strobe in.
}
void set1P(void){
	uint32_t k;
	LOADP_L;
	SDOP_L;			//  prepared data=0, only shit 1 bit
	SCKP_H;
		 k=0x05; while(k--) ;
	SCKP_L;
	LOADP_H; // strobe in.
}
void  allsetPower(void){
	uint32_t i=PH_POWER_PINS;
	uint32_t k;
	FIRE_OFF;		// make sure it is.
	LOADP_L;
	SCKP_L;
	SDOP_L;			//  prepared data=0 ( the PMOS, uses logic 0 to turn-on.)
	while	(i--){
		 k=0x05; while(k--) ;
	SCKP_H;
		 k=0x05; while(k--) ;
	SCKP_L;
	}
	LOADP_H; // strobe in.
}
void  resetPower(void){
	uint32_t i=PH_POWER_PINS;
	uint32_t k;
	FIRE_OFF;		// make sure it is.
	LOADP_L;
	SCKP_L;
	SDOP_H;			//  prepared data=1 ( the PMOS, uses logic 0 to turn-on.)
	while	(i--){
		 k=0x05; while(k--) ;
	SCKP_H;
		 k=0x05; while(k--) ;
	SCKP_L;
	}
	LOADP_H; // strobe in.
}
void  shift1P(void){
	uint32_t k;
	LOADP_L;
	SDOP_H;			//  prepared data=1 ( the PMOS, uses logic 0 to turn-on.)
		 k=0x05; while(k--) ;
	SCKP_H;
		 k=0x05; while(k--) ;
	SCKP_L;
	LOADP_H; // strobe in.
}

void PH_LoadA(uint32_t thisAddress, uint32_t nextAddress){
	FIRE_OFF;	// make sure it is.
}

void PH_LoadP(uint32_t thisPower, uint32_t nextPower){
	FIRE_OFF;	// make sure it is.
}
/* USER CODE END 4 */

#ifdef USE_FULL_ASSERT
/*******************************************************************************
* Function Name  : assert_failed
* Description    : Reports the name of the source file and the source line number
*                  where the assert_param error has occurred.
* Input          : - file: pointer to the source file name
*                  - line: assert_param error line source number
* Output         : None
* Return         : None
*******************************************************************************/
void assert_failed(uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {}
}
#endif

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
