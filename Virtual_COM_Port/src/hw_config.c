/**
  ******************************************************************************
  * @file    hw_config.c
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


/* Includes ------------------------------------------------------------------*/

#include "stm32_it.h"
#include "usb_lib.h"
#include "usb_prop.h"
#include "usb_desc.h"
#include "hw_config.h"
#include "usb_pwr.h"


/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
ErrorStatus HSEStartUpStatus;
USART_InitTypeDef USART_InitStructure;
EXTI_InitTypeDef EXTI_InitStructure;
uint8_t  USART_Rx_Buffer [USART_RX_DATA_SIZE]; 
uint32_t USART_Rx_ptr_in = 0;
uint32_t USART_Rx_ptr_out = 0;
uint32_t USART_Rx_length  = 0;
uint8_t   myError =0;

const char timestamp[] = JETDRIVER_STM32_REV;
const char ToHostmenu[] =
   "\n"
   "+******************** JetDriver-STM32 ***" __DATE__  "*****+\n"
   "| An insider program to set/reset parameters of JetDriver-STM32.        |\n"
   "+ cmd --+ ------------------- function ----------------------------+\n"
   "| !Pxx     | Pulsewidth (28=2.1us)                                      |\n"
   "| !Vo      |  VHP printhead high voltage VHp  turn ON         |\n"
   "| !Vf      |  VHP printhead high voltage VHp  turn OFF         |\n"
   "| !V+      |  VHP raise 1 count (0~31 total)  ~ 6.0V ~ 12.3V |\n"
   "| !V-      |  VHP lower 1 count (0~31 total)  ~ 6.0V ~ 12.3V |\n"
   "| !VH     |  VHP to lowest volt: !VH00  =~ 6.0V                     |\n"
   "| !VH1F |  VHP to highest volt: !VH1F =~ 12.3V                   |\n"
   "| @V     |  VHP  volt digits 0x0E = 14(dec) \n"
   "| @P      |  pulseWidth 0x1C = 28(dec) = 2.1us        |\n"
   "| @T      |  dwell time (firing freq related.)  0x1F4 = 500(dec) us   |\n"
   "| !?         |  This help menu                                                  |\n"
   "+---------+-----------------------------------------------------------+\n";

const char ToHostStatus[] =
   "\n__ JetDriver-STM32  Status Report : ";

const char ToRPImenu[] =
   "\n"
   "+******************** JetDriver-STM32 ****" __DATE__  "*****+\n"
   "| An insider program to set/reset parameters of JetDriver-STM32.        |\n"
   "+ cmd ---+ ------------------- function ----------------------------------+\n"
   "| !gxxxx  | Printing go , XXXX swathes   (rcv xxxx bytes(hex) then back to normal     |\n"
   "| !Txxx      | printing-inter-swath period (x014~xFFF us)            |\n"
   "| !Vo      |  VHP printhead high voltage VHp  turn ON         |\n"
   "| !Vf      |  VHP printhead high voltage VHp  turn OFF         |\n"
   "| !V+      |  VHP raise 1 count (0~31 total)  ~ 6.0V ~ 12.3V |\n"
   "| !V-      |  VHP lower 1 count (0~31 total)  ~ 6.0V ~ 12.3V |\n"
   "| @I       |   Get ID of PrintHead|\n"
   "|  !?         |  This help menu            |\n"
   "+---------+-----------------------------------------------------------+\n";
const char ToRPImessage[] = 
	 "\n"
   "+******************** JetDriver-STM32 *******************+\n"
   "  A message to tell you <g> only was just entered                                  |\n"
   "+---------+-------------------------------------------------------------+\n";
const char  ToRPIabort[] = 
	 "\n"
   "printing Task aborted\n" ;
 
// calculate  128KB = 128 * 1024 =  131072 bytes
// code= 13900, RO-data= 116996, total =  130896 bytes
//const char keepReserved[(110000+ 1024*4+208)]= "0"; // it exceeds
//const char keepReserved[(110000+ 1024*4+207)]= "0"; // it pass the compiler.
//  translate into HEX : 0x1BE7F.
//  I could use 0x1B000  to stores.  it it 110592 bytes.
//
const char keepReserved[0x1B000]= "0"; 

uint8_t  USB_Tx_State = 0;
static void IntToUnicode (uint32_t value , uint8_t *pbuf , uint8_t len);
static void IntToString    (uint16_t value , uint8_t *pbuf , uint8_t len) ;
static void Int32ToString (int         value , uint8_t *pbuf , uint8_t len) ;

/* Extern variables ----------------------------------------------------------*/

extern LINE_CODING linecoding;

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/*******************************************************************************
* Function Name  : Set_System
* Description    : Configures Main system clocks & power
* Input          : None.
* Return         : None.
*******************************************************************************/
void Set_System(void)
{
#if !defined(STM32L1XX_MD) && !defined(STM32L1XX_HD) && !defined(STM32L1XX_MD_PLUS)
  GPIO_InitTypeDef GPIO_InitStructure;
#endif /* STM32L1XX_MD && STM32L1XX_XD */  

#if defined(USB_USE_EXTERNAL_PULLUP)
  GPIO_InitTypeDef  GPIO_InitStructure;
#endif /* USB_USE_EXTERNAL_PULLUP */ 
  
  /*!< At this stage the microcontroller clock setting is already configured, 
       this is done through SystemInit() function which is called from startup
       file (startup_stm32f10x_xx.s) before to branch to application main.
       To reconfigure the default setting of SystemInit() function, refer to
       system_stm32f10x.c file
     */   
#if defined(STM32L1XX_MD) || defined(STM32L1XX_HD)|| defined(STM32L1XX_MD_PLUS) || defined(STM32F37X) || defined(STM32F30X)
  /* Enable the SYSCFG module clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
#endif /* STM32L1XX_XD */ 
   
#if !defined(STM32L1XX_MD) && !defined(STM32L1XX_HD) && !defined(STM32L1XX_MD_PLUS) && !defined(STM32F37X) && !defined(STM32F30X)
  /* Enable USB_DISCONNECT GPIO clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIO_DISCONNECT, ENABLE);

  /* Configure USB pull-up pin */
  GPIO_InitStructure.GPIO_Pin = USB_DISCONNECT_PIN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
  GPIO_Init(USB_DISCONNECT, &GPIO_InitStructure);
#endif /* STM32L1XX_MD && STM32L1XX_XD */
   
#if defined(USB_USE_EXTERNAL_PULLUP)
  /* Enable the USB disconnect GPIO clock */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIO_DISCONNECT, ENABLE);

  /* USB_DISCONNECT used as USB pull-up */
  GPIO_InitStructure.GPIO_Pin = USB_DISCONNECT_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(USB_DISCONNECT, &GPIO_InitStructure);  
#endif /* USB_USE_EXTERNAL_PULLUP */ 
  
#if defined(STM32F37X) || defined(STM32F30X)
  
  /* Enable the USB disconnect GPIO clock */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIO_DISCONNECT, ENABLE);
  
  /*Set PA11,12 as IN - USB_DM,DP*/
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11 | GPIO_Pin_12;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  
  /*SET PA11,12 for USB: USB_DM,DP*/
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource11, GPIO_AF_14);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource12, GPIO_AF_14);
  
  /* USB_DISCONNECT used as USB pull-up */
  GPIO_InitStructure.GPIO_Pin = USB_DISCONNECT_PIN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(USB_DISCONNECT, &GPIO_InitStructure);
#endif /* STM32F37X && STM32F30X */ 
  
  /* Configure the EXTI line 18 connected internally to the USB IP */
  EXTI_ClearITPendingBit(EXTI_Line18);
  EXTI_InitStructure.EXTI_Line = EXTI_Line18; 
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);
}

/*******************************************************************************
* Function Name  : Set_USBClock
* Description    : Configures USB Clock input (48MHz)
* Input          : None.
* Return         : None.
*******************************************************************************/
void Set_USBClock(void)
{
#if defined(STM32L1XX_MD) || defined(STM32L1XX_HD) || defined(STM32L1XX_MD_PLUS) 
  /* Enable USB clock */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USB, ENABLE);
  
#else 
  /* Select USBCLK source */
  RCC_USBCLKConfig(RCC_USBCLKSource_PLLCLK_1Div5);
  
  /* Enable the USB clock */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USB, ENABLE);
#endif /* STM32L1XX_MD */
}

/*******************************************************************************
* Function Name  : Enter_LowPowerMode
* Description    : Power-off system clocks and power while entering suspend mode
* Input          : None.
* Return         : None.
*******************************************************************************/
void Enter_LowPowerMode(void)
{
  /* Set the device state to suspend */
  bDeviceState = SUSPENDED;
}

/*******************************************************************************
* Function Name  : Leave_LowPowerMode
* Description    : Restores system clocks and power while exiting suspend mode
* Input          : None.
* Return         : None.
*******************************************************************************/
void Leave_LowPowerMode(void)
{
  DEVICE_INFO *pInfo = &Device_Info;

  /* Set the device state to the correct state */
  if (pInfo->Current_Configuration != 0)
  {
    /* Device configured */
    bDeviceState = CONFIGURED;
  }
  else
  {
    bDeviceState = ATTACHED;
  }
  /*Enable SystemCoreClock*/
  SystemInit();
}

/*******************************************************************************
* Function Name  : USB_Interrupts_Config
* Description    : Configures the USB interrupts
* Input          : None.
* Return         : None.
*******************************************************************************/
void USB_Interrupts_Config(void)
{
  NVIC_InitTypeDef NVIC_InitStructure; 
  
  /* 2 bit for pre-emption priority, 2 bits for subpriority */
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);  
  
#if defined(STM32L1XX_MD) || defined(STM32L1XX_HD)|| defined(STM32L1XX_MD_PLUS)
  NVIC_InitStructure.NVIC_IRQChannel = USB_LP_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  
    /* Enable the USB Wake-up interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = USB_FS_WKUP_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  
#elif defined(STM32F37X)
  /* Enable the USB interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = USB_LP_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  
  /* Enable the USB Wake-up interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = USBWakeUp_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  
#else
  NVIC_InitStructure.NVIC_IRQChannel = USB_LP_CAN1_RX0_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  
    /* Enable the USB Wake-up interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = USBWakeUp_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_Init(&NVIC_InitStructure);
#endif /* STM32L1XX_XD */

  /* Enable USART Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = EVAL_COM1_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_Init(&NVIC_InitStructure);
}

/*******************************************************************************
* Function Name  : USB_Cable_Config
* Description    : Software Connection/Disconnection of USB Cable
* Input          : None.
* Return         : Status
*******************************************************************************/
void USB_Cable_Config (FunctionalState NewState)
{
#if defined(STM32L1XX_MD) || defined (STM32L1XX_HD)|| (STM32L1XX_MD_PLUS)
  if (NewState != DISABLE)
  {
    STM32L15_USB_CONNECT;
  }
  else
  {
    STM32L15_USB_DISCONNECT;
  }  
  
#else /* USE_STM3210B_EVAL or USE_STM3210E_EVAL */
  if (NewState != DISABLE)
  {
    GPIO_ResetBits(USB_DISCONNECT, USB_DISCONNECT_PIN);
  }
  else
  {
    GPIO_SetBits(USB_DISCONNECT, USB_DISCONNECT_PIN);
  }
#endif /* STM32L1XX_MD */
}

/*******************************************************************************
* Function Name  :  USART_Config_Default.
* Description    :  configure the EVAL_COM1 with default values.
* Input          :  None.
* Return         :  None.
*******************************************************************************/
void USART_Config_Default(void)
{
  /* EVAL_COM1 default configuration */
  /* EVAL_COM1 configured as follow:
        - BaudRate = 9600 baud  
        - Word Length = 8 Bits
        - One Stop Bit
        - Parity Odd
        - Hardware flow control disabled
        - Receive and transmit enabled
  */
  USART_InitStructure.USART_BaudRate = 115200;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_Odd;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

  /* Configure and enable the USART */
  STM_EVAL_COMInit(COM1, &USART_InitStructure);

  /* Enable the USART Receive interrupt */
  USART_ITConfig(EVAL_COM1, USART_IT_RXNE, ENABLE);
}

/*******************************************************************************
* Function Name  :  USART_Config.
* Description    :  Configure the EVAL_COM1 according to the line coding structure.
* Input          :  None.
* Return         :  Configuration status
                    TRUE : configuration done with success
                    FALSE : configuration aborted.
*******************************************************************************/
bool USART_Config(void)
{

  /* set the Stop bit*/
  switch (linecoding.format)
  {
    case 0:
      USART_InitStructure.USART_StopBits = USART_StopBits_1;
      break;
    case 1:
      USART_InitStructure.USART_StopBits = USART_StopBits_1_5;
      break;
    case 2:
      USART_InitStructure.USART_StopBits = USART_StopBits_2;
      break;
    default :
    {
      USART_Config_Default();
      return (FALSE);
    }
  }

  /* set the parity bit*/
  switch (linecoding.paritytype)
  {
    case 0:
      USART_InitStructure.USART_Parity = USART_Parity_No;
      break;
    case 1:
      USART_InitStructure.USART_Parity = USART_Parity_Even;
      break;
    case 2:
      USART_InitStructure.USART_Parity = USART_Parity_Odd;
      break;
    default :
    {
      USART_Config_Default();
      return (FALSE);
    }
  }

  /*set the data type : only 8bits and 9bits is supported */
  switch (linecoding.datatype)
  {
    case 0x07:
      /* With this configuration a parity (Even or Odd) should be set */
      USART_InitStructure.USART_WordLength = USART_WordLength_8b;
      break;
    case 0x08:
      if (USART_InitStructure.USART_Parity == USART_Parity_No)
      {
        USART_InitStructure.USART_WordLength = USART_WordLength_8b;
      }
      else 
      {
        USART_InitStructure.USART_WordLength = USART_WordLength_9b;
      }
      
      break;
    default :
    {
      USART_Config_Default();
      return (FALSE);
    }
  }

  USART_InitStructure.USART_BaudRate = linecoding.bitrate;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
 
  /* Configure and enable the USART */
  STM_EVAL_COMInit(COM1, &USART_InitStructure);

  return (TRUE);
}


/*******************************************************************************
* Function Name  : Handle_USBAsynchXfer.
* Description    : send data to USB.
* Input          : None.
* Return         : none.
*******************************************************************************/
void Handle_USBAsynchXfer (void)
{
  
  uint16_t USB_Tx_ptr;
  uint16_t USB_Tx_length;
  
  if(USB_Tx_State != 1)
  {
    if (USART_Rx_ptr_out == USART_RX_DATA_SIZE)
    {
      USART_Rx_ptr_out = 0;
    }
    
    if(USART_Rx_ptr_out == USART_Rx_ptr_in) 
    {
      USB_Tx_State = 0; 
      return;
    }
    
    if(USART_Rx_ptr_out > USART_Rx_ptr_in) /* rollback */
    { 
      USART_Rx_length = USART_RX_DATA_SIZE - USART_Rx_ptr_out;
    }
    else 
    {
      USART_Rx_length = USART_Rx_ptr_in - USART_Rx_ptr_out;
    }
    
    if (USART_Rx_length > VIRTUAL_COM_PORT_DATA_SIZE)
    {
      USB_Tx_ptr = USART_Rx_ptr_out;
      USB_Tx_length = VIRTUAL_COM_PORT_DATA_SIZE;
      
      USART_Rx_ptr_out += VIRTUAL_COM_PORT_DATA_SIZE;	
      USART_Rx_length -= VIRTUAL_COM_PORT_DATA_SIZE;	
    }
    else
    {
      USB_Tx_ptr = USART_Rx_ptr_out;
      USB_Tx_length = USART_Rx_length;
      
      USART_Rx_ptr_out += USART_Rx_length;
      USART_Rx_length = 0;
    }
    USB_Tx_State = 1; 
    UserToPMABufferCopy(&USART_Rx_Buffer[USB_Tx_ptr], ENDP1_TXADDR, USB_Tx_length);
    SetEPTxCount(ENDP1, USB_Tx_length);
    SetEPTxValid(ENDP1); 
  }  
  
}
/*******************************************************************************
* Function Name  : USB_To_USART_Send_Data.
* Description    : send the received data from USB to the UART 0.
* Input          : data_buffer: data address.
                   Nb_bytes: number of bytes to send.
* Return         : none.
*******************************************************************************/
void USB_To_USART_Send_Data(uint8_t* data_buffer, uint8_t Nb_bytes)
{
  uint8_t   but[12]	;  //to stored converted string
  uint32_t i ,j;
  //uint32_t key_spot;	// if found that KEY, record that spot.
  
  for (i = 0; i < Nb_bytes; i++)
  {
	  uint8_t temp;
		int j;
		int thatWidth;
		temp = *(data_buffer+i);
		
		// ORIGINAL, only 2 lines followed.
   //USART_SendData(EVAL_COM1, *(data_buffer + i));
		// while(USART_GetFlagStatus(EVAL_COM1, USART_FLAG_TXE) == RESET); 

// ===== EXTRA dealing.  For FSM_KEY Sequence.			
		if (0==notINfsm) {
			// check the KEYs,
			
			USART_SendData(EVAL_COM1, temp);
			while(USART_GetFlagStatus(EVAL_COM1, USART_FLAG_TXE) == RESET); 
		   rcv_index=0;	// alway reset
		} else {
			accumulated_string[rcv_index++]= temp;	// save it for later parsing.
			if (rcv_index> PARSER_DATA_SIZE){
				rcv_index=0;		// avoid data overflow.
				notINfsm=0;		// also reset.
				fsm_parsed=FSM_DEFault;			
			}
		//  ---- debug portion -------  the rcv_index
		//	USART_SendData(EVAL_COM1, '0'+rcv_index);	// send out the index
		//	while(USART_GetFlagStatus(EVAL_COM1, USART_FLAG_TXE) == RESET); 
		}			

		
			if ('\r'==temp) {	// end-of-line , then reset FSM
				if (0!=notINfsm){
					USART_SendData(EVAL_COM1, temp);
					while(USART_GetFlagStatus(EVAL_COM1, USART_FLAG_TXE) == RESET); 
				}

			// ============= BEGIN switches of beauties ================================
			if (FSM_STM32store==fsm_parsed){	// 	goes into value set/reset.
				/* ---- debug portion -------
				// how many char/parameters input
				USART_SendData(EVAL_COM1, '0'+rcv_index);	// send out the index
		   	while(USART_GetFlagStatus(EVAL_COM1, USART_FLAG_TXE) == RESET); 
				
				// now print the command
				for (j=0; j<rcv_index; j++){
										USART_SendData(EVAL_COM1, accumulated_string[j] );
					          while(USART_GetFlagStatus(EVAL_COM1, USART_FLAG_TXE) == RESET); 
				}
				// now CR/LF
				USART_SendData(EVAL_COM1, '\n');
			  while(USART_GetFlagStatus(EVAL_COM1, USART_FLAG_TXE) == RESET); 
				*/
				//======== Real Parser ======
				switch (accumulated_string[0]) {
					case 'V':
						 if (rcv_index>=2) {
						  switch(accumulated_string[1]){
								case '+':  	VHPup(1); break;
								case '-':  	VHPdown(1); break;
								case 'o':  	VHP_ON;  break;
								case 'f':     VHP_OFF; break;
								case 'H':    VHPdown(32); 
										if (rcv_index>=4) {
											VHPup( printHead.VH_Volt=ParsedID(&accumulated_string[2], 2));
										}
										break;
							}
						}
						break;
					case 'E':	// Ennnn    : nnnn Encoder value set
						 if (rcv_index>=5){
							 printHead.encoder = ParsedID(&accumulated_string[1], 4);
						 }
						break;
					case 'I':	// Ixxx    : IDxxx
						 if (rcv_index>=4){
							 //printHead.ID = ParsedID(accumulated_string[1], &myError);
							 printHead.ID = ParsedID(&accumulated_string[1], 3);
						 }
						break;
					case 'g':	// go
						 stopFire=0;
						break;
					case '!':	// load swath pattern !0001020304050607080910111213141516171819202122232425262728293031323334353637383940414243
						// so your accumulated_string has to be at least 22*2 bytes to hold.
					  // modify that.
					if (rcv_index>=(22*2*2)){
						for (j=0; j<(22*2); j++){
							myPATTERN[j] = ParsedID(&accumulated_string[j*2],2);	// each 2 digit stores in 1 byte. we need 22*2 bytes. Therefore 2*22*2 = 88 bytes stored in accumulated_string.
						}
					}
						break;
					case 'P':	// pulseWidth
						 if (rcv_index>=3){
							 thatWidth = ParsedID(&accumulated_string[1], 2);	/// 2 digits input
							 if ( (thatWidth>= pulseWidth_MIN) && (thatWidth<=pulseWidth_MAX)) {
								  printHead.pulseWidth = thatWidth;		// assign it if  the value is valid.
							 }
						 }			 
						break;
					case 'T':	// timed dewell (delay time for non-jetting)
						 if (rcv_index>=4){
							 thatWidth = ParsedID(&accumulated_string[1], 3);	/// 3 digits input
							 if ( (thatWidth>= dwell_MIN) && (thatWidth<=dwell_MAX)) {
								  printHead.dwell = thatWidth;		// assign it if  the value is valid.
							 }
						 }			 
						break;
					case 'K':	// Kill the swath-task in RPI- waiting data
						if (rcv_index>=2){	// you may enter:  Koo  or KOO   (not only single character <K> . has to be 2 more character to work.
							 printHead.swathCounter=0;
 							 nibbleToggle=0;	// if we receiving the hex digits.(readable).
								stopFire=1;	// yes, please STOP firing/jetting.
							 // also reset  other mechanism.
							 	notINrfsm=0;
								rfsm_parsed=RFSM_DEFault;		
							// buffering management
							start2print=0;							
							bufferWrite_idx=0;				
							bufferRead_idx=0;
						  insideSwath=0;
							map001go=1;  // because stopFire=1, it won't print.
							map002go=1;	// because stopFire=1, it won't print.
							abortShow();	// Yes. tell USER I am back !!
						 }			 
						break;
						case 'm':	// default bitmap printing
							if (rcv_index>=3){	// you entered: m001   m002
								if ( 1==ParsedID(&accumulated_string[1], 3 )) {	/// 3 digits input
									  map001go = 22* (612+97);
									  if (0!=map002go ) map002go=1;	// to let it stop.
								} else {
									  map002go = 22* (612+97);
									 if (0!= map001go) map001go=1; // to let it stop
								}
							  stopFire=0;	// so it could fire.
						}
						break;
					case '?':	// Help
							showHelp();
						break;
				}
			}
			if (FSM_STM32fetch==fsm_parsed){	//  goes into value read/retrieve
				switch (accumulated_string[0]) {
					case '1':
					case '2':
					case '3':
					case '4':
						break;
					case 'V':	//  VH_Volt value read
					   IntToString(printHead.VH_Volt, &but[0], 4);				
						for (j=2; j<4 ;j++){	// 2 digits
							USART_Rx_Buffer[USART_Rx_ptr_in++]  = but[j];
							if(USART_Rx_ptr_in == USART_RX_DATA_SIZE){
								USART_Rx_ptr_in = 0;
							}
						}
						break;
					case 'E':	// Encoder set-value read
					   IntToString(printHead.encoder, &but[0], 4);				
						for (j=0; j<4 ;j++){	// 4 digits
							USART_Rx_Buffer[USART_Rx_ptr_in++]  = but[j];
							if(USART_Rx_ptr_in == USART_RX_DATA_SIZE){
								USART_Rx_ptr_in = 0;
							}
						}
						break;
					case 'P':	//  pulseWidth value read
					   IntToString(printHead.pulseWidth, &but[0], 4);				
						for (j=2; j<4 ;j++){		// 2 digits only
							USART_Rx_Buffer[USART_Rx_ptr_in++]  = but[j];
							if(USART_Rx_ptr_in == USART_RX_DATA_SIZE){
								USART_Rx_ptr_in = 0;
							}
						}
						break;						
					case 'T':	//  dwell time value read
					   IntToString(printHead.dwell, &but[0], 4);				
						for (j=1; j<4 ;j++){		// 3 digits only
							USART_Rx_Buffer[USART_Rx_ptr_in++]  = but[j];
							if(USART_Rx_ptr_in == USART_RX_DATA_SIZE){
								USART_Rx_ptr_in = 0;
							}
						}
						break;						
					case 'I':	// IDxxx
						 //VHP_OFF;
					   IntToString(printHead.ID, &but[0], 4);
						for (j=1; j<4 ;j++){		// 3 digits only
							USART_Rx_Buffer[USART_Rx_ptr_in++]  = but[j];
							if(USART_Rx_ptr_in == USART_RX_DATA_SIZE){
								USART_Rx_ptr_in = 0;
							}
						}
						break;
				case 'p':	// printing status
								showPrintingStatus();
					      Int32ToString(printHead.swathCounter, &but[0], 8);
						for (j=1; j<8 ;j++){		// 8 digits only
							USART_Rx_Buffer[USART_Rx_ptr_in++]  = but[j];
							if(USART_Rx_ptr_in == USART_RX_DATA_SIZE){
								USART_Rx_ptr_in = 0;
							}
						}
				
						break;

				case 'r':	// revision retrieve

						for (j=0; j<sizeof(timestamp) ;j++){		
							USART_Rx_Buffer[USART_Rx_ptr_in++]  = timestamp[j];
							if(USART_Rx_ptr_in == USART_RX_DATA_SIZE){
								USART_Rx_ptr_in = 0;
							}
						}
						USART_Rx_Buffer[USART_Rx_ptr_in++]  =  '\n';
						if	(USART_Rx_ptr_in == USART_RX_DATA_SIZE){
								USART_Rx_ptr_in = 0;
						}										
						USART_Rx_Buffer[USART_Rx_ptr_in++]  =  '\r';
						if	(USART_Rx_ptr_in == USART_RX_DATA_SIZE){
						USART_Rx_ptr_in = 0;
						}				
						break;


				
					case '?':	// status
						break;
				}				
			}
			// ============= END switches of beauties ================================
	
				notINfsm=0;
				fsm_parsed=FSM_DEFault;
			} 

		if (ESC_KEY1==temp) {		// the ESC_KEY1 = '\t'  ESC_KEY2 =  '!'
			 if (FSM_DEFault==fsm_parsed) {
				 fsm_parsed= FSM_KEYCheck;
			 }
		 }
		 if((0==notINfsm) && (FSM_KEYCheck==fsm_parsed)){
			
				if (ESC_KEY2==temp){
						fsm_parsed=FSM_STM32store ;
						notINfsm=1; // for checking
				}
			
					if (ESC_KEY3==temp){
							fsm_parsed=FSM_STM32fetch ;
							notINfsm=1; // for checking
					}
		  }
		// ===== EXTRA dealing.  For FSM_KEY Sequence.	 
  } 
}

/*******************************************************************************
* Function Name  : UART_To_USB_Send_Data.
* Description    : send the received data from UART 0 to USB.
* Input          : None.
* Return         : none.
*******************************************************************************/
void USART_To_USB_Send_Data(void)
{
	uint8_t temp;	// RFSM  used
  
  if (linecoding.datatype == 7)
  {
    USART_Rx_Buffer[USART_Rx_ptr_in] = USART_ReceiveData(EVAL_COM1) & 0x7F;
  }
  else if (linecoding.datatype == 8)
  {
    USART_Rx_Buffer[USART_Rx_ptr_in] = USART_ReceiveData(EVAL_COM1);
  }
	//  ===== EXTRA dealing.  For RFSM_KEY Sequence. 
  temp=USART_Rx_Buffer[USART_Rx_ptr_in];
	
		if (0==notINrfsm) 
		{
					if (0==printHead.quite)  {
						USART_Rx_ptr_in++;		// when downloading printSwath, disable to HOST.
					}
					rrcv_index=0;	// alway reset
		} else {
			acc_string[rrcv_index++]= temp;			// save it for later parsing.
				if 	(rrcv_index> PARSER_DATA_SIZE){
						rrcv_index=0;			// avoid data overflow.
						notINrfsm=0;				// also reset.
						rfsm_parsed=RFSM_DEFault;			
				}
		}			
		
		if (0==printHead.swathCounter) {
			if ('\r'==temp) {	// end-of-line , then reset FSM
				if (0!=notINrfsm){
							if (0==printHead.quite)  USART_Rx_ptr_in++;			
				}
			// ============= BEGIN switches of beauties ================================
			if (RFSM_STM32store==rfsm_parsed){	// 	goes into value set/reset.
				//======== Real Parser ======
				switch (acc_string[0]) {
					case 'V':
						 if (rrcv_index>=2){
						  switch(acc_string[1]){
								case '+':  	VHPup(1); break;
								case '-':  	VHPdown(1); break;
								case 'o':  	VHP_ON;  printHead.VHon=1; break;
								case 'f':     VHP_OFF; printHead.VHon=0; break;
								case 'H':    VHPdown(32); 
										if (rrcv_index>=4) {
											VHPup(printHead.VH_Volt = ParsedID(&acc_string[2], 2) );
										}
										break;
							}
						}
						break;
					case 'E':	// Encoder value set
						//VHP_OFF;
						break;
					case 'I':	// IDxxx

						break;
					case 'q':
							if (1==printHead.quite) 
							{ printHead.quite= 0; }
							else {printHead.quite= 1; }
						break;
					case 'g':	// gXXXX
						 if (rrcv_index>=5){
							 printHead.swathCounter = ParsedID(&acc_string[1], 4);
							 	//youCouldShow();
							 printHead.quite=1;	// also disable sending to Host.
						 }
						 if (0 != printHead.swathCounter ){
							 // prepare
							 nibbleToggle=0;	// if we receiving the hex digits.(readable).
								stopFire=0;	// yes, please go firing/jetting.
							 // also reset  other mechanism.
							 	notINrfsm=0;
								rfsm_parsed=RFSM_DEFault;		
						 } else {
							 //youCouldShow(); // under printHead.swathCount==0 condition.
						 }
						break;
					case '?':	// Help
							showHelpRPI();	// show Help on RPi
						break;
				}
			}
		// ============= END switches of beauties ================================		
				notINrfsm=0;
				rfsm_parsed=RFSM_DEFault;
	}  

		if (PAT_KEY1==temp) {		// the PAT_KEY1 = '\v'			// CTRL-K
			 if (RFSM_DEFault==rfsm_parsed) {
				 rfsm_parsed= RFSM_KEYCheck;
			 }
		 }
		 if((0==notINrfsm) && (RFSM_KEYCheck==rfsm_parsed)){
			
				if (PAT_KEY2==temp){
						rfsm_parsed=RFSM_STM32store ;
						notINrfsm=1; // for checking
				}
			
					if (PAT_KEY3==temp){
							rfsm_parsed=RFSM_STM32fetch ;
							notINrfsm=1; // for checking
					}
		  }
	//  ===== EXTRA dealing.  For RFSM_KEY Sequence. 
	} else{ 
		// When  0 != printHead.swathCounter 
// ============= swath input beauties  START ================================		
		acc_string[nibbleToggle]=temp;		// collect 2 bytes hex digit
		if (0==nibbleToggle) { nibbleToggle=1; ENC_ON; } else 
		{  nibbleToggle=0;
				ENC_OFF;
			// yes.  we got 2 bytes hex digit.
			myPATTERN[bufferWrite_idx++] = ParsedID(&acc_string[0], 2);
			if (bufferWrite_idx>bufferWrite_MAX){
				  bufferWrite_idx=0;				// in case overflow.
			}
			/* ==== SHOULD NOT HAPPENS HERE  ========
			if (1==start2print) {
				insideSwath++;
				if ((22 ) == insideSwath) {
					printHead.swathCounter--;
					insideSwath=0;
				}
			}
			==== SHOULD NOT HAPPENS HERE  ========
			*/
		}
		if (22*2*97==bufferWrite_idx) {
		   start2print=1;    // swath enough for print.
		   //insideSwath=0;	// reset for upcounting 22*2 bytes (to decrease swathCounter )
			 showPrintingStatus();
		}	
	}
// ============= swath input beauties  END  ================================		
			
	// ORIGINAL:
  //USART_Rx_ptr_in++;
  
  /* To avoid buffer overflow */
  if(USART_Rx_ptr_in == USART_RX_DATA_SIZE)
  {
    USART_Rx_ptr_in = 0;
  }
}
// ==============================================================================
void   VCOMprint(const  char  *pbuf, int whichMessage) {
	int i;
 switch (whichMessage) {
	 case 0:	for (i=0; i<sizeof(ToHostmenu) ;i++){
									USART_Rx_Buffer[USART_Rx_ptr_in++]  = pbuf[i];
									if(USART_Rx_ptr_in == USART_RX_DATA_SIZE){
									USART_Rx_ptr_in = 0;
									}
								}
							break;
	 case 1: 	for (i=0; i<sizeof(ToRPImenu) ;i++){
									USART_SendData(EVAL_COM1, pbuf[i]);
									while(USART_GetFlagStatus(EVAL_COM1, USART_FLAG_TXE) == RESET); 
								}
							break;
   case 2:  for (i=0; i<sizeof(ToRPImessage) ;i++){
									USART_SendData(EVAL_COM1, pbuf[i]);
									while(USART_GetFlagStatus(EVAL_COM1, USART_FLAG_TXE) == RESET); 
									}
							break;
   case 3:  for (i=0; i<sizeof(ToRPIabort) ;i++){
									USART_SendData(EVAL_COM1, pbuf[i]);
									while(USART_GetFlagStatus(EVAL_COM1, USART_FLAG_TXE) == RESET); 
		 
									// also send that message to Host
	 									USART_Rx_Buffer[USART_Rx_ptr_in++]  = pbuf[i];
										if(USART_Rx_ptr_in == USART_RX_DATA_SIZE){
												USART_Rx_ptr_in = 0;
											}
									}
							break;			
	 case 4: 	for (i=0; i<sizeof(ToHostStatus) ;i++){
									USART_Rx_Buffer[USART_Rx_ptr_in++]  = pbuf[i];
									if(USART_Rx_ptr_in == USART_RX_DATA_SIZE){
									USART_Rx_ptr_in = 0;
									}
								}
										USART_Rx_Buffer[USART_Rx_ptr_in++]  =  (1==start2print)? 'Y' :'N';
										if	(USART_Rx_ptr_in == USART_RX_DATA_SIZE){
												USART_Rx_ptr_in = 0;
										}
										USART_Rx_Buffer[USART_Rx_ptr_in++]  =  (0==stopFire)? 'Y' :'N';
										if	(USART_Rx_ptr_in == USART_RX_DATA_SIZE){
												USART_Rx_ptr_in = 0;
										}										
										USART_Rx_Buffer[USART_Rx_ptr_in++]  =  (1==printHead.VHon)? 'Y' :'N';
										if	(USART_Rx_ptr_in == USART_RX_DATA_SIZE){
												USART_Rx_ptr_in = 0;
										}																												
							break;											
	} // end of switch
//	VHP_OFF;	// for debug only.
}
/*******************************************************************************
* Function Name  : Get_SerialNum.
* Description    : Create the serial number string descriptor.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void Get_SerialNum(void)
{
  uint32_t Device_Serial0, Device_Serial1, Device_Serial2;

  Device_Serial0 = *(uint32_t*)ID1;
  Device_Serial1 = *(uint32_t*)ID2;
  Device_Serial2 = *(uint32_t*)ID3;  

  Device_Serial0 += Device_Serial2;

  if (Device_Serial0 != 0)
  {
    IntToUnicode (Device_Serial0, &Virtual_Com_Port_StringSerial[2] , 8);
    IntToUnicode (Device_Serial1, &Virtual_Com_Port_StringSerial[18], 4);
  }
}
/*******************************************************************************
* Function Name  : atoi ?
* Description    : Convert String back to uint16_t
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
//uint16_t ParsedID(uint8_t *pbuf, uint8_t *errorFlag){
uint16_t ParsedID(uint8_t *pbuf , uint8_t len){
  uint8_t 	idx = 0;
  uint16_t valueID=0;
	uint16_t  temp;
//	uint8_t  thisBuf[12];
	
//	*errorFlag=0;
  for( idx = 0 ; idx < len ; idx ++)
  {
		    if  ('A'>pbuf[ idx])
				  temp =  (pbuf[idx]-'0' );
				else {
					temp =  (pbuf[idx]-'A')+10;
					//if (temp>15) *errorFlag=1;
				}
				valueID = valueID*16+temp;
    }
	/*
	// what I enterred??
	 for( idx = 0 ; idx < 3 ; idx ++) {
		     USART_Rx_Buffer[USART_Rx_ptr_in++]  = pbuf[idx];
			if(USART_Rx_ptr_in == USART_RX_DATA_SIZE){
				USART_Rx_ptr_in = 0;
			}
		}
	 // then converted it back?
	     USART_Rx_Buffer[USART_Rx_ptr_in++]  = '\n';
			if(USART_Rx_ptr_in == USART_RX_DATA_SIZE){
				USART_Rx_ptr_in = 0;
			}
			IntToString(valueID, &thisBuf[0], 4);

			for( idx = 0 ; idx < 4 ; idx ++) {	// actually converted into 4 digits
		     USART_Rx_Buffer[USART_Rx_ptr_in++]  = thisBuf[idx];
			 if(USART_Rx_ptr_in == USART_RX_DATA_SIZE){
				 USART_Rx_ptr_in = 0;
			  }
		 }
			*/
			//
	return valueID;
}
/*******************************************************************************
* Function Name  : HexToChar.  (CH version)
* Description    : Convert Hex 16Bits value into char.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
static void IntToString (uint16_t value , uint8_t *pbuf , uint8_t len)
{
  uint8_t idx = 0;
  
  for( idx = 0 ; idx < len ; idx ++)
  {
    if( ((value >> 12)) < 0xA )
    {
      pbuf[ idx] = (value >> 12) + '0';
    }
    else
    {
      pbuf[ idx] = (value >> 12) -10+ 'A'; 
    }
    value = value << 4;
    //pbuf[ 2* idx + 1] = 0;
  }
}
/*******************************************************************************
* Function Name  : Int32ToString.  (CH version)
* Description    : Convert Hex 32Bits value into char.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
static void Int32ToString (int value , uint8_t *pbuf , uint8_t len)
{
  uint8_t idx = 0;
  
  for( idx = 0 ; idx < len ; idx ++)
  {
    if( ((value >> 28)) < 0xA )
    {
      pbuf[ idx] = (value >> 28) + '0';
    }
    else
    {
      pbuf[ idx] = (value >> 28) -10+ 'A'; 
    }
    value = value << 4;
    //pbuf[ 2* idx + 1] = 0;
  }
}

/*******************************************************************************
* Function Name  : HexToUnicode .  (original version)
* Description    : Convert Hex 32Bits value into char.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
static void IntToUnicode (uint32_t value , uint8_t *pbuf , uint8_t len)
{
  uint8_t idx = 0;
  
  for( idx = 0 ; idx < len ; idx ++)
  {
    if( ((value >> 28)) < 0xA )
    {
      pbuf[ 2* idx] = (value >> 28) + '0';
    }
    else
    {
      pbuf[2* idx] = (value >> 28) + 'A' - 10; 
    }
    
    value = value << 4;
    
    pbuf[ 2* idx + 1] = 0;
  }
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
