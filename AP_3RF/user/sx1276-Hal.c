/*
 * THE FOLLOWING FIRMWARE IS PROVIDED: (1) "AS IS" WITH NO WARRANTY; AND 
 * (2)TO ENABLE ACCESS TO CODING INFORMATION TO GUIDE AND FACILITATE CUSTOMER.
 * CONSEQUENTLY, SEMTECH SHALL NOT BE HELD LIABLE FOR ANY DIRECT, INDIRECT OR
 * CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE CONTENT
 * OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING INFORMATION
 * CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
 * 
 * Copyright (C) SEMTECH S.A.
 */
/*! 
 * \file       sx1276-Hal.c
 * \brief      SX1276 Hardware Abstraction Layer
 *
 * \version    2.0.B2 
 * \date       Nov 21 2012
 * \author     Miguel Luis
 *
 * Last modified by Miguel Luis on Jun 19 2013
 */
 
#include "stm32f4xx.h"
#include "sx1276-Hal.h"

#define      UserSX1276_1    p_sx1276_hw_operations = &sx1276_hw_operations1
#define      UserSX1276_2    p_sx1276_hw_operations = &sx1276_hw_operations2


volatile uint32_t TickCounter = 0;

struct _sx1276_hw_operations{
	void(*spi_init)(void);
	unsigned char(*spi_readwrite)(unsigned char);
	void(*spi_cs)(int);
	void(*io_init)(void);
	char(*io_read)(char);
	void(*reset)(char);
	void(*c_rxtx)(char);
	void(*irq_init)(void);
	void(*irq_hand_io0)(void);
	void(*irq_hand_io3)(void);
	void(*irq_hand_io4)(void);
}sx1276_hw_operations1,sx1276_hw_operations2,*p_sx1276_hw_operations;

void hw_init(void);

void SX1276InitIo( void )
{
	hw_init();
	p_sx1276_hw_operations->spi_init();
	p_sx1276_hw_operations->io_init();
	p_sx1276_hw_operations->irq_init();
}

void SX1276SetReset( uint8_t state )
{

    if( state == RADIO_RESET_ON )
    {
        // Set RESET pin to 0
			p_sx1276_hw_operations->reset(0); 
    }
    else
    {
			p_sx1276_hw_operations->reset(1); 
    }
}

void SX1276Write( uint8_t addr, uint8_t data )
{
    SX1276WriteBuffer( addr, &data, 1 );
}

void SX1276Read( uint8_t addr, uint8_t *data )
{
    SX1276ReadBuffer( addr, data, 1 );
}

void SX1276WriteBuffer( uint8_t addr, uint8_t *buffer, uint8_t size )
{
    uint8_t i;

    //NSS = 0;
    p_sx1276_hw_operations->spi_cs(0);

    p_sx1276_hw_operations->spi_readwrite( addr | 0x80 );
    for( i = 0; i < size; i++ )
    {
        p_sx1276_hw_operations->spi_readwrite( buffer[i] );
    }

    //NSS = 1;
    p_sx1276_hw_operations->spi_cs(1);
}

void SX1276ReadBuffer( uint8_t addr, uint8_t *buffer, uint8_t size )
{
    uint8_t i;

    //NSS = 0;
    p_sx1276_hw_operations->spi_cs(0);

    p_sx1276_hw_operations->spi_readwrite( addr & 0x7F );

    for( i = 0; i < size; i++ )
    {
        buffer[i] =  p_sx1276_hw_operations->spi_readwrite( 0 );
    }

    //NSS = 1;
    p_sx1276_hw_operations->spi_cs(1);
}

void SX1276WriteFifo( uint8_t *buffer, uint8_t size )
{
    SX1276WriteBuffer( 0, buffer, size );
}

void SX1276ReadFifo( uint8_t *buffer, uint8_t size )
{
    SX1276ReadBuffer( 0, buffer, size );
}

 uint8_t SX1276ReadDio0( void )
{
    return p_sx1276_hw_operations->io_read(0);
}

 uint8_t SX1276ReadDio1( void )
{
    return p_sx1276_hw_operations->io_read(1);
}

 uint8_t SX1276ReadDio2( void )
{
    return p_sx1276_hw_operations->io_read(2);
}

 uint8_t SX1276ReadDio3( void )
{
    return p_sx1276_hw_operations->io_read(3);
}

 uint8_t SX1276ReadDio4( void )
{
    return p_sx1276_hw_operations->io_read(4);
}

 uint8_t SX1276ReadDio5( void )
{
    return p_sx1276_hw_operations->io_read(5);
}

 void SX1276WriteRxTx( uint8_t txEnable )
{
    if( txEnable != 0 )
    {
			p_sx1276_hw_operations->c_rxtx(0);
    }
    else
    {
			p_sx1276_hw_operations->c_rxtx(1);
    }
}

/*-------------------------------------------------hardware layer----------------------------------------------------------*/
/**************************************************************************************************************************/



/*
 *  spi1    PA4-CS PA5-SCK PA6-MISO PA7-MOSI 
 *				 
 *
 *
 *
*/
void SPI1_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	SPI_InitTypeDef SPI_InitStruct;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1,ENABLE);
		
	GPIO_InitStruct.GPIO_Pin  = GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7;   //miso sck
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;	
	GPIO_Init(GPIOA, &GPIO_InitStruct);
	
	GPIO_InitStruct.GPIO_Pin  = GPIO_Pin_4;          //SS??
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_Init(GPIOA, &GPIO_InitStruct);	
	

	
	
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource5,GPIO_AF_SPI1);
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource6,GPIO_AF_SPI1);
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource7,GPIO_AF_SPI1);
	
	  /* Initialize the SPI_Direction member */
  SPI_InitStruct.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
  /* initialize the SPI_Mode member */
  SPI_InitStruct.SPI_Mode = SPI_Mode_Master;
  /* initialize the SPI_DataSize member */
  SPI_InitStruct.SPI_DataSize = SPI_DataSize_8b;
  /* Initialize the SPI_CPOL member */
  SPI_InitStruct.SPI_CPOL = SPI_CPOL_Low;
  /* Initialize the SPI_CPHA member */
  SPI_InitStruct.SPI_CPHA = SPI_CPHA_1Edge;
  /* Initialize the SPI_NSS member */
  SPI_InitStruct.SPI_NSS = SPI_NSS_Soft;
  /* Initialize the SPI_BaudRatePrescaler member */
  SPI_InitStruct.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;
  /* Initialize the SPI_FirstBit member */
  SPI_InitStruct.SPI_FirstBit = SPI_FirstBit_MSB;
  /* Initialize the SPI_CRCPolynomial member */
  SPI_InitStruct.SPI_CRCPolynomial = 7;
	
	SPI_Init(SPI1, &SPI_InitStruct);
	
	SPI_Cmd(SPI1,ENABLE);
	
}




unsigned char SPI1_ReadWrite(unsigned char data)
{
	while(SPI_I2S_GetFlagStatus(SPI1,SPI_I2S_FLAG_TXE)==RESET);
	SPI1->DR = data;
	while(SPI_I2S_GetFlagStatus(SPI1,SPI_I2S_FLAG_RXNE)==RESET);
	return SPI1->DR;
}


void SPI1_Cs(int flag)
{
	if(flag)
		GPIOA->BSRRL = GPIO_Pin_4;
	else
		GPIOA->BSRRH = GPIO_Pin_4;
}







/*
 *  spi2    PB12-CS PB13-SCK PB14-MISO PB15-MOSI 
 *				 
 *
 *
 *
*/
void SPI2_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	SPI_InitTypeDef SPI_InitStruct;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2,ENABLE);
		
	GPIO_InitStruct.GPIO_Pin  = GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15;   //miso sck
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;	
	GPIO_Init(GPIOB, &GPIO_InitStruct);
	
	GPIO_InitStruct.GPIO_Pin  = GPIO_Pin_12;          //SS??
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_Init(GPIOB, &GPIO_InitStruct);	

	GPIO_PinAFConfig(GPIOB,GPIO_PinSource13,GPIO_AF_SPI1);
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource14,GPIO_AF_SPI1);
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource15,GPIO_AF_SPI1);
	
	  /* Initialize the SPI_Direction member */
  SPI_InitStruct.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
  /* initialize the SPI_Mode member */
  SPI_InitStruct.SPI_Mode = SPI_Mode_Master;
  /* initialize the SPI_DataSize member */
  SPI_InitStruct.SPI_DataSize = SPI_DataSize_8b;
  /* Initialize the SPI_CPOL member */
  SPI_InitStruct.SPI_CPOL = SPI_CPOL_Low;
  /* Initialize the SPI_CPHA member */
  SPI_InitStruct.SPI_CPHA = SPI_CPHA_1Edge;
  /* Initialize the SPI_NSS member */
  SPI_InitStruct.SPI_NSS = SPI_NSS_Soft;
  /* Initialize the SPI_BaudRatePrescaler member */
  SPI_InitStruct.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;
  /* Initialize the SPI_FirstBit member */
  SPI_InitStruct.SPI_FirstBit = SPI_FirstBit_MSB;
  /* Initialize the SPI_CRCPolynomial member */
  SPI_InitStruct.SPI_CRCPolynomial = 7;
	
	SPI_Init(SPI2, &SPI_InitStruct);
	
	SPI_Cmd(SPI2,ENABLE);
	
}




unsigned char SPI2_ReadWrite(unsigned char data)
{
	while(SPI_I2S_GetFlagStatus(SPI2,SPI_I2S_FLAG_TXE)==RESET);
	SPI2->DR = data;
	while(SPI_I2S_GetFlagStatus(SPI2,SPI_I2S_FLAG_RXNE)==RESET);
	return SPI2->DR;
}


void SPI2_Cs(int flag)
{
	if(flag)
		GPIOB->BSRRL = GPIO_Pin_12;
	else
		GPIOB->BSRRH = GPIO_Pin_12;
}


void io1_init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA|RCC_AHB1Periph_GPIOB|RCC_AHB1Periph_GPIOC,ENABLE);	
	
	GPIO_InitStruct.GPIO_Pin  = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_4|GPIO_Pin_5; 
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOC, &GPIO_InitStruct);	

	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0;
	GPIO_Init(GPIOB, &GPIO_InitStruct);	
	
	GPIO_InitStruct.GPIO_Pin  = GPIO_Pin_1; //RESET
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_Init(GPIOB, &GPIO_InitStruct);	

	GPIO_InitStruct.GPIO_Pin  = GPIO_Pin_3; //FEM_CPS
	GPIO_Init(GPIOC, &GPIO_InitStruct);	

}

void io2_init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA|RCC_AHB1Periph_GPIOB|RCC_AHB1Periph_GPIOC,ENABLE);
	
	GPIO_InitStruct.GPIO_Pin  = GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_9; 
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOC, &GPIO_InitStruct);	
	
	GPIO_InitStruct.GPIO_Pin  = GPIO_Pin_8|GPIO_Pin_11; 
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &GPIO_InitStruct);		

	GPIO_InitStruct.GPIO_Pin  = GPIO_Pin_10; 
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_Init(GPIOB, &GPIO_InitStruct);			
	
	GPIO_InitStruct.GPIO_Pin  = GPIO_Pin_12; 
	GPIO_Init(GPIOA, &GPIO_InitStruct);		
}



char io1_read(char io)
{
	switch(io)
	{
		case 0:return (GPIOB->IDR&GPIO_Pin_0)?1:0;
		case 1:return (GPIOC->IDR&GPIO_Pin_5)?1:0;		
		case 2:return (GPIOC->IDR&GPIO_Pin_4)?1:0;	
		case 3:return (GPIOC->IDR&GPIO_Pin_0)?1:0;
		case 4:return (GPIOC->IDR&GPIO_Pin_1)?1:0;
		case 5:return (GPIOC->IDR&GPIO_Pin_2)?1:0;			
		default :return 0;
	}
}

char io2_read(char io)
{
	int fuck = GPIOC->IDR;
	fuck = fuck&GPIO_Pin_8;
	switch(io)
	{
		case 0:return (GPIOA->IDR&GPIO_Pin_11)?1:0;
		case 1:return (GPIOA->IDR&GPIO_Pin_8)?1:0;		
		case 2:return (GPIOC->IDR&GPIO_Pin_9)?1:0;	
		case 3:return (GPIOC->IDR&GPIO_Pin_8)?1:0;
		case 4:return (GPIOC->IDR&GPIO_Pin_7)?1:0;
		case 5:return (GPIOC->IDR&GPIO_Pin_6)?1:0;	
		default :return 0;		
	}
}

void s2_crxtx(char stat)
{
	if(stat)
		GPIOB->BSRRL = GPIO_Pin_10;
	else
		GPIOB->BSRRH = GPIO_Pin_10;
}


void s1_crxtx(char stat)
{
	if(stat)
		GPIOC->BSRRL = GPIO_Pin_3;
	else
		GPIOC->BSRRH = GPIO_Pin_3;
}


void s1_reset(char stat)
{
	if(stat)
	GPIOB->BSRRL = GPIO_Pin_1;
	else
	GPIOB->BSRRH = GPIO_Pin_1;
}


void s2_reset(char stat)
{
	if(stat)
	GPIOA->BSRRL = GPIO_Pin_12;
	else
	GPIOA->BSRRH = GPIO_Pin_12;
}

void irq1_init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;	
	EXTI_InitTypeDef EXTI_InitStruct;
	NVIC_InitTypeDef NVIC_InitStruct;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG,ENABLE );
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB|RCC_AHB1Periph_GPIOC,ENABLE);		
	
	GPIO_InitStruct.GPIO_Pin  = GPIO_Pin_1;  
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_Init(GPIOC, &GPIO_InitStruct);		

	GPIO_InitStruct.GPIO_Pin  = GPIO_Pin_0;  
	GPIO_Init(GPIOB, &GPIO_InitStruct);	
	
	//SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC,EXTI_PinSource0);
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB,EXTI_PinSource0);
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC,EXTI_PinSource1);
	
	
	EXTI_InitStruct.EXTI_Line = EXTI_Line0|EXTI_Line1;
  EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Rising;
  EXTI_InitStruct.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStruct);

	NVIC_InitStruct.NVIC_IRQChannel = EXTI0_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 5;
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStruct);
	
	NVIC_InitStruct.NVIC_IRQChannel = EXTI1_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 4;
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStruct);	
	
	
}





void irq2_init(void)
{

	GPIO_InitTypeDef GPIO_InitStruct;	
	EXTI_InitTypeDef EXTI_InitStruct;
	NVIC_InitTypeDef NVIC_InitStruct;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA|RCC_AHB1Periph_GPIOC,ENABLE);		
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG,ENABLE );
	
	GPIO_InitStruct.GPIO_Pin  = GPIO_Pin_7|GPIO_Pin_8;  
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_Init(GPIOC, &GPIO_InitStruct);		

	GPIO_InitStruct.GPIO_Pin  = GPIO_Pin_11;  
	GPIO_Init(GPIOA, &GPIO_InitStruct);	
	
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC,EXTI_PinSource7);
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC,EXTI_PinSource8);
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA,EXTI_PinSource11);
	
	EXTI_InitStruct.EXTI_Line = EXTI_Line7|EXTI_Line8|EXTI_Line11;
  EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Rising;
  EXTI_InitStruct.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStruct);
	
//	NVIC_InitStruct.NVIC_IRQChannel = EXTI9_5_IRQn;
//	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0;
//	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 6;
//	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
//	NVIC_Init(&NVIC_InitStruct);	

	NVIC_InitStruct.NVIC_IRQChannel = EXTI15_10_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 7;
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStruct);	



}

void EXTI0_IRQHandler()
{
	if(EXTI_GetFlagStatus(EXTI_Line0)==SET)
	{
		EXTI_ClearFlag(EXTI_Line0);
		sx1276_hw_operations1.irq_hand_io0();
//		sx1276_hw_operations1.irq_hand_io3();
	}
}

void EXTI1_IRQHandler()
{
	if(EXTI_GetFlagStatus(EXTI_Line1)==SET)
	{
		EXTI_ClearFlag(EXTI_Line1);
//		sx1276_hw_operations1.irq_hand_io4();
	}	
}

void EXTI9_5_IRQHandler()
{
	
	if(EXTI_GetFlagStatus(EXTI_Line7)==SET)
	{
		EXTI_ClearFlag(EXTI_Line7);
//		sx1276_hw_operations2.irq_hand_io4();
	}	
	
	if(EXTI_GetFlagStatus(EXTI_Line8)==SET)
	{
		EXTI_ClearFlag(EXTI_Line8);
//		sx1276_hw_operations2.irq_hand_io3();
	}		
	
}




void EXTI15_10_IRQHandler()
{
	
	if(EXTI_GetFlagStatus(EXTI_Line11)==SET)
	{
		EXTI_ClearFlag(EXTI_Line11);
		sx1276_hw_operations2.irq_hand_io0();
	}	
	
	
	
}	
	
	


void LED_Init(void);
void LED_Close(int ledsel);
void LED_Open(int ledsel);

void LED_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA|RCC_AHB1Periph_GPIOB|RCC_AHB1Periph_GPIOC,ENABLE);		
	
	GPIO_InitStruct.GPIO_Pin  = GPIO_Pin_10|GPIO_Pin_11|GPIO_Pin_12|GPIO_Pin_13;  
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_Init(GPIOC, &GPIO_InitStruct);

	GPIO_InitStruct.GPIO_Pin  = GPIO_Pin_4|GPIO_Pin_6|GPIO_Pin_8|GPIO_Pin_9;  	
	GPIO_Init(GPIOB, &GPIO_InitStruct);

	GPIO_InitStruct.GPIO_Pin  = GPIO_Pin_15;  	
	GPIO_Init(GPIOA, &GPIO_InitStruct);
		
	LED_Close(1);	
	LED_Close(2); 
	LED_Close(3); 
	LED_Close(4); 
	LED_Close(5); 
	LED_Close(6); 
	LED_Close(7); 
	LED_Close(8); 
	LED_Close(9);
}

void LED_Close(int ledsel)
{
	switch(ledsel)
	{
		case 1: GPIO_SetBits(GPIOC,GPIO_Pin_13); break;
		case 2: GPIO_SetBits(GPIOB,GPIO_Pin_9); break;
		case 3: GPIO_SetBits(GPIOB,GPIO_Pin_8); break;
		case 4: GPIO_SetBits(GPIOB,GPIO_Pin_6); break;
		case 5: GPIO_SetBits(GPIOB,GPIO_Pin_4); break;
		case 6: GPIO_SetBits(GPIOC,GPIO_Pin_12); break;
		case 7: GPIO_SetBits(GPIOC,GPIO_Pin_11); break;
		case 8: GPIO_SetBits(GPIOC,GPIO_Pin_10); break;
		case 9: GPIO_SetBits(GPIOA,GPIO_Pin_15); break;
		default : break;
	}
}

void LED_Open(int ledsel)
{
	switch(ledsel)
	{
		case 1: GPIO_ResetBits(GPIOC,GPIO_Pin_13); break;
		case 2: GPIO_ResetBits(GPIOB,GPIO_Pin_9); break;
		case 3: GPIO_ResetBits(GPIOB,GPIO_Pin_8); break;
		case 4: GPIO_ResetBits(GPIOB,GPIO_Pin_6); break;
		case 5: GPIO_ResetBits(GPIOB,GPIO_Pin_4); break;
		case 6: GPIO_ResetBits(GPIOC,GPIO_Pin_12); break;
		case 7: GPIO_ResetBits(GPIOC,GPIO_Pin_11); break;
		case 8: GPIO_ResetBits(GPIOC,GPIO_Pin_10); break;
		case 9: GPIO_ResetBits(GPIOA,GPIO_Pin_15); break;
		default : break;		
	}
}

/***********************************************************************************************
--FunName: Uart1_Init()
--Function Description:    ???UART1 ?????
--Param:   none
--Return:   none
--date:    2013.7.2
***********************************************************************************************/
void Uart1_Init(int baud)
{
	
	USART_InitTypeDef USART_InitStruct;
	GPIO_InitTypeDef GPIO_InitStruct;
	USART_ClockInitTypeDef USART_ClockInitStruct;
	NVIC_InitTypeDef NVIC_InitStruct;
	DMA_InitTypeDef DMA_InitStruct;		
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA|RCC_AHB1Periph_DMA2,ENABLE);
	
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource9,GPIO_AF_USART1);
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource10,GPIO_AF_USART1);
	
	GPIO_InitStruct.GPIO_Pin  = GPIO_Pin_9;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &GPIO_InitStruct);

	
	GPIO_InitStruct.GPIO_Pin  = GPIO_Pin_10;
	GPIO_Init(GPIOA, &GPIO_InitStruct);

	USART_ClockStructInit(&USART_ClockInitStruct);
	USART_ClockInit(USART1, &USART_ClockInitStruct);
	
	USART_InitStruct.USART_BaudRate = baud;
  USART_InitStruct.USART_WordLength = USART_WordLength_8b;
  USART_InitStruct.USART_StopBits = USART_StopBits_1;
  USART_InitStruct.USART_Parity = USART_Parity_No ;
  USART_InitStruct.USART_Mode = USART_Mode_Tx|USART_Mode_Rx;
  USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None; 
	USART_Init(USART1, &USART_InitStruct);
	USART_ITConfig(USART1,USART_IT_RXNE,ENABLE);
	USART_Cmd(USART1,ENABLE);
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	NVIC_InitStruct.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE ;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;
	NVIC_Init(&NVIC_InitStruct);
	
  DMA_InitStruct.DMA_Channel = DMA_Channel_4;

  /* Initialize the DMA_PeripheralBaseAddr member */
  DMA_InitStruct.DMA_PeripheralBaseAddr = (volatile int)&USART1->DR;

  /* Initialize the DMA_Memory0BaseAddr member */
  DMA_InitStruct.DMA_Memory0BaseAddr = 0;

  /* Initialize the DMA_DIR member */
  DMA_InitStruct.DMA_DIR = DMA_DIR_MemoryToPeripheral;

  /* Initialize the DMA_BufferSize member */
  DMA_InitStruct.DMA_BufferSize = 0;

  /* Initialize the DMA_PeripheralInc member */
  DMA_InitStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;

  /* Initialize the DMA_MemoryInc member */
  DMA_InitStruct.DMA_MemoryInc = DMA_MemoryInc_Enable;

  /* Initialize the DMA_PeripheralDataSize member */
  DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;

  /* Initialize the DMA_MemoryDataSize member */
  DMA_InitStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;

  /* Initialize the DMA_Mode member */
  DMA_InitStruct.DMA_Mode = DMA_Mode_Normal;

  /* Initialize the DMA_Priority member */
  DMA_InitStruct.DMA_Priority = DMA_Priority_Low;

  /* Initialize the DMA_FIFOMode member */
  DMA_InitStruct.DMA_FIFOMode = DMA_FIFOMode_Disable;

  /* Initialize the DMA_FIFOThreshold member */
  DMA_InitStruct.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;

  /* Initialize the DMA_MemoryBurst member */
  DMA_InitStruct.DMA_MemoryBurst = DMA_MemoryBurst_Single;

  /* Initialize the DMA_PeripheralBurst member */
  DMA_InitStruct.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_Init(DMA2_Stream7,&DMA_InitStruct);
	
	USART_DMACmd(USART1,USART_DMAReq_Tx,ENABLE);
}
/***********************************************************************************************
--FunName: Uart1_Send()
--Function Description:    ???UART1 ??????
--Param:   Data  ??????
--Return:   none
--date:    2013.7.2
***********************************************************************************************/
void Uart1_Send( char Data)
{
	while(RESET==USART_GetFlagStatus(USART1, USART_FLAG_TC));
	USART1->DR = (Data & (uint16_t)0x01FF);
}

void usart_sendstring(char *str)
{
//	while(*str)
//		Uart1_Send(*str++);
	int num = 0;
	char *ptr = str;
	while(*ptr++)
	{
		num++;
		if(num>500)return;
	}

	
	while(DMA2_Stream7->NDTR!=0);	
	
	DMA2_Stream7->NDTR = num;
	DMA2_Stream7->M0AR = (int)str;
	
	
	if(DMA_GetFlagStatus(DMA2_Stream7,DMA_FLAG_TCIF7)==SET)
		DMA_ClearFlag(DMA2_Stream7,DMA_FLAG_TCIF7);

	DMA_Cmd(DMA2_Stream7,ENABLE);
	
}


void uart_dma_send(unsigned char *ptr,int num)
{
	while(DMA2_Stream7->NDTR!=0);	
	
	DMA2_Stream7->NDTR = num;
	DMA2_Stream7->M0AR = (int)ptr;
	
	
	if(DMA_GetFlagStatus(DMA2_Stream7,DMA_FLAG_TCIF7)==SET)
		DMA_ClearFlag(DMA2_Stream7,DMA_FLAG_TCIF7);

	DMA_Cmd(DMA2_Stream7,ENABLE);	
}






void Timer2_Config(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
	NVIC_InitTypeDef NVIC_InitStruct;			
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);
	
	TIM_TimeBaseInitStruct.TIM_Period = 49;            //255 //15625
  TIM_TimeBaseInitStruct.TIM_Prescaler = 13125;    //2562  //41
  TIM_TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV1;
  TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInitStruct.TIM_RepetitionCounter = 0x0000;
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseInitStruct);
	
	TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE);


	NVIC_InitStruct.NVIC_IRQChannel = TIM2_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority =  0;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStruct);
	
	TIM_Cmd(TIM2,ENABLE);
}


extern int slot;
extern void ap_slot_handl(void);
extern unsigned char Tx_conter;
void TIM2_IRQHandler(void)
{
	
	if(TIM_GetITStatus(TIM2,TIM_IT_Update)==SET)
	{
		ap_slot_handl();
		TIM2->SR = (uint16_t)~TIM_IT_Update;
		Tx_conter++;
	}
	
	
}














void hw_init(void)
{
	sx1276_hw_operations1.spi_init = SPI1_Config;
	sx1276_hw_operations1.spi_readwrite = SPI1_ReadWrite;
	sx1276_hw_operations1.spi_cs = SPI1_Cs;
	sx1276_hw_operations1.io_init = io1_init;
	sx1276_hw_operations1.io_read = io1_read;
	sx1276_hw_operations1.reset = s1_reset;
	sx1276_hw_operations1.c_rxtx = s1_crxtx;
	sx1276_hw_operations1.irq_init = irq1_init;
	
	sx1276_hw_operations2.spi_init = SPI2_Config;
	sx1276_hw_operations2.spi_readwrite = SPI2_ReadWrite;
	sx1276_hw_operations2.spi_cs = SPI2_Cs;
	sx1276_hw_operations2.io_init = io2_init;
	sx1276_hw_operations2.io_read = io2_read;
	sx1276_hw_operations2.reset = s2_reset;
	sx1276_hw_operations2.c_rxtx = s2_crxtx;
	sx1276_hw_operations2.irq_init = irq2_init;	
	
	
//	Timer2_Config();
	p_sx1276_hw_operations = UserSX1276_2;
}



