#include "rf_hal.h"
#include "main.h"
#include "stm32f4xx_hal.h"
#include "sx1276.h"
#include "rf.h"




strcut_sx1276_hw_operations sx1276_hw_operations_1;
strcut_sx1276_hw_operations sx1276_hw_operations_2;
strcut_sx1276_hw_operations sx1276_hw_operations_3;



extern SPI_HandleTypeDef hspi1;
extern SPI_HandleTypeDef hspi4;
extern SPI_HandleTypeDef hspi5;





strcut_sx1276_hw_operations *p_sx1276_hw_operations = &sx1276_hw_operations_1;


volatile uint32_t TickCounter;
unsigned char spi_buff_no_use[0x7f];



void hw_init(void);

void SX1276InitIo( void )
{
	hw_init();
}


void SX1276Power( uint8_t state )
{
    if( state == 1 )
    {
        // Set RESET pin to 0
			p_sx1276_hw_operations->power(1); 
    }
    else
    {
			p_sx1276_hw_operations->power(0); 
    }
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
	 int delay = 100000;
		uint8_t _data[2];
		_data[0] = addr | 0x80;
	  _data[1] = data;
	
    p_sx1276_hw_operations->spi_cs(0);
		p_sx1276_hw_operations->spi_readwrite( _data,spi_buff_no_use,2);

    //NSS = 1;
    p_sx1276_hw_operations->spi_cs(1);
}

void SX1276Read( uint8_t addr, uint8_t *data )
{
		uint8_t tx[2],rx[2];
		tx[0] = addr;
	  tx[1] = 0xff;
	
    p_sx1276_hw_operations->spi_cs(0);

    p_sx1276_hw_operations->spi_readwrite( tx,rx,2);

    //NSS = 1;
    p_sx1276_hw_operations->spi_cs(1);
	
		*data = rx[1];
}

void SX1276WriteBuffer( uint8_t addr, uint8_t *buffer, uint8_t size )
{
    uint8_t i;
		
		i = addr | 0x80;
    //NSS = 0;
    p_sx1276_hw_operations->spi_cs(0);

    p_sx1276_hw_operations->spi_readwrite( &i,spi_buff_no_use,1);
		p_sx1276_hw_operations->spi_readwrite( buffer,spi_buff_no_use+1,size);

    //NSS = 1;
    p_sx1276_hw_operations->spi_cs(1);
}

void SX1276ReadBuffer( uint8_t addr, uint8_t *buffer, uint8_t size )
{
    uint8_t i;

		i = addr & 0x7F ;
    //NSS = 0;
    p_sx1276_hw_operations->spi_cs(0);

    p_sx1276_hw_operations->spi_readwrite( &i,spi_buff_no_use,1);

		p_sx1276_hw_operations->spi_readwrite( spi_buff_no_use,buffer,size);

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







int SPI1_ReadWrite(uint8_t *pTxData, uint8_t *pRxData, uint16_t Size)
{
	return HAL_SPI_TransmitReceive(&hspi1,pTxData,pRxData,Size,2);
}


void SPI1_Cs(int flag)
{
	if(flag)
		HAL_GPIO_WritePin(RF1_NSS_GPIO_Port,RF1_NSS_Pin,GPIO_PIN_SET);
	else
		HAL_GPIO_WritePin(RF1_NSS_GPIO_Port,RF1_NSS_Pin,GPIO_PIN_RESET);
}


int SPI4_ReadWrite(uint8_t *pTxData, uint8_t *pRxData, uint16_t Size)
{
	return HAL_SPI_TransmitReceive(&hspi4,pTxData,pRxData,Size,2);
}


void SPI4_Cs(int flag)
{
	if(flag)
		HAL_GPIO_WritePin(RF3_NSS_GPIO_Port,RF3_NSS_Pin,GPIO_PIN_SET);
	else
		HAL_GPIO_WritePin(RF3_NSS_GPIO_Port,RF3_NSS_Pin,GPIO_PIN_RESET);
}


int SPI5_ReadWrite(uint8_t *pTxData, uint8_t *pRxData, uint16_t Size)
{
	return HAL_SPI_TransmitReceive(&hspi5,pTxData,pRxData,Size,2);
}


void SPI5_Cs(int flag)
{
	if(flag)
		HAL_GPIO_WritePin(RF2_NSS_GPIO_Port,RF2_NSS_Pin,GPIO_PIN_SET);
	else
		HAL_GPIO_WritePin(RF2_NSS_GPIO_Port,RF2_NSS_Pin,GPIO_PIN_RESET);
}








char io1_read(char io)
{
	switch(io)
	{
		case 0:return (GPIOA->IDR&GPIO_PIN_3)?1:0;
		case 1:return (GPIOH->IDR&GPIO_PIN_5)?1:0;		
		case 2:return (GPIOH->IDR&GPIO_PIN_4)?1:0;	
		case 3:return (GPIOH->IDR&GPIO_PIN_3)?1:0;
		case 4:return (GPIOH->IDR&GPIO_PIN_2)?1:0;
		case 5:return (GPIOA->IDR&GPIO_PIN_2)?1:0;			
		default :return 0;
	}
}

char io2_read(char io)
{
	switch(io)
	{
		case 0:return (GPIOF->IDR&GPIO_PIN_5)?1:0;
		case 1:return (GPIOF->IDR&GPIO_PIN_4)?1:0;		
		case 2:return (GPIOF->IDR&GPIO_PIN_3)?1:0;	
		case 3:return (GPIOF->IDR&GPIO_PIN_2)?1:0;
		case 4:return (GPIOF->IDR&GPIO_PIN_1)?1:0;
		case 5:return (GPIOF->IDR&GPIO_PIN_0)?1:0;			
		default :return 0;
	}
}

char io3_read(char io)
{
	switch(io)
	{
		case 0:return (GPIOH->IDR&GPIO_PIN_8)?1:0;
		case 1:return (GPIOH->IDR&GPIO_PIN_7)?1:0;		
		case 2:return (GPIOH->IDR&GPIO_PIN_6)?1:0;	
		case 3:return (GPIOB->IDR&GPIO_PIN_11)?1:0;
		case 4:return (GPIOB->IDR&GPIO_PIN_10)?1:0;
		case 5:return (GPIOE->IDR&GPIO_PIN_15)?1:0;			
		default :return 0;
	}
}

void RF3_crxtx(char stat)
{
	if(stat)
		HAL_GPIO_WritePin(RF3_TRX_CTRL_GPIO_Port,RF3_TRX_CTRL_Pin,GPIO_PIN_SET);
	else
		HAL_GPIO_WritePin(RF3_TRX_CTRL_GPIO_Port,RF3_TRX_CTRL_Pin,GPIO_PIN_RESET);
}

void RF2_crxtx(char stat)
{
	if(stat)
		HAL_GPIO_WritePin(RF2_TRX_CTRL_GPIO_Port,RF2_TRX_CTRL_Pin,GPIO_PIN_SET);
	else
		HAL_GPIO_WritePin(RF2_TRX_CTRL_GPIO_Port,RF2_TRX_CTRL_Pin,GPIO_PIN_RESET);
}


void RF1_crxtx(char stat)
{
	if(stat)
		HAL_GPIO_WritePin(RF1_TRX_CTRL_GPIO_Port,RF1_TRX_CTRL_Pin,GPIO_PIN_SET);
	else
		HAL_GPIO_WritePin(RF1_TRX_CTRL_GPIO_Port,RF1_TRX_CTRL_Pin,GPIO_PIN_RESET);
}


void RF1_reset(char stat)
{
	if(stat)
		HAL_GPIO_WritePin(RF1_RESETN_GPIO_Port,RF1_RESETN_Pin,GPIO_PIN_SET);
	else
		HAL_GPIO_WritePin(RF1_RESETN_GPIO_Port,RF1_RESETN_Pin,GPIO_PIN_RESET);
}


void RF2_reset(char stat)
{
	if(stat)
		HAL_GPIO_WritePin(RF2_RESETN_GPIO_Port,RF2_RESETN_Pin,GPIO_PIN_SET);
	else
		HAL_GPIO_WritePin(RF2_RESETN_GPIO_Port,RF2_RESETN_Pin,GPIO_PIN_RESET);
}

void RF3_reset(char stat)
{
	if(stat)
		HAL_GPIO_WritePin(RF3_RESETN_GPIO_Port,RF3_RESETN_Pin,GPIO_PIN_SET);
	else
		HAL_GPIO_WritePin(RF3_RESETN_GPIO_Port,RF3_RESETN_Pin,GPIO_PIN_RESET);
}

void RF1_power(char stat)
{
	if(stat)
		HAL_GPIO_WritePin(RF1_EN_GPIO_Port,RF1_EN_Pin,GPIO_PIN_SET);
	else
		HAL_GPIO_WritePin(RF1_EN_GPIO_Port,RF1_EN_Pin,GPIO_PIN_RESET);
}


void RF2_power(char stat)
{
	if(stat)
		HAL_GPIO_WritePin(RF2_EN_GPIO_Port,RF2_EN_Pin,GPIO_PIN_SET);
	else
		HAL_GPIO_WritePin(RF2_EN_GPIO_Port,RF2_EN_Pin,GPIO_PIN_RESET);
}

void RF3_power(char stat)
{
	if(stat)
		HAL_GPIO_WritePin(RF3_EN_GPIO_Port,RF3_EN_Pin,GPIO_PIN_SET);
	else
		HAL_GPIO_WritePin(RF3_EN_GPIO_Port,RF3_EN_Pin,GPIO_PIN_RESET);
}








void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == GPIO_PIN_3)
		sx1276_hw_operations_1.irq_hand_io0();
	else if(GPIO_Pin == GPIO_PIN_5)
		sx1276_hw_operations_2.irq_hand_io0();
	else if(GPIO_Pin == GPIO_PIN_8)
		sx1276_hw_operations_3.irq_hand_io0();	
}


	
	




/*
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
*/








void hw_init(void)
{

	sx1276_hw_operations_1.spi_readwrite = SPI1_ReadWrite;
	sx1276_hw_operations_1.spi_cs = SPI1_Cs;
	sx1276_hw_operations_1.power = RF1_power;
	sx1276_hw_operations_1.io_read = io1_read;
	sx1276_hw_operations_1.reset = RF1_reset;
	sx1276_hw_operations_1.c_rxtx = RF1_crxtx;
	sx1276_hw_operations_1.irq_hand_io0 = RF1_irq_hand_io0;
	
	sx1276_hw_operations_2.spi_readwrite = SPI5_ReadWrite;
	sx1276_hw_operations_2.spi_cs = SPI5_Cs;
	sx1276_hw_operations_2.power = RF2_power;
	sx1276_hw_operations_2.io_read = io2_read;
	sx1276_hw_operations_2.reset = RF2_reset;
	sx1276_hw_operations_2.c_rxtx = RF2_crxtx;
	sx1276_hw_operations_2.irq_hand_io0 = RF2_irq_hand_io0;
	
	sx1276_hw_operations_3.spi_readwrite = SPI4_ReadWrite;
	sx1276_hw_operations_3.spi_cs = SPI4_Cs;
	sx1276_hw_operations_3.power = RF3_power;
	sx1276_hw_operations_3.io_read = io3_read;
	sx1276_hw_operations_3.reset = RF3_reset;
	sx1276_hw_operations_3.c_rxtx = RF3_crxtx;	
	sx1276_hw_operations_3.irq_hand_io0 = RF3_irq_hand_io0;	

}



void _UserSX1276_1(void) 
{   
do{p_sx1276_hw_operations = &sx1276_hw_operations_1; SX1276LR = ( tSX1276LR* )SX1276Regs_1; SX1276 = ( tSX1276* )SX1276Regs_1;}while(0);
}
void _UserSX1276_2(void)     
{do{p_sx1276_hw_operations = &sx1276_hw_operations_2; SX1276LR = ( tSX1276LR* )SX1276Regs_2; SX1276 = ( tSX1276* )SX1276Regs_2;}while(0);
}
void _UserSX1276_3(void)   
{  do{p_sx1276_hw_operations = &sx1276_hw_operations_3; SX1276LR = ( tSX1276LR* )SX1276Regs_3; SX1276 = ( tSX1276* )SX1276Regs_3;}while(0);
}




















