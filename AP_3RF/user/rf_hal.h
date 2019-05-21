#ifndef RF_HAL_H_
#define RF_HAL_H_


#include "stm32f4xx_hal.h"
#include "sx1276-LoRa.h"
#include "sx1276-Fsk.h"

typedef struct _sx1276_hw_operations{

	int (*spi_readwrite)(uint8_t *pTxData, uint8_t *pRxData, uint16_t Size);
	void(*spi_cs)(int);
	void(*power)(char);
	char(*io_read)(char);
	void(*reset)(char);
	void(*c_rxtx)(char);
	void(*irq_hand_io0)(void);
	void(*irq_hand_io3)(void);
	void(*irq_hand_io4)(void);
}strcut_sx1276_hw_operations;




#ifndef FUCK
#define FUCK
#include <stdint.h>
#define inline __inline
typedef enum { false = 0, true } bool;
#endif

extern volatile uint32_t TickCounter;
extern strcut_sx1276_hw_operations sx1276_hw_operations_1;
extern strcut_sx1276_hw_operations sx1276_hw_operations_2;
extern strcut_sx1276_hw_operations sx1276_hw_operations_3;
extern strcut_sx1276_hw_operations *p_sx1276_hw_operations;
extern tSX1276LR* SX1276LR;
extern tSX1276* SX1276;

extern uint8_t SX1276Regs_1[];
extern uint8_t SX1276Regs_2[];
extern uint8_t SX1276Regs_3[];


#define      UserSX1276_1    do{p_sx1276_hw_operations = &sx1276_hw_operations_1; SX1276LR = ( tSX1276LR* )SX1276Regs_1; SX1276 = ( tSX1276* )SX1276Regs_1;}while(0);
#define      UserSX1276_2    do{p_sx1276_hw_operations = &sx1276_hw_operations_2; SX1276LR = ( tSX1276LR* )SX1276Regs_2; SX1276 = ( tSX1276* )SX1276Regs_2;}while(0);
#define      UserSX1276_3    do{p_sx1276_hw_operations = &sx1276_hw_operations_3; SX1276LR = ( tSX1276LR* )SX1276Regs_3; SX1276 = ( tSX1276* )SX1276Regs_3;}while(0);

void _UserSX1276_1(void) ;
void _UserSX1276_2(void) ;
void _UserSX1276_3(void) ;

//#define      UserSX1276_1       _UserSX1276_1();
//#define      UserSX1276_2       _UserSX1276_2();
//#define      UserSX1276_3       _UserSX1276_3();


/*!
 * DIO state read functions mapping
 */
#define DIO0                                        SX1276ReadDio0( )
#define DIO1                                        SX1276ReadDio1( )
#define DIO2                                        SX1276ReadDio2( )
#define DIO3                                        SX1276ReadDio3( )
#define DIO4                                        SX1276ReadDio4( )
#define DIO5                                        SX1276ReadDio5( )


// RXTX pin control see errata note
#define RXTX( txEnable )                            SX1276WriteRxTx( txEnable );

#define GET_TICK_COUNT( )                           ( TickCounter )
#define TICK_RATE_MS( ms )                          ( ms )





typedef enum
{
    RADIO_RESET_OFF,
    RADIO_RESET_ON,
}tRadioResetState;
void SX1276Power( uint8_t state );
/*!
 * \brief Initializes the radio interface I/Os
 */
void SX1276InitIo( void );

/*!
 * \brief Set the radio reset pin state
 * 
 * \param state New reset pin state
 */
void SX1276SetReset( uint8_t state );

/*!
 * \brief Writes the radio register at the specified address
 *
 * \param [IN]: addr Register address
 * \param [IN]: data New register value
 */
void SX1276Write( uint8_t addr, uint8_t data );

/*!
 * \brief Reads the radio register at the specified address
 *
 * \param [IN]: addr Register address
 * \param [OUT]: data Register value
 */
void SX1276Read( uint8_t addr, uint8_t *data );

/*!
 * \brief Writes multiple radio registers starting at address
 *
 * \param [IN] addr   First Radio register address
 * \param [IN] buffer Buffer containing the new register's values
 * \param [IN] size   Number of registers to be written
 */
void SX1276WriteBuffer( uint8_t addr, uint8_t *buffer, uint8_t size );

/*!
 * \brief Reads multiple radio registers starting at address
 *
 * \param [IN] addr First Radio register address
 * \param [OUT] buffer Buffer where to copy the registers data
 * \param [IN] size Number of registers to be read
 */
void SX1276ReadBuffer( uint8_t addr, uint8_t *buffer, uint8_t size );

/*!
 * \brief Writes the buffer contents to the radio FIFO
 *
 * \param [IN] buffer Buffer containing data to be put on the FIFO.
 * \param [IN] size Number of bytes to be written to the FIFO
 */
void SX1276WriteFifo( uint8_t *buffer, uint8_t size );

/*!
 * \brief Reads the contents of the radio FIFO
 *
 * \param [OUT] buffer Buffer where to copy the FIFO read data.
 * \param [IN] size Number of bytes to be read from the FIFO
 */
void SX1276ReadFifo( uint8_t *buffer, uint8_t size );

/*!
 * \brief Gets the SX1276 DIO0 hardware pin status
 *
 * \retval status Current hardware pin status [1, 0]
 */
uint8_t SX1276ReadDio0( void );

/*!
 * \brief Gets the SX1276 DIO1 hardware pin status
 *
 * \retval status Current hardware pin status [1, 0]
 */
uint8_t SX1276ReadDio1( void );

/*!
 * \brief Gets the SX1276 DIO2 hardware pin status
 *
 * \retval status Current hardware pin status [1, 0]
 */
uint8_t SX1276ReadDio2( void );

/*!
 * \brief Gets the SX1276 DIO3 hardware pin status
 *
 * \retval status Current hardware pin status [1, 0]
 */
uint8_t SX1276ReadDio3( void );

/*!
 * \brief Gets the SX1276 DIO4 hardware pin status
 *
 * \retval status Current hardware pin status [1, 0]
 */
 uint8_t SX1276ReadDio4( void );

/*!
 * \brief Gets the SX1276 DIO5 hardware pin status
 *
 * \retval status Current hardware pin status [1, 0]
 */
 uint8_t SX1276ReadDio5( void );

/*!
 * \brief Writes the external RxTx pin value
 *
 * \remark see errata note
 *
 * \param [IN] txEnable [1: Tx, 0: Rx]
 */
 void SX1276WriteRxTx( uint8_t txEnable );











#endif

