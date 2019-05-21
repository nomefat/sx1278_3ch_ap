#include "sx1276.h"
#include "rf_hal.h"
#include "rf.h"
#include "sx1276-LoRa.h"
#include "sx1276-LoRaMisc.h"
#include "ap_s.h"
#include "flash.h"


unsigned char rf1_rx_buff[64];
unsigned char rf2_rx_buff[64];
unsigned char rf3_rx_buff[64];

int rf1_test_send_1000packet = 0;
int rf2_test_send_1000packet = 0;
int rf3_test_send_1000packet = 0;


unsigned char rf1_test_buff[11];
unsigned char rf2_test_buff[11];
unsigned char rf3_test_buff[11];

struct _test_1000p_recode
{
	int now_packet_seq;
	int rev_count;
}test_1000p_recode[3];

int rf1_test_mode = 0;
int rf2_test_mode = 0;
int rf3_test_mode = 0;


int rf1_rev_ap_syn_slot;
int rf2_rev_ap_syn_slot;
int rf3_rev_ap_syn_slot;

struct_syn_send_ok rf1_syn_send_ok;
struct_syn_send_ok rf2_syn_send_ok;
struct_syn_send_ok rf3_syn_send_ok;

struct_syn_rev_ok rf1_rev_ok;
struct_syn_rev_ok rf2_rev_ok;
struct_syn_rev_ok rf3_rev_ok;

extern char debug_send_buff[];

extern int slot(void);
extern void copy_string_to_double_buff(char *pstr);



const int rf_channel_table[41] = 
{
	470000000,471000000,472000000,473000000,
	474000000,475000000,476000000,477000000,
	478000000,479000000,480000000,481000000,
	482000000,483000000,484000000,485000000,
	486000000,487000000,488000000,489000000,
	490000000,491000000,492000000,493000000,
	494000000,495000000,496000000,497000000,
	498000000,499000000,500000000,501000000,
	502000000,503000000,504000000,505000000,
	506000000,507000000,508000000,509000000,
	510000000
};




int test1000p_rssi = 0;










void crc_add(void *pdata)
{
	int i = 0;
	uint8_t add = 0;
	
	for(i=0;i<10;i++)
		add += ((uint8_t *)pdata)[i];
	
	((uint8_t *)pdata)[10] = add;
}

int crc_add_judge(void *pdata)
{
	int i = 0;
	uint8_t add = 0;
	
	for(i=0;i<10;i++)
		add += ((uint8_t *)pdata)[i];
	
	return (((uint8_t *)pdata)[10] == add);
}



void RF_init(void)
{
	SX1276InitIo();
	
	UserSX1276_1
	SX1276Power(1);
	SX1276Init();
	RF1_set_ch(rf_channel_table[(ap_param.ap_channel>>24) & 0x7f]);

	
	UserSX1276_2
	SX1276Power(1);
	SX1276Init();	
	RF2_set_ch(rf_channel_table[(ap_param.ap_channel>>16) & 0x7f]);
	//SX1276LoRaStartRxPkt(); //切换到接收模式
	
	UserSX1276_3
	SX1276Power(1);
	SX1276Init();	
	RF3_set_ch(rf_channel_table[(ap_param.ap_channel>>8) & 0x7f]);
	
}


void debug_rf_send_syn_ok()
{
	if(rf1_syn_send_ok.syn_send_ok)
	{
		rf1_syn_send_ok.syn_send_ok = 0;
		sprintf(debug_send_buff,"rf1 num=%d slot=%d %dus syn send ok\r\n",rf1_syn_send_ok.number,rf1_syn_send_ok.now_slot,rf1_syn_send_ok.now_us);
		copy_string_to_double_buff(debug_send_buff);	
	}
	if(rf2_syn_send_ok.syn_send_ok)
	{
		rf2_syn_send_ok.syn_send_ok = 0;
		sprintf(debug_send_buff,"rf2 num=%d slot=%d %dus syn send ok\r\n",rf2_syn_send_ok.number,rf2_syn_send_ok.now_slot,rf2_syn_send_ok.now_us);
		copy_string_to_double_buff(debug_send_buff);	
	}
	if(rf3_syn_send_ok.syn_send_ok)
	{
		rf3_syn_send_ok.syn_send_ok = 0;
		sprintf(debug_send_buff,"rf3 num=%d slot=%d %dus syn send ok\r\n",rf3_syn_send_ok.number,rf3_syn_send_ok.now_slot,rf3_syn_send_ok.now_us);
		copy_string_to_double_buff(debug_send_buff);	
	}	
}





void RF1_set_ch(uint32_t ch)
{
	uint8_t opModePrev = RFLR_OPMODE_STANDBY;
	
	UserSX1276_1
	opModePrev = SX1276LR->RegOpMode & ~RFLR_OPMODE_MASK;
	SX1276LoRaSetOpMode( RFLR_OPMODE_STANDBY );
	SX1276LoRaSetRFFrequency(ch);
	SX1276LoRaSetOpMode( opModePrev );
	
}

void RF2_set_ch(uint32_t ch)
{
	uint8_t opModePrev = RFLR_OPMODE_STANDBY;
	
	UserSX1276_2
	opModePrev = SX1276LR->RegOpMode & ~RFLR_OPMODE_MASK;
	SX1276LoRaSetOpMode( RFLR_OPMODE_STANDBY );
	SX1276LoRaSetRFFrequency(ch);
	SX1276LoRaSetOpMode( opModePrev );
	
}

void RF3_set_ch(uint32_t ch)
{
	uint8_t opModePrev = RFLR_OPMODE_STANDBY;
	
	UserSX1276_3
	opModePrev = SX1276LR->RegOpMode & ~RFLR_OPMODE_MASK;
	SX1276LoRaSetOpMode( RFLR_OPMODE_STANDBY );
	SX1276LoRaSetRFFrequency(ch);
	SX1276LoRaSetOpMode( opModePrev );
	
}

void RF1_Send_packet_step1(void *pdata)
{

		UserSX1276_1
	
		SX1276LoRaSetOpMode( RFLR_OPMODE_STANDBY );	
	
    SX1276Write( REG_LR_FIFOADDRPTR, 0 );
	
	  SX1276WriteFifo( pdata, 11 );
                                        // TxDone               RxTimeout                   FhssChangeChannel          ValidHeader         
    SX1276LR->RegDioMapping1 = RFLR_DIOMAPPING1_DIO0_01 | RFLR_DIOMAPPING1_DIO1_00 | RFLR_DIOMAPPING1_DIO2_00 | RFLR_DIOMAPPING1_DIO3_01;
                                        // PllLock              Mode Ready
    SX1276LR->RegDioMapping2 = RFLR_DIOMAPPING2_DIO4_01 | RFLR_DIOMAPPING2_DIO5_00;
    SX1276WriteBuffer( REG_LR_DIOMAPPING1, &SX1276LR->RegDioMapping1, 2 );

//    SX1276LoRaSetOpMode( RFLR_OPMODE_TRANSMITTER );	

}

void RF2_Send_packet_step1(void *pdata)
{

		UserSX1276_2
	
		SX1276LoRaSetOpMode( RFLR_OPMODE_STANDBY );	
	
    SX1276Write( REG_LR_FIFOADDRPTR, 0 );
	
	  SX1276WriteFifo( pdata, 11 );
                                        // TxDone               RxTimeout                   FhssChangeChannel          ValidHeader         
    SX1276LR->RegDioMapping1 = RFLR_DIOMAPPING1_DIO0_01 | RFLR_DIOMAPPING1_DIO1_00 | RFLR_DIOMAPPING1_DIO2_00 | RFLR_DIOMAPPING1_DIO3_01;
                                        // PllLock              Mode Ready
    SX1276LR->RegDioMapping2 = RFLR_DIOMAPPING2_DIO4_01 | RFLR_DIOMAPPING2_DIO5_00;
    SX1276WriteBuffer( REG_LR_DIOMAPPING1, &SX1276LR->RegDioMapping1, 2 );

//    SX1276LoRaSetOpMode( RFLR_OPMODE_TRANSMITTER );	

}

void RF3_Send_packet_step1(void *pdata)
{

		UserSX1276_3
	
		SX1276LoRaSetOpMode( RFLR_OPMODE_STANDBY );	
	
    SX1276Write( REG_LR_FIFOADDRPTR, 0 );
	
	  SX1276WriteFifo( pdata, 11 );
                                        // TxDone               RxTimeout                   FhssChangeChannel          ValidHeader         
    SX1276LR->RegDioMapping1 = RFLR_DIOMAPPING1_DIO0_01 | RFLR_DIOMAPPING1_DIO1_00 | RFLR_DIOMAPPING1_DIO2_00 | RFLR_DIOMAPPING1_DIO3_01;
                                        // PllLock              Mode Ready
    SX1276LR->RegDioMapping2 = RFLR_DIOMAPPING2_DIO4_01 | RFLR_DIOMAPPING2_DIO5_00;
    SX1276WriteBuffer( REG_LR_DIOMAPPING1, &SX1276LR->RegDioMapping1, 2 );

//    SX1276LoRaSetOpMode( RFLR_OPMODE_TRANSMITTER );	

}

void RF1_Send_packet_step2(void)
{
	UserSX1276_1
	SX1276LoRaSetOpMode( RFLR_OPMODE_TRANSMITTER );	
}

void RF2_Send_packet_step2(void)
{
	UserSX1276_2
	SX1276LoRaSetOpMode( RFLR_OPMODE_TRANSMITTER );	
}

void RF3_Send_packet_step2(void)
{
	UserSX1276_3
	SX1276LoRaSetOpMode( RFLR_OPMODE_TRANSMITTER );	
}


void RF1_Send_packet(void *pdata)
{

		UserSX1276_1
	
		SX1276LoRaSetOpMode( RFLR_OPMODE_STANDBY );	
	
    SX1276Write( REG_LR_FIFOADDRPTR, 0 );
	
	  SX1276WriteFifo( pdata, 11 );
                                        // TxDone               RxTimeout                   FhssChangeChannel          ValidHeader         
    SX1276LR->RegDioMapping1 = RFLR_DIOMAPPING1_DIO0_01 | RFLR_DIOMAPPING1_DIO1_00 | RFLR_DIOMAPPING1_DIO2_00 | RFLR_DIOMAPPING1_DIO3_01;
                                        // PllLock              Mode Ready
    SX1276LR->RegDioMapping2 = RFLR_DIOMAPPING2_DIO4_01 | RFLR_DIOMAPPING2_DIO5_00;
    SX1276WriteBuffer( REG_LR_DIOMAPPING1, &SX1276LR->RegDioMapping1, 2 );

    SX1276LoRaSetOpMode( RFLR_OPMODE_TRANSMITTER );	

}

void RF2_Send_packet(void *pdata)
{

		UserSX1276_2
	
		SX1276LoRaSetOpMode( RFLR_OPMODE_STANDBY );	
	
    SX1276Write( REG_LR_FIFOADDRPTR, 0 );
	
	  SX1276WriteFifo( pdata, 11 );
                                        // TxDone               RxTimeout                   FhssChangeChannel          ValidHeader         
    SX1276LR->RegDioMapping1 = RFLR_DIOMAPPING1_DIO0_01 | RFLR_DIOMAPPING1_DIO1_00 | RFLR_DIOMAPPING1_DIO2_00 | RFLR_DIOMAPPING1_DIO3_01;
                                        // PllLock              Mode Ready
    SX1276LR->RegDioMapping2 = RFLR_DIOMAPPING2_DIO4_01 | RFLR_DIOMAPPING2_DIO5_00;
    SX1276WriteBuffer( REG_LR_DIOMAPPING1, &SX1276LR->RegDioMapping1, 2 );

    SX1276LoRaSetOpMode( RFLR_OPMODE_TRANSMITTER );	

}

void RF3_Send_packet(void *pdata)
{

		UserSX1276_3
	
		SX1276LoRaSetOpMode( RFLR_OPMODE_STANDBY );	
	
    SX1276Write( REG_LR_FIFOADDRPTR, 0 );
	
	  SX1276WriteFifo( pdata, 11 );
                                        // TxDone               RxTimeout                   FhssChangeChannel          ValidHeader         
    SX1276LR->RegDioMapping1 = RFLR_DIOMAPPING1_DIO0_01 | RFLR_DIOMAPPING1_DIO1_00 | RFLR_DIOMAPPING1_DIO2_00 | RFLR_DIOMAPPING1_DIO3_01;
                                        // PllLock              Mode Ready
    SX1276LR->RegDioMapping2 = RFLR_DIOMAPPING2_DIO4_01 | RFLR_DIOMAPPING2_DIO5_00;
    SX1276WriteBuffer( REG_LR_DIOMAPPING1, &SX1276LR->RegDioMapping1, 2 );

    SX1276LoRaSetOpMode( RFLR_OPMODE_TRANSMITTER );	

}

static uint32_t send_timeout = 0;
extern int rf1_sending;
extern int32_t test_1000p_send_flag;
extern tLoRaSettings LoRaSettings;
extern struct_test1000p test1000p;
extern uint32_t test_packet_count;
int32_t rev_test_count;
int32_t rev_rssi;
int32_t rev_test_count2;
int32_t rev_rssi2;
	 int32_t test_delay_time = 20;
	 uint32_t time_slot = 0;
uint16_t test_sensor_rev_count_start = 0;
void test_1000packet_send(void)
{
	uint32_t i;
	int32_t delay;


	
	if(HAL_GetTick()%20 == 0 && (rf1_test_mode == 2018 || rf2_test_mode == 2018 ||rf3_test_mode == 2018))
	{
	
		if(rf1_test_send_1000packet)
		{
			rf1_test_buff[0] = 0xaa;
			rf1_test_buff[1] = 0x55;
			rf1_test_buff[2] = (1000-rf1_test_send_1000packet)&0xff;
			rf1_test_buff[3] = ((1000-rf1_test_send_1000packet)>>8)&0xff;
			
			HAL_GPIO_WritePin(LED5_GPIO_Port,LED5_Pin,GPIO_PIN_RESET)	;
			crc_add(rf1_test_buff);
			RF1_Send_packet(rf1_test_buff);		
			rf1_test_send_1000packet--;
		}

		if(rf2_test_send_1000packet)
		{
			rf2_test_buff[0] = 0xaa;
			rf2_test_buff[1] = 0x55;
			rf2_test_buff[2] = (1000-rf2_test_send_1000packet)&0xff;
			rf2_test_buff[3] = ((1000-rf2_test_send_1000packet)>>8)&0xff;
			
			HAL_GPIO_WritePin(LED4_GPIO_Port,LED4_Pin,GPIO_PIN_RESET)	;
			crc_add(rf2_test_buff);
			RF2_Send_packet(rf2_test_buff);		
			rf2_test_send_1000packet--;
		}

		if(rf3_test_send_1000packet)
		{
			rf3_test_buff[0] = 0xaa;
			rf3_test_buff[1] = 0x55;
			rf3_test_buff[2] = (1000-rf3_test_send_1000packet)&0xff;
			rf3_test_buff[3] = ((1000-rf3_test_send_1000packet)>>8)&0xff;
			
			HAL_GPIO_WritePin(LED3_GPIO_Port,LED3_Pin,GPIO_PIN_RESET)	;
			crc_add(rf3_test_buff);
			RF3_Send_packet(rf3_test_buff);		
			rf3_test_send_1000packet--;
		}	
		
		while(HAL_GetTick()%20 == 0 );
	}
	
	if(rf1_test_mode == 2019)
	{
		if(test_1000p_send_flag == 1)
		{
			delay = 10000000;
			while(delay--);
			test1000p.cmd = 3;
			//RF1_Send_packet(&test1000p);	
			delay = 10000000;
			while(test_1000p_send_flag == 1 && delay)
			{
				delay--;
			}
		
		}
		else if(test_1000p_send_flag == 2)
		{
//			UserSX1276_2
//			SX1276Power(0);
//			delay = 1000000;
//			while(delay--);			
//			SX1276Power(1);
//			SX1276Init();
//			RF1_set_ch(test1000p.freq*1000000);
//			RF2_Send_packet(&test1000p);
//			SX1276LoRaStartRxPkt(); //切换到接收模式			
			
			UserSX1276_1
			SX1276Power(0);
			delay = 1000000;
			while(delay--);			
			SX1276Power(1);
			SX1276Init();
			RF1_set_ch(test1000p.freq*1000000);
			
			test_1000p_send_flag = 3;
			test_delay_time = test1000p.time*2/5;
			delay = 1000000;
			while(delay--);	
			test1000p.packet_seq = 0;
			send_timeout = 0;
			rev_test_count = 0;
			rev_rssi = 0;
			test_sensor_rev_count_start = 0;
		}
		else if(test_1000p_send_flag == 3)
		{
			if(slot_count - time_slot > test_delay_time)
			{
				time_slot = slot_count;
				test1000p.cmd = 2;
				test1000p.packet_count = test_packet_count;
				RF1_Send_packet(&test1000p);	

//				if(test1000p.packet_seq >= test1000p.packet_count)
//					test_1000p_send_flag = -1;
			}
		}
	}
	
}








#define RSSI_OFFSET_LF                              -155.0
#define RSSI_OFFSET_HF                              -150.0

#define NOISE_ABSOLUTE_ZERO                         -174.0

#define NOISE_FIGURE_LF                                4.0
#define NOISE_FIGURE_HF                                6.0 
int8_t RxPacketRssiValue_1,RxPacketRssiValue_2,RxPacketRssiValue_3;
extern const double RssiOffsetLF[];
extern tLoRaSettings LoRaSettings;
extern const double SignalBwLog[];


void RF1_irq_hand_io0(void)
{
	char *pstr;
	static int error;
	uint8_t rxSnrEstimate;
	int8_t RxPacketSnrEstimate;
	uint32_t seq;
	uint32_t freq;
	struct_syn_packet *p_syn = (struct_syn_packet *)rf1_rx_buff;
	struct_sensor_rftest_packet *p_testrf = (struct_sensor_rftest_packet *)rf1_rx_buff;
	struct_sensor_status_packet *p_sensor_status = (struct_sensor_status_packet *)rf1_rx_buff;	
	struct_test1000p *p_test1000p = (struct_test1000p *)rf1_rx_buff;


	
	UserSX1276_1
	
	SX1276Read( REG_LR_IRQFLAGS, &SX1276LR->RegIrqFlags );
	
	if((SX1276LR->RegIrqFlags & RFLR_IRQFLAGS_TXDONE)==RFLR_IRQFLAGS_TXDONE) //TxDone
	{
		HAL_GPIO_WritePin(LED5_GPIO_Port,LED5_Pin,GPIO_PIN_SET)	;
		SX1276Write( REG_LR_IRQFLAGS, RFLR_IRQFLAGS_TXDONE  );
		SX1276LoRaStartRxPkt(); //切换到接收模式
		
		rf1_sending = 0;
		if(rf1_test_mode == 2018)
		{
			freq = (SX1276LR->RegFrfMsb << 16) + (SX1276LR->RegFrfMid << 8) + (SX1276LR->RegFrfLsb);
			freq = (int)(FREQ_STEP *freq);			
			sprintf(debug_send_buff,"rf1_%d send ok %d\r\n",freq,1000-rf1_test_send_1000packet);
			copy_string_to_double_buff(debug_send_buff);				
		}
		else if(rf1_test_mode == 2019)
		{
			
			freq = (SX1276LR->RegFrfMsb << 16) + (SX1276LR->RegFrfMid << 8) + (SX1276LR->RegFrfLsb);
			freq = (int)(FREQ_STEP *freq);	
			if(test_1000p_send_flag == 0)
			{				
				sprintf(debug_send_buff," rf1_test_%d set_sensor ",freq);
				copy_string_to_double_buff(debug_send_buff);
			}	
			else if(test_1000p_send_flag == 1)
			{
				test_1000p_send_flag = 2;
				sprintf(debug_send_buff,"rf1_test_%d begin\r\n",freq);
				copy_string_to_double_buff(debug_send_buff);				
			}
			else if(test_1000p_send_flag == 3)
			{
				
				sprintf(debug_send_buff,"\rrf1_test_%d send %d   ",freq,test1000p.packet_seq);
				copy_string_to_double_buff(debug_send_buff);	
				test1000p.packet_seq++;
//				if(test1000p.packet_seq == 0)
//				{
//					send_timeout++;
//					if(send_timeout>50)
//						test_1000p_send_flag = -1;
//				}
//				else
//				{
//					test1000p.packet_seq++;
//				}				
			}
		}
		else
		{
			rf1_syn_send_ok.syn_send_ok = 1;
			rf1_syn_send_ok.number++;
			rf1_syn_send_ok.now_slot = slot();
			rf1_syn_send_ok.now_us = (840820 - SysTick->VAL)/168;
			//sprintf(debug_send_buff,"rf1 num=%d slot=%d %dus syn send ok\r\n",slot_count/50,slot(),(840820 - SysTick->VAL)/168);
			//copy_string_to_double_buff(debug_send_buff);				
		}
		return;
	}

  if( ( SX1276LR->RegIrqFlags & RFLR_IRQFLAGS_PAYLOADCRCERROR ) == RFLR_IRQFLAGS_PAYLOADCRCERROR )//CRC ERROR
  {
     // Clear Irq					
     SX1276Write( REG_LR_IRQFLAGS, RFLR_IRQFLAGS_PAYLOADCRCERROR  );            
		 return;
  }
	
	if((SX1276LR->RegIrqFlags & RFLR_IRQFLAGS_RXDONE)==RFLR_IRQFLAGS_RXDONE)  //RxDone
	{
		SX1276Write( REG_LR_IRQFLAGS, RFLR_IRQFLAGS_RXDONE  );

		SX1276Read( REG_LR_FIFORXCURRENTADDR, &SX1276LR->RegFifoRxCurrentAddr );
			

		SX1276LR->RegFifoAddrPtr = SX1276LR->RegFifoRxCurrentAddr;

		SX1276Write( REG_LR_FIFOADDRPTR, SX1276LR->RegFifoAddrPtr);	

		SX1276ReadFifo( rf1_rx_buff, 11 );	

		SX1276LoRaSetOpMode( RFLR_OPMODE_STANDBY );
		SX1276Write( REG_LR_FIFOADDRPTR, 0 );
		SX1276LoRaSetOpMode( RFLR_OPMODE_RECEIVER );


		SX1276Read( REG_LR_PKTSNRVALUE, &rxSnrEstimate );
		if( rxSnrEstimate & 0x80 ) // The SNR sign bit is 1
		{
				// Invert and divide by 4
				RxPacketSnrEstimate = ( ( ~rxSnrEstimate + 1 ) & 0xFF ) >> 2;
				RxPacketSnrEstimate = -RxPacketSnrEstimate;
		}
		else
		{
				// Divide by 4
				RxPacketSnrEstimate = ( rxSnrEstimate & 0xFF ) >> 2;
		}		

			if( RxPacketSnrEstimate < 0 )
		{
				RxPacketRssiValue_1 = NOISE_ABSOLUTE_ZERO + 10.0 * SignalBwLog[LoRaSettings.SignalBw] + NOISE_FIGURE_LF + ( double )RxPacketSnrEstimate;
		}
		else
		{    
				SX1276Read( REG_LR_PKTRSSIVALUE, &SX1276LR->RegPktRssiValue );
				RxPacketRssiValue_1 = RssiOffsetLF[LoRaSettings.SignalBw] + ( double )SX1276LR->RegPktRssiValue;
		}		

		rf1_rev_ok.rf_rev_ok++;
		rf1_rev_ok.number++;
		rf1_rev_ok.now_slot = slot();
		rf1_rev_ok.now_us = (840820 - SysTick->VAL)/168;
		
		if(rf1_test_mode == 2018)
		{
			freq = (SX1276LR->RegFrfMsb << 16) + (SX1276LR->RegFrfMid << 8) + (SX1276LR->RegFrfLsb);
			freq = (int)(FREQ_STEP *freq);
			
			if(crc_add_judge(rf1_rx_buff) == 0)
			{
				sprintf(debug_send_buff,"rf1_%d rev %d seq=%d crc_error\r\n",freq,RxPacketRssiValue_1,test_1000p_recode[0].now_packet_seq);
				copy_string_to_double_buff(debug_send_buff);
				
			}
			else if(rf1_rx_buff[0] == 0xaa && rf1_rx_buff[1] == 0x55 )
			{
				seq = rf1_rx_buff[2] + rf1_rx_buff[3]*256;
				if(test_1000p_recode[0].now_packet_seq > seq)
				{
					test_1000p_recode[0].rev_count = 0;
					test1000p_rssi = 0;
				}
				test_1000p_recode[0].now_packet_seq = seq;
				test_1000p_recode[0].rev_count++;

				test1000p_rssi += RxPacketRssiValue_1;
				sprintf(debug_send_buff,"rf1_%d rev %d avg=%d seq=%d count=%d\r\n",freq,RxPacketRssiValue_1,test1000p_rssi/test_1000p_recode[0].rev_count,test_1000p_recode[0].now_packet_seq,test_1000p_recode[0].rev_count);
				copy_string_to_double_buff(debug_send_buff);		
			}				
		}
		else if(rf1_test_mode == 2019)
		{
//			sprintf(debug_send_buff,"rf1_test rev %d %d  ",p_test1000p->packet_seq,RxPacketRssiValue_1);
//			copy_string_to_double_buff(debug_send_buff);	
			if(p_test1000p->head == 0x55aa && p_test1000p->cmd == 3)
			{
				test_1000p_send_flag = 1;
				sprintf(debug_send_buff," rf1_test rev setack\r\n");
				copy_string_to_double_buff(debug_send_buff);				
			}
			else if(p_test1000p->head == 0x55aa && p_test1000p->cmd == 2)
			{
				if(test1000p.packet_seq == 0)
					test1000p.packet_seq++;
				if(test_sensor_rev_count_start == 0)
					test_sensor_rev_count_start = p_test1000p->packet_count;
				rev_test_count++;
				rev_rssi += RxPacketRssiValue_1;
				sprintf(debug_send_buff,"rf1_test rev [%d/%d] %d/%d       [%d/%d] %d       ",p_test1000p->packet_seq,p_test1000p->packet_count,p_test1000p->rssi, RxPacketRssiValue_1,rev_test_count,p_test1000p->packet_count-test_sensor_rev_count_start,rev_rssi/rev_test_count);
				copy_string_to_double_buff(debug_send_buff);	
				time_slot = slot_count - 16;				
			}
		}
		else if(p_syn->head.type == PACKAGE_TYPE_SYNC&& cal_crc_table(rf1_rx_buff,10) == rf1_rx_buff[10])
		{
			rf1_rev_ap_syn_slot = slot();
			sprintf(debug_send_buff,"rf1 rev syn seq=%d slot=%d tick=%d rssi=%d %X %X %X %X %X %X %X %X %X %X %X\r\n",p_syn->head.packet_num,slot(),(840820 - SysTick->VAL)/168,RxPacketRssiValue_1,
			rf1_rx_buff[0],rf1_rx_buff[1],rf1_rx_buff[2],rf1_rx_buff[3],rf1_rx_buff[4],rf1_rx_buff[5],rf1_rx_buff[6],rf1_rx_buff[7],rf1_rx_buff[8],
			rf1_rx_buff[9],rf1_rx_buff[10]);		
			copy_string_to_double_buff(debug_send_buff);			
		}
		else if(p_syn->head.type == PACKAGE_TYPE_TESTRF)
		{			
			sprintf(debug_send_buff,"rf1 rev testrf seq=%d slot=%d tick=%d rssi=%d\r\n",p_testrf->seq,slot(),(840820 - SysTick->VAL)/168,RxPacketRssiValue_1);
			copy_string_to_double_buff(debug_send_buff);			
		}
		else if(p_syn->head.type == PACKAGE_TYPE_STATUS && cal_crc_table(rf1_rx_buff,10) == rf1_rx_buff[10])
		{
			sprintf(debug_send_buff,"rf1 rev sensor_status_packet id=%04X seq=%d slot=%d tick=%d band_id=%04X volt=%d,v=%d mode=%d ch=%d sensor_rssi=%d rssi=%d\r\n",p_sensor_status->head.dev_id,p_sensor_status->head.packet_num,slot(),(840820 - SysTick->VAL)/168,
			p_sensor_status->band_id,p_sensor_status->battary_volt,p_sensor_status->firm_version,p_sensor_status->mode,p_sensor_status->rf_channel, p_sensor_status->rssi,RxPacketRssiValue_1);
			copy_string_to_double_buff(debug_send_buff);				
		}			
	}
}

void RF2_irq_hand_io0(void)
{
	char *pstr;
	static int error;
	uint8_t rxSnrEstimate;
	int8_t RxPacketSnrEstimate;
	uint32_t seq;
	uint32_t freq;
	struct_syn_packet *p_syn = (struct_syn_packet *)rf2_rx_buff;
	struct_sensor_rftest_packet *p_testrf = (struct_sensor_rftest_packet *)rf2_rx_buff;
	struct_sensor_status_packet *p_sensor_status = (struct_sensor_status_packet *)rf2_rx_buff;
	struct_test1000p *p_test1000p = (struct_test1000p *)rf2_rx_buff;
	
	UserSX1276_2
	
	SX1276Read( REG_LR_IRQFLAGS, &SX1276LR->RegIrqFlags );
	
	if((SX1276LR->RegIrqFlags & RFLR_IRQFLAGS_TXDONE)==RFLR_IRQFLAGS_TXDONE) //TxDone
	{
		HAL_GPIO_WritePin(LED4_GPIO_Port,LED4_Pin,GPIO_PIN_SET)	;
		SX1276Write( REG_LR_IRQFLAGS, RFLR_IRQFLAGS_TXDONE  );
		SX1276LoRaStartRxPkt(); //切换到接收模式
		if(rf2_test_mode == 2018)
		{
			freq = (SX1276LR->RegFrfMsb << 16) + (SX1276LR->RegFrfMid << 8) + (SX1276LR->RegFrfLsb);
			freq = (int)(FREQ_STEP *freq);				
			sprintf(debug_send_buff,"rf2_%d send ok %d\r\n",freq,1000-rf2_test_send_1000packet);
			copy_string_to_double_buff(debug_send_buff);				
		}		
		else
		{
			rf2_syn_send_ok.syn_send_ok = 1;
			rf2_syn_send_ok.number++;
			rf2_syn_send_ok.now_slot = slot();
			rf2_syn_send_ok.now_us = (840820 - SysTick->VAL)/168;
			//sprintf(debug_send_buff,"rf2 num=%d slot=%d %dus syn send ok\r\n",slot_count/50,slot(),(840820 - SysTick->VAL)/168);
			//copy_string_to_double_buff(debug_send_buff);				
		}
		return;
	}

  if( ( SX1276LR->RegIrqFlags & RFLR_IRQFLAGS_PAYLOADCRCERROR ) == RFLR_IRQFLAGS_PAYLOADCRCERROR )//CRC ERROR
  {
     // Clear Irq					
     SX1276Write( REG_LR_IRQFLAGS, RFLR_IRQFLAGS_PAYLOADCRCERROR  );            
		 return;
  }
	
	if((SX1276LR->RegIrqFlags & RFLR_IRQFLAGS_RXDONE)==RFLR_IRQFLAGS_RXDONE)  //RxDone
	{
		SX1276Write( REG_LR_IRQFLAGS, RFLR_IRQFLAGS_RXDONE  );

		SX1276Read( REG_LR_FIFORXCURRENTADDR, &SX1276LR->RegFifoRxCurrentAddr );
			

		SX1276LR->RegFifoAddrPtr = SX1276LR->RegFifoRxCurrentAddr;

		SX1276Write( REG_LR_FIFOADDRPTR, SX1276LR->RegFifoAddrPtr);	

		SX1276ReadFifo( rf2_rx_buff, 11 );	

		SX1276LoRaSetOpMode( RFLR_OPMODE_STANDBY );
		SX1276Write( REG_LR_FIFOADDRPTR, 0 );
		SX1276LoRaSetOpMode( RFLR_OPMODE_RECEIVER );

		

		SX1276Read( REG_LR_PKTSNRVALUE, &rxSnrEstimate );
		if( rxSnrEstimate & 0x80 ) // The SNR sign bit is 1
		{
				// Invert and divide by 4
				RxPacketSnrEstimate = ( ( ~rxSnrEstimate + 1 ) & 0xFF ) >> 2;
				RxPacketSnrEstimate = -RxPacketSnrEstimate;
		}
		else
		{
				// Divide by 4
				RxPacketSnrEstimate = ( rxSnrEstimate & 0xFF ) >> 2;
		}		

			if( RxPacketSnrEstimate < 0 )
		{
				RxPacketRssiValue_2 = NOISE_ABSOLUTE_ZERO + 10.0 * SignalBwLog[LoRaSettings.SignalBw] + NOISE_FIGURE_LF + ( double )RxPacketSnrEstimate;
		}
		else
		{    
				SX1276Read( REG_LR_PKTRSSIVALUE, &SX1276LR->RegPktRssiValue );
				RxPacketRssiValue_2 = RssiOffsetLF[LoRaSettings.SignalBw] + ( double )SX1276LR->RegPktRssiValue;
		}		

		rf2_rev_ok.rf_rev_ok++;
		rf2_rev_ok.number++;
		rf2_rev_ok.now_slot = slot();
		rf2_rev_ok.now_us = (840820 - SysTick->VAL)/168;
		
		if(rf2_test_mode == 2018)
		{
			freq = (SX1276LR->RegFrfMsb << 16) + (SX1276LR->RegFrfMid << 8) + (SX1276LR->RegFrfLsb);
			freq = (int)(FREQ_STEP *freq);
			
			if(crc_add_judge(rf2_rx_buff) == 0)
			{
				sprintf(debug_send_buff,"rf2_%d rev %d seq=%d crc_error\r\n",freq,RxPacketRssiValue_2,test_1000p_recode[1].now_packet_seq);
				copy_string_to_double_buff(debug_send_buff);
				
			}
			else if(rf2_rx_buff[0] == 0xaa && rf2_rx_buff[1] == 0x55 )
			{
				seq = rf2_rx_buff[2] + rf2_rx_buff[3]*256;
				if(test_1000p_recode[1].now_packet_seq > seq)
				{
					test_1000p_recode[1].rev_count = 0;
					test1000p_rssi = 0;
				}				
				test_1000p_recode[1].now_packet_seq = seq;
				test_1000p_recode[1].rev_count++;
				
				test1000p_rssi += RxPacketRssiValue_2;
				sprintf(debug_send_buff,"rf2_%d rev %d avg=%d seq=%d count=%d\r\n",freq,RxPacketRssiValue_2,test1000p_rssi/test_1000p_recode[1].rev_count,test_1000p_recode[1].now_packet_seq,test_1000p_recode[1].rev_count);
				copy_string_to_double_buff(debug_send_buff);		
			}				
		}
		else if(rf1_test_mode == 2019)
		{
			if(p_test1000p->head == 0x55aa && p_test1000p->cmd == 2)
			{
				if(test1000p.packet_seq == 0)
					test1000p.packet_seq++;
				
				rev_test_count2++;
				rev_rssi2 += RxPacketRssiValue_2;
				sprintf(debug_send_buff,"rf2_test rev [%d/%d] %d/%d       %d %d       ",p_test1000p->packet_seq,p_test1000p->packet_count,p_test1000p->rssi, RxPacketRssiValue_2,rev_test_count2,rev_rssi2/rev_test_count2);
				copy_string_to_double_buff(debug_send_buff);						
			}
		}		
		else if(p_syn->head.type == PACKAGE_TYPE_SYNC && cal_crc_table(rf2_rx_buff,10) == rf2_rx_buff[10])
		{
			rf2_rev_ap_syn_slot = slot();
			sprintf(debug_send_buff,"rf2 rev syn seq=%d slot=%d tick=%d rssi=%d %X %X %X %X %X %X %X %X %X %X %X\r\n",p_syn->head.packet_num,slot(),(840820 - SysTick->VAL)/168,RxPacketRssiValue_2,
			rf2_rx_buff[0],rf2_rx_buff[1],rf2_rx_buff[2],rf2_rx_buff[3],rf2_rx_buff[4],rf2_rx_buff[5],rf2_rx_buff[6],rf2_rx_buff[7],rf2_rx_buff[8],
			rf2_rx_buff[9],rf2_rx_buff[10]);			copy_string_to_double_buff(debug_send_buff);			
		}
		else if(p_syn->head.type == PACKAGE_TYPE_TESTRF)
		{			
			sprintf(debug_send_buff,"rf2 rev testrf seq=%d slot=%d tick=%d rssi=%d\r\n",p_testrf->seq,slot(),(840820 - SysTick->VAL)/168,RxPacketRssiValue_2);
			copy_string_to_double_buff(debug_send_buff);			
		}	
		else if(p_syn->head.type == PACKAGE_TYPE_STATUS && cal_crc_table(rf2_rx_buff,10) == rf2_rx_buff[10])
		{
			sprintf(debug_send_buff,"rf2 rev sensor_status_packet id=%04X seq=%d slot=%d tick=%d band_id=%04X volt=%d,v=%d mode=%d ch=%d sensor_rssi=%d rssi=%d\r\n",p_sensor_status->head.dev_id,p_sensor_status->head.packet_num,slot(),(840820 - SysTick->VAL)/168,
			p_sensor_status->band_id,p_sensor_status->battary_volt,p_sensor_status->firm_version,p_sensor_status->mode,p_sensor_status->rf_channel, p_sensor_status->rssi,RxPacketRssiValue_2);
			copy_string_to_double_buff(debug_send_buff);				
		}		
	}
}

void RF3_irq_hand_io0(void)
{
	char *pstr;
	static int error;
	uint8_t rxSnrEstimate;
	int8_t RxPacketSnrEstimate;
	uint32_t seq;
	uint32_t freq;
	struct_syn_packet *p_syn = (struct_syn_packet *)rf3_rx_buff;	
	struct_sensor_rftest_packet *p_testrf = (struct_sensor_rftest_packet *)rf3_rx_buff;
	struct_sensor_status_packet *p_sensor_status = (struct_sensor_status_packet *)rf3_rx_buff;
	
	UserSX1276_3
	
	SX1276Read( REG_LR_IRQFLAGS, &SX1276LR->RegIrqFlags );
	
	if((SX1276LR->RegIrqFlags & RFLR_IRQFLAGS_TXDONE)==RFLR_IRQFLAGS_TXDONE) //TxDone
	{
		HAL_GPIO_WritePin(LED3_GPIO_Port,LED3_Pin,GPIO_PIN_SET)	;
		SX1276Write( REG_LR_IRQFLAGS, RFLR_IRQFLAGS_TXDONE  );
		SX1276LoRaStartRxPkt(); //切换到接收模式

		if(rf3_test_mode == 2018)
		{
			freq = (SX1276LR->RegFrfMsb << 16) + (SX1276LR->RegFrfMid << 8) + (SX1276LR->RegFrfLsb);
			freq = (int)(FREQ_STEP *freq);			
			sprintf(debug_send_buff,"rf3_%d send ok %d\r\n",freq,1000-rf3_test_send_1000packet);
			copy_string_to_double_buff(debug_send_buff);				
		}		
		else
		{
			rf3_syn_send_ok.syn_send_ok = 1;
			rf3_syn_send_ok.number++;
			rf3_syn_send_ok.now_slot = slot();
			rf3_syn_send_ok.now_us = (840820 - SysTick->VAL)/168;
			//sprintf(debug_send_buff,"rf3 num=%d slot=%d %dus syn send ok\r\n",slot_count/50,slot(),(840820 - SysTick->VAL)/168);
			//copy_string_to_double_buff(debug_send_buff);				
		}		
		return;
	}

  if( ( SX1276LR->RegIrqFlags & RFLR_IRQFLAGS_PAYLOADCRCERROR ) == RFLR_IRQFLAGS_PAYLOADCRCERROR )//CRC ERROR
  {
     // Clear Irq					
     SX1276Write( REG_LR_IRQFLAGS, RFLR_IRQFLAGS_PAYLOADCRCERROR  );            
		 return;
  }
	
	if((SX1276LR->RegIrqFlags & RFLR_IRQFLAGS_RXDONE)==RFLR_IRQFLAGS_RXDONE)  //RxDone
	{
		SX1276Write( REG_LR_IRQFLAGS, RFLR_IRQFLAGS_RXDONE  );

		SX1276Read( REG_LR_FIFORXCURRENTADDR, &SX1276LR->RegFifoRxCurrentAddr );
			

		SX1276LR->RegFifoAddrPtr = SX1276LR->RegFifoRxCurrentAddr;

		SX1276Write( REG_LR_FIFOADDRPTR, SX1276LR->RegFifoAddrPtr);	

		SX1276ReadFifo( rf3_rx_buff, 11 );	

		SX1276LoRaSetOpMode( RFLR_OPMODE_STANDBY );
		SX1276Write( REG_LR_FIFOADDRPTR, 0 );
		SX1276LoRaSetOpMode( RFLR_OPMODE_RECEIVER );


		SX1276Read( REG_LR_PKTSNRVALUE, &rxSnrEstimate );
		if( rxSnrEstimate & 0x80 ) // The SNR sign bit is 1
		{
				// Invert and divide by 4
				RxPacketSnrEstimate = ( ( ~rxSnrEstimate + 1 ) & 0xFF ) >> 2;
				RxPacketSnrEstimate = -RxPacketSnrEstimate;
		}
		else
		{
				// Divide by 4
				RxPacketSnrEstimate = ( rxSnrEstimate & 0xFF ) >> 2;
		}		

			if( RxPacketSnrEstimate < 0 )
		{
				RxPacketRssiValue_3 = NOISE_ABSOLUTE_ZERO + 10.0 * SignalBwLog[LoRaSettings.SignalBw] + NOISE_FIGURE_LF + ( double )RxPacketSnrEstimate;
		}
		else
		{    
				SX1276Read( REG_LR_PKTRSSIVALUE, &SX1276LR->RegPktRssiValue );
				RxPacketRssiValue_3 = RssiOffsetLF[LoRaSettings.SignalBw] + ( double )SX1276LR->RegPktRssiValue;
		}		

		rf3_rev_ok.rf_rev_ok++;
		rf3_rev_ok.number++;
		rf3_rev_ok.now_slot = slot();
		rf3_rev_ok.now_us = (840820 - SysTick->VAL)/168;
		
		if(rf3_test_mode == 2018)
		{
			freq = (SX1276LR->RegFrfMsb << 16) + (SX1276LR->RegFrfMid << 8) + (SX1276LR->RegFrfLsb);
			freq = (int)(FREQ_STEP *freq);
			
			if(crc_add_judge(rf3_rx_buff) == 0)
			{
				sprintf(debug_send_buff,"rf3_%d rev %d seq=%d crc_error\r\n",freq,RxPacketRssiValue_3,test_1000p_recode[2].now_packet_seq);
				copy_string_to_double_buff(debug_send_buff);
				
			}
			else if(rf3_rx_buff[0] == 0xaa && rf3_rx_buff[1] == 0x55 )
			{
				seq = rf3_rx_buff[2] + rf3_rx_buff[3]*256;
				if(test_1000p_recode[2].now_packet_seq > seq)
				{
					test_1000p_recode[2].rev_count = 0;
					test1000p_rssi = 0;
				}				
				test_1000p_recode[2].now_packet_seq = seq;
				test_1000p_recode[2].rev_count++;

				test1000p_rssi += RxPacketRssiValue_3;
				sprintf(debug_send_buff,"rf3_%d rev %d avg=%d seq=%d count=%d\r\n",freq,RxPacketRssiValue_3,test1000p_rssi/test_1000p_recode[2].rev_count,test_1000p_recode[2].now_packet_seq,test_1000p_recode[2].rev_count);
				copy_string_to_double_buff(debug_send_buff);		
			}				
		}
		else if(p_syn->head.type == PACKAGE_TYPE_SYNC && cal_crc_table(rf3_rx_buff,10) == rf3_rx_buff[10])
		{
			rf3_rev_ap_syn_slot = slot();
			sprintf(debug_send_buff,"rf3 rev syn seq=%d slot=%d tick=%d rssi=%d %X %X %X %X %X %X %X %X %X %X %X\r\n",p_syn->head.packet_num,slot(),(840820 - SysTick->VAL)/168,RxPacketRssiValue_3,
			rf3_rx_buff[0],rf3_rx_buff[1],rf3_rx_buff[2],rf3_rx_buff[3],rf3_rx_buff[4],rf3_rx_buff[5],rf3_rx_buff[6],rf3_rx_buff[7],rf3_rx_buff[8],
			rf3_rx_buff[9],rf3_rx_buff[10]);
			copy_string_to_double_buff(debug_send_buff);			
		}
		else if(p_syn->head.type == PACKAGE_TYPE_TESTRF)
		{			
			sprintf(debug_send_buff,"rf3 rev testrf seq=%d slot=%d tick=%d rssi=%d\r\n",p_testrf->seq,slot(),(840820 - SysTick->VAL)/168,RxPacketRssiValue_3);
			copy_string_to_double_buff(debug_send_buff);			
		}	
		else if(p_syn->head.type == PACKAGE_TYPE_STATUS && cal_crc_table(rf3_rx_buff,10) == rf3_rx_buff[10])
		{
			sprintf(debug_send_buff,"rf3 rev sensor_status_packet id=%04X seq=%d slot=%d tick=%d band_id=%04X volt=%d,v=%d mode=%d ch=%d sensor_rssi=%d rssi=%d\r\n",p_sensor_status->head.dev_id,p_sensor_status->head.packet_num,slot(),(840820 - SysTick->VAL)/168,
			p_sensor_status->band_id,p_sensor_status->battary_volt,p_sensor_status->firm_version,p_sensor_status->mode,p_sensor_status->rf_channel, p_sensor_status->rssi,RxPacketRssiValue_3);
			copy_string_to_double_buff(debug_send_buff);				
		}
	}
}





static const unsigned char crc_table[] =
{
    0x00,0x31,0x62,0x53,0xc4,0xf5,0xa6,0x97,0xb9,0x88,0xdb,0xea,0x7d,0x4c,0x1f,0x2e,
    0x43,0x72,0x21,0x10,0x87,0xb6,0xe5,0xd4,0xfa,0xcb,0x98,0xa9,0x3e,0x0f,0x5c,0x6d,
    0x86,0xb7,0xe4,0xd5,0x42,0x73,0x20,0x11,0x3f,0x0e,0x5d,0x6c,0xfb,0xca,0x99,0xa8,
    0xc5,0xf4,0xa7,0x96,0x01,0x30,0x63,0x52,0x7c,0x4d,0x1e,0x2f,0xb8,0x89,0xda,0xeb,
    0x3d,0x0c,0x5f,0x6e,0xf9,0xc8,0x9b,0xaa,0x84,0xb5,0xe6,0xd7,0x40,0x71,0x22,0x13,
    0x7e,0x4f,0x1c,0x2d,0xba,0x8b,0xd8,0xe9,0xc7,0xf6,0xa5,0x94,0x03,0x32,0x61,0x50,
    0xbb,0x8a,0xd9,0xe8,0x7f,0x4e,0x1d,0x2c,0x02,0x33,0x60,0x51,0xc6,0xf7,0xa4,0x95,
    0xf8,0xc9,0x9a,0xab,0x3c,0x0d,0x5e,0x6f,0x41,0x70,0x23,0x12,0x85,0xb4,0xe7,0xd6,
    0x7a,0x4b,0x18,0x29,0xbe,0x8f,0xdc,0xed,0xc3,0xf2,0xa1,0x90,0x07,0x36,0x65,0x54,
    0x39,0x08,0x5b,0x6a,0xfd,0xcc,0x9f,0xae,0x80,0xb1,0xe2,0xd3,0x44,0x75,0x26,0x17,
    0xfc,0xcd,0x9e,0xaf,0x38,0x09,0x5a,0x6b,0x45,0x74,0x27,0x16,0x81,0xb0,0xe3,0xd2,
    0xbf,0x8e,0xdd,0xec,0x7b,0x4a,0x19,0x28,0x06,0x37,0x64,0x55,0xc2,0xf3,0xa0,0x91,
    0x47,0x76,0x25,0x14,0x83,0xb2,0xe1,0xd0,0xfe,0xcf,0x9c,0xad,0x3a,0x0b,0x58,0x69,
    0x04,0x35,0x66,0x57,0xc0,0xf1,0xa2,0x93,0xbd,0x8c,0xdf,0xee,0x79,0x48,0x1b,0x2a,
    0xc1,0xf0,0xa3,0x92,0x05,0x34,0x67,0x56,0x78,0x49,0x1a,0x2b,0xbc,0x8d,0xde,0xef,
    0x82,0xb3,0xe0,0xd1,0x46,0x77,0x24,0x15,0x3b,0x0a,0x59,0x68,0xff,0xce,0x9d,0xac
};


unsigned char cal_crc_table(unsigned char *ptr, unsigned char len) 
{
    unsigned char  crc = 0x00;

    while (len--)
    {
        crc = crc_table[crc ^ *ptr++];
    }
    return (crc);
}



