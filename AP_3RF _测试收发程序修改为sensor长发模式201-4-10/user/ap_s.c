#include "rf.h"
#include "main.h"
#include "ap_s.h"
#include "flash.h"
#include "rf_hal.h"
#include "debug_uart.h"



#define FREAME_SLOT_NUM   50        //一帧50个时间槽

extern int rf1_test_mode;
extern int rf2_test_mode;
extern int rf3_test_mode;


volatile uint32_t slot_count = 0;


struct_syn_packet syn_packet;
struct_ack_packet ack_packet;




int slot(void)
{
	return (slot_count%FREAME_SLOT_NUM);
}




void tick_init(void)
{
	
}

void send_syn(void)
{
	int time = 0 ,t;
	
	syn_packet.head.type = PACKAGE_TYPE_SYNC;
	syn_packet.head.packet_num++;
	syn_packet.head.dev_id = ap_param.band_id;
	if(syn_packet.head.packet_num%4 == 0)
	{
		syn_packet.sec++;
		if(syn_packet.sec > 29)
			syn_packet.sec = 0;
	}
	syn_packet.crc=  cal_crc_table((unsigned char *)&syn_packet,10);
	
	HAL_GPIO_WritePin(LED3_GPIO_Port,LED3_Pin,GPIO_PIN_RESET)	;
	HAL_GPIO_WritePin(LED4_GPIO_Port,LED4_Pin,GPIO_PIN_RESET)	;
	HAL_GPIO_WritePin(LED5_GPIO_Port,LED5_Pin,GPIO_PIN_RESET)	;
	
	RF1_Send_packet_step1(&syn_packet);
	HAL_GPIO_WritePin(LED5_GPIO_Port,LED5_Pin,GPIO_PIN_SET)	;	
	RF2_Send_packet_step1(&syn_packet);
	HAL_GPIO_WritePin(LED4_GPIO_Port,LED4_Pin,GPIO_PIN_SET)	;	
	RF3_Send_packet_step1(&syn_packet);	
	HAL_GPIO_WritePin(LED3_GPIO_Port,LED3_Pin,GPIO_PIN_SET)	;


  RF2_Send_packet_step2();
	HAL_GPIO_WritePin(LED4_GPIO_Port,LED4_Pin,GPIO_PIN_RESET)	;

  RF3_Send_packet_step2();		
	HAL_GPIO_WritePin(LED3_GPIO_Port,LED3_Pin,GPIO_PIN_RESET)	;
	
  RF1_Send_packet_step2();
	HAL_GPIO_WritePin(LED5_GPIO_Port,LED5_Pin,GPIO_PIN_RESET)	;	
	

}

void send_ack(void)
{
	int time = 0 ,t;
	
	if(ack_packet.slot_0 == 0 && ack_packet.slot_1 == 0 && ack_packet.slot_2 == 0 
		&& ack_packet.slot_3 == 0 && ack_packet.slot_4 == 0 && ack_packet.slot_5 == 0)
		return;
	
	ack_packet.head.type = PACKAGE_TYPE_ACK;
	ack_packet.head.packet_num++;
	ack_packet.head.dev_id = ap_param.band_id;


	ack_packet.crc=  cal_crc_table((unsigned char *)&ack_packet,10);
	
	HAL_GPIO_WritePin(LED3_GPIO_Port,LED3_Pin,GPIO_PIN_RESET)	;
	HAL_GPIO_WritePin(LED4_GPIO_Port,LED4_Pin,GPIO_PIN_RESET)	;
	HAL_GPIO_WritePin(LED5_GPIO_Port,LED5_Pin,GPIO_PIN_RESET)	;
	
	RF1_Send_packet_step1(&ack_packet);
	HAL_GPIO_WritePin(LED5_GPIO_Port,LED5_Pin,GPIO_PIN_SET)	;	
	RF2_Send_packet_step1(&ack_packet);
	HAL_GPIO_WritePin(LED4_GPIO_Port,LED4_Pin,GPIO_PIN_SET)	;	
	RF3_Send_packet_step1(&ack_packet);	
	HAL_GPIO_WritePin(LED3_GPIO_Port,LED3_Pin,GPIO_PIN_SET)	;


  RF2_Send_packet_step2();
	HAL_GPIO_WritePin(LED4_GPIO_Port,LED4_Pin,GPIO_PIN_RESET)	;

  RF3_Send_packet_step2();		
	HAL_GPIO_WritePin(LED3_GPIO_Port,LED3_Pin,GPIO_PIN_RESET)	;
	
  RF1_Send_packet_step2();
	HAL_GPIO_WritePin(LED5_GPIO_Port,LED5_Pin,GPIO_PIN_RESET)	;	
	
	ack_packet.slot_0 = 0;
	ack_packet.slot_1 = 0;
	ack_packet.slot_2 = 0;
	ack_packet.slot_3 = 0;
	ack_packet.slot_4 = 0;
	ack_packet.slot_5 = 0;
}



void make_ack(int slot)
{
	int s = slot - 8;
	
	if(s/8 == 0)
	{
		ack_packet.slot_0 |= 1<<(s%8);
	}
	else if(s/8 == 1)
	{
		ack_packet.slot_1 |= 1<<(s%8);
	}	
	else if(s/8 == 2)
	{
		ack_packet.slot_2 |= 1<<(s%8);
	}	
	else if(s/8 == 3)
	{
		ack_packet.slot_3 |= 1<<(s%8);
	}	
	else if(s/8 == 4)
	{
		ack_packet.slot_4 |= 1<<(s%8);
	}	
	else if(s/8 == 5)
	{
		ack_packet.slot_5 |= 1<<(s%8);
	}		
}

/*
	func： main调用该函数
			3路RF中断接收到数据后存放在buff中 并记录时间和flag 该函数在main中处理buff数据




*/
void rf_rev_data_handle()
{
	uint32_t freq;
	if(rf1_rev_ok.rf_rev_ok)
	{
		rf1_rev_ok.rf_rev_ok = 0;
	if(rf1_test_mode == 2018)
		{
/*			freq = (SX1276LR->RegFrfMsb << 16) + (SX1276LR->RegFrfMid << 8) + (SX1276LR->RegFrfLsb);
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
			*/
			;
	}
	
	if(rf2_rev_ok.rf_rev_ok)
	{
		rf2_rev_ok.rf_rev_ok = 0;
	}
	
	if(rf3_rev_ok.rf_rev_ok)
	{
		rf3_rev_ok.rf_rev_ok = 0;
	}	
}
}














struct_sensor_rftest_packet sensor_rftest_packet;
int rf1_sending = 0;
void send_sensor_rftest_packet(void)
{
	
	unsigned char no_use_slot[7];
	int i;
	int ret = 0;
	
	for(i=0;i<7;i++)
	{
		if(rf1_rev_ap_syn_slot+i-1 < 0)
			no_use_slot[i] = 49;
		else if(rf1_rev_ap_syn_slot+i-1 > 49)
			no_use_slot[i] = 0;
		else
			no_use_slot[i] = rf1_rev_ap_syn_slot+i-1;
		
		if(slot() == no_use_slot[i])
			ret = -1;
	}
	
	if(ret == -1 || sensor_rftest_packet.head.dev_id == slot())
		return;
	
	sensor_rftest_packet.head.dev_id = slot();
	sensor_rftest_packet.head.type = PACKAGE_TYPE_TESTRF;
	sensor_rftest_packet.head.packet_num++;
	sensor_rftest_packet.seq = slot_count/FREAME_SLOT_NUM;
	
	if(rf1_sending == 0){
		RF1_Send_packet(&sensor_rftest_packet);	
		rf1_sending = 1;}
}




















void HAL_SYSTICK_Callback(void)
{
	
	slot_count++;
#if(RF_LORA_CONTINUE_SEND == 1)
	return;
#endif
	if(HAL_GetTick()<1000 || rf1_test_mode >= 2018 || rf2_test_mode >= 2018 ||rf3_test_mode >= 2018)
		return;
	
	
	
	
	switch(slot())
	{
		case 0: send_syn(); break;
		case 2: send_ack(); break;
		default:break;
	}
	
	
	
	
	
		HAL_GPIO_TogglePin(LED1_GPIO_Port,LED1_Pin);
	
}






