#ifndef RF_H_
#define RF_H_

#include "stm32f4xx_hal.h"



#define RF_LORA_CONTINUE_SEND         0        //以lora模式长发


#pragma pack(1)
#pragma anon_unions
typedef struct _test1000p
{
	uint16_t head;
	uint16_t id;
	uint8_t cmd;
	union{
		struct
		{
			uint8_t sf;
			uint8_t bw;
			uint16_t freq;
			uint16_t time;
		};
		struct
		{
			uint16_t packet_count;
			uint16_t packet_seq;
			int8_t rssi;
			uint8_t aaa;
		};
	};
}struct_test1000p;
#pragma pack()

typedef struct _syn_send_ok
{
	unsigned int syn_send_ok;
	unsigned int number;
	unsigned int now_slot;
	unsigned int now_us;
}struct_syn_send_ok;

typedef struct _syn_rev_ok
{
	unsigned int rf_rev_ok;
	unsigned int number;
	unsigned int now_slot;
	unsigned int now_us;
}struct_syn_rev_ok;


extern int rf1_rev_ap_syn_slot;
extern int rf2_rev_ap_syn_slot;
extern int rf3_rev_ap_syn_slot;

extern struct_syn_rev_ok rf1_rev_ok;
extern struct_syn_rev_ok rf2_rev_ok;
extern struct_syn_rev_ok rf3_rev_ok;


extern unsigned char rf1_rx_buff[64];
extern unsigned char rf2_rx_buff[64];
extern unsigned char rf3_rx_buff[64];




extern const int rf_channel_table[41];


void RF_init(void);


void RF1_set_ch(uint32_t ch);
void RF2_set_ch(uint32_t ch);
void RF3_set_ch(uint32_t ch);



void RF1_Send_packet_step1(void *pdata);
void RF2_Send_packet_step1(void *pdata);
void RF3_Send_packet_step1(void *pdata);

void RF1_Send_packet_step2(void);
void RF2_Send_packet_step2(void);
void RF3_Send_packet_step2(void);


void RF1_irq_hand_io0(void);
void RF2_irq_hand_io0(void);
void RF3_irq_hand_io0(void);


void RF1_Send_packet(void *pdata);
void RF2_Send_packet(void *pdata);
void RF3_Send_packet(void *pdata);

void debug_rf_send_syn_ok();

void test_1000packet_send(void);

unsigned char cal_crc_table(unsigned char *ptr, unsigned char len) ;




void help(char *param);













#endif













