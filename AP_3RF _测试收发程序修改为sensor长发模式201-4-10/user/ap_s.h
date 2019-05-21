#ifndef AP_S_H_
#define AP_S_H_





#define PACKAGE_TYPE_SYNC             1
#define PACKAGE_TYPE_STATUS           2
#define PACKAGE_TYPE_EVENT            3
#define PACKAGE_TYPE_ACK              4
#define PACKAGE_TYPE_UPGRADE          5
#define PACKAGE_TYPE_UPSTATUS         6
#define PACKAGE_TYPE_TESTRF           7




#pragma anon_unions
#pragma pack(1)


typedef struct packet_head
{
	struct{
		unsigned char type:3;
		unsigned char packet_num:5;			
	};
	unsigned short dev_id;
}struct_packet_head;


typedef struct _syn_packet
{
	struct_packet_head head;
	unsigned char sec;
	unsigned short sensor_id;
	unsigned char cmd_type;
	struct{
		unsigned char cmd_1;
		unsigned char cmd_2;
		unsigned char cmd_3;
	};
	unsigned char crc;
}struct_syn_packet;


typedef struct _ack_packet
{
	struct_packet_head head;

	unsigned char slot_0;
	unsigned char slot_1;
	unsigned char slot_2;
	unsigned char slot_3;
	unsigned char slot_4;
	unsigned char slot_5;
	unsigned char nouse;
	unsigned char crc;
}struct_ack_packet;


typedef struct _sensor_event
{
	struct{
		unsigned short on_off:1;
		unsigned short second:5;
		unsigned short ms:10;
	};
	unsigned char speed;
}struct_sensor_event;


typedef struct _sensor_event_packet
{
	struct_packet_head head;	
	struct_sensor_event event1;
	struct_sensor_event event2;
	unsigned char resend_times;
	unsigned char crc;
}struct_sensor_event_packet;


typedef struct _sensor_status_packet
{
	struct_packet_head head;	
	signed char rssi;
	unsigned short band_id;
	unsigned char rf_channel;
	unsigned char firm_version;
	unsigned char mode;
	unsigned char battary_volt;
	unsigned char crc;
}struct_sensor_status_packet;


//sensor rf 测试包   每帧发送  用来测试网络性能
typedef struct _sensor_rftest_packet
{
	struct_packet_head head;	
	unsigned int seq;  //包序号
	struct{
		unsigned char on_off:1;    //当前是否压车
		unsigned char have_ack:1;  //本帧是否收到ack
		unsigned char net_30s:1;   //是否进入30s稳定入网
	};
	unsigned char rev_ack_rssi;  //收到ack的信号强度 没有填写0
	unsigned char nouse;
	unsigned char crc;
}struct_sensor_rftest_packet;





void send_sensor_rftest_packet(void);


extern volatile unsigned int slot_count;



#pragma pack()




#endif


