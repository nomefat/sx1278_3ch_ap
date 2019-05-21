#include "main.h"
#include "stm32f4xx_hal.h"
#include "debug_uart.h"
#include "string.h"
#include "rf_hal.h"
#include "math.h"
#include "rf.h"
#include "flash.h"
#include "ap_s.h"

extern UART_HandleTypeDef huart6;





#define Q_LEN 256           //队列长度
char debug_uart_dma_buff[Q_LEN];       //队列数组

char debug_uart_buff[Q_LEN+1];

char debug_send_buff[Q_LEN];

struct_debug_double_buff debug_send_buff1;
struct_debug_double_buff debug_send_buff2;


struct _cmd_param_int{
	int param_num;
	int param[10];
}cmd_param_int;


struct _cmd_list{
	char *cmd;
	void (*func)(char *param);
};

#define CMD_CALLBACK_LIST_BEGIN struct _cmd_list cmd_list[] = {NULL,NULL,
#define CMD_CALLBACK_LIST_END NULL,NULL};
#define CMD_CALLBACK(cmd_string,callback)	cmd_string,callback,


uint16_t rf_test_1000p_time_delay[7][10] = 
{
400,300,250,200,150,100,100,80,60,50,
600,500,350,350,200,150,120,100,80,60,
1200,900,600,500,350,250,200,120,100,80,
2300,1750,1200,900,600,500,350,200,150,100,
3800,2900,2000,1500,1000,800,550,300,200,120,
7500,5700,3800,2900,2000,1500,1000,550,300,200,
14900,11200,7500,5700,3800,2900,2000,1000,550,300,

};



int copy_string_to_double_buff(char *pstr)
{
	
	int len = strlen(pstr);
	
	if(debug_send_buff1.len != 0xffffffff)
	{
		if(len<(DEBUG_DOUBLE_BUFF_LEN-debug_send_buff1.len))
		{			
			memcpy(debug_send_buff1.data+debug_send_buff1.len,pstr,len);
			debug_send_buff1.len += len;
			
			return 0;
		}	
	}
	
	if(debug_send_buff2.len != 0xffffffff)
	{
		if(len<(DEBUG_DOUBLE_BUFF_LEN-debug_send_buff2.len))
		{			
			memcpy(debug_send_buff2.data+debug_send_buff2.len,pstr,len);
			debug_send_buff2.len += len;
			return 0;
		}	
	}
	return -1;
}

/*****************

有main函数调用 双缓冲有数据就启动DMA发送


******************/
void debug_send_double_buff_poll(void)
{
	int len;
	
	if(huart6.gState == HAL_UART_STATE_READY)
	{
		if(debug_send_buff1.len == 0xffffffff)
		{
			debug_send_buff1.len = 0;
		}
		else if(debug_send_buff2.len == 0xffffffff)
			debug_send_buff2.len = 0;
	}
	else
		return;
	
	
	//缓冲中有数据需要发送  2个缓冲并没有正在发送
	//if(debug_send_buff1.len != 0 && debug_send_buff1.len != 0xffffffff && debug_send_buff2.len != 0xffffffff)
	if(debug_send_buff1.len != 0 && debug_send_buff1.len != 0xffffffff && huart6.gState == HAL_UART_STATE_READY)
	{		
		
		len = debug_send_buff1.len;
		debug_send_buff1.len = 0xffffffff;  //表明这个缓冲正在发送，不能操作
		HAL_UART_Transmit_DMA(&huart6,(uint8_t *)debug_send_buff1.data,len);		
	}
	//else if(debug_send_buff2.len != 0 && debug_send_buff2.len != 0xffffffff && debug_send_buff1.len != 0xffffffff)
	else if(debug_send_buff2.len != 0 && debug_send_buff2.len != 0xffffffff && huart6.gState == HAL_UART_STATE_READY)
	{		
		len = debug_send_buff2.len;
		debug_send_buff2.len = 0xffffffff;
		HAL_UART_Transmit_DMA(&huart6,(uint8_t *)debug_send_buff2.data,len);		
	}
}




void debug_uart_send_string(char *pstr)
{
	int num = 0;
	char *ptr = pstr;
	while(*ptr++)
	{
		num++;
		if(num>5000)return;
	}
	
	HAL_UART_Transmit_DMA(&huart6,(uint8_t *)pstr,num);
	
}





/**********************************************************************************************
***func: 用于单片第一次开启DMA接收
***     
***date: 2017/6/9
*** nome
***********************************************************************************************/
void start_from_debug_dma_receive()
{
	SET_BIT((&huart6)->Instance->CR1, USART_CR1_IDLEIE);  //打开串口空闲中断
	HAL_UART_Receive_DMA(&huart6,debug_uart_dma_buff,Q_LEN);	 //打开DMA接收
}




/**********************************************************************************************
***func:串口空闲中断回调函数
***     空闲中断用来判断一包数据的结束
***date: 2017/6/9
*** nome
***********************************************************************************************/
void uart_from_debug_idle_callback()
{
	HAL_DMA_Abort((&huart6)->hdmarx);
	huart6.RxState = HAL_UART_STATE_READY;
	huart6.hdmarx->State = HAL_DMA_STATE_READY;
	debug_uart_buff[0] = Q_LEN-DMA2_Stream1->NDTR;
	memcpy(debug_uart_buff+1,(char*)debug_uart_dma_buff,Q_LEN-DMA2_Stream1->NDTR);
	
	HAL_UART_Receive_DMA(&huart6,debug_uart_dma_buff,Q_LEN);	 //打开DMA接收

}


/*
 * 功能：从队列中取出一个完整的字符串命令。
 * 失败：返回-1
 * 成功：返回0
 *cmd 存放命令的指针，param 存放参数的指针。
*/
int get_q_string(char *cmd,char *param)
{
	int i = 1;
	int timeout = 0;

	
	for(;;){
		if(debug_uart_buff[i] == ' ')
		{
			cmd[i-1] = 0; 
			i++;
			break;
		}
		else if(debug_uart_buff[i] == '\r' || debug_uart_buff[i] == '\n')
		{
			debug_uart_buff[0] = 0;
			cmd[i-1] = 0; 
			memset(param,0,32);
			return 0;
		}
		if(i>debug_uart_buff[0])
			return -1;
		cmd[i-1] = debug_uart_buff[i];
		i++;
				
	}


	for(;;){

		if(debug_uart_buff[i] == '\r' || debug_uart_buff[i] == '\n')
		{
			debug_uart_buff[0] = 0;
			*param = 0; 
			return 0;
		}
		if(i>debug_uart_buff[0])
			return -1;	
		
		*param++ = debug_uart_buff[i];
		i++;	
	}

	return 0;
}

int str_to_int(char *str)
{
	int i = 0,j = 0;
	int ret = 0;
	for(;;){
	if(str[i++]==0||i>20)
		break;
	}
	j = i = i-2;
	for(;i>=0;i--)
	{
		ret += (str[i]-'0')*(pow(10,(j-i)));
	}
	return ret;
}

//
struct _cmd_param_int* get_str_param(char *param)
{
	char *ptr_now = param;
	char *ptr_pre = param;
	int i = 0;
	
	cmd_param_int.param_num = 0;
	for(;;){                     //分割参数，按照空格

		if(*ptr_now==' ')
		{
			*ptr_now = 0;
			cmd_param_int.param[i++] = str_to_int(ptr_pre);
			ptr_pre = ptr_now+1;
			cmd_param_int.param_num++;
		}
		ptr_now++;
		if(*ptr_now==0)
		{
			if(ptr_now != ptr_pre)
			{
			cmd_param_int.param[i] = str_to_int(ptr_pre);
			cmd_param_int.param_num++;
			}
			return &cmd_param_int;
		}		
			
		if(ptr_now-param>100)		
			return &cmd_param_int;
	}
	
}

void print_now_rfmode()
{

}



void oncmd_rf_set_ch(char *param)
{
	struct _cmd_param_int* ps_pram = get_str_param(param);
//	uint8_t *ptr = (uint8_t *)&ap_param.ap_channel;
	
	if(ps_pram->param[0]>3 || ps_pram->param[0]<1)
	{
		copy_string_to_double_buff("选择RF错误(1-3)\r\n");
		return;
	}	
/*	if(ps_pram->param[1]>30 || ps_pram->param[1]<0)
	{
		copy_string_to_double_buff("RF通道不合法(0-2)\r\n");
		return;
	}	
*/	

	if(ps_pram->param[0] == 1)
	{
		if(ps_pram->param[1]<41)
		{
			RF1_set_ch(rf_channel_table[ps_pram->param[1]]);
			((unsigned char *)&ap_param.ap_channel)[3] = ps_pram->param[1];
			ap_param_write_flash_flag = 1;
		}
		else
			RF1_set_ch(ps_pram->param[1]);
	}
	else if(ps_pram->param[0] == 2)
	{
		if(ps_pram->param[1]<41)
		{
			RF2_set_ch(rf_channel_table[ps_pram->param[1]]);
			((unsigned char *)&ap_param.ap_channel)[2] = ps_pram->param[1];
			ap_param_write_flash_flag = 1;
		}
		else
			RF2_set_ch(ps_pram->param[1]);
	}
	else if(ps_pram->param[0] == 3)
	{
		if(ps_pram->param[1]<41)
		{
			RF3_set_ch(rf_channel_table[ps_pram->param[1]]);
			((unsigned char *)&ap_param.ap_channel)[1] = ps_pram->param[1];
			ap_param_write_flash_flag = 1;
		}
		else
			RF3_set_ch(ps_pram->param[1]);
	}

	sprintf(debug_send_buff,"setrfch ok %d %d\r\n",ps_pram->param[0],ps_pram->param[1]);
	copy_string_to_double_buff(debug_send_buff);	
}


extern int rf1_test_mode;
extern int rf2_test_mode;
extern int rf3_test_mode;

void test_1000_packet_mode(char *param)
{
	struct _cmd_param_int* ps_pram = get_str_param(param);
	
	if(ps_pram->param_num == 0)
	{
		sprintf(debug_send_buff,"testmode %d %d %d\r\n",rf1_test_mode%2017,rf2_test_mode%2017,rf3_test_mode%2017);
		copy_string_to_double_buff(debug_send_buff);	
		return;
	}
	
	if(ps_pram->param[0] == 1)
	{
		if(ps_pram->param[1] == 1)
		{
			rf1_test_mode = 2018;
			rf2_test_mode = 0;
			rf3_test_mode = 0;
			UserSX1276_1
			SX1276LoRaSetOpMode( RFLR_OPMODE_RECEIVER );	
			copy_string_to_double_buff("RF1 已关闭同步包发送，进入只接受模式\r\n");
		}
		else
		{
			rf1_test_mode = 0;
			copy_string_to_double_buff("已打开同步包发送，进入正常工作模式\r\n");			
		}

		
	}
	else if(ps_pram->param[0] == 2)
	{
		if(ps_pram->param[1] == 1)
		{
			rf1_test_mode = 0;
			rf2_test_mode = 2018;
			rf3_test_mode = 0;
			UserSX1276_2
			SX1276LoRaSetOpMode( RFLR_OPMODE_RECEIVER );	
			copy_string_to_double_buff("RF2 已关闭同步包发送，进入只接受模式\r\n");
		}
		else
		{
			rf2_test_mode = 0;
			copy_string_to_double_buff("已打开同步包发送，进入正常工作模式\r\n");			
		}
	}
	else if(ps_pram->param[0] == 3)
	{
		if(ps_pram->param[1] == 1)
		{
			rf1_test_mode = 0;
			rf2_test_mode = 0;
			rf3_test_mode = 2018;
			UserSX1276_3
			SX1276LoRaSetOpMode( RFLR_OPMODE_RECEIVER );	
			copy_string_to_double_buff("RF3 已关闭同步包发送，进入只接受模式\r\n");
		}
		else
		{
			rf3_test_mode = 0;
			copy_string_to_double_buff("已打开同步包发送，进入正常工作模式\r\n");			
		}
	}	
}


extern int rf1_test_send_1000packet;
extern int rf2_test_send_1000packet;
extern int rf3_test_send_1000packet;


void default_1000p_test(void)
{
		rf1_test_mode = 2018;
		rf2_test_mode = 0;
		rf3_test_mode = 0;
		rf1_test_send_1000packet = 1000;
}





static void test_1000packet_send_(char *param)
{
	struct _cmd_param_int* ps_pram = get_str_param(param);
	

	if(ps_pram->param[0] == 1){
		rf1_test_mode = 2018;
		rf2_test_mode = 0;
		rf3_test_mode = 0;
		rf1_test_send_1000packet = 1000;}
	else if(ps_pram->param[0] == 2){
		rf1_test_mode = 0;
		rf2_test_mode = 2018;
		rf3_test_mode = 0;
		rf2_test_send_1000packet = 1000;}
	else if(ps_pram->param[0] == 3){
		rf1_test_mode = 0;
		rf2_test_mode = 0;
		rf3_test_mode = 2018;
		rf3_test_send_1000packet = 1000;	}
	else
		return;
	copy_string_to_double_buff("开始发送1000包\r\n");
}



extern int enable_send_sensor_rftest;

static void send_sensor_rftest_enable(char *param)
{
	struct _cmd_param_int* ps_pram = get_str_param(param);
	

	if(ps_pram->param[0] == 1){
		enable_send_sensor_rftest = 1;
				rf1_test_mode = 0;
		rf2_test_mode = 0;
		rf3_test_mode = 2018;
		copy_string_to_double_buff("开始模拟sensor发送testrf数据包\r\n");
	}
	else
{
		enable_send_sensor_rftest = 0;
			rf1_test_mode = 0;
		rf2_test_mode = 0;
		rf3_test_mode = 0;
		copy_string_to_double_buff("停止模拟sensor发送testrf数据包\r\n");
	}

}


extern void SX1276LoRaSetRFPower( int8_t power );

void setRFPower(char *param)
{
	struct _cmd_param_int* ps_pram = get_str_param(param);
	
	if(ps_pram->param[0] == 1)
	{
		UserSX1276_1
		SX1276LoRaSetRFPower(ps_pram->param[1]);
	}
	else if(ps_pram->param[0] == 2)
	{
		UserSX1276_2
		SX1276LoRaSetRFPower(ps_pram->param[1]);
	}
	else if(ps_pram->param[0] == 3)
	{
		UserSX1276_3
		SX1276LoRaSetRFPower(ps_pram->param[1]);
	}

	sprintf(debug_send_buff,"setrfpower ok %d %d\r\n",ps_pram->param[0],ps_pram->param[1]);
	copy_string_to_double_buff(debug_send_buff);
	
}

uint32_t test_1000p_timedelay ;
volatile int32_t test_1000p_send_flag;
volatile struct_test1000p test1000p;
uint32_t test_packet_count;
extern tLoRaSettings LoRaSettings;
extern void SX1276Init(void);
extern	 int32_t test_delay_time ;
extern	 uint32_t time_slot ;
void set_test1000p_sf_bw_freq(char *param)
{
	struct _cmd_param_int* ps_pram = get_str_param(param);
	int32_t delay;
	
//	if(ps_pram->param[0] == 256 ||ps_pram->param[0] == 257 ||ps_pram->param[0] == 13124 || ps_pram->param[0] == 21862)
		if(1 == 1)
	{
			UserSX1276_1
			LoRaSettings.RFFrequency = 495000000;
			LoRaSettings.SignalBw = 6;
			LoRaSettings.SpreadingFactor = 9;		
			SX1276Power(0);
			delay = 1000000;
			while(delay--);			
			SX1276Power(1);
			SX1276Init();
			RF1_set_ch(495000000);
			delay = 1000000;
			while(delay--);					
		LoRaSettings.RFFrequency = ps_pram->param[3] * 1000000;
		LoRaSettings.SignalBw = ps_pram->param[2];
		LoRaSettings.SpreadingFactor = ps_pram->param[1];
		test_packet_count = ps_pram->param[4];
		rf1_test_mode = 2019;
		test_1000p_send_flag = 1;
		test1000p.head = 0x55aa;
		test1000p.cmd = 1;
		test1000p.freq = ps_pram->param[3];
		test1000p.id = ps_pram->param[0];
		test1000p.sf = ps_pram->param[1];
		test1000p.bw = ps_pram->param[2];
		test1000p.time = rf_test_1000p_time_delay[test1000p.sf-6][test1000p.bw];
		test_1000p_timedelay = test1000p.time;
		RF1_Send_packet(&test1000p);
	  test_delay_time = 20;
	  time_slot = 0;		
	}
	else if(ps_pram->param[0] == 2)
	{
		UserSX1276_2
		SX1276LoRaSetRFPower(ps_pram->param[1]);
	}
	else if(ps_pram->param[0] == 3)
	{
		UserSX1276_3
		SX1276LoRaSetRFPower(ps_pram->param[1]);
	}

	sprintf(debug_send_buff,"\r\nrf%d set test param %d %d %d ",ps_pram->param[0],ps_pram->param[1],ps_pram->param[2],ps_pram->param[3]);
	copy_string_to_double_buff(debug_send_buff);	
}


void power_on_auto_send_test()
{
		int32_t delay;
			UserSX1276_1
			LoRaSettings.RFFrequency = 495000000;
			LoRaSettings.SignalBw = 6;
			LoRaSettings.SpreadingFactor = 9;		
			SX1276Power(0);
			delay = 1000000;
			while(delay--);			
			SX1276Power(1);
			SX1276Init();
			RF1_set_ch(495000000);
			delay = 1000000;
			while(delay--);					
		LoRaSettings.RFFrequency = 505 * 1000000;
		LoRaSettings.SignalBw = 9;
		LoRaSettings.SpreadingFactor = 6;
		test_packet_count = 10000000;
		rf1_test_mode = 2019;
		test_1000p_send_flag = 1;
		test1000p.head = 0x55aa;
		test1000p.cmd = 1;
		test1000p.freq = 505;
		test1000p.id = 2;
		test1000p.sf = 6;
		test1000p.bw = 9;
		test1000p.time = rf_test_1000p_time_delay[test1000p.sf-6][test1000p.bw];
		test_1000p_timedelay = test1000p.time;
		RF1_Send_packet(&test1000p);
	  test_delay_time = 20;
	  time_slot = 0;	
}




void get_ap_param(char *param)
{
		sprintf(debug_send_buff,"034ap:version=%d.%d\r\napid=%x\r\nch=%d %d %d\r\nserverip=%d.%d.%d.%d port=%d\r\nlive=%d\r\n",
	AP_VERSION>>8,AP_VERSION&0xff,ap_param.ap_id,(ap_param.ap_channel>>24)&0XFF,(ap_param.ap_channel>>16)&0XFF,(ap_param.ap_channel>>8)&0XFF
	,ap_param.gprs_server_ip&0xff,(ap_param.gprs_server_ip>>8)&0xff,(ap_param.gprs_server_ip>>16)&0xff,ap_param.gprs_server_ip>>24,ap_param.gprs_server_port,slot_count/200);
		debug_uart_send_string(debug_send_buff);	
}



void help(char *param)
{
	copy_string_to_double_buff("*******************************************************\r\n\
034---AP3RF_V0.1_1000p_test \r\n\
ap....................打印AP参数\r\n\
setch [1-3] [freq or 0-40]........设置RF通道  [RF1-3] [FREQ]\r\n\
setpower [1-3] [power]........设置RF通道  [RF1-3] [power]\r\n\
testmode [1-3][0-1].............进入1000包接收模式或者关闭\r\n\
send1000 [1-3].............启动发送1000包[RF1-3]\r\n\
send [1-0].............1 启动 0 关闭发送模拟sensor发送rf测试包\r\n\
*******************************************************\r\n");

}







//在此处添加你的命令字符串和回调函数
CMD_CALLBACK_LIST_BEGIN


CMD_CALLBACK("?",help)	
CMD_CALLBACK("ap",get_ap_param)	
CMD_CALLBACK("setch",oncmd_rf_set_ch)	
CMD_CALLBACK("testmode",test_1000_packet_mode)	
CMD_CALLBACK("send1000",test_1000packet_send_)
CMD_CALLBACK("send",send_sensor_rftest_enable)
CMD_CALLBACK("setpower",setRFPower)
CMD_CALLBACK("t",set_test1000p_sf_bw_freq)


CMD_CALLBACK_LIST_END









char cmd[100];
char param[32];
int get_cmd(void)
{
	int i = 0;
	if(get_q_string(cmd,param) == -1)
		return 0;
	
	for(;;){
		if(strcmp(cmd,cmd_list[i].cmd)==0)
			return i;	
		if(cmd_list[++i].cmd==NULL)
			return 0;
	}
}



void debug_cmd_handle(void)
{
	int func_index = get_cmd();
	if(func_index<=0)
		return;
	cmd_list[func_index].func(param);
}







