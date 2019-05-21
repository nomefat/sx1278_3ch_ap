#ifndef _DEBUG_UART_
#define _DEBUG_UART_



#define DEBUG_DOUBLE_BUFF_LEN 1024

void debug_uart_send_string(char *pstr);
void print_version();
void default_1000p_test(void);

extern char debug_send_buff[];

void start_from_debug_dma_receive();
void debug_cmd_handle(void);
void debug_send_double_buff_poll(void);


typedef struct _debug_double_buff
{
	unsigned int len;
	char data[DEBUG_DOUBLE_BUFF_LEN];
}struct_debug_double_buff;



#endif


