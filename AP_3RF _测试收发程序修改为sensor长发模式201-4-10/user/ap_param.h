#ifndef _AP_PARAM_H_
#define _AP_PARAM_H_


#include "stm32f4xx_hal.h"


#define AP_VERSION  0X0001






#pragma pack(1)



typedef struct _ap_param{

	uint16_t ap_id;                                                           //     N1只读
	uint16_t ap_version;                                                      //     N1只读
	uint16_t rp_version;                //eeprom 中rp固件版本号                       N1只读
	uint16_t sensor_version;						   //eeprom 中sensor固件版本号                N1只读	
	uint16_t band_id;	
	uint32_t ap_channel;
	uint32_t gprs_server_ip;
	uint16_t gprs_server_port;
	uint8_t ap_syn_param[6];


}struct_ap_param;



typedef struct{
	
 uint8_t uiCmd;  //命令2-检测器重校准 3-检测器模式设置  11 -参数全配置 
 uint16_t uiPoll;   //目的ID
 
 uint16_t uiBindId; //绑定ID
 struct{

  uint16_t uimySlot:8,     //8位-本机时间槽
     uiSlotStateE:8; //8位-开关时间槽扩展
  
 }paraB; 
 uint16_t uiSlotStateL;//开关时间槽低16
 uint16_t uiSlotStateM;//开关时间槽中16
 uint16_t uiSlotStateH;//开关时间槽高16
 struct{  
  uint8_t uiGrade:3, //跟的同步包级别0-3
    uiChannel:5;//设置的通道0-31
 }paraA;
 
}rp_param;




typedef struct{

 uint8_t ucSensorMode;
 rp_param ParaFram;
 
}struct_sensor_rp_param;



typedef struct _dev_list{

	uint16_t bind_id;
	uint16_t ap_channel;
	



}struct_dev_list;



#pragma pack()


extern struct_ap_param ap_param;


void init_ap_param(void);



#endif


