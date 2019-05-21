
#include "main.h"
#include "stm32f4xx_hal.h"
#include "ap_param.h"
#include "flash.h"
#include "string.h"





extern CRC_HandleTypeDef hcrc;



uint8_t flash_buff[256] = {0};
uint8_t ap_param_write_flash_flag = 0;

struct_ap_param ap_param;


void init_ap_param(void)
{
	
	ap_param.ap_id = *(uint16_t *)0x0800001c;
	if(ap_param.ap_id == 0)
		ap_param.ap_id = HAL_CRC_Calculate(&hcrc,(uint32_t *)0x1fff7a10,3);
	ap_param.ap_version = AP_VERSION;
	ap_param.band_id = ap_param.ap_id;
	ap_param.ap_channel = 0;
	ap_param.ap_syn_param[0] = 0x10;
	ap_param.gprs_server_ip = (74<<24)|(83<<16)|(239<<8)|219;
	ap_param.gprs_server_port = 40005;
}









void read_ap_param_flash(void)
{
	struct_flash_head_crc *p_head1;
	struct_flash_head_crc *p_head2;
	
	uint32_t crc;
	FLASH_EraseInitTypeDef EraseInit;
	uint32_t SectorError	;
	int i = 0;
	int8_t times = 0;
	
	p_head1 = (struct_flash_head_crc *)FLASH_AP_PARAM_BEGIN_1;
	p_head2 = (struct_flash_head_crc *)FLASH_AP_PARAM_BEGIN_2;

read:

	times++;
	if(times>3)
		return;
	
	if(p_head1->head == 0xaaaa5555)
	{
		memset(flash_buff,0,sizeof(struct_ap_param)+8);
		memcpy(flash_buff,(uint8_t*)(FLASH_AP_PARAM_BEGIN_1+8),sizeof(struct_ap_param));
		crc = HAL_CRC_Calculate(&hcrc,(uint32_t *)flash_buff,sizeof(struct_ap_param)/4+1);
		if(crc == p_head1->crc)
		{
			memcpy(&ap_param,flash_buff,sizeof(struct_ap_param));
			ap_param.ap_version = AP_VERSION;
			ap_param.ap_id = *(uint16_t *)0x0800001c;
			return;
		}
		
	}
	if(p_head2->head == 0xaaaa5555)
	{
		memset(flash_buff,0,sizeof(struct_ap_param)+8);
		memcpy(flash_buff,(uint8_t*)(FLASH_AP_PARAM_BEGIN_2+8),sizeof(struct_ap_param));
		crc = HAL_CRC_Calculate(&hcrc,(uint32_t *)flash_buff,sizeof(struct_ap_param)/4+1);
		if(crc == p_head2->crc)
		{
			memcpy(&ap_param,flash_buff,sizeof(struct_ap_param));
			ap_param.ap_version = AP_VERSION;
			ap_param.ap_id = *(uint16_t *)0x0800001c;
			return;
		}
	}

	
write:
	memset(flash_buff,0,sizeof(struct_ap_param)+8);
	memcpy(flash_buff,&ap_param,sizeof(struct_ap_param));
	crc = HAL_CRC_Calculate(&hcrc,(uint32_t *)flash_buff,sizeof(struct_ap_param)/4+1);
	
	EraseInit.TypeErase = FLASH_TYPEERASE_SECTORS;
	EraseInit.Banks = FLASH_BANK_2;
	EraseInit.NbSectors = 1;
	EraseInit.Sector = FLASH_SECTOR_12;
	EraseInit.VoltageRange = FLASH_VOLTAGE_RANGE_3;
	HAL_FLASHEx_Erase(&EraseInit,&SectorError);

	EraseInit.Sector = FLASH_SECTOR_13;
	HAL_FLASHEx_Erase(&EraseInit,&SectorError);	
	
	HAL_FLASH_Unlock();
	
	for(i=0;i<sizeof(struct_ap_param);i++)
	{
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE,FLASH_AP_PARAM_BEGIN_1+8+i,flash_buff[i]);
	}
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,FLASH_AP_PARAM_BEGIN_1,0xaaaa5555);	
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,FLASH_AP_PARAM_BEGIN_1+4,crc);	
	
	for(i=0;i<sizeof(struct_ap_param);i++)
	{
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE,FLASH_AP_PARAM_BEGIN_2+8+i,flash_buff[i]);
	}	
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,FLASH_AP_PARAM_BEGIN_2,0xaaaa5555);	
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,FLASH_AP_PARAM_BEGIN_2+4,crc);		
	
	HAL_FLASH_Lock();
	goto read;
	
}




void write_ap_param_flash(void)
{
	struct_flash_head_crc *p_head1;
	struct_flash_head_crc *p_head2;
	
	uint32_t crc;
	FLASH_EraseInitTypeDef EraseInit;
	uint32_t SectorError	;
	int i = 0;
	
	int8_t times = 0;
	
	
	p_head1 = (struct_flash_head_crc *)FLASH_AP_PARAM_BEGIN_1;
	p_head2 = (struct_flash_head_crc *)FLASH_AP_PARAM_BEGIN_2;


write:
	times++;
	if(times>3)
		return;
	memset(flash_buff,0,sizeof(struct_ap_param)+8);
	memcpy(flash_buff,&ap_param,sizeof(struct_ap_param));
	crc = HAL_CRC_Calculate(&hcrc,(uint32_t *)flash_buff,sizeof(struct_ap_param)/4+1);
	
	HAL_FLASH_Unlock();
	
	EraseInit.TypeErase = FLASH_TYPEERASE_SECTORS;
	EraseInit.Banks = FLASH_BANK_2;
	EraseInit.NbSectors = 1;
	EraseInit.Sector = FLASH_SECTOR_12;
	EraseInit.VoltageRange = FLASH_VOLTAGE_RANGE_3;
	HAL_FLASHEx_Erase(&EraseInit,&SectorError);

	EraseInit.Sector = FLASH_SECTOR_13;
	HAL_FLASHEx_Erase(&EraseInit,&SectorError);	
	
	
	
	for(i=0;i<sizeof(struct_ap_param);i++)
	{
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE,FLASH_AP_PARAM_BEGIN_1+8+i,flash_buff[i]);
	}
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,FLASH_AP_PARAM_BEGIN_1,0xaaaa5555);	
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,FLASH_AP_PARAM_BEGIN_1+4,crc);	
	
	for(i=0;i<sizeof(struct_ap_param);i++)
	{
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE,FLASH_AP_PARAM_BEGIN_2+8+i,flash_buff[i]);
	}	
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,FLASH_AP_PARAM_BEGIN_2,0xaaaa5555);	
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,FLASH_AP_PARAM_BEGIN_2+4,crc);	

	HAL_FLASH_Lock();
	
	if(p_head1->head == 0xaaaa5555)
	{
		memset(flash_buff,0,sizeof(struct_ap_param)+8);
		memcpy(flash_buff,(uint8_t*)(FLASH_AP_PARAM_BEGIN_1+8),sizeof(struct_ap_param));
		crc = HAL_CRC_Calculate(&hcrc,(uint32_t *)flash_buff,sizeof(struct_ap_param)/4+1);
		if(crc == p_head1->crc)
		{
//			memcpy(&ap_param,flash_buff,sizeof(struct_ap_param));
			ap_param_write_flash_flag = 0;
			return;
		}
		
	}
	if(p_head2->head == 0xaaaa5555)
	{
		memset(flash_buff,0,sizeof(struct_ap_param)+8);
		memcpy(flash_buff,(uint8_t*)(FLASH_AP_PARAM_BEGIN_2+8),sizeof(struct_ap_param));
		crc = HAL_CRC_Calculate(&hcrc,(uint32_t *)flash_buff,sizeof(struct_ap_param)/4+1);
		if(crc == p_head2->crc)
		{
//			memcpy(&ap_param,flash_buff,sizeof(struct_ap_param)); ddd
			ap_param_write_flash_flag = 0;
			return;
		}
	}
	
	goto write;
	
	
}


int32_t write_bin_flash(uint32_t address,uint8_t *pdata,uint32_t size)
{
	int i = 0;
	uint8_t *p_readdata = (uint8_t *)address;
	
	if(address < 0x08004000)
		return -1;
	
	HAL_FLASH_Unlock();
	for(i=0;i<size;i++)
	{
		
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE,address+i,pdata[i]);
	}	
	
	HAL_FLASH_Lock();
	
	for(i=0;i<size;i++)
	{
		if(p_readdata[i] != pdata[i])
			return -1;
	}
	
	return 0;
}
