
#include "flash.h"

//__static_funtion___

//static void deleteBuffer8(uint8_t* data, uint16_t _LENGTH_);
//static void deleteBuffer16(uint16_t* data, uint16_t _LENGTH_);
static void deleteBuffer32(uint32_t* data, uint16_t _LENGTH_);


//______________________________________________________________________________________________________


/* Base address of the Flash sectors */
#define ADDR_FLASH_SECTOR_0    ((uint32_t)0x08000000) /* Base @ of Sector 0, 16 Kbytes */
#define ADDR_FLASH_SECTOR_1    ((uint32_t)0x08004000) /* Base @ of Sector 1, 16 Kbytes */
#define ADDR_FLASH_SECTOR_2    ((uint32_t)0x08008000) /* Base @ of Sector 2, 16 Kbytes */
#define ADDR_FLASH_SECTOR_3    ((uint32_t)0x0800C000) /* Base @ of Sector 3, 16 Kbytes */
#define ADDR_FLASH_SECTOR_4    ((uint32_t)0x08010000) /* Base @ of Sector 4, 64 Kbytes */
#define ADDR_FLASH_SECTOR_5    ((uint32_t)0x08020000) /* Base @ of Sector 5, 128 Kbytes */
#define ADDR_FLASH_SECTOR_6    ((uint32_t)0x08040000) /* Base @ of Sector 6, 128 Kbytes */
#define ADDR_FLASH_SECTOR_7    ((uint32_t)0x08060000) /* Base @ of Sector 7, 128 Kbytes */
#define ADDR_FLASH_SECTOR_8    ((uint32_t)0x08080000) /* Base @ of Sector 8, 128 Kbytes */
#define ADDR_FLASH_SECTOR_9    ((uint32_t)0x080A0000) /* Base @ of Sector 9, 128 Kbytes */
#define ADDR_FLASH_SECTOR_10   ((uint32_t)0x080C0000) /* Base @ of Sector 10, 128 Kbytes */
#define ADDR_FLASH_SECTOR_11   ((uint32_t)0x080E0000) /* Base @ of Sector 11, 128 Kbytes */
/**
 * @brief Gets the sector of a given address
 * @param None
 * @retval The sector of a given address
 */
uint32_t GetSector(uint32_t Address)
{
	uint32_t sector = 0;

	if((Address < ADDR_FLASH_SECTOR_1) && (Address >= ADDR_FLASH_SECTOR_0))
	{
	sector = FLASH_SECTOR_0;
	}
	else if((Address < ADDR_FLASH_SECTOR_2) && (Address >= ADDR_FLASH_SECTOR_1))
	{
	sector = FLASH_SECTOR_1;
	}
	else if((Address < ADDR_FLASH_SECTOR_3) && (Address >= ADDR_FLASH_SECTOR_2))
	{
	sector = FLASH_SECTOR_2;
	}
	else if((Address < ADDR_FLASH_SECTOR_4) && (Address >= ADDR_FLASH_SECTOR_3))
	{
	sector = FLASH_SECTOR_3;
	}
	else if((Address < ADDR_FLASH_SECTOR_5) && (Address >= ADDR_FLASH_SECTOR_4))
	{
	sector = FLASH_SECTOR_4;
	}
	else if((Address < ADDR_FLASH_SECTOR_6) && (Address >= ADDR_FLASH_SECTOR_5))
	{
	sector = FLASH_SECTOR_5;
	}
	else if((Address < ADDR_FLASH_SECTOR_7) && (Address >= ADDR_FLASH_SECTOR_6))
	{
	sector = FLASH_SECTOR_6;
	}
	else if((Address < ADDR_FLASH_SECTOR_8) && (Address >= ADDR_FLASH_SECTOR_7))
	{
	sector = FLASH_SECTOR_7;
	}
	else if((Address < ADDR_FLASH_SECTOR_9) && (Address >= ADDR_FLASH_SECTOR_8))
	{
	sector = FLASH_SECTOR_7;
	}
	else if((Address < ADDR_FLASH_SECTOR_10) && (Address >= ADDR_FLASH_SECTOR_9))
	{
	sector = FLASH_SECTOR_7;
	}
	else if((Address < ADDR_FLASH_SECTOR_11) && (Address >= ADDR_FLASH_SECTOR_10))
	{
	sector = FLASH_SECTOR_7;
	}
	else if((Address < 0x080FFFFF) && (Address >= ADDR_FLASH_SECTOR_11))
	{
	sector = FLASH_SECTOR_7;
	}
	return sector;
}


//______________________________________________________________________________________________________


uint32_t Int_to_Uint_Convert(int32_t _DATA_)
{
	return  (uint32_t)_DATA_;
}

void Int_to_Uint_16bit_Array_Convert(int16_t* _DATA_S, uint16_t* _DATA_D_, uint16_t _LENGTH_)
{
	for(uint16_t i = 0; i <_LENGTH_;i++)
	{
		_DATA_D_[i]  = (uint16_t)_DATA_S[i];
	}
}

void Int_to_Uint_32bit_Array_Convert(int32_t* _DATA_S, uint32_t* _DATA_D_, uint16_t _LENGTH_)
{
	for(uint16_t i = 0; i <_LENGTH_;i++)
	{
		_DATA_D_[i]  = (uint32_t)_DATA_S[i];
	}
}

int32_t Uint_to_Int_Convert(int32_t _DATA_)
{
	return  (int32_t)_DATA_;
}

void Uint_to_Int_16bit_Array_Convert(uint16_t* _DATA_S, int16_t* _DATA_D_, uint16_t _LENGTH_)
{
	for(uint16_t i = 0; i <_LENGTH_;i++)
	{
		_DATA_D_[i]  = (int16_t)_DATA_S[i];
	}
}

void Uint_to_Int_32bit_Array_Convert(uint32_t* _DATA_S, int32_t* _DATA_D_, uint16_t _LENGTH_)
{
	for(uint16_t i = 0; i <_LENGTH_;i++)
	{
		_DATA_D_[i]  = (int32_t)_DATA_S[i];
	}
}




/*
    //_________STATIC FUNTION__________NO USE____

*/

//static void deleteBuffer8(uint8_t* data, uint16_t _LENGTH_)
//{
//	for(uint16_t i = 0; i < _LENGTH_; i++)
//	{
//		data[i] = 0;
//	}
//}
//
//static void deleteBuffer16(uint16_t* data, uint16_t _LENGTH_)
//{
//	for(uint16_t i = 0; i < _LENGTH_; i++)
//	{
//		data[i] = 0;
//	}
//}
static void deleteBuffer32(uint32_t* data, uint16_t _LENGTH_)
{
	for(uint16_t i = 0; i < _LENGTH_; i++)
	{
		data[i] = 0;
	}
}
// erase sector from addr
void Flash_Erase(uint32_t addr, uint32_t numberSectorToErase)
{ 
	HAL_FLASH_Unlock();
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP);
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_OPERR);
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_WRPERR);
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_PGAERR);
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_PGPERR);
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_PGSERR);
	uint32_t SectorError;
	FLASH_EraseInitTypeDef EraseInitStruct;
	EraseInitStruct.Banks = 1;
	EraseInitStruct.Sector = GetSector(addr);
	EraseInitStruct.NbSectors = numberSectorToErase;
	EraseInitStruct.TypeErase = FLASH_TYPEERASE_SECTORS;
	EraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_3;
	if(HAL_FLASHEx_Erase(&EraseInitStruct, &SectorError) != HAL_OK)
	{
	/* X???y ra l???i trong khi x??a Sector s??? c???n th??m m???t s??? code ????? x??? l?? l???i n??y SectorError s??? ch???a sector b??? l???i,v?? sau ???? ????? bi???t m?? l???i tr??n sector n??y b???n c???n g???i h??m 'HAL_FLASH_GetError()'*/
	//	uint32_t errorcode = HAL_FLASH_GetError();
		Error_Handler();
	}
	HAL_FLASH_Lock();
}

//______________________________________________________________________________________
// READ FLASH FUNCTION
/*
 * - Des: this function read a 32 bit value from flash
 * - para: @_ADDRESS_DATA_ : address we will get value
 * - return the value read
 * */
uint32_t Flash_Read_Uint(uint32_t _ADDRESS_DATA_)
{
	return (*(__IO uint32_t*) _ADDRESS_DATA_);
}
/*
 * - Des: this function read a string and return string to an array
 * -para: @_Array_DATA_ : Array save read data
 * 		  @_ADDRESS_DATA_: address start to read
 * 		  @_LENGTH_ : byte to read
 * */
void Flash_Read_Array_32bit(uint32_t* _Array_DATA_, uint32_t _ADDRESS_DATA_, uint32_t _LENGTH_)
{
	deleteBuffer32(_Array_DATA_,_LENGTH_);
	for(uint16_t i = 0; i < _LENGTH_; i++)
	{
		_Array_DATA_[i] = Flash_Read_Uint (_ADDRESS_DATA_ + i * sizeof (uint32_t));
	}
}

uint8_t Flash_Write_Uin32t(uint32_t _DATA_, uint32_t _ADDRESS_DATA_)
{

//	Flash_Erase(_ADDRESS_DATA_, 1);
	HAL_FLASH_Unlock();
	HAL_FLASH_Program (FLASH_TYPEPROGRAM_WORD, _ADDRESS_DATA_, _DATA_);
	HAL_FLASH_Lock();
	uint32_t read_back = Flash_Read_Uint (_ADDRESS_DATA_);
	if (read_back == _DATA_)
	{
		return 1; // send data ok
	}
	else
	{
		return 0; //send wrong data
	}
}
uint8_t Flash_Write_Array_32bit(uint32_t* _Array_DATA_, uint32_t _ADDRESS_DATA_, uint32_t _LENGTH_)
{
	HAL_FLASH_Unlock();
	for(uint32_t i = 0; i < _LENGTH_; i++)
	{
		HAL_FLASH_Program (FLASH_TYPEPROGRAM_WORD, _ADDRESS_DATA_ + i * sizeof (uint32_t), _Array_DATA_[i]);
	}
	HAL_FLASH_Lock();
	uint32_t Array_To_Check [_LENGTH_];// Creat a array to Check Back
	Flash_Read_Array_32bit (Array_To_Check, _ADDRESS_DATA_, _LENGTH_);
	if (memcmp (Array_To_Check, _Array_DATA_, _LENGTH_) == 0)
	{
		return 1;
	}
	else
	{
		return 0;
	}
}

