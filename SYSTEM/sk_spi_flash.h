#ifndef __SK_SPI_FLASH_H__
#define __SK_SPI_FLASH_H__

/********************** W25Q64 Flash Command Defination ***********************/
#define SPI_FLASH_PerWritePageSize      256

#define W25X_WriteEnable                0x06 
#define W25X_WriteDisable               0x04 
#define W25X_ReadStatusReg_1            0x05
#define W25X_ReadStatusReg_2            0x35 
#define W25X_WriteStatusReg             0x01 
#define W25X_ReadData                   0x03 
#define W25X_FastReadData               0x0B 
#define W25X_FastReadDual               0x3B 
#define W25X_PageProgram                0x02 
#define W25X_64K_BlockErase             0xD8 
#define W25X_32K_BlockErase             0x52 
#define W25X_4K_SectorErase             0x20 
#define W25X_ChipErase                  0xC7 
#define W25X_PowerDown                  0xB9 
#define W25X_ReleasePowerDown           0xAB 
#define W25X_DeviceID                   0xAB 
#define W25X_ManufactDeviceID           0x90 
#define W25X_JedecDeviceID              0x9F 

#define Busy_Flag                       0x01  /* Write In Progress (WIP) flag */
#define Dummy_Byte                      0xFF  /* Dummy Data */

typedef struct {
    uint8_t manufacturer_id;
    uint8_t memory_type_id;
    uint8_t capacity_id;
    uint8_t device_id;
} W25Q64_ID_t;

void SK_SPIFlashInit(void);
void SK_GetFlashID(W25Q64_ID_t *stW25Q64_ID);
void SK_GetFlashDeviceID(W25Q64_ID_t *stW25Q64_ID);
void SK_SPI_FLASH_Erase(uint32_t SectorAddr, uint8_t EraseType);
void SK_SPI_FLASH_ChipErase(void);
void SK_SPI_FLASH_ChipErase(void);
void SK_SPI_FLASH_PageWrite(uint8_t* pBuffer, uint32_t WriteAddr, uint32_t NumByteToWrite);


#endif // __SK_SPI_FLASH_H__
