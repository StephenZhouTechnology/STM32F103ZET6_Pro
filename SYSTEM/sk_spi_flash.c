#include "stm32f10x.h"
#include "stm32f10x_spi.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"

#include "sk_spi_flash.h"

#define SK_SPI_FLASH_CS_HIGH()           GPIO_SetBits(GPIOB, GPIO_Pin_12)
#define SK_SPI_FLASH_CS_LOW()            GPIO_ResetBits(GPIOB, GPIO_Pin_12)

/*******************************************************************************
* Function Name  : SK_SPIPortInit
* Description    : Configure the I/O port for SPI2.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
static void _SK_SPI2PortInit(void)
{
    GPIO_InitTypeDef stGpioInit;

    /*!< Configure pins: SCK */
    stGpioInit.GPIO_Pin = GPIO_Pin_13;
    stGpioInit.GPIO_Speed = GPIO_Speed_50MHz;
    stGpioInit.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOB, &stGpioInit);

    /*!< Configure pins: MISO */
    stGpioInit.GPIO_Pin = GPIO_Pin_14;
    stGpioInit.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOB, &stGpioInit);

    /*!< Configure pins: MOSI */
    stGpioInit.GPIO_Pin = GPIO_Pin_15;
    stGpioInit.GPIO_Speed = GPIO_Speed_50MHz;
    stGpioInit.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOB, &stGpioInit);

    /*!< Configure pins: CS */
    stGpioInit.GPIO_Pin = GPIO_Pin_12;
    stGpioInit.GPIO_Speed = GPIO_Speed_50MHz;
    stGpioInit.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOB, &stGpioInit);
}

/*******************************************************************************
* Function Name  : _SK_SPI2BusInit
* Description    : Configure the SPI2 Bus to adpte the W25Q64 Flash.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
static void _SK_SPI2BusInit(void)
{
    SPI_InitTypeDef  SPI_InitStructure;

    SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
    SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
    SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
    SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
    SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
    SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
    SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4; // PCLK = 36M, SPI2 CLK = PCLK/4 = 9M
    SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
    SPI_InitStructure.SPI_CRCPolynomial = 7;

    SPI_Init(SPI2, &SPI_InitStructure);
    SPI_Cmd(SPI2, ENABLE);
}

/*******************************************************************************
* Function Name  : SK_SPIInit
* Description    : Initializes the peripherals used by the SPI FLASH driver.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void SK_SPIFlashInit(void)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);

    _SK_SPI2PortInit();

    SK_SPI_FLASH_CS_HIGH();

    _SK_SPI2BusInit();
}

/*******************************************************************************
********************* W25Q64 Device Driver Defination **************************
*******************************************************************************/

/*******************************************************************************
* Function Name  : SPI_FLASH_SendByte
* Description    : Sends a byte through the SPI interface and return the byte
*                  received from the SPI bus.
* Input          : byte : byte to send.
* Output         : None
* Return         : The value of the received byte.
*******************************************************************************/
static uint8_t SPI_FLASH_SendByte(uint8_t byte)
{
  /* Loop while DR register in not emplty */
  while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET);

  /* Send byte through the SPI2 peripheral */
  SPI_I2S_SendData(SPI2, byte);

  /* Wait to receive a byte */
  while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == RESET);

  /* Return the byte read from the SPI bus */
  return SPI_I2S_ReceiveData(SPI2);
}

/*******************************************************************************
* Function Name  : SK_GetFlashID
* Description    : Reads FLASH identification.
* Input          : W25Q64_ID_t pointer 
* Output         : None
* Return         : FLASH identification
*******************************************************************************/
void SK_GetFlashID(W25Q64_ID_t *stW25Q64_ID)
{
    SK_SPI_FLASH_CS_LOW();

    SPI_FLASH_SendByte(W25X_JedecDeviceID);

    stW25Q64_ID->manufacturer_id = SPI_FLASH_SendByte(Dummy_Byte);
    stW25Q64_ID->memory_type_id = SPI_FLASH_SendByte(Dummy_Byte);
    stW25Q64_ID->capacity_id = SPI_FLASH_SendByte(Dummy_Byte);

    SK_SPI_FLASH_CS_HIGH();
}

/*******************************************************************************
* Function Name  : SK_GetFlashDeviceID
* Description    : Reads FLASH Device identification.
* Input          : W25Q64_ID_t pointer
* Output         : None
* Return         : FLASH identification
*******************************************************************************/
void SK_GetFlashDeviceID(W25Q64_ID_t *stW25Q64_ID)
{
    SK_SPI_FLASH_CS_LOW();

    SPI_FLASH_SendByte(W25X_DeviceID);
    SPI_FLASH_SendByte(Dummy_Byte);
    SPI_FLASH_SendByte(Dummy_Byte);
    SPI_FLASH_SendByte(Dummy_Byte);

    stW25Q64_ID->device_id = SPI_FLASH_SendByte(Dummy_Byte);

    SK_SPI_FLASH_CS_HIGH();
}

/*******************************************************************************
* Function Name  : SK_SPI_FLASH_WriteEnable
* Description    : Enables the write access to the FLASH.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void SK_SPI_FLASH_WriteEnable(void)
{
  /* Select the FLASH: Chip Select low */
  SK_SPI_FLASH_CS_LOW();

  /* Send "Write Enable" instruction */
  SPI_FLASH_SendByte(W25X_WriteEnable);

  /* Deselect the FLASH: Chip Select high */
  SK_SPI_FLASH_CS_HIGH();
}

/*******************************************************************************
* Function Name  : SK_SPI_FLASH_WriteDisable
* Description    : Enables the write access to the FLASH.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void SK_SPI_FLASH_WriteDisable(void)
{
  /* Select the FLASH: Chip Select low */
  SK_SPI_FLASH_CS_LOW();

  /* Send "Write Enable" instruction */
  SPI_FLASH_SendByte(W25X_WriteDisable);

  /* Deselect the FLASH: Chip Select high */
  SK_SPI_FLASH_CS_HIGH();
}

/*******************************************************************************
* Function Name  : SK_SPI_FLASH_WaitBusy
* Description    : Polls the status of the Write In Progress (WIP) flag in the
*                  FLASH's status register and loop until write opertaion
*                  has completed.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void SK_SPI_FLASH_WaitBusy(void)
{
  u8 FLASH_Status = 0;

  /* Select the FLASH: Chip Select low */
  SK_SPI_FLASH_CS_LOW();

  /* Send "Read Status Register" instruction */
  SPI_FLASH_SendByte(W25X_ReadStatusReg_1);

  /* Loop as long as the memory is busy with a write cycle */
  do
  {
    /* Send a dummy byte to generate the clock needed by the FLASH
    and put the value of the status register in FLASH_Status variable */
    FLASH_Status = SPI_FLASH_SendByte(Dummy_Byte);
  }
  while ((FLASH_Status & Busy_Flag) == SET); /* Write in progress */

  /* Deselect the FLASH: Chip Select high */
  SK_SPI_FLASH_CS_HIGH();
}


/*******************************************************************************
* Function Name  : SK_SPI_FLASH_Erase
* Description    : Erases the specified FLASH
* Input          : SectorAddr: address of the sector to erase.
* Input          : EraseType : The type of erase.
*                : W25X_4K_SectorErase (4KB Erase)
*                : W25X_32K_BlockErase (32KB Erase)
*                : W25X_64K_BlockErase (64KB Erase)
* Output         : None
* Return         : None
*******************************************************************************/
void SK_SPI_FLASH_Erase(uint32_t SectorAddr, uint8_t EraseType)
{
  /* Send write enable instruction */
  SK_SPI_FLASH_WriteEnable();
  SK_SPI_FLASH_WaitBusy();
  /* Sector Erase */
  /* Select the FLASH: Chip Select low */
  SK_SPI_FLASH_CS_LOW();
  /* Send Sector Erase instruction */
  SPI_FLASH_SendByte(EraseType);
  /* Send SectorAddr high nibble address byte */
  SPI_FLASH_SendByte((SectorAddr & 0xFF0000) >> 16);
  /* Send SectorAddr medium nibble address byte */
  SPI_FLASH_SendByte((SectorAddr & 0xFF00) >> 8);
  /* Send SectorAddr low nibble address byte */
  SPI_FLASH_SendByte(SectorAddr & 0xFF);
  /* Deselect the FLASH: Chip Select high */
  SK_SPI_FLASH_CS_HIGH();
  /* Wait the end of Flash writing */
  SK_SPI_FLASH_WaitBusy();
}

/*******************************************************************************
* Function Name  : SPI_FLASH_ChipErase
* Description    : Erases the whole FLASH.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void SK_SPI_FLASH_ChipErase(void)
{
  /* Send write enable instruction */
  SK_SPI_FLASH_WriteEnable();
  SK_SPI_FLASH_WaitBusy();

  /* Bulk Erase */
  /* Select the FLASH: Chip Select low */
  SK_SPI_FLASH_CS_LOW();
  /* Send Bulk Erase instruction  */
  SPI_FLASH_SendByte(W25X_ChipErase);
  /* Deselect the FLASH: Chip Select high */
  SK_SPI_FLASH_CS_HIGH();

  /* Wait the end of Flash writing */
  SK_SPI_FLASH_WaitBusy();
}

/*******************************************************************************
* Function Name  : SK_SPI_FLASH_BufferRead
* Description    : Reads a block of data from the FLASH.
* Input          : - pBuffer : pointer to the buffer that receives the data read
*                    from the FLASH.
*                  - ReadAddr : FLASH's internal address to read from.
*                  - NumByteToRead : number of bytes to read from the FLASH.
* Output         : None
* Return         : None
*******************************************************************************/
void SK_SPI_FLASH_BufferRead(uint8_t* pBuffer, uint32_t ReadAddr, uint32_t NumByteToRead)
{
  /* Select the FLASH: Chip Select low */
  SK_SPI_FLASH_CS_LOW();

  /* Send "Read from Memory " instruction */
  SPI_FLASH_SendByte(W25X_ReadData);

  /* Send ReadAddr high nibble address byte to read from */
  SPI_FLASH_SendByte((ReadAddr & 0xFF0000) >> 16);
  /* Send ReadAddr medium nibble address byte to read from */
  SPI_FLASH_SendByte((ReadAddr& 0xFF00) >> 8);
  /* Send ReadAddr low nibble address byte to read from */
  SPI_FLASH_SendByte(ReadAddr & 0xFF);

  while (NumByteToRead--)
  {
    *pBuffer = SPI_FLASH_SendByte(Dummy_Byte);
    pBuffer++;
  }
  /* Deselect the FLASH: Chip Select high */
  SK_SPI_FLASH_CS_HIGH();
}

/*******************************************************************************
* Function Name  : SK_SPI_FLASH_PageWrite
* Description    : Writes more than one byte to the FLASH with a single WRITE
*                  cycle(Page WRITE sequence). The number of byte can't exceed
*                  the FLASH page size.
* Input          : - pBuffer : pointer to the buffer  containing the data to be
*                    written to the FLASH.
*                  - WriteAddr : FLASH's internal address to write to.
*                  - NumByteToWrite : number of bytes to write to the FLASH,
*                    must be equal or less than "SPI_FLASH_PageSize" value.
* Output         : None
* Return         : None
*******************************************************************************/
void SK_SPI_FLASH_PageWrite(uint8_t *pBuffer, uint32_t WriteAddr, uint32_t NumByteToWrite)
{
  /* Enable the write access to the FLASH */
  SK_SPI_FLASH_WriteEnable();

  /* Select the FLASH: Chip Select low */
  SK_SPI_FLASH_CS_LOW();
  /* Send "Write to Memory " instruction */
  SPI_FLASH_SendByte(W25X_PageProgram);
  /* Send WriteAddr high nibble address byte to write to */
  SPI_FLASH_SendByte((WriteAddr & 0xFF0000) >> 16);
  /* Send WriteAddr medium nibble address byte to write to */
  SPI_FLASH_SendByte((WriteAddr & 0xFF00) >> 8);
  /* Send WriteAddr low nibble address byte to write to */
  SPI_FLASH_SendByte(WriteAddr & 0xFF);

  if(NumByteToWrite > SPI_FLASH_PerWritePageSize)
  {
     NumByteToWrite = SPI_FLASH_PerWritePageSize;
  }

  /* while there is data to be written on the FLASH */
  while (NumByteToWrite--)
  {
    /* Send the current byte */
    SPI_FLASH_SendByte(*pBuffer);
    /* Point on the next byte to be written */
    pBuffer++;
  }

  /* Deselect the FLASH: Chip Select high */
  SK_SPI_FLASH_CS_HIGH();
  /* Wait the end of Flash writing */
  SK_SPI_FLASH_WaitBusy();
}

