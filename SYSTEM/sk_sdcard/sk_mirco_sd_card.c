#include "sk_mirco_sd_card.h"
#include "stm32f10x.h"
#include "stm32f10x_sdio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_dma.h"

#include "core_cm3.h"

/** 
  * @brief  Card Identification Data: CID Register   
  */
typedef struct
{
  __IO uint8_t  ManufacturerID;       /*!< ManufacturerID */
  __IO uint16_t OEM_AppliID;          /*!< OEM/Application ID */
  __IO uint32_t ProdName1;            /*!< Product Name part1 */
  __IO uint8_t  ProdName2;            /*!< Product Name part2*/
  __IO uint8_t  ProdRev;              /*!< Product Revision */
  __IO uint32_t ProdSN;               /*!< Product Serial Number */
  __IO uint8_t  Reserved1;            /*!< Reserved1 */
  __IO uint16_t ManufactDate;         /*!< Manufacturing Date */
  __IO uint8_t  CID_CRC;              /*!< CID CRC */
  __IO uint8_t  Reserved2;            /*!< always 1 */
} SD_CID;

/** 
  * @brief  Card Specific Data: CSD Register   
  */ 
typedef struct
{
  __IO uint8_t  CSDStruct;            /*!< CSD structure */
  __IO uint8_t  SysSpecVersion;       /*!< System specification version */
  __IO uint8_t  Reserved1;            /*!< Reserved */
  __IO uint8_t  TAAC;                 /*!< Data read access-time 1 */
  __IO uint8_t  NSAC;                 /*!< Data read access-time 2 in CLK cycles */
  __IO uint8_t  MaxBusClkFrec;        /*!< Max. bus clock frequency */
  __IO uint16_t CardComdClasses;      /*!< Card command classes */
  __IO uint8_t  RdBlockLen;           /*!< Max. read data block length */
  __IO uint8_t  PartBlockRead;        /*!< Partial blocks for read allowed */
  __IO uint8_t  WrBlockMisalign;      /*!< Write block misalignment */
  __IO uint8_t  RdBlockMisalign;      /*!< Read block misalignment */
  __IO uint8_t  DSRImpl;              /*!< DSR implemented */
  __IO uint8_t  Reserved2;            /*!< Reserved */
  __IO uint32_t DeviceSize;           /*!< Device Size */
  __IO uint8_t  MaxRdCurrentVDDMin;   /*!< Max. read current @ VDD min */
  __IO uint8_t  MaxRdCurrentVDDMax;   /*!< Max. read current @ VDD max */
  __IO uint8_t  MaxWrCurrentVDDMin;   /*!< Max. write current @ VDD min */
  __IO uint8_t  MaxWrCurrentVDDMax;   /*!< Max. write current @ VDD max */
  __IO uint8_t  DeviceSizeMul;        /*!< Device size multiplier */
  __IO uint8_t  EraseGrSize;          /*!< Erase group size */
  __IO uint8_t  EraseGrMul;           /*!< Erase group size multiplier */
  __IO uint8_t  WrProtectGrSize;      /*!< Write protect group size */
  __IO uint8_t  WrProtectGrEnable;    /*!< Write protect group enable */
  __IO uint8_t  ManDeflECC;           /*!< Manufacturer default ECC */
  __IO uint8_t  WrSpeedFact;          /*!< Write speed factor */
  __IO uint8_t  MaxWrBlockLen;        /*!< Max. write data block length */
  __IO uint8_t  WriteBlockPaPartial;  /*!< Partial blocks for write allowed */
  __IO uint8_t  Reserved3;            /*!< Reserded */
  __IO uint8_t  ContentProtectAppli;  /*!< Content protection application */
  __IO uint8_t  FileFormatGrouop;     /*!< File format group */
  __IO uint8_t  CopyFlag;             /*!< Copy flag (OTP) */
  __IO uint8_t  PermWrProtect;        /*!< Permanent write protection */
  __IO uint8_t  TempWrProtect;        /*!< Temporary write protection */
  __IO uint8_t  FileFormat;           /*!< File Format */
  __IO uint8_t  ECC;                  /*!< ECC code */
  __IO uint8_t  CSD_CRC;              /*!< CSD CRC */
  __IO uint8_t  Reserved4;            /*!< always 1*/
} SD_CSD;

typedef struct {
    uint32_t CardType;
    uint16_t RCA;
    SD_CID   SD_cid;
    SD_CSD   SD_csd;
    uint32_t SD_scr_raw_data[2];
    uint32_t CardCapacity;
    uint32_t CardBlockSize;
} SK_SDCARD_st;

/** 
  * @brief  SD Card States 
  */   
typedef enum
{
  SD_CARD_READY                  = ((uint32_t)0x00000001),
  SD_CARD_IDENTIFICATION         = ((uint32_t)0x00000002),
  SD_CARD_STANDBY                = ((uint32_t)0x00000003),
  SD_CARD_TRANSFER               = ((uint32_t)0x00000004),
  SD_CARD_SENDING                = ((uint32_t)0x00000005),
  SD_CARD_RECEIVING              = ((uint32_t)0x00000006),
  SD_CARD_PROGRAMMING            = ((uint32_t)0x00000007),
  SD_CARD_DISCONNECTED           = ((uint32_t)0x00000008),
  SD_CARD_ERROR                  = ((uint32_t)0x000000FF)
} SDCardState;

/** 
  * @brief  SDIO Transfer state  
  */   
typedef enum
{
  SD_TRANSFER_OK  = 0,
  SD_TRANSFER_BUSY = 1,
  SD_TRANSFER_ERROR
} SDTransferState;

typedef enum {FAILED = 0, PASSED = !FAILED} TestStatus;

/*!< SDIOCLK = HCLK, SDIO_CK = HCLK/(2 + SDIO_TRANSFER_CLK_DIV) */
#define SDIO_INIT_CLK_DIV                (0xB2)          // 400KHz
#define SDIO_TRANSFER_CLK_DIV            ((uint8_t)0x01) // 24MHz

/** 
  * @brief SDIO Commands  Index 
  */
#define SD_CMD_GO_IDLE_STATE                       ((uint8_t)0)
#define SD_CMD_SEND_OP_COND                        ((uint8_t)1)
#define SD_CMD_ALL_SEND_CID                        ((uint8_t)2)
#define SD_CMD_SET_REL_ADDR                        ((uint8_t)3) /*!< SDIO_SEND_REL_ADDR for SD Card */
#define SD_CMD_SET_DSR                             ((uint8_t)4)
#define SD_CMD_SDIO_SEN_OP_COND                    ((uint8_t)5)
#define SD_CMD_HS_SWITCH                           ((uint8_t)6)
#define SD_CMD_SEL_DESEL_CARD                      ((uint8_t)7)
#define SD_CMD_HS_SEND_EXT_CSD                     ((uint8_t)8)
#define SD_CMD_SEND_CSD                            ((uint8_t)9)
#define SD_CMD_SEND_CID                            ((uint8_t)10)
#define SD_CMD_READ_DAT_UNTIL_STOP                 ((uint8_t)11) /*!< SD Card doesn't support it */
#define SD_CMD_STOP_TRANSMISSION                   ((uint8_t)12)
#define SD_CMD_SEND_STATUS                         ((uint8_t)13)
#define SD_CMD_HS_BUSTEST_READ                     ((uint8_t)14)
#define SD_CMD_GO_INACTIVE_STATE                   ((uint8_t)15)
#define SD_CMD_SET_BLOCKLEN                        ((uint8_t)16)
#define SD_CMD_READ_SINGLE_BLOCK                   ((uint8_t)17)
#define SD_CMD_READ_MULT_BLOCK                     ((uint8_t)18)
#define SD_CMD_HS_BUSTEST_WRITE                    ((uint8_t)19)
#define SD_CMD_WRITE_DAT_UNTIL_STOP                ((uint8_t)20) /*!< SD Card doesn't support it */
#define SD_CMD_SET_BLOCK_COUNT                     ((uint8_t)23) /*!< SD Card doesn't support it */
#define SD_CMD_WRITE_SINGLE_BLOCK                  ((uint8_t)24)
#define SD_CMD_WRITE_MULT_BLOCK                    ((uint8_t)25)
#define SD_CMD_PROG_CID                            ((uint8_t)26) /*!< reserved for manufacturers */
#define SD_CMD_PROG_CSD                            ((uint8_t)27)
#define SD_CMD_SET_WRITE_PROT                      ((uint8_t)28)
#define SD_CMD_CLR_WRITE_PROT                      ((uint8_t)29)
#define SD_CMD_SEND_WRITE_PROT                     ((uint8_t)30)
#define SD_CMD_SD_ERASE_GRP_START                  ((uint8_t)32) /*!< To set the address of the first write
                                                                  block to be erased. (For SD card only) */
#define SD_CMD_SD_ERASE_GRP_END                    ((uint8_t)33) /*!< To set the address of the last write block of the
                                                                  continuous range to be erased. (For SD card only) */
#define SD_CMD_ERASE_GRP_START                     ((uint8_t)35) /*!< To set the address of the first write block to be erased.
                                                                  (For MMC card only spec 3.31) */

#define SD_CMD_ERASE_GRP_END                       ((uint8_t)36) /*!< To set the address of the last write block of the
                                                                  continuous range to be erased. (For MMC card only spec 3.31) */

#define SD_CMD_ERASE                               ((uint8_t)38)
#define SD_CMD_FAST_IO                             ((uint8_t)39) /*!< SD Card doesn't support it */
#define SD_CMD_GO_IRQ_STATE                        ((uint8_t)40) /*!< SD Card doesn't support it */
#define SD_CMD_LOCK_UNLOCK                         ((uint8_t)42)
#define SD_CMD_APP_CMD                             ((uint8_t)55)
#define SD_CMD_GEN_CMD                             ((uint8_t)56)
#define SD_CMD_NO_CMD                              ((uint8_t)64)

/** 
  * @brief Following commands are SD Card Specific commands.
  *        SDIO_APP_CMD ：CMD55 should be sent before sending these commands. 
  */
#define SD_CMD_APP_SD_SET_BUSWIDTH                 ((uint8_t)6)  /*!< For SD Card only */
#define SD_CMD_SD_APP_STAUS                        ((uint8_t)13) /*!< For SD Card only */
#define SD_CMD_SD_APP_SEND_NUM_WRITE_BLOCKS        ((uint8_t)22) /*!< For SD Card only */
#define SD_CMD_SD_APP_OP_COND                      ((uint8_t)41) /*!< For SD Card only */
#define SD_CMD_SD_APP_SET_CLR_CARD_DETECT          ((uint8_t)42) /*!< For SD Card only */
#define SD_CMD_SD_APP_SEND_SCR                     ((uint8_t)51) /*!< For SD Card only */
#define SD_CMD_SDIO_RW_DIRECT                      ((uint8_t)52) /*!< For SD I/O Card only */
#define SD_CMD_SDIO_RW_EXTENDED                    ((uint8_t)53) /*!< For SD I/O Card only */

/** 
  * @brief Following commands are SD Card Specific security commands.
  *        SDIO_APP_CMD should be sent before sending these commands. 
  */
#define SD_CMD_SD_APP_GET_MKB                      ((uint8_t)43) /*!< For SD Card only */
#define SD_CMD_SD_APP_GET_MID                      ((uint8_t)44) /*!< For SD Card only */
#define SD_CMD_SD_APP_SET_CER_RN1                  ((uint8_t)45) /*!< For SD Card only */
#define SD_CMD_SD_APP_GET_CER_RN2                  ((uint8_t)46) /*!< For SD Card only */
#define SD_CMD_SD_APP_SET_CER_RES2                 ((uint8_t)47) /*!< For SD Card only */
#define SD_CMD_SD_APP_GET_CER_RES1                 ((uint8_t)48) /*!< For SD Card only */
#define SD_CMD_SD_APP_SECURE_READ_MULTIPLE_BLOCK   ((uint8_t)18) /*!< For SD Card only */
#define SD_CMD_SD_APP_SECURE_WRITE_MULTIPLE_BLOCK  ((uint8_t)25) /*!< For SD Card only */
#define SD_CMD_SD_APP_SECURE_ERASE                 ((uint8_t)38) /*!< For SD Card only */
#define SD_CMD_SD_APP_CHANGE_SECURE_AREA           ((uint8_t)49) /*!< For SD Card only */
#define SD_CMD_SD_APP_SECURE_WRITE_MKB             ((uint8_t)48) /*!< For SD Card only */

/** 
  * @brief  Mask for errors Card Status R1 (OCR Register) 
  */
#define SD_OCR_ADDR_OUT_OF_RANGE        ((uint32_t)0x80000000)
#define SD_OCR_ADDR_MISALIGNED          ((uint32_t)0x40000000)
#define SD_OCR_BLOCK_LEN_ERR            ((uint32_t)0x20000000)
#define SD_OCR_ERASE_SEQ_ERR            ((uint32_t)0x10000000)
#define SD_OCR_BAD_ERASE_PARAM          ((uint32_t)0x08000000)
#define SD_OCR_WRITE_PROT_VIOLATION     ((uint32_t)0x04000000)
#define SD_OCR_LOCK_UNLOCK_FAILED       ((uint32_t)0x01000000)
#define SD_OCR_COM_CRC_FAILED           ((uint32_t)0x00800000)
#define SD_OCR_ILLEGAL_CMD              ((uint32_t)0x00400000)
#define SD_OCR_CARD_ECC_FAILED          ((uint32_t)0x00200000)
#define SD_OCR_CC_ERROR                 ((uint32_t)0x00100000)
#define SD_OCR_GENERAL_UNKNOWN_ERROR    ((uint32_t)0x00080000)
#define SD_OCR_STREAM_READ_UNDERRUN     ((uint32_t)0x00040000)
#define SD_OCR_STREAM_WRITE_OVERRUN     ((uint32_t)0x00020000)
#define SD_OCR_CID_CSD_OVERWRIETE       ((uint32_t)0x00010000)
#define SD_OCR_WP_ERASE_SKIP            ((uint32_t)0x00008000)
#define SD_OCR_CARD_ECC_DISABLED        ((uint32_t)0x00004000)
#define SD_OCR_ERASE_RESET              ((uint32_t)0x00002000)
#define SD_OCR_AKE_SEQ_ERROR            ((uint32_t)0x00000008)
#define SD_OCR_ERRORBITS                ((uint32_t)0xFDFFE008)


#define SD_CHECK_PATTERN                ((uint32_t)0x000001AA)
#define SDIO_SEND_IF_COND               ((uint32_t)0x00000008)
#define SD_OCR_ERRORBITS                ((uint32_t)0xFDFFE008)
#define SD_MAX_VOLT_TRIAL               ((uint32_t)0x0000FFFF)
#define SD_VOLTAGE_WINDOW_SD            ((uint32_t)0x80100000)
#define SD_HIGH_CAPACITY                ((uint32_t)0x40000000)
#define SD_STD_CAPACITY                 ((uint32_t)0x00000000)

#define SD_R6_GENERAL_UNKNOWN_ERROR     ((uint32_t)0x00002000)
#define SD_R6_ILLEGAL_CMD               ((uint32_t)0x00004000)
#define SD_R6_COM_CRC_FAILED            ((uint32_t)0x00008000)

#define SD_DATATIMEOUT                  ((uint32_t)0xFFFFFFFF)
#define SD_0TO7BITS                     ((uint32_t)0x000000FF)
#define SD_8TO15BITS                    ((uint32_t)0x0000FF00)
#define SD_16TO23BITS                   ((uint32_t)0x00FF0000)
#define SD_24TO31BITS                   ((uint32_t)0xFF000000)
#define SD_MAX_DATA_LENGTH              ((uint32_t)0x01FFFFFF)

#define SD_WIDE_BUS_SUPPORT             ((uint32_t)0x00040000)
#define SD_1_BIT_WIDE                   ((uint32_t)0x00000000)
#define SD_4_BIT_WIDE                   ((uint32_t)0x00000002)

#define SD_BLOCK_SIZE_512               (512)
#define NUMBER_OF_BLOCKS                10  /* For Multi Blocks operation (Read/Write) */
#define MULTI_BUFFER_SIZE               (SD_BLOCK_SIZE_512 * NUMBER_OF_BLOCKS)

#define SDIO_FIFO_ADDRESS                ((uint32_t)0x40018080)

SK_SDCARD_st g_stSDCARD = {0};
__IO uint32_t StopCondition = 0;
__IO SD_Error TransferError = SD_OK;
__IO uint32_t TransferEnd = 0;

static void SK_SD_NVIC_Config(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;

    /* Configure the NVIC Preemption Priority Bits */
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);

    NVIC_InitStructure.NVIC_IRQChannel = SDIO_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

static void SK_SD_GPIOConfig(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure;
    /*!< GPIOC and GPIOD Periph clock enable */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD , ENABLE);
    
    /*!< Configure PC.08, PC.09, PC.10, PC.11, PC.12 pin: D0, D1, D2, D3, CLK pin */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_12;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
    
    /*!< Configure PD.02 CMD line */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
    GPIO_Init(GPIOD, &GPIO_InitStructure); 
}

static void SK_SDIO_Config(void)
{
    /*!< Enable the SDIO AHB Clock */
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_SDIO, ENABLE);
    SDIO_DeInit();
}

static void SK_SDIO_DMA2Config(void)
{
    /*!< Enable the DMA2 Clock */
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA2, ENABLE);
}

void SD_DMA_RxConfig(uint32_t *BufferDST, uint32_t BufferSize)
{
  DMA_InitTypeDef DMA_InitStructure;

  DMA_ClearFlag(DMA2_FLAG_TC4 | DMA2_FLAG_TE4 | DMA2_FLAG_HT4 | DMA2_FLAG_GL4);

  /*!< DMA2 Channel-4 disable */
  DMA_Cmd(DMA2_Channel4, DISABLE);

  /*!< DMA2 Channel4 Config */
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)SDIO_FIFO_ADDRESS;
  DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)BufferDST;
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
  DMA_InitStructure.DMA_BufferSize = BufferSize / 4;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Word;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
  DMA_Init(DMA2_Channel4, &DMA_InitStructure);

  /*!< DMA2 Channel4 enable */
  DMA_Cmd(DMA2_Channel4, ENABLE); 
}

void SD_DMA_TxConfig(uint32_t *BufferSRC, uint32_t BufferSize)
{

  DMA_InitTypeDef DMA_InitStructure;

  DMA_ClearFlag(DMA2_FLAG_TC4 | DMA2_FLAG_TE4 | DMA2_FLAG_HT4 | DMA2_FLAG_GL4);

  /*!< DMA2 Channel4 disable */
  DMA_Cmd(DMA2_Channel4, DISABLE);

  /*!< DMA2 Channel4 Config */
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)SDIO_FIFO_ADDRESS;
  DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)BufferSRC;
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
  DMA_InitStructure.DMA_BufferSize = BufferSize / 4;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Word;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
  DMA_Init(DMA2_Channel4, &DMA_InitStructure);

  /*!< DMA2 Channel4 enable */
  DMA_Cmd(DMA2_Channel4, ENABLE);  
}

/************************** SD CMD Defination Start *********************************/
void SK_SdioSendCmd_0(void)
{
    SDIO_CmdInitTypeDef stSdioCmdInit;

    stSdioCmdInit.SDIO_Argument = 0x0;
    stSdioCmdInit.SDIO_CmdIndex = SD_CMD_GO_IDLE_STATE; //cmd0
    stSdioCmdInit.SDIO_Response = SDIO_Response_No;
    stSdioCmdInit.SDIO_Wait = SDIO_Wait_No;
    stSdioCmdInit.SDIO_CPSM = SDIO_CPSM_Enable;
    SDIO_SendCommand(&stSdioCmdInit);
}

void SK_SdioSendCmd_8(void)
{
    SDIO_CmdInitTypeDef stSdioCmdInit;

    stSdioCmdInit.SDIO_Argument = SD_CHECK_PATTERN;
    stSdioCmdInit.SDIO_CmdIndex = SDIO_SEND_IF_COND;  // cmd8
    stSdioCmdInit.SDIO_Response = SDIO_Response_Short;// Rsp = r7
    stSdioCmdInit.SDIO_Wait = SDIO_Wait_No;
    stSdioCmdInit.SDIO_CPSM = SDIO_CPSM_Enable;
    SDIO_SendCommand(&stSdioCmdInit);
}

void SK_SdioSendCmd_55(uint32_t rca)
{
    SDIO_CmdInitTypeDef stSdioCmdInit;

    stSdioCmdInit.SDIO_Argument = (uint32_t) rca << 16;
    stSdioCmdInit.SDIO_CmdIndex = SD_CMD_APP_CMD;
    stSdioCmdInit.SDIO_Response = SDIO_Response_Short; //r1
    stSdioCmdInit.SDIO_Wait = SDIO_Wait_No;
    stSdioCmdInit.SDIO_CPSM = SDIO_CPSM_Enable;
    SDIO_SendCommand(&stSdioCmdInit);
}

void SK_SdioSendAppCmd_41(void)
{
    SDIO_CmdInitTypeDef stSdioCmdInit;

    stSdioCmdInit.SDIO_Argument = SD_VOLTAGE_WINDOW_SD | SD_HIGH_CAPACITY;
    stSdioCmdInit.SDIO_CmdIndex = SD_CMD_SD_APP_OP_COND;
    stSdioCmdInit.SDIO_Response = SDIO_Response_Short;  //r3
    stSdioCmdInit.SDIO_Wait = SDIO_Wait_No;
    stSdioCmdInit.SDIO_CPSM = SDIO_CPSM_Enable;
    SDIO_SendCommand(&stSdioCmdInit);
}

void SK_SdioSendAppCmd_51(void)
{
    SDIO_CmdInitTypeDef stSdioCmdInit;

    /*!< Send ACMD51 SD_APP_SEND_SCR with argument as 0 */
    stSdioCmdInit.SDIO_Argument = 0x0;
    stSdioCmdInit.SDIO_CmdIndex = SD_CMD_SD_APP_SEND_SCR;
    stSdioCmdInit.SDIO_Response = SDIO_Response_Short;  //r1
    stSdioCmdInit.SDIO_Wait = SDIO_Wait_No;
    stSdioCmdInit.SDIO_CPSM = SDIO_CPSM_Enable;
    SDIO_SendCommand(&stSdioCmdInit);
}

void SK_SdioSendAppCmd_6(uint32_t bit_wide)
{
    SDIO_CmdInitTypeDef stSdioCmdInit;

    stSdioCmdInit.SDIO_Argument = bit_wide;
    stSdioCmdInit.SDIO_CmdIndex = SD_CMD_APP_SD_SET_BUSWIDTH;
    stSdioCmdInit.SDIO_Response = SDIO_Response_Short;
    stSdioCmdInit.SDIO_Wait = SDIO_Wait_No;
    stSdioCmdInit.SDIO_CPSM = SDIO_CPSM_Enable;
    SDIO_SendCommand(&stSdioCmdInit);
}

void SK_SdioSendAppCmd_23(uint32_t num_of_blk)
{
    SDIO_CmdInitTypeDef stSdioCmdInit;

    stSdioCmdInit.SDIO_Argument = num_of_blk;
    stSdioCmdInit.SDIO_CmdIndex = SD_CMD_SET_BLOCK_COUNT;  //cmd23
    stSdioCmdInit.SDIO_Response = SDIO_Response_Short;
    stSdioCmdInit.SDIO_Wait = SDIO_Wait_No;
    stSdioCmdInit.SDIO_CPSM = SDIO_CPSM_Enable;
    SDIO_SendCommand(&stSdioCmdInit);
}

void SK_SdioSendCmd_2(void)
{
    SDIO_CmdInitTypeDef stSdioCmdInit;

    stSdioCmdInit.SDIO_Argument = 0x0;
    stSdioCmdInit.SDIO_CmdIndex = SD_CMD_ALL_SEND_CID;
    stSdioCmdInit.SDIO_Response = SDIO_Response_Long;
    stSdioCmdInit.SDIO_Wait = SDIO_Wait_No;
    stSdioCmdInit.SDIO_CPSM = SDIO_CPSM_Enable;
    SDIO_SendCommand(&stSdioCmdInit);
}

void SK_SdioSendCmd_3(void)
{
    SDIO_CmdInitTypeDef stSdioCmdInit;

    stSdioCmdInit.SDIO_Argument = 0x00;
    stSdioCmdInit.SDIO_CmdIndex = SD_CMD_SET_REL_ADDR;  //cmd3
    stSdioCmdInit.SDIO_Response = SDIO_Response_Short; //r6
    stSdioCmdInit.SDIO_Wait = SDIO_Wait_No;
    stSdioCmdInit.SDIO_CPSM = SDIO_CPSM_Enable;
    SDIO_SendCommand(&stSdioCmdInit);
}

void SK_SdioSendCmd_9(void)
{
    SDIO_CmdInitTypeDef stSdioCmdInit;

    stSdioCmdInit.SDIO_Argument = (uint32_t)(g_stSDCARD.RCA << 16);
    stSdioCmdInit.SDIO_CmdIndex = SD_CMD_SEND_CSD;
    stSdioCmdInit.SDIO_Response = SDIO_Response_Long;
    stSdioCmdInit.SDIO_Wait = SDIO_Wait_No;
    stSdioCmdInit.SDIO_CPSM = SDIO_CPSM_Enable;
    SDIO_SendCommand(&stSdioCmdInit);
}

// Erase Start Command
void SK_SdioSendCmd_32(uint32_t start_addr)
{
    SDIO_CmdInitTypeDef stSdioCmdInit;

    /*!< Send CMD32 SD_ERASE_GRP_START with argument as addr  */
    stSdioCmdInit.SDIO_Argument = start_addr;
    stSdioCmdInit.SDIO_CmdIndex = SD_CMD_SD_ERASE_GRP_START;
    stSdioCmdInit.SDIO_Response = SDIO_Response_Short;  //R1
    stSdioCmdInit.SDIO_Wait = SDIO_Wait_No;
    stSdioCmdInit.SDIO_CPSM = SDIO_CPSM_Enable;
    SDIO_SendCommand(&stSdioCmdInit);
}

// Erase End Command
void SK_SdioSendCmd_33(uint32_t end_addr)
{
    SDIO_CmdInitTypeDef stSdioCmdInit;

    /*!< Send CMD33 SD_ERASE_GRP_END with argument as addr  */
    stSdioCmdInit.SDIO_Argument = end_addr;
    stSdioCmdInit.SDIO_CmdIndex = SD_CMD_SD_ERASE_GRP_END;
    stSdioCmdInit.SDIO_Response = SDIO_Response_Short;  //R1
    stSdioCmdInit.SDIO_Wait = SDIO_Wait_No;
    stSdioCmdInit.SDIO_CPSM = SDIO_CPSM_Enable;
    SDIO_SendCommand(&stSdioCmdInit);
}

// Erase Command
void SK_SdioSendCmd_38(void)
{
    SDIO_CmdInitTypeDef stSdioCmdInit;

    stSdioCmdInit.SDIO_Argument = 0;
    stSdioCmdInit.SDIO_CmdIndex = SD_CMD_ERASE;
    stSdioCmdInit.SDIO_Response = SDIO_Response_Short;  //R1
    stSdioCmdInit.SDIO_Wait = SDIO_Wait_No;
    stSdioCmdInit.SDIO_CPSM = SDIO_CPSM_Enable;
    SDIO_SendCommand(&stSdioCmdInit);
}

// Request the Card Status Registers
void SK_SdioSendCmd_13(uint32_t rca)
{
    SDIO_CmdInitTypeDef stSdioCmdInit;

    stSdioCmdInit.SDIO_Argument = (uint32_t) rca << 16;
    stSdioCmdInit.SDIO_CmdIndex = SD_CMD_SEND_STATUS;
    stSdioCmdInit.SDIO_Response = SDIO_Response_Short;
    stSdioCmdInit.SDIO_Wait = SDIO_Wait_No;
    stSdioCmdInit.SDIO_CPSM = SDIO_CPSM_Enable;
    SDIO_SendCommand(&stSdioCmdInit);
}

void SK_SdioSendCmd_16(uint16_t block_size)
{
    SDIO_CmdInitTypeDef stSdioCmdInit;

    /*!< Set Block Size for Card，cmd16,if sdhc card the block size is 512, this will not effect */
    stSdioCmdInit.SDIO_Argument = (uint32_t) block_size;
    stSdioCmdInit.SDIO_CmdIndex = SD_CMD_SET_BLOCKLEN;
    stSdioCmdInit.SDIO_Response = SDIO_Response_Short;   //r1
    stSdioCmdInit.SDIO_Wait = SDIO_Wait_No;
    stSdioCmdInit.SDIO_CPSM = SDIO_CPSM_Enable;
    SDIO_SendCommand(&stSdioCmdInit);
}

void SK_SdioSendCmd_17(uint32_t addr)
{
    SDIO_CmdInitTypeDef stSdioCmdInit;

    /*!< Send CMD17 READ_SINGLE_BLOCK */
    stSdioCmdInit.SDIO_Argument = (uint32_t)addr;
    stSdioCmdInit.SDIO_CmdIndex = SD_CMD_READ_SINGLE_BLOCK;
    stSdioCmdInit.SDIO_Response = SDIO_Response_Short;
    stSdioCmdInit.SDIO_Wait = SDIO_Wait_No;
    stSdioCmdInit.SDIO_CPSM = SDIO_CPSM_Enable;
    SDIO_SendCommand(&stSdioCmdInit);
}

void SK_SdioSendCmd_18(uint32_t addr)
{
    SDIO_CmdInitTypeDef stSdioCmdInit;

    /*!< Send CMD18 READ_MULT_BLOCK with argument data address */
    stSdioCmdInit.SDIO_Argument = (uint32_t)addr; //起始地址
    stSdioCmdInit.SDIO_CmdIndex = SD_CMD_READ_MULT_BLOCK;
    stSdioCmdInit.SDIO_Response = SDIO_Response_Short; //r1
    stSdioCmdInit.SDIO_Wait = SDIO_Wait_No;
    stSdioCmdInit.SDIO_CPSM = SDIO_CPSM_Enable;
    SDIO_SendCommand(&stSdioCmdInit);
}

void SK_SdioSendCmd_24(uint32_t WriteAddr)
{
    SDIO_CmdInitTypeDef stSdioCmdInit;

    /*!< Send CMD24 WRITE_SINGLE_BLOCK */
    stSdioCmdInit.SDIO_Argument = WriteAddr;    //写入地址
    stSdioCmdInit.SDIO_CmdIndex = SD_CMD_WRITE_SINGLE_BLOCK;
    stSdioCmdInit.SDIO_Response = SDIO_Response_Short;     //r1
    stSdioCmdInit.SDIO_Wait = SDIO_Wait_No;
    stSdioCmdInit.SDIO_CPSM = SDIO_CPSM_Enable;
    SDIO_SendCommand(&stSdioCmdInit);
}

void SK_SdioSendCmd_25(uint32_t WriteAddr)
{
    SDIO_CmdInitTypeDef stSdioCmdInit;

    /*!< Send CMD25 WRITE_MULT_BLOCK with argument data address */
    stSdioCmdInit.SDIO_Argument = (uint32_t)WriteAddr;
    stSdioCmdInit.SDIO_CmdIndex = SD_CMD_WRITE_MULT_BLOCK;
    stSdioCmdInit.SDIO_Response = SDIO_Response_Short;
    stSdioCmdInit.SDIO_Wait = SDIO_Wait_No;
    stSdioCmdInit.SDIO_CPSM = SDIO_CPSM_Enable;
    SDIO_SendCommand(&stSdioCmdInit);
}

void SK_SdioSendCmd_12(void)
{
  /*!< Send CMD12 STOP_TRANSMISSION  */
  SDIO->ARG = 0x0;
  SDIO->CMD = 0x44C;
}

/************************** SD CMD Defination End *********************************/

static SD_Error SK_CmdError(void)
{
    SD_Error errorstatus = SD_OK;
    uint32_t timeout;

    timeout = SDIO_CMD0TIMEOUT; /*!< 10000 */

    /* Check If CMD have been Send out */
    while ((timeout > 0) && (SDIO_GetFlagStatus(SDIO_FLAG_CMDSENT) == RESET))
    {
        timeout--;
    }

    if (timeout == 0)
    {
        errorstatus = SD_CMD_RSP_TIMEOUT;
        return (errorstatus);
    }

    /*!< Clear all the static flags */
    SDIO_ClearFlag(SDIO_STATIC_FLAGS);

    return(errorstatus);
}

static SD_Error SK_CheckCmdRsp_7(void)
{
    SD_Error errorstatus = SD_OK;
    uint32_t crc_fail_status, timeout_status, cmd_rsp_status;
    uint32_t timeout = 0;

    for (timeout = 0; timeout < SDIO_CMD0TIMEOUT; timeout++)
    {
        crc_fail_status = SDIO_GetFlagStatus(SDIO_FLAG_CCRCFAIL);
        timeout_status = SDIO_GetFlagStatus(SDIO_FLAG_CTIMEOUT);
        cmd_rsp_status = SDIO_GetFlagStatus(SDIO_FLAG_CMDREND);

        /*!< CRC Fail check */
        if (crc_fail_status)
        {
            errorstatus = SD_CMD_CRC_FAIL;
            SDIO_ClearFlag(SDIO_FLAG_CCRCFAIL);
            break;
        }

        /*!< Card is not V2.0 complient or card does not support the set voltage range */
        if (timeout_status)
        {
            errorstatus = SD_CMD_RSP_TIMEOUT;
            SDIO_ClearFlag(SDIO_FLAG_CTIMEOUT);
            break;
        }

        /*!< Rsp Fail check */
        if (cmd_rsp_status)
        {
            errorstatus = SD_OK;
            SDIO_ClearFlag(SDIO_FLAG_CMDREND);
            break;
        }
    }

    if (SDIO_CMD0TIMEOUT == timeout)
        errorstatus = SD_CMD_RSP_TIMEOUT;

    return errorstatus;
}

static SD_Error SK_CheckCmdRsp_1(uint8_t cmd)
{

  while (!(SDIO->STA & (SDIO_FLAG_CCRCFAIL | SDIO_FLAG_CMDREND | SDIO_FLAG_CTIMEOUT)));

  SDIO->ICR = SDIO_STATIC_FLAGS;

  return (SD_Error)(SDIO->RESP1 & SD_OCR_ERRORBITS);
}

static SD_Error SK_CheckCmdRsp_3(void)
{
    SD_Error errorstatus = SD_OK;
    uint32_t status;

    status = SDIO->STA;

    while (!(status & (SDIO_FLAG_CCRCFAIL | SDIO_FLAG_CMDREND | SDIO_FLAG_CTIMEOUT)))
    {
        status = SDIO->STA;
    }

    if (status & SDIO_FLAG_CTIMEOUT)
    {
        errorstatus = SD_CMD_RSP_TIMEOUT;
        SDIO_ClearFlag(SDIO_FLAG_CTIMEOUT);
        return(errorstatus);
    }
    /*!< Clear all the static flags */
    SDIO_ClearFlag(SDIO_STATIC_FLAGS);
    return(errorstatus);
}

static SD_Error SK_CheckCmdRsp_2(void)
{
    SD_Error errorstatus = SD_OK;
    uint32_t status;

    status = SDIO->STA;

    while (!(status & (SDIO_FLAG_CCRCFAIL | SDIO_FLAG_CTIMEOUT | SDIO_FLAG_CMDREND)))
    {
        status = SDIO->STA;
    }

    if (status & SDIO_FLAG_CTIMEOUT)
    {
        errorstatus = SD_CMD_RSP_TIMEOUT;
        SDIO_ClearFlag(SDIO_FLAG_CTIMEOUT);
        return(errorstatus);
    }
    else if (status & SDIO_FLAG_CCRCFAIL)
    {
        errorstatus = SD_CMD_CRC_FAIL;
        SDIO_ClearFlag(SDIO_FLAG_CCRCFAIL);
        return(errorstatus);
    }

    /*!< Clear all the static flags */
    SDIO_ClearFlag(SDIO_STATIC_FLAGS);

    return(errorstatus);

}

static SD_Error SK_CheckCmdRsp_6(void)
{
    SD_Error errorstatus = SD_OK;
    uint32_t response;

    do {
        errorstatus = SK_CheckCmdRsp_3();
        SK_SD_RETURN_CHECK(errorstatus);

        if (SD_CMD_SET_REL_ADDR != SDIO_GetCommandResponse())
        {
            errorstatus = SD_ILLEGAL_CMD;
            break;
        }

        response = SDIO_GetResponse(SDIO_RESP1);

        if (response & SD_R6_GENERAL_UNKNOWN_ERROR)
        {
            errorstatus = SD_GENERAL_UNKNOWN_ERROR;
            break;
        }

        if (response & SD_R6_ILLEGAL_CMD)
        {
            errorstatus = SD_ILLEGAL_CMD;
            break;
        }

        if (response & SD_R6_COM_CRC_FAILED)
        {
            errorstatus = SD_COM_CRC_FAILED;
            break;
        }

        g_stSDCARD.RCA = (uint16_t)(response >> 16);

    } while (0);

    return errorstatus;
}

static void _parseCID(uint32_t *CID_RAW_DATA)
{
    uint8_t tmp = 0;

    /*!< Byte 0 */
    tmp = (uint8_t)((CID_RAW_DATA[0] & 0xFF000000) >> 24);
    g_stSDCARD.SD_cid.ManufacturerID = tmp;
    
    /*!< Byte 1 */
    tmp = (uint8_t)((CID_RAW_DATA[0] & 0x00FF0000) >> 16);
    g_stSDCARD.SD_cid.OEM_AppliID = tmp << 8;
    
    /*!< Byte 2 */
    tmp = (uint8_t)((CID_RAW_DATA[0] & 0x000000FF00) >> 8);
    g_stSDCARD.SD_cid.OEM_AppliID |= tmp;
    
    /*!< Byte 3 */
    tmp = (uint8_t)(CID_RAW_DATA[0] & 0x000000FF);
    g_stSDCARD.SD_cid.ProdName1 = tmp << 24;
    
    /*!< Byte 4 */
    tmp = (uint8_t)((CID_RAW_DATA[1] & 0xFF000000) >> 24);
    g_stSDCARD.SD_cid.ProdName1 |= tmp << 16;
    
    /*!< Byte 5 */
    tmp = (uint8_t)((CID_RAW_DATA[1] & 0x00FF0000) >> 16);
    g_stSDCARD.SD_cid.ProdName1 |= tmp << 8;
    
    /*!< Byte 6 */
    tmp = (uint8_t)((CID_RAW_DATA[1] & 0x0000FF00) >> 8);
    g_stSDCARD.SD_cid.ProdName1 |= tmp;
    
    /*!< Byte 7 */
    tmp = (uint8_t)(CID_RAW_DATA[1] & 0x000000FF);
    g_stSDCARD.SD_cid.ProdName2 = tmp;
    
    /*!< Byte 8 */
    tmp = (uint8_t)((CID_RAW_DATA[2] & 0xFF000000) >> 24);
    g_stSDCARD.SD_cid.ProdRev = tmp;
    
    /*!< Byte 9 */
    tmp = (uint8_t)((CID_RAW_DATA[2] & 0x00FF0000) >> 16);
    g_stSDCARD.SD_cid.ProdSN = tmp << 24;
    
    /*!< Byte 10 */
    tmp = (uint8_t)((CID_RAW_DATA[2] & 0x0000FF00) >> 8);
    g_stSDCARD.SD_cid.ProdSN |= tmp << 16;
    
    /*!< Byte 11 */
    tmp = (uint8_t)(CID_RAW_DATA[2] & 0x000000FF);
    g_stSDCARD.SD_cid.ProdSN |= tmp << 8;
    
    /*!< Byte 12 */
    tmp = (uint8_t)((CID_RAW_DATA[3] & 0xFF000000) >> 24);
    g_stSDCARD.SD_cid.ProdSN |= tmp;
    
    /*!< Byte 13 */
    tmp = (uint8_t)((CID_RAW_DATA[3] & 0x00FF0000) >> 16);
    g_stSDCARD.SD_cid.Reserved1 |= (tmp & 0xF0) >> 4;
    g_stSDCARD.SD_cid.ManufactDate = (tmp & 0x0F) << 8;
    
    /*!< Byte 14 */
    tmp = (uint8_t)((CID_RAW_DATA[3] & 0x0000FF00) >> 8);
    g_stSDCARD.SD_cid.ManufactDate |= tmp;
    
    /*!< Byte 15 */
    tmp = (uint8_t)(CID_RAW_DATA[3] & 0x000000FF);
    g_stSDCARD.SD_cid.CID_CRC = (tmp & 0xFE) >> 1;
    g_stSDCARD.SD_cid.Reserved2 = 1;
}

static void _parseCSD(uint32_t *CSD_RAW_DATA)
{
    uint8_t tmp = 0;

    /*!< Byte 0 */
    tmp = (uint8_t)((CSD_RAW_DATA[0] & 0xFF000000) >> 24);
    g_stSDCARD.SD_csd.CSDStruct = (tmp & 0xC0) >> 6;
    g_stSDCARD.SD_csd.SysSpecVersion = (tmp & 0x3C) >> 2;
    g_stSDCARD.SD_csd.Reserved1 = tmp & 0x03;
    
    /*!< Byte 1 */
    tmp = (uint8_t)((CSD_RAW_DATA[0] & 0x00FF0000) >> 16);
    g_stSDCARD.SD_csd.TAAC = tmp;
    
    /*!< Byte 2 */
    tmp = (uint8_t)((CSD_RAW_DATA[0] & 0x0000FF00) >> 8);
    g_stSDCARD.SD_csd.NSAC = tmp;
    
    /*!< Byte 3 */
    tmp = (uint8_t)(CSD_RAW_DATA[0] & 0x000000FF);
    g_stSDCARD.SD_csd.MaxBusClkFrec = tmp;
    
    /*!< Byte 4 */
    tmp = (uint8_t)((CSD_RAW_DATA[1] & 0xFF000000) >> 24);
    g_stSDCARD.SD_csd.CardComdClasses = tmp << 4;
    
    /*!< Byte 5 */
    tmp = (uint8_t)((CSD_RAW_DATA[1] & 0x00FF0000) >> 16);
    g_stSDCARD.SD_csd.CardComdClasses |= (tmp & 0xF0) >> 4;
    g_stSDCARD.SD_csd.RdBlockLen = tmp & 0x0F;
    
    /*!< Byte 6 */
    tmp = (uint8_t)((CSD_RAW_DATA[1] & 0x0000FF00) >> 8);
    g_stSDCARD.SD_csd.PartBlockRead = (tmp & 0x80) >> 7;
    g_stSDCARD.SD_csd.WrBlockMisalign = (tmp & 0x40) >> 6;
    g_stSDCARD.SD_csd.RdBlockMisalign = (tmp & 0x20) >> 5;
    g_stSDCARD.SD_csd.DSRImpl = (tmp & 0x10) >> 4;
    g_stSDCARD.SD_csd.Reserved2 = 0; /*!< Reserved */
    
    if ((g_stSDCARD.CardType == SDIO_STD_CAPACITY_SD_CARD_V1_1) || (g_stSDCARD.CardType == SDIO_STD_CAPACITY_SD_CARD_V2_0))
    {
        g_stSDCARD.SD_csd.DeviceSize = (tmp & 0x03) << 10;

        /*!< Byte 7 */
        tmp = (uint8_t)(CSD_RAW_DATA[1] & 0x000000FF);
        g_stSDCARD.SD_csd.DeviceSize |= (tmp) << 2;

        /*!< Byte 8 */
        tmp = (uint8_t)((CSD_RAW_DATA[2] & 0xFF000000) >> 24);
        g_stSDCARD.SD_csd.DeviceSize |= (tmp & 0xC0) >> 6;

        g_stSDCARD.SD_csd.MaxRdCurrentVDDMin = (tmp & 0x38) >> 3;
        g_stSDCARD.SD_csd.MaxRdCurrentVDDMax = (tmp & 0x07);

        /*!< Byte 9 */
        tmp = (uint8_t)((CSD_RAW_DATA[2] & 0x00FF0000) >> 16);
        g_stSDCARD.SD_csd.MaxWrCurrentVDDMin = (tmp & 0xE0) >> 5;
        g_stSDCARD.SD_csd.MaxWrCurrentVDDMax = (tmp & 0x1C) >> 2;
        g_stSDCARD.SD_csd.DeviceSizeMul = (tmp & 0x03) << 1;
        /*!< Byte 10 */
        tmp = (uint8_t)((CSD_RAW_DATA[2] & 0x0000FF00) >> 8);
        g_stSDCARD.SD_csd.DeviceSizeMul |= (tmp & 0x80) >> 7;

        g_stSDCARD.CardCapacity = (g_stSDCARD.SD_csd.DeviceSize + 1) ;
        g_stSDCARD.CardCapacity *= (1 << (g_stSDCARD.SD_csd.DeviceSizeMul + 2));
        g_stSDCARD.CardBlockSize = 1 << (g_stSDCARD.SD_csd.RdBlockLen);
        g_stSDCARD.CardCapacity *= g_stSDCARD.CardBlockSize;
    }
    else if (g_stSDCARD.CardType == SDIO_HIGH_CAPACITY_SD_CARD)
    {
        /*!< Byte 7 */
        tmp = (uint8_t)(CSD_RAW_DATA[1] & 0x000000FF);
        g_stSDCARD.SD_csd.DeviceSize = (tmp & 0x3F) << 16;

        /*!< Byte 8 */
        tmp = (uint8_t)((CSD_RAW_DATA[2] & 0xFF000000) >> 24);

        g_stSDCARD.SD_csd.DeviceSize |= (tmp << 8);

        /*!< Byte 9 */
        tmp = (uint8_t)((CSD_RAW_DATA[2] & 0x00FF0000) >> 16);

        g_stSDCARD.SD_csd.DeviceSize |= (tmp);

        /*!< Byte 10 */
        tmp = (uint8_t)((CSD_RAW_DATA[2] & 0x0000FF00) >> 8);

        g_stSDCARD.CardCapacity = (g_stSDCARD.SD_csd.DeviceSize + 1) * 512 * 1024;
        g_stSDCARD.CardBlockSize = 512;    
    }

    g_stSDCARD.SD_csd.EraseGrSize = (tmp & 0x40) >> 6;
    g_stSDCARD.SD_csd.EraseGrMul = (tmp & 0x3F) << 1;
    
    /*!< Byte 11 */
    tmp = (uint8_t)(CSD_RAW_DATA[2] & 0x000000FF);
    g_stSDCARD.SD_csd.EraseGrMul |= (tmp & 0x80) >> 7;
    g_stSDCARD.SD_csd.WrProtectGrSize = (tmp & 0x7F);
    
    /*!< Byte 12 */
    tmp = (uint8_t)((CSD_RAW_DATA[3] & 0xFF000000) >> 24);
    g_stSDCARD.SD_csd.WrProtectGrEnable = (tmp & 0x80) >> 7;
    g_stSDCARD.SD_csd.ManDeflECC = (tmp & 0x60) >> 5;
    g_stSDCARD.SD_csd.WrSpeedFact = (tmp & 0x1C) >> 2;
    g_stSDCARD.SD_csd.MaxWrBlockLen = (tmp & 0x03) << 2;
    
    /*!< Byte 13 */
    tmp = (uint8_t)((CSD_RAW_DATA[3] & 0x00FF0000) >> 16);
    g_stSDCARD.SD_csd.MaxWrBlockLen |= (tmp & 0xC0) >> 6;
    g_stSDCARD.SD_csd.WriteBlockPaPartial = (tmp & 0x20) >> 5;
    g_stSDCARD.SD_csd.Reserved3 = 0;
    g_stSDCARD.SD_csd.ContentProtectAppli = (tmp & 0x01);
    
    /*!< Byte 14 */
    tmp = (uint8_t)((CSD_RAW_DATA[3] & 0x0000FF00) >> 8);
    g_stSDCARD.SD_csd.FileFormatGrouop = (tmp & 0x80) >> 7;
    g_stSDCARD.SD_csd.CopyFlag = (tmp & 0x40) >> 6;
    g_stSDCARD.SD_csd.PermWrProtect = (tmp & 0x20) >> 5;
    g_stSDCARD.SD_csd.TempWrProtect = (tmp & 0x10) >> 4;
    g_stSDCARD.SD_csd.FileFormat = (tmp & 0x0C) >> 2;
    g_stSDCARD.SD_csd.ECC = (tmp & 0x03);
    
    /*!< Byte 15 */
    tmp = (uint8_t)(CSD_RAW_DATA[3] & 0x000000FF);
    g_stSDCARD.SD_csd.CSD_CRC = (tmp & 0xFE) >> 1;
    g_stSDCARD.SD_csd.Reserved4 = 1;
}

/* Set SDIO_CK to 24MHz in Transfer Mode */
static void SK_SetSDIOFreqToHighSpeed(void)
{
    SDIO_InitTypeDef stSdioInit;

    /*SDIOCLK = HCLK, SDIO_CK = HCLK/(2 + SDIO_TRANSFER_CLK_DIV) = 24MHz */
    stSdioInit.SDIO_ClockDiv = SDIO_TRANSFER_CLK_DIV;   
    stSdioInit.SDIO_ClockEdge = SDIO_ClockEdge_Rising;
    stSdioInit.SDIO_ClockBypass = SDIO_ClockBypass_Disable;
    stSdioInit.SDIO_ClockPowerSave = SDIO_ClockPowerSave_Disable;
    stSdioInit.SDIO_BusWide = SDIO_BusWide_1b;
    stSdioInit.SDIO_HardwareFlowControl = SDIO_HardwareFlowControl_Disable;
    SDIO_Init(&stSdioInit);
}

/* Set SDIO_CK to 400KHz in Identifacation Mode */
static void SK_SetSDIOFreqToLowSpeed(void)
{
    SDIO_InitTypeDef stSdioInit;

    /*!< SDIOCLK = HCLK, SDIO_CK = HCLK/(2 + SDIO_INIT_CLK_DIV) */
    /*!< SDIO_CK for Initialization should not exceed 400 KHz */
    /* HCLK = 72MHz, SDIOCLK = 72MHz, SDIO_CK = HCLK/(178 + 2) = 400 KHz */
    stSdioInit.SDIO_ClockDiv = SDIO_INIT_CLK_DIV;
    stSdioInit.SDIO_ClockEdge = SDIO_ClockEdge_Rising;
    stSdioInit.SDIO_ClockBypass = SDIO_ClockBypass_Disable;
    stSdioInit.SDIO_ClockPowerSave = SDIO_ClockPowerSave_Disable;
    stSdioInit.SDIO_BusWide = SDIO_BusWide_1b;
    stSdioInit.SDIO_HardwareFlowControl = SDIO_HardwareFlowControl_Disable;
    SDIO_Init(&stSdioInit);

}

static void SK_SetSDIO4BitWide(void)
{
    SDIO_InitTypeDef stSdioInit;

    /*!< Configure the SDIO peripheral */
    stSdioInit.SDIO_ClockDiv = SDIO_TRANSFER_CLK_DIV; 
    stSdioInit.SDIO_ClockEdge = SDIO_ClockEdge_Rising;
    stSdioInit.SDIO_ClockBypass = SDIO_ClockBypass_Disable;
    stSdioInit.SDIO_ClockPowerSave = SDIO_ClockPowerSave_Disable;
    stSdioInit.SDIO_BusWide = SDIO_BusWide_4b;
    stSdioInit.SDIO_HardwareFlowControl = SDIO_HardwareFlowControl_Disable;
    SDIO_Init(&stSdioInit);
}

static SD_Error SK_SD_SelectDeselect(uint32_t addr)
{
  SD_Error errorstatus = SD_OK;
  SDIO_CmdInitTypeDef stSdioCmdInit;

  /*!< Send CMD7 SDIO_SEL_DESEL_CARD */
  stSdioCmdInit.SDIO_Argument =  addr;
  stSdioCmdInit.SDIO_CmdIndex = SD_CMD_SEL_DESEL_CARD;
  stSdioCmdInit.SDIO_Response = SDIO_Response_Short;
  stSdioCmdInit.SDIO_Wait = SDIO_Wait_No;
  stSdioCmdInit.SDIO_CPSM = SDIO_CPSM_Enable;
  SDIO_SendCommand(&stSdioCmdInit);

  errorstatus = SK_CheckCmdRsp_1(SD_CMD_SEL_DESEL_CARD);

  return(errorstatus);
}

static SD_Error SK_Get_SCR(uint16_t rca, uint32_t *pscr)
{
    SD_Error errorstatus = SD_OK;
    SDIO_DataInitTypeDef SDIO_DataInitStructure;
    SDIO_CmdInitTypeDef stSdioCmdInit;
    uint16_t delay_time;
    uint32_t index = 0;
    uint32_t tempscr[2] = {0, 0};

    do {
        // No need to send the CMD16 to change the Block length, SDHC is fixed in 512-Bytes
        /*!< Set Block Size To 8 Bytes */ 
        stSdioCmdInit.SDIO_Argument = (uint32_t)8;
        stSdioCmdInit.SDIO_CmdIndex = SD_CMD_SET_BLOCKLEN; //  cmd16
        stSdioCmdInit.SDIO_Response = SDIO_Response_Short;  //r1
        stSdioCmdInit.SDIO_Wait = SDIO_Wait_No;
        stSdioCmdInit.SDIO_CPSM = SDIO_CPSM_Enable;
        SDIO_SendCommand(&stSdioCmdInit);

        errorstatus = SK_CheckCmdRsp_1(SD_CMD_SET_BLOCKLEN);
        SK_SD_RETURN_CHECK(errorstatus);

        SK_SdioSendCmd_55(g_stSDCARD.RCA);
        errorstatus = SK_CheckCmdRsp_1(SD_CMD_APP_CMD);
        SK_SD_RETURN_CHECK(errorstatus);

        SDIO_DataInitStructure.SDIO_DataTimeOut = SD_DATATIMEOUT;
        SDIO_DataInitStructure.SDIO_DataLength = 8; //8-bytes
        SDIO_DataInitStructure.SDIO_DataBlockSize = SDIO_DataBlockSize_8b  ;// Set Block Size 8-bytes 
        SDIO_DataInitStructure.SDIO_TransferDir = SDIO_TransferDir_ToSDIO;
        SDIO_DataInitStructure.SDIO_TransferMode = SDIO_TransferMode_Block;
        SDIO_DataInitStructure.SDIO_DPSM = SDIO_DPSM_Enable;
        SDIO_DataConfig(&SDIO_DataInitStructure);

        for(delay_time = 0; delay_time < 20; delay_time++)
            __nop();

        /*!< Send ACMD51 SD_APP_SEND_SCR with argument as 0 */
        SK_SdioSendAppCmd_51();
        errorstatus = SK_CheckCmdRsp_1(SD_CMD_SD_APP_SEND_SCR);
        SK_SD_RETURN_CHECK(errorstatus);

        // Wait for Status.    FIFO OverFlow         Data CRC ERR         Data Timeout          Recv Data         No Start bit
        while (!(SDIO->STA & (SDIO_FLAG_RXOVERR | SDIO_FLAG_DCRCFAIL | SDIO_FLAG_DTIMEOUT | SDIO_FLAG_DBCKEND | SDIO_FLAG_STBITERR)))
        {            
            if (SDIO_GetFlagStatus(SDIO_FLAG_RXDAVL) != RESET)
            {   
                *(tempscr + index) = SDIO_ReadData();
                    index++; 
        
                if(index > 1 ) 
                    break;
            }
        }

        if (SDIO_GetFlagStatus(SDIO_FLAG_DTIMEOUT) != RESET)
        {
            SDIO_ClearFlag(SDIO_FLAG_DTIMEOUT);
            errorstatus = SD_DATA_TIMEOUT;
            break;
        }
        else if (SDIO_GetFlagStatus(SDIO_FLAG_DCRCFAIL) != RESET)
        {
            SDIO_ClearFlag(SDIO_FLAG_DCRCFAIL);
            errorstatus = SD_DATA_CRC_FAIL;
            break;
        }
        else if (SDIO_GetFlagStatus(SDIO_FLAG_RXOVERR) != RESET)
        {
            SDIO_ClearFlag(SDIO_FLAG_RXOVERR);
            errorstatus = SD_RX_OVERRUN;
            break;
        }
        else if (SDIO_GetFlagStatus(SDIO_FLAG_STBITERR) != RESET)
        {
            SDIO_ClearFlag(SDIO_FLAG_STBITERR);
            errorstatus = SD_START_BIT_ERR;
            break;
        }

        /*!< Clear all the static flags */
        SDIO_ClearFlag(SDIO_STATIC_FLAGS);

        *(pscr + 1) = ((tempscr[0] & SD_0TO7BITS) << 24) | ((tempscr[0] & SD_8TO15BITS) << 8) | ((tempscr[0] & SD_16TO23BITS) >> 8) | ((tempscr[0] & SD_24TO31BITS) >> 24);
        *(pscr) = ((tempscr[1] & SD_0TO7BITS) << 24) | ((tempscr[1] & SD_8TO15BITS) << 8) | ((tempscr[1] & SD_16TO23BITS) >> 8) | ((tempscr[1] & SD_24TO31BITS) >> 24);
    } while (0);

    return errorstatus;
}

static SDTransferState SK_GetCurrentCardState()
{
    SDTransferState ret;
    SD_Error errorstatus = SD_OK;
    __IO uint32_t respR1 = 0;
    SDCardState state;

    SK_SdioSendCmd_13(g_stSDCARD.RCA);
    errorstatus = SK_CheckCmdRsp_1(SD_CMD_SEND_STATUS);

    if (errorstatus != SD_OK)
        return SD_TRANSFER_ERROR;

    respR1 = SDIO_GetResponse(SDIO_RESP1);
    state = (SDCardState)((respR1 >> 9) & 0x0F);

    if (SD_CARD_TRANSFER == state)
        ret = SD_TRANSFER_OK;
    else if (SD_CARD_ERROR == state)
        ret = SD_TRANSFER_ERROR;
    else
        ret = SD_TRANSFER_BUSY;

    return ret;
}

static SD_Error SK_Enable_4Bit_WideBus(void)
{
    SD_Error errorstatus = SD_OK;

    do {
        /// First get the SCR to ensure the card support the 4 bit data transfer
        errorstatus = SK_Get_SCR(g_stSDCARD.RCA, g_stSDCARD.SD_scr_raw_data);
        SK_SD_RETURN_CHECK(errorstatus);

        if (g_stSDCARD.SD_scr_raw_data[1] & SD_WIDE_BUS_SUPPORT)
        {
             /*!< Send CMD55 APP_CMD with argument as card's RCA.*/
            SK_SdioSendCmd_55(g_stSDCARD.RCA);
            errorstatus = SK_CheckCmdRsp_1(SD_CMD_APP_CMD);
            SK_SD_RETURN_CHECK(errorstatus);

            /*!< Send ACMD6 APP_CMD with argument as 2 for wide bus mode */
            SK_SdioSendAppCmd_6(SD_4_BIT_WIDE);
            errorstatus = SK_CheckCmdRsp_1(SD_CMD_APP_SD_SET_BUSWIDTH);
            SK_SD_RETURN_CHECK(errorstatus);

            SK_SetSDIO4BitWide();
        }
    } while (0);

    return errorstatus;
}

static SD_Error SK_IsCardProgramming(uint8_t *pstatus)
{
    SD_Error errorstatus = SD_OK;
    __IO uint32_t respR1 = 0, status = 0;

    SK_SdioSendCmd_13(g_stSDCARD.RCA);

    status = SDIO->STA;

    while (!(status & (SDIO_FLAG_CCRCFAIL | SDIO_FLAG_CMDREND | SDIO_FLAG_CTIMEOUT)))
    {
        status = SDIO->STA;
    }

    do {
        if (status & SDIO_FLAG_CTIMEOUT)
        {
            errorstatus = SD_CMD_RSP_TIMEOUT;
            SDIO_ClearFlag(SDIO_FLAG_CTIMEOUT);
            break;
        }

        if (status & SDIO_FLAG_CCRCFAIL)
        {
            errorstatus = SD_CMD_CRC_FAIL;
            SDIO_ClearFlag(SDIO_FLAG_CCRCFAIL);
            break;
        }

        status = (uint32_t)SDIO_GetCommandResponse();

        if (status != SD_CMD_SEND_STATUS)
        {
            errorstatus = SD_ILLEGAL_CMD;
            break;
        }

        /*!< Clear all the static flags */
        SDIO_ClearFlag(SDIO_STATIC_FLAGS);

        /*!< We have received response, retrieve it for analysis  */
        respR1 = SDIO_GetResponse(SDIO_RESP1);

        /*!< Find out card status */
        *pstatus = (uint8_t) ((respR1 >> 9) & 0x0000000F);   //status[12:9] :cardstate 

        if ((respR1 & SD_OCR_ERRORBITS) == 0x00)
        {
            return(errorstatus);
        }

        if (respR1 & SD_OCR_ADDR_OUT_OF_RANGE)
        {
            return(SD_ADDR_OUT_OF_RANGE);
        }

        if (respR1 & SD_OCR_ADDR_MISALIGNED)
        {
            return(SD_ADDR_MISALIGNED);
        }

        if (respR1 & SD_OCR_BLOCK_LEN_ERR)
        {
            return(SD_BLOCK_LEN_ERR);
        }

        if (respR1 & SD_OCR_ERASE_SEQ_ERR)
        {
            return(SD_ERASE_SEQ_ERR);
        }

        if (respR1 & SD_OCR_BAD_ERASE_PARAM)
        {
            return(SD_BAD_ERASE_PARAM);
        }

        if (respR1 & SD_OCR_WRITE_PROT_VIOLATION)
        {
            return(SD_WRITE_PROT_VIOLATION);
        }

        if (respR1 & SD_OCR_LOCK_UNLOCK_FAILED)
        {
            return(SD_LOCK_UNLOCK_FAILED);
        }

        if (respR1 & SD_OCR_COM_CRC_FAILED)
        {
            return(SD_COM_CRC_FAILED);
        }

        if (respR1 & SD_OCR_ILLEGAL_CMD)
        {
            return(SD_ILLEGAL_CMD);
        }

        if (respR1 & SD_OCR_CARD_ECC_FAILED)
        {
            return(SD_CARD_ECC_FAILED);
        }

        if (respR1 & SD_OCR_CC_ERROR)
        {
            return(SD_CC_ERROR);
        }

        if (respR1 & SD_OCR_GENERAL_UNKNOWN_ERROR)
        {
            return(SD_GENERAL_UNKNOWN_ERROR);
        }

        if (respR1 & SD_OCR_STREAM_READ_UNDERRUN)
        {
            return(SD_STREAM_READ_UNDERRUN);
        }

        if (respR1 & SD_OCR_STREAM_WRITE_OVERRUN)
        {
            return(SD_STREAM_WRITE_OVERRUN);
        }

        if (respR1 & SD_OCR_CID_CSD_OVERWRIETE)
        {
            return(SD_CID_CSD_OVERWRITE);
        }

        if (respR1 & SD_OCR_WP_ERASE_SKIP)
        {
            return(SD_WP_ERASE_SKIP);
        }

        if (respR1 & SD_OCR_CARD_ECC_DISABLED)
        {
            return(SD_CARD_ECC_DISABLED);
        }

        if (respR1 & SD_OCR_ERASE_RESET)
        {
            return(SD_ERASE_RESET);
        }

        if (respR1 & SD_OCR_AKE_SEQ_ERROR)
        {
            return(SD_AKE_SEQ_ERROR);
        }

    } while (0);

    return errorstatus;
}

static SD_Error SK_SdcardPowerOn(void)
{
    SD_Error errorstatus = SD_OK;
    uint32_t response = 0, count = 0, validvoltage = 0;

    g_stSDCARD.CardType = SDIO_STD_CAPACITY_SD_CARD_V1_1;

    /*!< Power ON Sequence -----------------------------------------------------*/
    /*!< Configure the SDIO peripheral */
    /*!< SDIOCLK = HCLK, SDIO_CK = HCLK/(2 + SDIO_INIT_CLK_DIV) */
    /*!< SDIO_CK for Initialization should not exceed 400 KHz */
    /* HCLK = 72MHz, SDIOCLK = 72MHz, SDIO_CK = HCLK/(178 + 2) = 400 KHz */
    SK_SetSDIOFreqToLowSpeed();

    /// STM32 SDIO CARD Clock Power On
    SDIO_SetPowerState(SDIO_PowerState_ON);

    SDIO_ClockCmd(ENABLE);

    do {

        /*!< CMD0: GO_IDLE_STATE ---------------------------------------------------*/
        SK_SdioSendCmd_0();
        errorstatus = SK_CmdError();
        SK_SD_RETURN_CHECK(errorstatus);

        /*!< CMD8: SEND_IF_COND ----------------------------------------------------*/
        SK_SdioSendCmd_8();
        errorstatus = SK_CheckCmdRsp_7();
        SK_SD_RETURN_CHECK(errorstatus);

        // Support Card V2.0
        g_stSDCARD.CardType = SDIO_STD_CAPACITY_SD_CARD_V2_0;

        /*!< CMD55: SD_CMD_APP_CMD ------------------------------------------------ */
        SK_SdioSendCmd_55(0x00);
        errorstatus = SK_CheckCmdRsp_1(SD_CMD_APP_CMD);
        SK_SD_RETURN_CHECK(errorstatus);

        while ((!validvoltage) && (count < SD_MAX_VOLT_TRIAL))
        {
            /*!< CMD55: SD_CMD_APP_CMD ------------------------------------------------ */
            SK_SdioSendCmd_55(0x00);
            errorstatus = SK_CheckCmdRsp_1(SD_CMD_APP_CMD);
            if (errorstatus != SD_OK)
                return errorstatus;

            /*!< ACMD41: SD_CMD_APP_CMD ------------------------------------------------ */
            SK_SdioSendAppCmd_41();
            errorstatus = SK_CheckCmdRsp_3();
            if (errorstatus != SD_OK)
                return errorstatus;

            response = SDIO_GetResponse(SDIO_RESP1);
            validvoltage = (((response >> 31) == 1) ? 1 : 0);

            count++;
        }

        if (count >= SD_MAX_VOLT_TRIAL)
        {
            errorstatus = SD_INVALID_VOLTRANGE;
            break;
        }

        if (response & SD_HIGH_CAPACITY)
        {
            g_stSDCARD.CardType = SDIO_HIGH_CAPACITY_SD_CARD;
        }

    } while (0);

    return errorstatus;
}

static SD_Error SK_SD_InitializeCards(void)
{
    SD_Error errorstatus = SD_OK;
    uint32_t CID_RAW_DATA[4] = {0};
    uint32_t CSD_RAW_DATA[4] = {0};

    do {
        if (SDIO_GetPowerState() == SDIO_PowerState_OFF)
        {
            errorstatus = SD_REQUEST_NOT_APPLICABLE;
            break;
        }

        if (SDIO_SECURE_DIGITAL_IO_CARD != g_stSDCARD.CardType)
        {
            /*!< CMD2: ALL_SEND_CID ------------------------------------------------ */
            SK_SdioSendCmd_2();
            errorstatus = SK_CheckCmdRsp_2();
            SK_SD_RETURN_CHECK(errorstatus);

            CID_RAW_DATA[0] = SDIO_GetResponse(SDIO_RESP1);
            CID_RAW_DATA[1] = SDIO_GetResponse(SDIO_RESP2);
            CID_RAW_DATA[2] = SDIO_GetResponse(SDIO_RESP3);
            CID_RAW_DATA[3] = SDIO_GetResponse(SDIO_RESP4);

            /*!< CMD3: SD_CMD_SET_REL_ADDR ------------------------------------------ */
            SK_SdioSendCmd_3();
            errorstatus = SK_CheckCmdRsp_6();
            SK_SD_RETURN_CHECK(errorstatus);

            /*!< CMD9: SD_CMD_SEND_CSD ------------------------------------------ */
            SK_SdioSendCmd_9();
            errorstatus = SK_CheckCmdRsp_2();
            SK_SD_RETURN_CHECK(errorstatus);

            CSD_RAW_DATA[0] = SDIO_GetResponse(SDIO_RESP1);
            CSD_RAW_DATA[1] = SDIO_GetResponse(SDIO_RESP2);
            CSD_RAW_DATA[2] = SDIO_GetResponse(SDIO_RESP3);
            CSD_RAW_DATA[3] = SDIO_GetResponse(SDIO_RESP4);

            _parseCID(CID_RAW_DATA);
            _parseCSD(CSD_RAW_DATA);
        }
    } while (0);

    return errorstatus;
}

void SK_SDIO_SDCARD_Init(void)
{
    SD_Error errorstatus = SD_OK;

    SK_SD_NVIC_Config();
    SK_SD_GPIOConfig();
    SK_SDIO_Config();
    SK_SDIO_DMA2Config();

    do {
        errorstatus = SK_SdcardPowerOn();
        SK_SD_RETURN_CHECK(errorstatus);

        errorstatus = SK_SD_InitializeCards();
        SK_SD_RETURN_CHECK(errorstatus);

        // Now in Transfer Mode, just change to 24Mhz Clock
        SK_SetSDIOFreqToHighSpeed();

        errorstatus = SK_SD_SelectDeselect((uint32_t)(g_stSDCARD.RCA << 16));
        SK_SD_RETURN_CHECK(errorstatus);

        errorstatus = SK_Enable_4Bit_WideBus();
        SK_SD_RETURN_CHECK(errorstatus);
    } while (0);
}

SD_Error SK_SD_Erase(uint32_t start_addr, uint32_t end_addr)
{
    SD_Error errorstatus = SD_OK;
    uint32_t delay = 0;
    __IO uint32_t maxdelay = 0;
    uint8_t cardstate = 0;

    maxdelay = 120000 / ((SDIO->CLKCR & 0xFF) + 2);

    if (g_stSDCARD.CardType == SDIO_HIGH_CAPACITY_SD_CARD)
    {
        start_addr /= SD_BLOCK_SIZE_512;
        end_addr   /= SD_BLOCK_SIZE_512;
    }

    do {
        /*!< Send CMD32 SD_ERASE_GRP_START with argument as addr  */
        SK_SdioSendCmd_32(start_addr);
        errorstatus = SK_CheckCmdRsp_1(SD_CMD_SD_ERASE_GRP_START);
        SK_SD_RETURN_CHECK(errorstatus);

        /*!< Send CMD33 SD_ERASE_GRP_END with argument as addr  */
        SK_SdioSendCmd_33(end_addr);
        errorstatus = SK_CheckCmdRsp_1(SD_CMD_SD_ERASE_GRP_END);
        SK_SD_RETURN_CHECK(errorstatus);

        /*!< Send CMD38 ERASE */
        SK_SdioSendCmd_38();
        errorstatus = SK_CheckCmdRsp_1(SD_CMD_ERASE);
        SK_SD_RETURN_CHECK(errorstatus);

        for (delay = 0; delay < maxdelay; delay++);

        /*!< Wait till the card is in programming state */
        errorstatus = SK_IsCardProgramming(&cardstate);
        while ((errorstatus == SD_OK) && ((SD_CARD_PROGRAMMING == cardstate) || (SD_CARD_RECEIVING == cardstate)))
        {
            errorstatus = SK_IsCardProgramming(&cardstate);
        }
    } while (0);

    return errorstatus;
}

SD_Error SK_SD_ReadBlock(uint8_t *read_buf, uint32_t read_addr, uint16_t blk_size)
{
    SD_Error errorstatus = SD_OK;
    SDIO_DataInitTypeDef SDIO_DataInitStructure;

    TransferError = SD_OK;
    TransferEnd = 0;
    StopCondition = 0;

    SDIO->DCTRL = 0x0;

    if (g_stSDCARD.CardType == SDIO_HIGH_CAPACITY_SD_CARD)
    {
        blk_size   = SD_BLOCK_SIZE_512;
        read_addr /= SD_BLOCK_SIZE_512;
    }

    do {
        /*!< Send CMD16 SD_CMD_SET_BLOCKLEN with argument as block size  */
        SK_SdioSendCmd_16(blk_size);
        errorstatus = SK_CheckCmdRsp_1(SD_CMD_SET_BLOCKLEN);
        SK_SD_RETURN_CHECK(errorstatus);

        /*!< Configure the Data and make the state machine go into the WAIT_R State */
        SDIO_DataInitStructure.SDIO_DataTimeOut = SD_DATATIMEOUT;
        SDIO_DataInitStructure.SDIO_DataLength = blk_size;
        SDIO_DataInitStructure.SDIO_DataBlockSize = SDIO_DataBlockSize_512b;
        SDIO_DataInitStructure.SDIO_TransferDir = SDIO_TransferDir_ToSDIO;
        SDIO_DataInitStructure.SDIO_TransferMode = SDIO_TransferMode_Block;
        SDIO_DataInitStructure.SDIO_DPSM = SDIO_DPSM_Enable;
        SDIO_DataConfig(&SDIO_DataInitStructure);

        /*!< Send CMD17 READ_SINGLE_BLOCK */
        SK_SdioSendCmd_17(read_addr);
        errorstatus = SK_CheckCmdRsp_1(SD_CMD_READ_SINGLE_BLOCK);
        SK_SD_RETURN_CHECK(errorstatus);

        // Use DMA2
        SDIO_ITConfig(SDIO_IT_DATAEND, ENABLE);
        SDIO_DMACmd(ENABLE);
        SD_DMA_RxConfig((uint32_t *)read_buf, blk_size);

    } while (0);

    return errorstatus;
}

SD_Error SK_SD_ReadMultiBlocks(uint8_t *readbuff, uint32_t ReadAddr, uint16_t BlockSize, uint32_t NumberOfBlocks)
{
    SD_Error errorstatus = SD_OK;
    SDIO_DataInitTypeDef SDIO_DataInitStructure;

    TransferError = SD_OK;
    TransferEnd = 0;
    StopCondition = 1;

    SDIO->DCTRL = 0x0;

    if (g_stSDCARD.CardType == SDIO_HIGH_CAPACITY_SD_CARD)
    {
        BlockSize = SD_BLOCK_SIZE_512;
        ReadAddr /= SD_BLOCK_SIZE_512;
    }

    do {
        /*!< Send CMD16 SD_CMD_SET_BLOCKLEN with argument as block size  */
        SK_SdioSendCmd_16(BlockSize);
        errorstatus = SK_CheckCmdRsp_1(SD_CMD_SET_BLOCKLEN);
        SK_SD_RETURN_CHECK(errorstatus);

        /*!< Configure the Data and make the state machine go into the WAIT_R State */
        SDIO_DataInitStructure.SDIO_DataTimeOut = SD_DATATIMEOUT;
        SDIO_DataInitStructure.SDIO_DataLength = NumberOfBlocks * BlockSize;
        SDIO_DataInitStructure.SDIO_DataBlockSize = SDIO_DataBlockSize_512b;
        SDIO_DataInitStructure.SDIO_TransferDir = SDIO_TransferDir_ToSDIO;
        SDIO_DataInitStructure.SDIO_TransferMode = SDIO_TransferMode_Block;
        SDIO_DataInitStructure.SDIO_DPSM = SDIO_DPSM_Enable;
        SDIO_DataConfig(&SDIO_DataInitStructure);

        /*!< Send CMD18 READ_MULT_BLOCK with argument data address */
        SK_SdioSendCmd_18(ReadAddr);
        errorstatus = SK_CheckCmdRsp_1(SD_CMD_SET_BLOCKLEN);
        SK_SD_RETURN_CHECK(errorstatus);

        SDIO_ITConfig(SDIO_IT_DATAEND, ENABLE);
        SDIO_DMACmd(ENABLE);
        SD_DMA_RxConfig((uint32_t *)readbuff, (NumberOfBlocks * BlockSize));

    } while (0);

    return errorstatus;
}

SD_Error SK_SD_WriteBlock(uint8_t *writebuff, uint32_t WriteAddr, uint16_t BlockSize)
{
    SD_Error errorstatus = SD_OK;
    SDIO_DataInitTypeDef SDIO_DataInitStructure;

    TransferError = SD_OK;
    TransferEnd = 0;
    StopCondition = 0;

    SDIO->DCTRL = 0x0;

    if (g_stSDCARD.CardType == SDIO_HIGH_CAPACITY_SD_CARD)
    {
        BlockSize  = SD_BLOCK_SIZE_512;
        WriteAddr /= SD_BLOCK_SIZE_512;
    }

    do {
        /*!< Send CMD16 SD_CMD_SET_BLOCKLEN with argument as block size  */
        SK_SdioSendCmd_16(BlockSize);
        errorstatus = SK_CheckCmdRsp_1(SD_CMD_SET_BLOCKLEN);
        SK_SD_RETURN_CHECK(errorstatus);

        /*!< Send CMD24 WRITE_SINGLE_BLOCK */
        SK_SdioSendCmd_24(WriteAddr);
        errorstatus = SK_CheckCmdRsp_1(SD_CMD_WRITE_SINGLE_BLOCK);
        SK_SD_RETURN_CHECK(errorstatus);

        /*!< Configure the Data and make the state machine go into the WAIT_S State */
        SDIO_DataInitStructure.SDIO_DataTimeOut = SD_DATATIMEOUT;
        SDIO_DataInitStructure.SDIO_DataLength = BlockSize;
        SDIO_DataInitStructure.SDIO_DataBlockSize = SDIO_DataBlockSize_512b;
        SDIO_DataInitStructure.SDIO_TransferDir = SDIO_TransferDir_ToCard;
        SDIO_DataInitStructure.SDIO_TransferMode = SDIO_TransferMode_Block;
        SDIO_DataInitStructure.SDIO_DPSM = SDIO_DPSM_Enable;
        SDIO_DataConfig(&SDIO_DataInitStructure);

        // DMA2 Mode
        SDIO_ITConfig(SDIO_IT_DATAEND, ENABLE);
        SD_DMA_TxConfig((uint32_t *)writebuff, BlockSize);
        SDIO_DMACmd(ENABLE);

    } while (0);

    return errorstatus;
}

SD_Error SK_SD_WriteMultiBlocks(uint8_t *writebuff, uint32_t WriteAddr, uint16_t BlockSize, uint32_t NumberOfBlocks)
{
    SD_Error errorstatus = SD_OK;
    SDIO_DataInitTypeDef SDIO_DataInitStructure;
    __IO uint32_t count = 0;
    
    TransferError = SD_OK;
    TransferEnd = 0;
    StopCondition = 1;
    
    SDIO->DCTRL = 0x0;
    
    if (g_stSDCARD.CardType == SDIO_HIGH_CAPACITY_SD_CARD)
    {
      BlockSize  = SD_BLOCK_SIZE_512;
      WriteAddr /= SD_BLOCK_SIZE_512;
    }

    do {
        /*!< Send CMD16 SD_CMD_SET_BLOCKLEN with argument as block size  */
        SK_SdioSendCmd_16(BlockSize);
        errorstatus = SK_CheckCmdRsp_1(SD_CMD_SET_BLOCKLEN);
        SK_SD_RETURN_CHECK(errorstatus);

        /*!< Pre-Erased : To Improve Performance. First CMD55 */
        SK_SdioSendCmd_55(g_stSDCARD.RCA);
        errorstatus = SK_CheckCmdRsp_1(SD_CMD_APP_CMD);
        SK_SD_RETURN_CHECK(errorstatus);

        /*!< Pre-Erased : To Improve Performance. Second ACMD23 */
        SK_SdioSendAppCmd_23(NumberOfBlocks);
        errorstatus = SK_CheckCmdRsp_1(SD_CMD_SET_BLOCK_COUNT);
        SK_SD_RETURN_CHECK(errorstatus);

        /*!< Send CMD25 WRITE_MULT_BLOCK with argument data address */
        SK_SdioSendCmd_25(WriteAddr);
        errorstatus = SK_CheckCmdRsp_1(SD_CMD_WRITE_MULT_BLOCK);
        SK_SD_RETURN_CHECK(errorstatus);

        /*!< Configure the Data and make the state machine go into the WAIT_S State */
        SDIO_DataInitStructure.SDIO_DataTimeOut = SD_DATATIMEOUT;
        SDIO_DataInitStructure.SDIO_DataLength = NumberOfBlocks * BlockSize;
        SDIO_DataInitStructure.SDIO_DataBlockSize = SDIO_DataBlockSize_512b;
        SDIO_DataInitStructure.SDIO_TransferDir = SDIO_TransferDir_ToCard;
        SDIO_DataInitStructure.SDIO_TransferMode = SDIO_TransferMode_Block;
        SDIO_DataInitStructure.SDIO_DPSM = SDIO_DPSM_Enable;
        SDIO_DataConfig(&SDIO_DataInitStructure);

        SDIO_ITConfig(SDIO_IT_DATAEND, ENABLE);
        SDIO_DMACmd(ENABLE);
        SD_DMA_TxConfig((uint32_t *)writebuff, (NumberOfBlocks * BlockSize));

    } while (0);

    return errorstatus;
}

uint32_t SD_DMAEndOfTransferStatus(void)
{
    //Channel4 transfer complete flag. 
    return (uint32_t)DMA_GetFlagStatus(DMA2_FLAG_TC4);
}

SD_Error SD_WaitReadOperation(void)
{
    SD_Error errorstatus = SD_OK;

    // Wait for Transfer ok
    while ((SD_DMAEndOfTransferStatus() == RESET) && (TransferEnd == 0) && (TransferError == SD_OK))
    {}

    if (TransferError != SD_OK)
    {
        return(TransferError);
    }

    return(errorstatus);
}

SD_Error SD_WaitWriteOperation(void)
{
    SD_Error errorstatus = SD_OK;

    // Wait for Transfer ok
    while ((SD_DMAEndOfTransferStatus() == RESET) && (TransferEnd == 0) && (TransferError == SD_OK))
    {}

    if (TransferError != SD_OK)
    {
    return(TransferError);
    }

    /*!< Clear all the static flags */
    SDIO_ClearFlag(SDIO_STATIC_FLAGS);

    return(errorstatus);
}

SD_Error SD_ProcessIRQSrc(void)
{
    if (StopCondition == 1)
    {
        /*!< Send CMD12 STOP_TRANSMISSION to stop the data transfer */
        SK_SdioSendCmd_12();
        TransferError = SK_CheckCmdRsp_1(SD_CMD_STOP_TRANSMISSION);
    }
    else
    {
        TransferError = SD_OK;
    }

    SDIO_ClearITPendingBit(SDIO_IT_DATAEND);
    SDIO_ITConfig(SDIO_IT_DATAEND, DISABLE);

    TransferEnd = 1;

    return(TransferError);
}

void SDIO_IRQHandler(void) 
{
    /* Process All SDIO Interrupt Sources */
    SD_ProcessIRQSrc();
}

void Fill_Buffer(uint8_t *pBuffer, uint32_t BufferLength, uint32_t Offset)
{
    uint16_t index = 0;

    /* Put in global buffer same values */
    for (index = 0; index < BufferLength; index++ )
    {
        pBuffer[index] = index + Offset;
    }
}

TestStatus Buffercmp(uint8_t* pBuffer1, uint8_t* pBuffer2, uint32_t BufferLength)
{
    while (BufferLength--)
    {
        if (*pBuffer1 != *pBuffer2)
        {
            return FAILED;
        }
        pBuffer1++;
        pBuffer2++;
    }

    return PASSED;
}

uint8_t Buffer_MultiBlock_Tx[MULTI_BUFFER_SIZE] = {0};
uint8_t Buffer_MultiBlock_Rx[MULTI_BUFFER_SIZE] = {0};
uint8_t Buffer_Block_Tx[MULTI_BUFFER_SIZE] = {0};
uint8_t Buffer_Block_Rx[MULTI_BUFFER_SIZE] = {0};

volatile TestStatus EraseStatus = FAILED, TransferStatus1 = FAILED, TransferStatus2 = FAILED;

/* Test Interface */
SD_Error SD_EraseTest(void)
{
    /* Erase NumberOfBlocks Blocks of WRITE_BL_LEN(512 Bytes) */
    SD_Error Status = SD_OK;
    // Erase start at 0x00, length
    Status = SK_SD_Erase(0x00, MULTI_BUFFER_SIZE);

    if (SD_OK == Status)
    {
        Status = SK_SD_ReadMultiBlocks(Buffer_MultiBlock_Rx, 0x00, SD_BLOCK_SIZE_512, NUMBER_OF_BLOCKS);
        Status = SD_WaitReadOperation();
        while(SK_GetCurrentCardState() != SD_TRANSFER_OK);
    }

    return Status;
}

SD_Error SD_SingleBlockTest(void)
{
    SD_Error Status = SD_OK;

    Fill_Buffer(Buffer_Block_Tx, SD_BLOCK_SIZE_512, 0x320F);

    Status = SK_SD_WriteBlock(Buffer_Block_Tx, 0x00, SD_BLOCK_SIZE_512);
    Status = SD_WaitWriteOperation();
    while(SK_GetCurrentCardState() != SD_TRANSFER_OK);

    if (Status == SD_OK)
    {
        Status = SK_SD_ReadBlock(Buffer_Block_Rx, 0x00, SD_BLOCK_SIZE_512);
        Status = SD_WaitReadOperation();
        while(SK_GetCurrentCardState() != SD_TRANSFER_OK);
    }

    if (Status == SD_OK)
    {
        TransferStatus1 = Buffercmp(Buffer_Block_Tx, Buffer_Block_Rx, SD_BLOCK_SIZE_512);
    }

    return Status;
}

SD_Error SD_MultiBlockTest(void)
{
    SD_Error Status = SD_OK;

    Fill_Buffer(Buffer_MultiBlock_Tx, MULTI_BUFFER_SIZE, 0x0);

    Status = SK_SD_WriteMultiBlocks(Buffer_MultiBlock_Tx, 0x00, SD_BLOCK_SIZE_512, NUMBER_OF_BLOCKS);
    Status = SD_WaitWriteOperation();
    while(SK_GetCurrentCardState() != SD_TRANSFER_OK);

    if (Status == SD_OK)
    {
        Status = SK_SD_ReadMultiBlocks(Buffer_MultiBlock_Rx, 0x00, SD_BLOCK_SIZE_512, NUMBER_OF_BLOCKS);
        /* Check if the Transfer is finished */
        Status = SD_WaitReadOperation();
        while(SK_GetCurrentCardState() != SD_TRANSFER_OK);
    }

    if (Status == SD_OK)
    {
        TransferStatus2 = Buffercmp(Buffer_MultiBlock_Tx, Buffer_MultiBlock_Rx, MULTI_BUFFER_SIZE);
    }

    return Status;
}

