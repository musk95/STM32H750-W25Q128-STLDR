#ifndef INC_W25Q128F_H_
#define INC_W25Q128F_H_

typedef enum {
	W25Q128F_INIT,
	W25Q128F_OK,
	W25Q128F_BUSY
} W25Q128F_State_t;

/* Reset Operations */
#define ENABLE_RESET                         0x66 //
#define RESET_DEVICE                         0x99 //

/* Read Operations */
#define READ_DATA_CMD                        0x03 //
#define FAST_READ_DATA_CMD                   0x0b //
#define FAST_READ_DUAL_OUTPUT                0x3b //
#define FAST_READ_QUAD_OUTPUT                0x6b //

/* Write Operations */
#define WRITE_ENABLE_CMD                     0x06 //

/* Register Operations */
#define READ_STATUS_REG_CMD                  0x05 //
#define READ_STATUS_REG2_CMD                 0x35 //
#define READ_STATUS_REG3_CMD                 0x15 //
#define WRITE_STATUS_REG2_CMD                0x31 //
#define WRITE_STATUS_REG3_CMD                0x11 //
#define VOLATILE_SR_WRITE_ENABLE             0x50 //

/* Program Operations */
#define PAGE_PROG_CMD                        0x02 //

/* Erase Operations */
#define BLOCK_ERASE_CMD                      0xD8 // 64Kbyte block erase

#define DUMMY_CLOCK_CYCLES_READ_QUAD         8 //

HAL_StatusTypeDef W25Q128F_AutoPollingMemReady(void);
HAL_StatusTypeDef W25Q128F_Init(void);
HAL_StatusTypeDef W25Q128F_EraseBlock(uint32_t flash_address);
HAL_StatusTypeDef W25Q128F_Write(uint32_t flash_address, uint8_t *s, int32_t l);
HAL_StatusTypeDef W25Q128F_ReadQuad(uint32_t flash_address, uint8_t *d, int32_t l);
HAL_StatusTypeDef W25Q128F_Read1Line(uint32_t flash_address, uint8_t *d, int32_t l);
HAL_StatusTypeDef W25Q128F_ReadFast1Line(uint32_t flash_address, uint8_t *d, int32_t l);
HAL_StatusTypeDef W25Q128F_ReadDual(uint32_t flash_address, uint8_t *d, int32_t l);
HAL_StatusTypeDef W25Q128F_MemoryMapped(void);

#endif /* INC_W25Q128F_H_ */
