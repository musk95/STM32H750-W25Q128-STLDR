/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : Loader_Src.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "w25q128f.h"


/* Private variables ---------------------------------------------------------*/
extern QSPI_HandleTypeDef hqspi;

extern void SystemClock_Config(void);
extern void ResetClock_Config(void);
extern void MX_QUADSPI_Init(void);
extern void MX_USART1_UART_Init(void);
extern void MX_GPIO_Init(void);

void *memset(void *buf, int ch, size_t n);

/**
  * @brief  The application entry point.
  * @retval int
  */
int Init(uint8_t MemMappedMode)
{
	SystemInit();

    hqspi.Instance = QUADSPI;
    HAL_QSPI_DeInit(&hqspi);
    HAL_DeInit();

    HAL_Init();

    /* Configure the system clock */
    SystemClock_Config();

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_QUADSPI_Init();
    W25Q128F_Init();

    return 1;
}

#define QSPI_PAGESIZE 0x100
#define QSPI_FLASHSIZE (16*1024*1024)
#define QSPI_BLOCKSIZE 0x10000

int WriteEnable (void)
{
    return 1;
}

int ResetMemory(void)
{
    return Init(0);
}

/*******************************************************************************
 Description :
 Write data to the device
 Inputs :
 				Address 	: Write location
 				Size 		: Length in bytes
 				buffer 		: Address where to get the data to write
 outputs :
 				"1" 	        : Operation succeeded
 Info :
 Note : Mandatory for all types except SRAM and PSRAM
********************************************************************************/
int Write (uint32_t Address, uint32_t Size, uint8_t* buffer)
{
    uint32_t NumOfPage = 0, NumOfSingle = 0, Addr = 0, count = 0, temp = 0;
    uint32_t   QSPI_DataNum = 0;

    if (Address >= 0x90000000) {
        Address -= 0x90000000;
    }

    Addr = Address % QSPI_PAGESIZE;
    count = QSPI_PAGESIZE - Addr;
    NumOfPage =  Size / QSPI_PAGESIZE;
    NumOfSingle = Size % QSPI_PAGESIZE;

    if (Addr == 0) /*!< Address is QSPI_PAGESIZE aligned  */
    {
        if (NumOfPage == 0) /*!< NumByteToWrite < QSPI_PAGESIZE */
        {
            QSPI_DataNum = Size;
            if (QSPI_DataNum != 0) {
                W25Q128F_Write(Address, buffer, QSPI_DataNum);
                W25Q128F_AutoPollingMemReady();
            }
        }
        else /*!< Size > QSPI_PAGESIZE */
        {
            while (NumOfPage--)
            {
                QSPI_DataNum = QSPI_PAGESIZE;
                W25Q128F_Write(Address, buffer, QSPI_DataNum);
                W25Q128F_AutoPollingMemReady();
                Address +=  QSPI_PAGESIZE;
                buffer += QSPI_PAGESIZE;

            }

            QSPI_DataNum = NumOfSingle;
            if (QSPI_DataNum != 0) {
                W25Q128F_Write(Address, buffer, QSPI_DataNum);
                W25Q128F_AutoPollingMemReady();
            }

        }
    }
    else /*!< Address is not QSPI_PAGESIZE aligned  */
    {
        if (NumOfPage == 0) /*!< Size < QSPI_PAGESIZE */
        {
            if (NumOfSingle > count) /*!< (Size + Address) > QSPI_PAGESIZE */
            {
                temp = NumOfSingle - count;
                QSPI_DataNum = count;
                if (QSPI_DataNum != 0) {
                    W25Q128F_Write(Address, buffer, QSPI_DataNum);
                    W25Q128F_AutoPollingMemReady();
                }
                Address +=  count;
                buffer += count;

                QSPI_DataNum = temp;
                if (QSPI_DataNum != 0) {
                    W25Q128F_Write(Address, buffer, QSPI_DataNum);
                    W25Q128F_AutoPollingMemReady();
                }
            }
            else
            {
                QSPI_DataNum = Size;
                if (QSPI_DataNum != 0) {
                    W25Q128F_Write(Address, buffer, QSPI_DataNum);
                    W25Q128F_AutoPollingMemReady();
                }
            }
        }
        else /*!< Size > QSPI_PAGESIZE */
        {
            Size -= count;
            NumOfPage =  Size / QSPI_PAGESIZE;
            NumOfSingle = Size % QSPI_PAGESIZE;

            QSPI_DataNum = count;

            if (QSPI_DataNum != 0) {
                W25Q128F_Write(Address, buffer, QSPI_DataNum);
                W25Q128F_AutoPollingMemReady();
            }
            Address +=  count;
            buffer += count;

            while (NumOfPage--)
            {
                QSPI_DataNum = QSPI_PAGESIZE;

                if (QSPI_DataNum != 0) {
                    W25Q128F_Write(Address, buffer, QSPI_DataNum);
                    W25Q128F_AutoPollingMemReady();
                }
                Address +=  QSPI_PAGESIZE;
                buffer += QSPI_PAGESIZE;
            }

            if (NumOfSingle != 0)
            {
                QSPI_DataNum = NumOfSingle;

                if (QSPI_DataNum != 0) {
                    W25Q128F_Write(Address, buffer, QSPI_DataNum);
                    W25Q128F_AutoPollingMemReady();
                }
            }
        }
    }

    return 1;
}



/*******************************************************************************
 Description :
 Erase a full sector in the device
 Inputs :
 				SectrorAddress	: Start of sector
 outputs :
 				"1" : Operation succeeded
 				"0" : Operation failure
 Note : Not Mandatory for SRAM PSRAM and NOR_FLASH
********************************************************************************/
int SectorErase (uint32_t EraseStartAddress ,uint32_t EraseEndAddress)
{
    uint32_t BlockAddr;

    if (EraseStartAddress >= 0x90000000) {
        EraseStartAddress -= 0x90000000;
    }
    if (EraseEndAddress >= 0x90000000) {
        EraseEndAddress -= 0x90000000;
    }

    EraseStartAddress = EraseStartAddress -  EraseStartAddress % 0x10000;

    while (EraseEndAddress>=EraseStartAddress)
    {
        BlockAddr = EraseStartAddress & 0x0FFFFFFF;
        W25Q128F_EraseBlock( BlockAddr);
        W25Q128F_AutoPollingMemReady();
        EraseStartAddress += 0x10000;
    }

    return 1;
}

/*******************************************************************************
 Description :
 Erase a full sector in the device
 Inputs :
 				SectrorAddress	: Start of sector
 outputs :
 				"1" : Operation succeeded
 				"0" : Operation failure
 Note : Not Mandatory for SRAM PSRAM and NOR_FLASH
********************************************************************************/
int MassErase (void)
{
    uint32_t BlockAddr = 0;

    while (BlockAddr < QSPI_FLASHSIZE)
    {
        W25Q128F_EraseBlock( BlockAddr);
        W25Q128F_AutoPollingMemReady();
        BlockAddr += QSPI_BLOCKSIZE;
    }

    return 1;
}

/*******************************************************************************
 Description :
 Read data from the device
 Inputs :
 				Address 	: Write location
 				Size 		: Length in bytes
 				buffer 		: Address where to get the data to write
 outputs :
 				"1" 		: Operation succeeded
 				"0" 		: Operation failure
 Note : Not Mandatory
********************************************************************************/
int Read (uint32_t Address, uint32_t Size, uint8_t* Buffer)
{
    int i = 0;

    W25Q128F_MemoryMapped();

    for (i=0; i < Size;i++)
    {
        *(uint8_t*)Buffer++ = *(uint8_t*)Address;
        Address ++;
    }

    return 1;
}

/*******************************************************************************
 Description :
 Verify the data
 Inputs :
 				MemoryAddr 	: Write location
 				RAMBufferAddr 	: RAM Address
 				Size 		: Length in bytes
 outputs :
 				"0" 		: Operation succeeded
 Note : Not Mandatory
********************************************************************************/
int Verify (uint32_t MemoryAddr, uint32_t RAMBufferAddr, uint32_t Size)
{
    uint32_t VerifiedData = 0;
    Size*=4;

    W25Q128F_MemoryMapped();

    while (Size>VerifiedData)
    {
        if ( *(uint8_t*)MemoryAddr++ != *((uint8_t*)RAMBufferAddr + VerifiedData))
            return (MemoryAddr + VerifiedData);

        VerifiedData++;
    }

    return 0;
}

void *memset(void *buf, int ch, size_t n)
{
    char *p = buf;
    while (n-- > 0) {
        *p++ = (char)ch;
    }
    return p;
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
