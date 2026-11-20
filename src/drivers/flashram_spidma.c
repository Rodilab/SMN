/*
 * flashram_spidma.c - External FLASH RAM SPI driver using DMA
 *
 * Author: Dan Green (danngreen1@gmail.com)
 * (c) 2019
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 * See http://creativecommons.org/licenses/MIT/ for more information.
 *
 * -----------------------------------------------------------------------------
 */

// Include proper chip defs here:
#include "drivers/flash_S25FL127.h"

#include "drivers/flashram_spidma.h"
#include "globals.h"
#include "hal_handlers.h"
#include "gpio_pins.h"


FlashRamChip s_flash_chip;
FlashRamChip *flash_chip = &s_flash_chip; 

static DMA_HandleTypeDef flashram_spidma_tx;
static DMA_HandleTypeDef flashram_spidma_rx;
SPI_HandleTypeDef	flashram_spi;

enum sFlashErrors sflash_error=0;
volatile enum sFlashStates sflash_state=0;


// Private functions:
void create_flash_chip(void);
void sFLASH_write_enable(void);
void sFLASH_WaitForWriteEnd(void);
void sFLASH_SPI_GPIO_init(uint32_t nss_mode);
void sFLASH_SPIDMA_init(void);
void sFLASH_write_page_DMA(uint8_t* txBuffer, uint32_t write_addr, uint16_t num_bytes);
void sFLASH_write_page(uint8_t* txBuffer, uint32_t write_addr, uint16_t num_bytes);

static inline void select_chip(void);
static inline void select_chip(void)	{  PIN_LOW(flash_chip->CS.gpio, flash_chip->CS.pin);	} //CS Low
static inline void deselect_chip(void);
static inline void deselect_chip(void)	{  PIN_HIGH(flash_chip->CS.gpio, flash_chip->CS.pin);	} //CS High

//Public:
enum sFlashStates get_flash_state(void) { return sflash_state; }
#define delay_32ns(x) do {  register unsigned int i;  for (i = 0; i < x; ++i)   __asm__ __volatile__ ("nop\n\t":::"memory"); } while (0)

static uint8_t g_cmd[4];

//
// READING
//

// reads, and then waits until read is done
void sFLASH_read_buffer(uint8_t* rxBuffer, uint32_t read_addr, uint16_t num_bytes)
{
	sFLASH_read_buffer_DMA(rxBuffer, read_addr, num_bytes);
	while (get_flash_state() != sFLASH_NOTBUSY) {;}
}

void sFLASH_read_buffer_DMA(uint8_t* rxBuffer, uint32_t read_addr, uint16_t num_bytes)
{
	g_cmd[0] = sFLASH_CMD_READ;
	g_cmd[1] = read_addr >> 16;
	g_cmd[2] = read_addr >> 8;
	g_cmd[3] = read_addr & 0xFF;

	sflash_state = sFLASH_READCMD;
	select_chip();
	if (HAL_SPI_Transmit_DMA(&flashram_spi, g_cmd, 4) != HAL_OK)
		sflash_error |= sFLASH_SPI_DMA_TX_ERROR;

	while (sflash_state != sFLASH_NOTBUSY)  { ; }

	sflash_state = sFLASH_READING;
	if (HAL_SPI_Receive_DMA(&flashram_spi, rxBuffer, num_bytes) != HAL_OK)
		sflash_error |= sFLASH_SPI_DMA_RX_ERROR;
}

//
// WRITING
//

// writes, and then waits until write DMA is done
// void sFLASH_write_buffer(uint8_t* txBuffer, uint32_t write_addr, uint16_t num_bytes)
// {
// 	sFLASH_write_buffer_DMA(txBuffer, write_addr, num_bytes);
// 	while (get_flash_state() != sFLASH_NOTBUSY) {;}
// 	while (!sFLASH_is_chip_ready()) {;}
// }

void sFLASH_write_page(uint8_t* txBuffer, uint32_t write_addr, uint16_t num_bytes)
{
	if (!num_bytes) return;
	sFLASH_write_page_DMA(txBuffer, write_addr, num_bytes);
	while (get_flash_state() != sFLASH_NOTBUSY) {;}
	while (!sFLASH_is_chip_ready()) {;}
}

void sFLASH_write_page_DMA(uint8_t* txBuffer, uint32_t write_addr, uint16_t num_bytes)
{
	if (num_bytes > sFLASH_SPI_PAGESIZE) {
		num_bytes = sFLASH_SPI_PAGESIZE;
		sflash_error |= sFLASH_SPI_PAGE_OF_WARN;
	}

	sFLASH_write_enable();

	g_cmd[0] = sFLASH_CMD_WRITE;
	g_cmd[1] = write_addr >> 16;
	g_cmd[2] = write_addr >> 8;
	g_cmd[3] = write_addr & 0xFF;

	sflash_state = sFLASH_WRITECMD;
	select_chip();
	if (HAL_SPI_Transmit_DMA(&flashram_spi, g_cmd, 4) != HAL_OK)
		sflash_error |= sFLASH_SPI_DMA_TX_ERROR;

	while (sflash_state != sFLASH_NOTBUSY)  { ; }

	sflash_state = sFLASH_WRITING;
	if (HAL_SPI_Transmit_DMA(&flashram_spi, txBuffer, num_bytes) != HAL_OK)
		sflash_error |= sFLASH_SPI_DMA_TX_ERROR;
}

void sFLASH_write_buffer(uint8_t* txBuffer, uint32_t write_addr, uint16_t num_bytes)
{
	uint8_t page_num = 0, single_num = 0, page_offset = 0, count = 0, temp = 0;

	page_offset = write_addr % sFLASH_SPI_PAGESIZE;
	count = sFLASH_SPI_PAGESIZE - page_offset;
	page_num =  num_bytes / sFLASH_SPI_PAGESIZE;
	single_num = num_bytes % sFLASH_SPI_PAGESIZE;

	if (page_offset == 0) // write_addr is sFLASH_SPI_PAGESIZE aligned
	{
		if (page_num == 0) // num_bytes < sFLASH_SPI_PAGESIZE
		{
			sFLASH_write_page(txBuffer, write_addr, num_bytes);
		}
		else // num_bytes > sFLASH_SPI_PAGESIZE 
		{
			while (page_num--)
			{
				sFLASH_write_page(txBuffer, write_addr, sFLASH_SPI_PAGESIZE);
				write_addr +=  sFLASH_SPI_PAGESIZE;
				txBuffer += sFLASH_SPI_PAGESIZE;
			}

			sFLASH_write_page(txBuffer, write_addr, single_num);
		}
	}
	else // write_addr is not sFLASH_SPI_PAGESIZE aligned 
	{
		if (page_num == 0) // num_bytes < sFLASH_SPI_PAGESIZE
		{
			if (single_num > count) // (num_bytes + write_addr) > sFLASH_SPI_PAGESIZE
			{
				temp = single_num - count;

				sFLASH_write_page(txBuffer, write_addr, count);
				write_addr +=  count;
				txBuffer += count;

				sFLASH_write_page(txBuffer, write_addr, temp);
			}
			else
			{
				sFLASH_write_page(txBuffer, write_addr, num_bytes);
			}
		}
		else // num_bytes > sFLASH_SPI_PAGESIZE
		{
			num_bytes -= count;
			page_num =  num_bytes / sFLASH_SPI_PAGESIZE;
			single_num = num_bytes % sFLASH_SPI_PAGESIZE;

			sFLASH_write_page(txBuffer, write_addr, count);
			write_addr +=  count;
			txBuffer += count;

			while (page_num--)
			{
				sFLASH_write_page(txBuffer, write_addr, sFLASH_SPI_PAGESIZE);
				write_addr +=  sFLASH_SPI_PAGESIZE;
				txBuffer += sFLASH_SPI_PAGESIZE;
			}

			if (single_num != 0)
			{
				sFLASH_write_page(txBuffer, write_addr, single_num);
			}
		}
	}
}

uint8_t write_enable_cmd = sFLASH_CMD_WREN;
void sFLASH_write_enable(void)
{
	sflash_state = sFLASH_WRITECMD;
	select_chip();
	if (HAL_SPI_Transmit_DMA(&flashram_spi, &write_enable_cmd, 1) != HAL_OK)
		sflash_error |= sFLASH_SPI_DMA_TX_ERROR;

	while (sflash_state != sFLASH_NOTBUSY)  { ; }
	deselect_chip();
}

//
// ERASING
//

void sFLASH_erase_sector(uint32_t SectorAddr)
{
	sFLASH_erase_sector_background(SectorAddr);
	while (!sFLASH_is_chip_ready()) {;}
}

// After issuing the erase command it does not wait until the flash chip is ready
// You MUST not do any more FLASH commands until sFLASH_is_chip_ready() returns true
// (typically 150ms per sector, but datasheet says max is 12 seconds!)
void sFLASH_erase_sector_background(uint32_t SectorAddr)
{
	uint32_t aligned_addr = sFLASH_align2sector(SectorAddr);

	sFLASH_write_enable();

	if (aligned_addr < sFLASH_SPI_FIRST_64K_ADDR)
		g_cmd[0] = sFLASH_CMD_PE;
	else
		g_cmd[0] = sFLASH_CMD_SE;

	g_cmd[1] = aligned_addr >> 16;
	g_cmd[2] = aligned_addr >> 8;
	g_cmd[3] = aligned_addr & 0xFF;

	sflash_state = sFLASH_ERASING;
	select_chip();
	if (HAL_SPI_Transmit_DMA(&flashram_spi, g_cmd, 4) != HAL_OK)
		sflash_error |= sFLASH_SPI_DMA_TX_ERROR;

	while (sflash_state != sFLASH_NOTBUSY)  { ; }
	deselect_chip();
}

void sFLASH_erase_chip(void)
{
	sFLASH_write_enable();

	g_cmd[0] = sFLASH_CMD_BE;

	sflash_state = sFLASH_ERASING;
	select_chip();
	if (HAL_SPI_Transmit_DMA(&flashram_spi, g_cmd, 1) != HAL_OK)
		sflash_error |= sFLASH_SPI_DMA_TX_ERROR;

	while (sflash_state != sFLASH_NOTBUSY)  { ; }
	deselect_chip();
}

//
// Queries chip for Write In Progress (WIP) status
//
uint8_t read_sr_cmd[2] = {sFLASH_CMD_RDSR, sFLASH_DUMMY_BYTE};
uint8_t sr_data[2];

uint8_t sFLASH_is_chip_ready(void)
{
	sflash_state = sFLASH_READCMD;

	select_chip();
	if (HAL_SPI_TransmitReceive_DMA(&flashram_spi, read_sr_cmd, sr_data, 2) != HAL_OK)
		sflash_error |= sFLASH_SPI_DMA_TX_ERROR;

	while (sflash_state != sFLASH_NOTBUSY)  { ; }
	deselect_chip();

	if (sr_data[1] & sFLASH_WIP_FLAG)
		return 0; //WIP is set ==> not ready
	else
		return 1; //WIP is cleared ==> ready
}

//
// Polls the status of the Write In Progress (WIP) flag in the FLASH's
// status register and loop until write operation has completed.
//
void sFLASH_wait_for_chip_ready(void)
{	
	sflash_state = sFLASH_READCMD;

	select_chip();	
	if (HAL_SPI_Transmit_DMA(&flashram_spi, read_sr_cmd, 1) != HAL_OK)
		sflash_error |= sFLASH_SPI_DMA_TX_ERROR;
	while (sflash_state != sFLASH_NOTBUSY)  { ; }

	// Loop as long as the memory is busy with a write cycle
	do
	{
		// Send a dummy byte to generate the clock needed by the FLASH 
		if (HAL_SPI_Receive_DMA(&flashram_spi, sr_data, 1) != HAL_OK)
			sflash_error |= sFLASH_SPI_DMA_TX_ERROR;

		if (sr_data[0] & sFLASH_E_ERR_FLAG) 
			sflash_error |= sFLASH_ERASE_ERROR;

		if (sr_data[0] & sFLASH_P_ERR_FLAG) 
			sflash_error |= sFLASH_PROG_ERROR;
	}
	while ((sr_data[0] & sFLASH_WIP_FLAG) == SET); // Write in progress 

	deselect_chip();
}


//
// INIT
//

void sFLASH_init(void)
{
	create_flash_chip();
	sFLASH_SPIDMA_init();
}

void create_flash_chip(void)
{
	flash_chip = &s_flash_chip;
	flash_chip->SPIx 				= SPI3;

	flash_chip->SCK.pin				= GPIO_PIN_10;
	flash_chip->SCK.gpio 			= GPIOC;
	flash_chip->SCK.af				= GPIO_AF6_SPI3;

	flash_chip->MISO.pin			= GPIO_PIN_11;
	flash_chip->MISO.gpio			= GPIOC;
	flash_chip->MISO.af				= GPIO_AF6_SPI3;

	flash_chip->MOSI.pin			= GPIO_PIN_12;
	flash_chip->MOSI.gpio 			= GPIOC;
	flash_chip->MOSI.af				= GPIO_AF6_SPI3;

	flash_chip->CS.pin				= GPIO_PIN_15;
	flash_chip->CS.gpio 			= GPIOA;
	flash_chip->CS.af				= GPIO_AF6_SPI3;

	flash_chip->IO2.pin				= GPIO_PIN_11;
	flash_chip->IO2.gpio 			= GPIOA;

	flash_chip->IO3.pin				= GPIO_PIN_12;
	flash_chip->IO3.gpio 			= GPIOA;
}

void sFLASH_SPIDMA_init(void)
{
	uint32_t nss_mode = SPI_NSS_SOFT;

	sFLASH_SPI_GPIO_init(nss_mode);

	flashram_spi.Instance               = flash_chip->SPIx;
	flashram_spi.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
	flashram_spi.Init.Direction         = SPI_DIRECTION_2LINES;
	flashram_spi.Init.CLKPhase          = SPI_PHASE_1EDGE;
	flashram_spi.Init.CLKPolarity       = SPI_POLARITY_LOW;
	flashram_spi.Init.DataSize          = SPI_DATASIZE_8BIT;
	flashram_spi.Init.FirstBit          = SPI_FIRSTBIT_MSB;
	flashram_spi.Init.TIMode            = SPI_TIMODE_DISABLE;
	flashram_spi.Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLE;
	flashram_spi.Init.CRCPolynomial     = 7;
	flashram_spi.Init.NSS               = nss_mode;
	flashram_spi.Init.Mode 				= SPI_MODE_MASTER;

	if (HAL_SPI_Init(&flashram_spi) != HAL_OK)
		sflash_error |= sFLASH_SPI_DMA_INIT_ERROR;

	DMAx_CLK_ENABLE();

    flashram_spidma_tx.Instance                 = SPIx_TX_DMA_STREAM;
    flashram_spidma_tx.Init.Channel             = SPIx_TX_DMA_CHANNEL;
    flashram_spidma_tx.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
    flashram_spidma_tx.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL;
    flashram_spidma_tx.Init.MemBurst            = DMA_MBURST_SINGLE;
    flashram_spidma_tx.Init.PeriphBurst         = DMA_MBURST_SINGLE;
    flashram_spidma_tx.Init.Direction           = DMA_MEMORY_TO_PERIPH;
    flashram_spidma_tx.Init.PeriphInc           = DMA_PINC_DISABLE;
    flashram_spidma_tx.Init.MemInc              = DMA_MINC_ENABLE;
    flashram_spidma_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    flashram_spidma_tx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
    flashram_spidma_tx.Init.Mode                = DMA_NORMAL;
    flashram_spidma_tx.Init.Priority            = DMA_PRIORITY_LOW;

    HAL_DMA_Init(&flashram_spidma_tx);
    __HAL_LINKDMA(&flashram_spi, hdmatx, flashram_spidma_tx);

    flashram_spidma_rx.Instance                 = SPIx_RX_DMA_STREAM;
    flashram_spidma_rx.Init.Channel             = SPIx_RX_DMA_CHANNEL;
    flashram_spidma_rx.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
    flashram_spidma_rx.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL;
    flashram_spidma_rx.Init.MemBurst            = DMA_MBURST_SINGLE;
    flashram_spidma_rx.Init.PeriphBurst         = DMA_MBURST_SINGLE;
    flashram_spidma_rx.Init.Direction           = DMA_PERIPH_TO_MEMORY;
    flashram_spidma_rx.Init.PeriphInc           = DMA_PINC_DISABLE;
    flashram_spidma_rx.Init.MemInc              = DMA_MINC_ENABLE;
    flashram_spidma_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    flashram_spidma_rx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
    flashram_spidma_rx.Init.Mode                = DMA_NORMAL;
    flashram_spidma_rx.Init.Priority            = DMA_PRIORITY_HIGH;

    HAL_DMA_Init(&flashram_spidma_rx);
    __HAL_LINKDMA(&flashram_spi, hdmarx, flashram_spidma_rx);

    HAL_NVIC_SetPriority(SPIx_DMA_TX_IRQn, 0, 2);
    HAL_NVIC_EnableIRQ(SPIx_DMA_TX_IRQn);

    HAL_NVIC_SetPriority(SPIx_DMA_RX_IRQn, 0, 2);
    HAL_NVIC_EnableIRQ(SPIx_DMA_RX_IRQn);
}

void sFLASH_SPI_GPIO_init(uint32_t nss_mode)
{
	GPIO_InitTypeDef gpio;

	#ifdef SPI1
		if (flash_chip->SPIx == SPI1)	 
			__HAL_RCC_SPI1_CLK_ENABLE();
	#endif
	#ifdef SPI2
		if (flash_chip->SPIx == SPI2)	 
			__HAL_RCC_SPI2_CLK_ENABLE();
	#endif
	#ifdef SPI3
		if (flash_chip->SPIx == SPI3)	 
			__HAL_RCC_SPI3_CLK_ENABLE();
	#endif
	#ifdef SPI4
		if (flash_chip->SPIx == SPI4)	 
			__HAL_RCC_SPI4_CLK_ENABLE();
	#endif
	#ifdef SPI5
		if (flash_chip->SPIx == SPI5)	
		 __HAL_RCC_SPI5_CLK_ENABLE();
	#endif
	#ifdef SPI6
		if (flash_chip->SPIx == SPI6)	 
			__HAL_RCC_SPI6_CLK_ENABLE();
	#endif

	// Assume GPIO RCC is enabled already, or if not do it here:
	//		__HAL_RCC_GPIOx_CLK_ENABLE()

	gpio.Speed 	= GPIO_SPEED_FREQ_VERY_HIGH;
	gpio.Pull  	= GPIO_PULLDOWN;


	// SPI SCK pin configuration
	gpio.Mode 		= GPIO_MODE_AF_PP;
	gpio.Pin 		= flash_chip->SCK.pin;
	gpio.Alternate 	= flash_chip->SCK.af;
	HAL_GPIO_Init(flash_chip->SCK.gpio, &gpio);

	// SPI  MOSI pin configuration
	gpio.Mode 		= GPIO_MODE_AF_PP;
	gpio.Pin 		= flash_chip->MOSI.pin;
	gpio.Alternate 	= flash_chip->MOSI.af;
	HAL_GPIO_Init(flash_chip->MOSI.gpio, &gpio);

	// SPI  MISO pin configuration
	gpio.Mode 		= GPIO_MODE_AF_PP;
	gpio.Pin 		= flash_chip->MISO.pin;
	gpio.Alternate 	= flash_chip->MISO.af;
	HAL_GPIO_Init(flash_chip->MISO.gpio, &gpio);  

	// SPI  CS pin configuration
	if (nss_mode == SPI_NSS_SOFT)
	{
		gpio.Mode 		= GPIO_MODE_OUTPUT_PP;
		gpio.Speed 		= GPIO_SPEED_FREQ_VERY_HIGH;
		gpio.Pull  		= GPIO_PULLUP;
		gpio.Pin 		= flash_chip->CS.pin;
		HAL_GPIO_Init(flash_chip->CS.gpio, &gpio);

		//Deselect the chip: CS high
		PIN_HIGH(flash_chip->CS.gpio, flash_chip->CS.pin);
	} 
	else if (nss_mode == SPI_NSS_HARD_OUTPUT)
	{
		gpio.Mode 		= GPIO_MODE_AF_PP;
		gpio.Pull  		= GPIO_PULLUP;
		gpio.Pin 		= flash_chip->CS.pin;
		gpio.Alternate 	= flash_chip->CS.af;
		HAL_GPIO_Init(flash_chip->CS.gpio, &gpio);  
	}

	gpio.Mode 		= GPIO_MODE_OUTPUT_PP;
	gpio.Speed 		= GPIO_SPEED_FREQ_HIGH;
	gpio.Pull  		= GPIO_PULLUP;

	gpio.Pin 		= flash_chip->IO2.pin;
	HAL_GPIO_Init(flash_chip->IO2.gpio, &gpio);
	PIN_HIGH(flash_chip->IO2.gpio, flash_chip->IO2.pin);

	gpio.Pin 		= flash_chip->IO3.pin;
	HAL_GPIO_Init(flash_chip->IO3.gpio, &gpio);
	PIN_HIGH(flash_chip->IO3.gpio, flash_chip->IO3.pin);
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
	if (sflash_state == sFLASH_READING || sflash_state == sFLASH_WRITING)
		deselect_chip();
	sflash_state = sFLASH_NOTBUSY;
}
void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
	if (sflash_state == sFLASH_WRITING)
		deselect_chip();
	sflash_state = sFLASH_NOTBUSY;
}
void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
	if (sflash_state == sFLASH_READING)
		deselect_chip();
	sflash_state = sFLASH_NOTBUSY;
}
void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
	deselect_chip();
	sflash_state = sFLASH_ERROR;
}
void SPIx_DMA_RX_IRQHandler(void)
{
	HAL_DMA_IRQHandler(flashram_spi.hdmarx);
}
void SPIx_DMA_TX_IRQHandler(void)
{
	HAL_DMA_IRQHandler(flashram_spi.hdmatx);
}
void SPIx_IRQHandler(void)
{
	HAL_SPI_IRQHandler(&flashram_spi);
}


//
// TESTS
//
uint8_t test_bytes[sFLASH_SPI_PAGESIZE];
uint32_t sFLASH_test_sector(uint32_t test_start)
{
	uint32_t i;
	uint32_t bad_bytes=0;
	uint16_t num_bytes = sFLASH_SPI_PAGESIZE;

	test_start = sFLASH_align2sector(test_start);

	for (i=0; i<num_bytes; i++)
		test_bytes[i] = (5+i) & 0xFF;

	sFLASH_erase_sector(test_start);

	sFLASH_write_buffer(test_bytes, test_start, num_bytes);

	for (i=0; i<num_bytes; i++)
		test_bytes[i] = 0;

//	while(!sFLASH_is_chip_ready() );

	//sFLASH_wait_for_chip_ready();

	//Read back the page using low-level commands
	sFLASH_read_buffer(test_bytes, test_start, num_bytes);

	for (i=0; i<num_bytes; i++)
	{
		if (test_bytes[i] != ((5+i) & 0xFF) )
			bad_bytes++;
	}

	return bad_bytes;
}

// SMN resume worker. Legacy synchronous entry points above are unchanged.
// Main-loop ownership is held until the chip is ready, including WIP time.
#include "drivers/smn_flash_async.h"
#include "external_flash_layout.h"

enum ResumeSpiStage {
    RS_IDLE, RS_WREN, RS_WREN_DONE, RS_HEADER, RS_HEADER_DONE, RS_DATA, RS_DATA_DONE,
    RS_STATUS, RS_STATUS_DONE, RS_FAILED
};
static enum ResumeSpiStage resume_spi_stage;
static uint8_t resume_command[4], resume_status_command[2] = {sFLASH_CMD_RDSR, 0};
static uint8_t resume_status[2];
static uint8_t *resume_data;
static uint16_t resume_size;
static uint32_t resume_started;
static uint8_t resume_clock_started;

static void resume_spi_priorities(uint8_t normal)
{
    // Existing flash IRQs share preemption priority 0 with audio/LFO. During
    // automatic storage, put them BELOW every priority task, without masking it.
    HAL_NVIC_SetPriority(SPIx_DMA_TX_IRQn, normal ? 0 : 3, normal ? 2 : 3);
    HAL_NVIC_SetPriority(SPIx_DMA_RX_IRQn, normal ? 0 : 3, normal ? 2 : 3);
}

uint8_t smn_flash_idle(void)
{
    return resume_spi_stage == RS_IDLE && sflash_state == sFLASH_NOTBUSY;
}

static uint8_t resume_spi_begin(uint8_t command, uint32_t address,
                                uint8_t *data, uint16_t size)
{
    // Hard address guard: even a journal bug cannot reach SWN data.
    const uint32_t start = SMN_RESUME_SECTOR_A * sFLASH_SPI_4K_SECTOR_SIZE;
    const uint32_t end = (SMN_RESUME_SECTOR_B + 1) * sFLASH_SPI_4K_SECTOR_SIZE;
    if (!smn_flash_idle() || address < start || address >= end ||
        size > end - address) return 0;
    resume_command[0] = command;
    resume_command[1] = address >> 16;
    resume_command[2] = address >> 8;
    resume_command[3] = address;
    resume_data = data;
    resume_size = size;
    resume_clock_started = 0;
    resume_spi_priorities(0);
    resume_spi_stage = command == sFLASH_CMD_READ ? RS_HEADER : RS_WREN;
    return 1;
}

uint8_t smn_flash_read(uint32_t address, uint8_t *data, uint16_t size)
{
    if (!data || !size) return 0;
    return resume_spi_begin(sFLASH_CMD_READ, address, data, size);
}

uint8_t smn_flash_program(uint32_t address, uint8_t *data, uint16_t size)
{
    if (!data || !size || size > sFLASH_SPI_PAGESIZE ||
        (address % sFLASH_SPI_PAGESIZE) + size > sFLASH_SPI_PAGESIZE) return 0;
    return resume_spi_begin(sFLASH_CMD_WRITE, address, data, size);
}

uint8_t smn_flash_erase(uint32_t address)
{
    if (address % sFLASH_SPI_4K_SECTOR_SIZE) return 0;
    return resume_spi_begin(sFLASH_CMD_PE, address, 0, 0);
}

static enum SmnFlashResult resume_spi_fail(void)
{
    // Fail closed: never let a legacy call enter its unbounded wait loops on a
    // failed/unknown bus. Prioritized audio/LFO continue; reboot permits retry.
    deselect_chip();
    resume_spi_stage = RS_FAILED;
    return SMN_FLASH_FAILED;
}

enum SmnFlashResult smn_flash_poll(uint32_t ticks)
{
    if (resume_spi_stage == RS_FAILED) return SMN_FLASH_FAILED;
    if (resume_spi_stage == RS_IDLE) return SMN_FLASH_DONE;
    if (!resume_clock_started) {
        resume_started = ticks;
        resume_clock_started = 1;
    }
    if (sflash_state == sFLASH_ERROR ||
        (uint32_t)(ticks - resume_started) > 15000u * TICKS_PER_MS)
        return resume_spi_fail();
    if (sflash_state != sFLASH_NOTBUSY) return SMN_FLASH_BUSY;

    HAL_StatusTypeDef result = HAL_OK;
    switch (resume_spi_stage) {
    case RS_WREN:
        g_cmd[0] = sFLASH_CMD_WREN;
        sflash_state = sFLASH_READCMD;
        select_chip();
        result = HAL_SPI_Transmit_DMA(&flashram_spi, g_cmd, 1);
        resume_spi_stage = RS_WREN_DONE;
        break;
    case RS_WREN_DONE:
        deselect_chip();
        // Separate service call guarantees the minimum CS-high time after
        // WREN; no busy delay and no immediate high/low GPIO pulse.
        resume_spi_stage = RS_HEADER;
        break;
    case RS_HEADER:
        select_chip();
        sflash_state = sFLASH_READCMD;
        result = HAL_SPI_Transmit_DMA(&flashram_spi, resume_command, 4);
        resume_spi_stage = RS_HEADER_DONE;
        break;
    case RS_HEADER_DONE:
        if (resume_command[0] == sFLASH_CMD_PE) {
            deselect_chip(); // Erase starts on this edge, without waiting.
            resume_spi_stage = RS_STATUS;
        } else resume_spi_stage = RS_DATA;
        break;
    case RS_DATA:
        if (resume_command[0] == sFLASH_CMD_READ) {
            sflash_state = sFLASH_READING;
            result = HAL_SPI_Receive_DMA(&flashram_spi, resume_data, resume_size);
        } else {
            sflash_state = sFLASH_WRITING;
            result = HAL_SPI_Transmit_DMA(&flashram_spi, resume_data, resume_size);
        }
        resume_spi_stage = RS_DATA_DONE;
        break;
    case RS_DATA_DONE:
        // Existing completion callbacks have released CS.
        if (resume_command[0] == sFLASH_CMD_READ) {
            resume_spi_stage = RS_IDLE;
            resume_spi_priorities(1);
            return SMN_FLASH_DONE;
        }
        resume_spi_stage = RS_STATUS;
        break;
    case RS_STATUS:
        sflash_state = sFLASH_READCMD;
        select_chip();
        result = HAL_SPI_TransmitReceive_DMA(&flashram_spi, resume_status_command,
                                             resume_status, 2);
        resume_spi_stage = RS_STATUS_DONE;
        break;
    case RS_STATUS_DONE:
        deselect_chip();
        if (resume_status[1] & (sFLASH_E_ERR_FLAG | sFLASH_P_ERR_FLAG))
            return resume_spi_fail();
        if (resume_status[1] & sFLASH_WIP_FLAG) resume_spi_stage = RS_STATUS;
        else {
            resume_spi_stage = RS_IDLE;
            resume_spi_priorities(1);
            return SMN_FLASH_DONE;
        }
        break;
    default: return resume_spi_fail();
    }
    if (result != HAL_OK) return resume_spi_fail();
    return SMN_FLASH_BUSY;
}
