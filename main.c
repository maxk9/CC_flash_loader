/*

Copyright (c) 2012-2014 RedBearLab

Permission is hereby granted, free of charge, to any person
obtaining a copy of this software and associated documentation files (the "Software"), 
to deal in the Software without restriction, including without 
limitation the rights to use, copy, modify, merge, publish, distribute, 
sublicense, and/or sell copies of the Software, and to permit persons 
to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be 
included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES 
OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, 
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, 
TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH 
THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

*/


/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include "stm32F10x.h"
#include "STM32vldiscovery.h"




#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)

#define  PIN_DD_OUT  	GPIO_Pin_13
#define  PIN_DD_IN  	GPIO_Pin_12
#define  PIN_DC		GPIO_Pin_14
#define  PIN_RESET  GPIO_Pin_15

#define  PIN_DD_OUT_LOW  GPIO_ResetBits(GPIOB,PIN_DD_OUT)
#define  PIN_DD_OUT_HIGH  GPIO_SetBits(GPIOB,PIN_DD_OUT)
#define  PIN_DC_LOW  GPIO_ResetBits(GPIOB,PIN_DC)
#define  PIN_DC_HIGH  GPIO_SetBits(GPIOB,PIN_DC)
#define  PIN_RES_LOW  GPIO_ResetBits(GPIOB,PIN_RESET)
#define  PIN_RES_HIGH  GPIO_SetBits(GPIOB,PIN_RESET)

#define  PIN_DD_READ  GPIO_ReadInputDataBit(GPIOB,PIN_DD_IN)	//must be short circult on PIN_DD_OUT


/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

// Start addresses on DUP (Increased buffer size improves performance)
#define ADDR_BUF0                   0x0000 // Buffer (512 bytes)
#define ADDR_DMA_DESC_0             0x0200 // DMA descriptors (8 bytes)
#define ADDR_DMA_DESC_1             (ADDR_DMA_DESC_0 + 8)

// DMA channels used on DUP
#define CH_DBG_TO_BUF0              0x01   // Channel 0
#define CH_BUF0_TO_FLASH            0x02   // Channel 1

// Debug commands
#define CMD_CHIP_ERASE              0x10
#define CMD_WR_CONFIG               0x19
#define CMD_RD_CONFIG               0x24
#define CMD_READ_STATUS             0x30
#define CMD_RESUME                  0x4C
#define CMD_DEBUG_INSTR_1B          (0x54|1)
#define CMD_DEBUG_INSTR_2B          (0x54|2)
#define CMD_DEBUG_INSTR_3B          (0x54|3)
#define CMD_BURST_WRITE             0x80
#define CMD_GET_CHIP_ID             0x68

// Debug status bitmasks
#define STATUS_CHIP_ERASE_BUSY_BM   0x80 // New debug interface
#define STATUS_PCON_IDLE_BM         0x40
#define STATUS_CPU_HALTED_BM        0x20
#define STATUS_PM_ACTIVE_BM         0x10
#define STATUS_HALT_STATUS_BM       0x08
#define STATUS_DEBUG_LOCKED_BM      0x04
#define STATUS_OSC_STABLE_BM        0x02
#define STATUS_STACK_OVERFLOW_BM    0x01

// DUP registers (XDATA space address)
#define DUP_DBGDATA                 0x6260  // Debug interface data buffer
#define DUP_FCTL                    0x6270  // Flash controller
#define DUP_FADDRL                  0x6271  // Flash controller addr
#define DUP_FADDRH                  0x6272  // Flash controller addr
#define DUP_FWDATA                  0x6273  // Clash controller data buffer
#define DUP_CLKCONSTA               0x709E  // Sys clock status
#define DUP_CLKCONCMD               0x70C6  // Sys clock configuration
#define DUP_MEMCTR                  0x70C7  // Flash bank xdata mapping
#define DUP_DMA1CFGL                0x70D2  // Low byte, DMA config ch. 1
#define DUP_DMA1CFGH                0x70D3  // Hi byte , DMA config ch. 1
#define DUP_DMA0CFGL                0x70D4  // Low byte, DMA config ch. 0
#define DUP_DMA0CFGH                0x70D5  // Low byte, DMA config ch. 0
#define DUP_DMAARM                  0x70D6  // DMA arming register

// Commands to Bootloader
#define SBEGIN	0x01
#define SDATA	0x02
#define SRSP	0x03
#define SEND	0x04
#define ERRO	0x05
#define WAITING	0x00
#define RECEIVING	0x01







/* Private macro -------------------------------------------------------------*/


// Utility macros
//! Low nibble of 16bit variable
#define LOBYTE(w)           ((uint8_t)(w))
//! High nibble of 16bit variable
#define HIBYTE(w)           ((uint8_t)(((uint16_t)(w) >> 8) & 0xFF))

/* Private consts ------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
/******************************************************************************
 VARIABLES*/
//! DUP DMA descriptor
const unsigned char dma_desc_0[8] =
{
    // Debug Interface -> Buffer
    HIBYTE(DUP_DBGDATA),            // src[15:8]
    LOBYTE(DUP_DBGDATA),            // src[7:0]
    HIBYTE(ADDR_BUF0),              // dest[15:8]
    LOBYTE(ADDR_BUF0),              // dest[7:0]
    0,                              // len[12:8] - filled in later
    0,                              // len[7:0]
    31,                             // trigger: DBG_BW
    0x11                            // increment destination
};
//! DUP DMA descriptor
const unsigned char dma_desc_1[8] =
{
    // Buffer -> Flash controller
    HIBYTE(ADDR_BUF0),              // src[15:8]
    LOBYTE(ADDR_BUF0),              // src[7:0]
    HIBYTE(DUP_FWDATA),             // dest[15:8]
    LOBYTE(DUP_FWDATA),             // dest[7:0]
    0,                              // len[12:8] - filled in later
    0,                              // len[7:0]
    18,                             // trigger: FLASH
    0x42,                           // increment source
};





// USART Receiver buffer
#define RX_BUFFER_SIZE 350
volatile uint8_t rx_buffer[RX_BUFFER_SIZE];
volatile uint16_t rx_wr_index=0,rx_rd_index=0;
volatile uint16_t rx_counter=0;
volatile uint8_t rx_buffer_overflow=0;

// USART Transmitter buffer
#define TX_BUFFER_SIZE 350
volatile uint8_t  tx_buffer[TX_BUFFER_SIZE];
volatile uint16_t tx_wr_index=0,tx_rd_index=0;
volatile uint16_t tx_counter=0;


static __IO uint32_t TimingDelay;
/* Private function prototypes -----------------------------------------------*/
void Delay(uint32_t nTime);
void TimingDelay_Decrement(void);
void My_GPIO_init(void);

uint8_t Serial_read(void);
void Serial_write(uint8_t c);
uint8_t Serial_available(void);
void My_USART_Init(void);
void Delay_nop(uint32_t nTime);

/* Private functions ---------------------------------------------------------*/

/**
* @brief  This function handles SysTick Handler.
* @param  None
* @retval None
*/
void SysTick_Handler(void)
{
	if ( TimingDelay )
	{ 
		TimingDelay--;
	}
}

void USART1_IRQHandler(void)
{ 
	if(USART_GetFlagStatus(USART1, USART_FLAG_RXNE) == SET)
	{
		if ((USART1->SR & (USART_FLAG_NE|USART_FLAG_FE|USART_FLAG_PE|USART_FLAG_ORE)) == 0)
		{                       
			rx_buffer[rx_wr_index++]=(uint8_t)(USART_ReceiveData(USART1)& 0xFF);
			
			if (rx_wr_index == RX_BUFFER_SIZE) 
				rx_wr_index=0;
				
			if (++rx_counter == RX_BUFFER_SIZE)
			{
				rx_counter=0;
				rx_buffer_overflow=1;
			}
		}
		else USART_ReceiveData(USART1);//вообще здесь нужен обработчик ошибок, а мы просто пропускаем битый байт
	}

	if(USART_GetFlagStatus(USART1, USART_FLAG_ORE) == SET) //прерывание по переполнению буфера
	{
		USART_ReceiveData(USART1); //в идеале пишем здесь обработчик переполнения буфера, но мы просто сбрасываем этот флаг прерывания чтением из регистра данных.
	}

	if(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == SET)
	{   
		if (tx_counter)
		{
			--tx_counter;
			USART_SendData(USART1,tx_buffer[tx_rd_index++]);
			if (tx_rd_index == TX_BUFFER_SIZE) 
				tx_rd_index=0;
		}
		else
		{
			USART_ITConfig(USART1, USART_IT_TXE, DISABLE);                  
		}
	}
}











/**************************************************************************//**
* @brief    Writes a byte on the debug interface. Requires DD to be
*           output when function is called.
* @param    data    Byte to write
* @return   None.
******************************************************************************/
#pragma inline
void write_debug_byte(uint8_t data)
{
	uint8_t i;
	for (i = 0; i < 8; i++)
	{
		// Set clock high and put data on DD line
		PIN_DC_HIGH;		
		if(data & 0x80)
		{
			PIN_DD_OUT_HIGH;
			
		}
		else
		{
			PIN_DD_OUT_LOW;
		}
		//Delay(1);
		Delay_nop(1);
		data <<= 1;
		PIN_DC_LOW;
		Delay_nop(1);
		//Delay(1);
	}
}

/**************************************************************************//**
* @brief    Reads a byte from the debug interface. Requires DD to be
*           input when function is called.
* @return   Returns the byte read.
******************************************************************************/
#pragma inline
uint8_t read_debug_byte(void)
{
	uint8_t i;
	uint8_t data = 0x00;
	for (i = 0; i < 8; i++)
	{
		PIN_DC_HIGH;	 // DC high
		data <<= 1;
	//	Delay(1);
	Delay_nop(1);
		if(SET == PIN_DD_READ)
		{
			data |= 0x01;
		}        
		PIN_DC_LOW;	// DC low
Delay_nop(1);
		//Delay(1);
	}
	return data;
}

/**************************************************************************//**
* @brief    Function waits for DUP to indicate that it is ready. The DUP will
*           pulls DD line low when it is ready. Requires DD to be input when
*           function is called.
* @return   Returns 0 if function timed out waiting for DD line to go low
* @return   Returns 1 when DUP has indicated it is ready.
******************************************************************************/
#pragma inline
uint8_t wait_dup_ready(void)
{
	// DUP pulls DD low when ready
	uint16_t count = 0;
	while ((SET == PIN_DD_READ) && count < 16)
	{
		// Clock out 8 bits before checking if DD is low again
		read_debug_byte();
		count++;
	}
	return (count == 16) ? 0 : 1;
}

/**************************************************************************//**
* @brief    Issues a command on the debug interface. Only commands that return
*           one output byte are supported.
* @param    cmd             Command byte
* @param    cmd_bytes       Pointer to the array of data bytes following the
*                           command byte [0-3]
* @param    num_cmd_bytes   The number of data bytes (input to DUP) [0-3]
* @return   Data returned by command
******************************************************************************/
uint8_t debug_command(uint8_t cmd, uint8_t *cmd_bytes, uint16_t num_cmd_bytes)
{
	uint8_t i;
	uint8_t output = 0;
	// Make sure DD is output
	//pinMode(DD, OUTPUT);
	// Send command
	write_debug_byte(cmd);
	// Send bytes
	for (i = 0; i < num_cmd_bytes; i++)
	{
		write_debug_byte(cmd_bytes[i]);
	}
	// Set DD as input
	//pinMode(DD, INPUT);
	//digitalWrite(DD, SET);
	PIN_DD_OUT_HIGH;
	// Wait for data to be ready
	wait_dup_ready();
	// Read returned byte
	output = read_debug_byte();
	// Set DD as output
	//pinMode(DD, OUTPUT);

	return output;
}

/**************************************************************************//**
* @brief    Resets the DUP into debug mode. Function assumes that
*           the programmer I/O has already been configured using e.g.
*           ProgrammerInit().
* @return   None.
******************************************************************************/
void debug_init(void)
{
	volatile uint8_t i;

	// Send two flanks on DC while keeping RESET_N low
	// All low (incl. RESET_N)

	PIN_DD_OUT_LOW;
	
	PIN_DC_LOW;

	PIN_RES_LOW;
	Delay(100);   // Wait 10us
	
	PIN_DC_HIGH;// DC high
	Delay(10);   // Wait
	
	PIN_DC_LOW;// DC low
	Delay(10);   // Wait 10us
	
	PIN_DC_HIGH;// DC high
	Delay(10);   // Wait 10us
	
	PIN_DC_LOW;// DC low
	Delay(10);   // Wait 10us
	
	PIN_RES_HIGH; // Release RESET_N
	Delay(100);   // Wait 10us
}

/**************************************************************************//**
* @brief    Reads the chip ID over the debug interface using the
*           GET_CHIP_ID command.
* @return   Returns the chip id returned by the DUP
******************************************************************************/
uint8_t read_chip_id(void)
{
	uint8_t id = 0;

	// Make sure DD is output
	
	Delay(1);// Wait 1us
	// Send command
	write_debug_byte(CMD_GET_CHIP_ID);
	// Set DD as input
	
	PIN_DD_OUT_HIGH;
	Delay(1);// Wait 1us
	// Wait for data to be ready
	if(wait_dup_ready() == 1)
	{
		// Read ID and revision
		id = read_debug_byte(); // ID
		read_debug_byte();      // Revision (discard)
	}
	

	return id;
}

/**************************************************************************//**
* @brief    Sends a block of data over the debug interface using the
*           BURST_WRITE command.
* @param    src         Pointer to the array of input bytes
* @param    num_bytes   The number of input bytes
* @return   None.
******************************************************************************/
void burst_write_block(uint8_t *src, uint16_t num_bytes)
{
	uint16_t i;

	write_debug_byte(CMD_BURST_WRITE | HIBYTE(num_bytes));
	write_debug_byte(LOBYTE(num_bytes));
	for (i = 0; i < num_bytes; i++)
	{
		write_debug_byte(src[i]);
	}

	PIN_DD_OUT_HIGH;
	// Wait for DUP to be ready
	wait_dup_ready();
	read_debug_byte(); // ignore output
	
}

/**************************************************************************//**
* @brief    Issues a CHIP_ERASE command on the debug interface and waits for it
*           to complete.
* @return   None.
******************************************************************************/
void chip_erase(void)
{
	volatile uint8_t status;
	// Send command
	debug_command(CMD_CHIP_ERASE, 0, 0);

	// Wait for status bit 7 to go low
	do {
		status = debug_command(CMD_READ_STATUS, 0, 0);
	} while((status & STATUS_CHIP_ERASE_BUSY_BM));
}

/**************************************************************************//**
* @brief    Writes a block of data to the DUP's XDATA space.
* @param    address     XDATA start address
* @param    values      Pointer to the array of bytes to write
* @param    num_bytes   Number of bytes to write
* @return   None.
******************************************************************************/
void write_xdata_memory_block(uint32_t address, const uint8_t *values, uint16_t num_bytes)
{
	uint8_t instr[3];
	uint16_t i;

	// MOV DPTR, address
	instr[0] = 0x90;
	instr[1] = HIBYTE(address);
	instr[2] = LOBYTE(address);
	debug_command(CMD_DEBUG_INSTR_3B, instr, 3);

	for (i = 0; i < num_bytes; i++)
	{
		// MOV A, values[i]
		instr[0] = 0x74;
		instr[1] = values[i];
		debug_command(CMD_DEBUG_INSTR_2B, instr, 2);

		// MOV @DPTR, A
		instr[0] = 0xF0;
		debug_command(CMD_DEBUG_INSTR_1B, instr, 1);

		// INC DPTR
		instr[0] = 0xA3;
		debug_command(CMD_DEBUG_INSTR_1B, instr, 1);
	}
}

/**************************************************************************//**
* @brief    Writes a byte to a specific address in the DUP's XDATA space.
* @param    address     XDATA address
* @param    value       Value to write
* @return   None.
******************************************************************************/
void write_xdata_memory(uint32_t address, uint8_t value)
{
	uint8_t instr[3];

	// MOV DPTR, address
	instr[0] = 0x90;
	instr[1] = HIBYTE(address);
	instr[2] = LOBYTE(address);
	debug_command(CMD_DEBUG_INSTR_3B, instr, 3);

	// MOV A, values[i]
	instr[0] = 0x74;
	instr[1] = value;
	debug_command(CMD_DEBUG_INSTR_2B, instr, 2);

	// MOV @DPTR, A
	instr[0] = 0xF0;
	debug_command(CMD_DEBUG_INSTR_1B, instr, 1);
}

/**************************************************************************//**
* @brief    Read a byte from a specific address in the DUP's XDATA space.
* @param    address     XDATA address
* @return   Value read from XDATA
******************************************************************************/
uint8_t read_xdata_memory(uint32_t address)
{
	uint8_t instr[3];

	// MOV DPTR, address
	instr[0] = 0x90;
	instr[1] = HIBYTE(address);
	instr[2] = LOBYTE(address);
	debug_command(CMD_DEBUG_INSTR_3B, instr, 3);

	// MOVX A, @DPTR
	instr[0] = 0xE0;
	return debug_command(CMD_DEBUG_INSTR_1B, instr, 1);
}

/**************************************************************************//**
* @brief    Reads 1-32767 bytes from DUP's flash to a given buffer on the
*           programmer.
* @param    bank        Flash bank to read from [0-7]
* @param    address     Flash memory start address [0x0000 - 0x7FFF]
* @param    values      Pointer to destination buffer.
* @return   None.
******************************************************************************/
void read_flash_memory_block(uint8_t bank,uint32_t flash_addr, uint16_t num_bytes, uint8_t *values)
{
	uint8_t instr[3];
	uint16_t i;
	uint16_t xdata_addr = (0x8000 + flash_addr);

	// 1. Map flash memory bank to XDATA address 0x8000-0xFFFF
	write_xdata_memory(DUP_MEMCTR, bank);

	// 2. Move data pointer to XDATA address (MOV DPTR, xdata_addr)
	instr[0] = 0x90;
	instr[1] = HIBYTE(xdata_addr);
	instr[2] = LOBYTE(xdata_addr);
	debug_command(CMD_DEBUG_INSTR_3B, instr, 3);

	for (i = 0; i < num_bytes; i++)
	{
		// 3. Move value pointed to by DPTR to accumulator (MOVX A, @DPTR)
		instr[0] = 0xE0;
		values[i] = debug_command(CMD_DEBUG_INSTR_1B, instr, 1);

		// 4. Increment data pointer (INC DPTR)
		instr[0] = 0xA3;
		debug_command(CMD_DEBUG_INSTR_1B, instr, 1);
	}
}

/**************************************************************************//**
* @brief    Writes 4-2048 bytes to DUP's flash memory. Parameter \c num_bytes
*           must be a multiple of 4.
* @param    src         Pointer to programmer's source buffer (in XDATA space)
* @param    start_addr  FLASH memory start address [0x0000 - 0x7FFF]
* @param    num_bytes   Number of bytes to transfer [4-1024]
* @return   None.
******************************************************************************/
void write_flash_memory_block(uint8_t *src, uint32_t start_addr, uint16_t num_bytes)
{
	uint8_t len[2];
	
	
	
	// 1. Write the 2 DMA descriptors to RAM
	write_xdata_memory_block(ADDR_DMA_DESC_0, dma_desc_0, 8);
	write_xdata_memory_block(ADDR_DMA_DESC_1, dma_desc_1, 8);

	// 2. Update LEN value in DUP's DMA descriptors
	len[0]=HIBYTE(num_bytes);
	len[1]=LOBYTE(num_bytes);
	
	write_xdata_memory_block((ADDR_DMA_DESC_0+4), len, 2);  // LEN, DBG => ram
	write_xdata_memory_block((ADDR_DMA_DESC_1+4), len, 2);  // LEN, ram => flash

	// 3. Set DMA controller pointer to the DMA descriptors
	write_xdata_memory(DUP_DMA0CFGH, HIBYTE(ADDR_DMA_DESC_0));
	write_xdata_memory(DUP_DMA0CFGL, LOBYTE(ADDR_DMA_DESC_0));
	write_xdata_memory(DUP_DMA1CFGH, HIBYTE(ADDR_DMA_DESC_1));
	write_xdata_memory(DUP_DMA1CFGL, LOBYTE(ADDR_DMA_DESC_1));

	// 4. Set Flash controller start address (wants 16MSb of 18 bit address)
	write_xdata_memory(DUP_FADDRH, HIBYTE( (start_addr)));//>>2) ));
	write_xdata_memory(DUP_FADDRL, LOBYTE( (start_addr)));//>>2) ));

	// 5. Arm DBG=>buffer DMA channel and start burst write
	write_xdata_memory(DUP_DMAARM, CH_DBG_TO_BUF0);
	burst_write_block(src, num_bytes);

	// 6. Start programming: buffer to flash
	write_xdata_memory(DUP_DMAARM, CH_BUF0_TO_FLASH);
	write_xdata_memory(DUP_FCTL, 0x0A);//0x06

	// 7. Wait until flash controller is done
	while (read_xdata_memory(DUP_FCTL) & 0x80);
}

void RunDUP(void)
{

	// Send two flanks on DC while keeping RESET_N low
	// All low (incl. RESET_N)
	//digitalWrite(DD, RESET);
	PIN_DD_OUT_LOW;
	//digitalWrite(DC, RESET);
	PIN_DC_LOW;
	//digitalWrite(RESET, RESET);
	PIN_RES_LOW;
	Delay(10);  // Wait 10us

	//digitalWrite(RESET, SET);
	PIN_RES_HIGH;
	Delay(10);   // Wait 10us
}
















/**
* @brief  Main program.
* @param  None
* @retval None
*/

int main(void)
{
	uint8_t chip_id = 0;
	uint8_t debug_config = 0;
	uint8_t Continue = 0;
	uint8_t Verify = 0;
	uint16_t i;
	uint8_t ch;
	uint8_t Done = 0;
	uint8_t State = WAITING;
	uint8_t  rxBuf[514]; 
	uint8_t read_data[512];
	uint16_t BufIndex = 0;
	uint32_t addr = 0x00000000;
	uint16_t CheckSum,CheckSum_t;
	uint32_t bank;
	uint32_t  offset;
		
		
		
	STM32vldiscovery_LEDInit(LED3);
	STM32vldiscovery_LEDInit(LED4);
	
	STM32vldiscovery_LEDOn(LED3);
	STM32vldiscovery_LEDOn(LED4);
	
	My_GPIO_init();
	My_USART_Init();
	
	/* Setup SysTick Timer for 1 us? interrupts  */
	if (SysTick_Config(SystemCoreClock / 1000000UL))
	{ 
		while (1);	/* Capture error */ 
	}
	SysTick->CTRL  &= ~SysTick_CTRL_ENABLE_Msk;	//stop systick
	
	STM32vldiscovery_LEDOff(LED4);
	
	//Включаем прерывания по приему байта
	USART_ITConfig( USART1, USART_IT_RXNE, ENABLE );
//	USART_ITConfig(USART1, USART_IT_TC, ENABLE);
	NVIC_EnableIRQ( USART1_IRQn );
	
	__enable_irq();
	
	
	/* main while */
	while(1)
	{
	
		chip_id = 0;
		debug_config = 0;
		Continue = 0;
		Verify = 0;
		
		while(!Continue)     // Wait for starting
		{  
			if(Serial_available()==2)
			{      
				if(Serial_read() == SBEGIN)
				{
					Verify = Serial_read();
					debug_init();
					chip_id = read_chip_id();
					if((chip_id == 0) || (chip_id == 0xFF)) 
					{
						Serial_write(ERRO);  
					//	return; // No chip detected, run loop again.
					}
					else
						{
						Continue = 1;
						STM32vldiscovery_LEDOn(LED4);
						}
				}
				else
				{
					Serial_read(); // Clear RX buffer
				}
			}
		}
	
		
		
		RunDUP();
		debug_init();
		
		chip_erase();
		RunDUP();
		debug_init();
		
		// Switch DUP to external crystal osc. (XOSC) and wait for it to be stable.
		// This is recommended if XOSC is available during programming. If
		// XOSC is not available, comment out these two lines.
		write_xdata_memory(DUP_CLKCONCMD, 0x80);
		while (read_xdata_memory(DUP_CLKCONSTA) != 0x80);//0x80)
		
		// Enable DMA (Disable DMA_PAUSE bit in debug configuration)
		debug_config = 0x22;
		debug_command(CMD_WR_CONFIG, &debug_config, 1);
		
		// Program data (start address must be word aligned [32 bit])
		Serial_write(SRSP);    // Request data blocks


		Done = 0;
		State = WAITING;
		BufIndex = 0;
		addr = 0x0000;
  
		while(!Done)
		{
			while(Serial_available())
			{
				    
				ch = Serial_read();        
				switch (State)
				{
					// Bootloader is waiting for a new block, each block begin with a flag byte
					case WAITING:
						{
							if(SDATA == ch)  // Incoming bytes are data
							{
								State = RECEIVING;
							}
							else if(SEND == ch)   // End receiving firmware
							{
								Done = 1;           // Exit while(1) in main function
							}
							break;
						}      
					// Bootloader is receiving block data  
					case RECEIVING:
						{          
							rxBuf[BufIndex] = ch;
							BufIndex++;            
							if (BufIndex == 514) // If received one block, write it to flash
							{	
								STM32vldiscovery_LEDToggle(LED4);
								
								BufIndex = 0;              
								CheckSum = 0x0000;
								for( i=0; i<512; i++)
								{
									CheckSum += rxBuf[i];
								}
								CheckSum_t = rxBuf[512]<<8 | rxBuf[513];
								
								if(CheckSum_t != CheckSum)
								{	// Fail
									State = WAITING;
									Serial_write(ERRO);                    
									chip_erase();
									Done = 1;
									break;
								} 
								else
									write_flash_memory_block(rxBuf, addr, 512); // src, address, count                    
									
								if(Verify && !Done)
								{
									bank = addr / (512 * 16);
									offset = (addr % (512 * 16)) * 4;
									read_flash_memory_block((uint8_t)bank, offset, 512, read_data); // Bank, address, count, dest.            
									for( i = 0; i < 512; i++) 
									{
										if(read_data[i] != rxBuf[i]) 
										{
											// Fail
											State = WAITING;
											Serial_write(ERRO);                    
											chip_erase();
											Done = 1;
											break;
										}
									}
								}
								
								if(!Done)
								{
									addr += (uint16_t)128;              
									State = WAITING;
									Serial_write(SRSP);
								}
							}
							break;
						}      
				default:
					break;
				}
			}
		}
		
		STM32vldiscovery_LEDOff(LED4);
		RunDUP();

	}
}


















void My_GPIO_init(void)
{
	//Объявляем переменную port типа GPIO_InitTypeDef
	GPIO_InitTypeDef port;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	GPIO_StructInit(&port);

	port.GPIO_Mode = GPIO_Mode_Out_OD ;
	port.GPIO_Pin = PIN_DD_OUT;
	port.GPIO_Speed = GPIO_Speed_50MHz;  
	GPIO_Init(GPIOB, &port); 

	port.GPIO_Mode = GPIO_Mode_Out_PP ;
	port.GPIO_Pin = PIN_DC | PIN_RESET | GPIO_Pin_11;
	port.GPIO_Speed = GPIO_Speed_50MHz;  
	GPIO_Init(GPIOB, &port); 
	
	GPIO_SetBits( GPIOB, (PIN_RESET | GPIO_Pin_11) );	



	port.GPIO_Mode = GPIO_Mode_IPU;   
	port.GPIO_Pin = PIN_DD_IN;  //for Poll UP DD
	port.GPIO_Speed = GPIO_Speed_50MHz;   
	GPIO_Init(GPIOB, &port);
}

void My_USART_Init(void)
{
	USART_InitTypeDef usart;
	GPIO_InitTypeDef port;
	
	RCC_APB2PeriphClockCmd((RCC_APB2Periph_GPIOA | RCC_APB2Periph_USART1), ENABLE);
	
	
	GPIO_StructInit(&port);
	
	port.GPIO_Mode = GPIO_Mode_AF_PP;
	port.GPIO_Pin = GPIO_Pin_9;
	port.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOA, &port);

	port.GPIO_Mode = GPIO_Mode_IPU;
	port.GPIO_Pin = GPIO_Pin_10;
	port.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOA, &port);	

	USART_StructInit(&usart);
	usart.USART_BaudRate = 115200;
	USART_Init(USART1, &usart);

	USART_Cmd(USART1, ENABLE);
}
/**
* @brief  Inserts a Delay time.
* @param  nTime: specifies the Delay time length, in us.
* @retval None
*/
void Delay(uint32_t nTime)
{ 
	
	TimingDelay = nTime;
	
	SysTick->VAL   = 0UL;
	SysTick->CTRL  |= SysTick_CTRL_ENABLE_Msk;//enable systick

	while(TimingDelay != 0);
	
	SysTick->CTRL  &= ~SysTick_CTRL_ENABLE_Msk;//stop systick
}
void Delay_nop(uint32_t nTime)
{ 
	while(nTime--)
		__nop();
}

/**
* @brief  Decrements the TimingDelay variable.
* @param  None
* @retval None
*/
void TimingDelay_Decrement( void )
{
	
}








uint8_t Serial_available(void)
{
	return rx_counter;
}

uint8_t Serial_read(void)
{
	uint8_t data;
	
	//while (rx_counter==0);
	
	data=rx_buffer[rx_rd_index++];
	
	if (rx_rd_index == RX_BUFFER_SIZE) 
		rx_rd_index=0;
	
	USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);
	--rx_counter;
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
	return data;
}

void Serial_write(uint8_t c)
{
	while (tx_counter == TX_BUFFER_SIZE);
	
	USART_ITConfig(USART1, USART_IT_TXE, DISABLE);
	
	if (tx_counter || (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET))
	{
		tx_buffer[tx_wr_index++]=c;

		if (tx_wr_index == TX_BUFFER_SIZE) 
			tx_wr_index=0;
			
		++tx_counter;
		
		USART_ITConfig(USART1, USART_IT_TXE, ENABLE);
	}
	else
		USART_SendData(USART1,c);

}







#ifdef  USE_FULL_ASSERT
/**
* @brief  Reports the name of the source file and the source line number
*         where the assert_param error has occurred.
* @param  file: pointer to the source file name
* @param  line: assert_param error line source number
* @retval None
*/
void assert_failed(uint8_t* file, uint32_t line)
{ 
	/* User can add his own implementation to report the file name and line number,
	ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

	/* Infinite loop */
	while (1)
	{
	}
}
#endif


/**
* @brief  Retargets the C library printf function to the USART.
* @param  None
* @retval None
*/
PUTCHAR_PROTOTYPE
{
	/* Place your implementation of fputc here */
	/* e.g. write a character to the USART */
	// USART_SendData(EVAL_COM1, (uint8_t) ch);

	/* Loop until the end of transmission */
	// while (USART_GetFlagStatus(EVAL_COM1, USART_FLAG_TC) == RESET)
	// {}

	return ch;
}



/**
* @}
*/

/**
* @}
*/

/******************* (C) COPYRIGHT 2010 STMicroelectronics *****END OF FILE****/
