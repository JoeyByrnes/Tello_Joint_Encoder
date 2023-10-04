#include <dwt_delay.hpp>
#include "alt_main.h"
#include "stm32g4xx_hal.h"
#include <Protocentral_ADS1220.hpp>
#include <string.h>
#include <stdio.h>
#include "main.h"
#include <math.h>

#define CPU_FREQ_HZ (float)170e6

#define AS4047D_CS1_Port GPIOA
#define AS4047D_CS1_Pin GPIO_PIN_3
#define ENC_CS_HIGH AS4047D_CS1_Port->BSRR=(uint32_t)AS4047D_CS1_Pin
#define ENC_CS_LOW AS4047D_CS1_Port->BRR=(uint32_t)AS4047D_CS1_Pin
#define READ_POS_REG 0xFFFF
#define READ_POS_REG_COMP 0xFFFE
#define READ_VEL_REG 0xFFFC

#define HALF_ENC_CPR 8192

//#define CAN_DEVICE_ID_ENC (THIS IS UNUSED NOW, ID is read from FLASH)
#define CAN_DEVICE_ID_ENC_ADDRESS ((uint8_t*)0x0800FFF0)
#define CAN_MASTER_ID (uint8_t)0
#define CAN_DATA_SIZE 8    // Number of data bytes in a CAN packet

uint8_t encoder_packet[8]; // Packet 1 with 8 bytes

FDCAN_TxHeaderTypeDef txHeader;

uint8_t can_id = 0;

uint16_t last_raw_pos = 0;
int num_turns = 0;

union FloatToBytes {
    float f;
    uint8_t b[sizeof(float)];
};

union Int32ToBytes {
    int32_t i;
    uint8_t b[sizeof(int32_t)];
};

union Int16ToBytes {
    int16_t i;
    uint8_t b[sizeof(int16_t)];
};


bool first_reading = true;

extern UART_HandleTypeDef huart2;

extern SPI_HandleTypeDef hspi1;

extern DMA_HandleTypeDef hdma_usart2_tx;

extern FDCAN_HandleTypeDef hfdcan1;


uint16_t AS5147_Read_Pos()
{
	ENC_CS_LOW;
	while ((SPI1->SR & SPI_SR_BSY) != 0);
	SPI1->DR = READ_POS_REG;
	SPI1->CR1 |= SPI_CR1_SPE;
	while ((SPI1->SR & SPI_SR_RXNE) == 0);
	uint16_t result = SPI1->DR;
	while ((SPI1->SR & SPI_SR_TXE) == 0);
	while ((SPI1->SR & SPI_SR_BSY) != 0);
	SPI1->CR1 &= ~(SPI_CR1_SPE);
	ENC_CS_HIGH;
	return (result & 0x3FFF);
}

uint16_t AS5147_Read_Vel()
{
	ENC_CS_LOW;
	while ((SPI1->SR & SPI_SR_BSY) != 0);
	SPI1->DR = READ_VEL_REG;
	SPI1->CR1 |= SPI_CR1_SPE;
	while ((SPI1->SR & SPI_SR_RXNE) == 0);
	uint16_t result = SPI1->DR;
	while ((SPI1->SR & SPI_SR_TXE) == 0);
	while ((SPI1->SR & SPI_SR_BSY) != 0);
	SPI1->CR1 &= ~(SPI_CR1_SPE);
	ENC_CS_HIGH;
	return (result & 0x3FFF);
}

int32_t twoscomp14bit_to_int(int16_t value) {
    // Extract the sign bit from the 14-bit number (bit 13)
    int32_t sign_bit = (value & 0x2000) ? 0xFFFFC000 : 0x00000000;

    // Extend the 14-bit number to 32 bits while preserving its sign
    int32_t result = (int32_t)(value | sign_bit);

    return result;
}


int alt_main()
{
	/* Initialization */
	can_id = *(CAN_DEVICE_ID_ENC_ADDRESS);

	txHeader.Identifier = (can_id-20); // will set master IDs to 0 through 9 to avoid conflicts
	txHeader.IdType = FDCAN_STANDARD_ID;
	txHeader.TxFrameType = FDCAN_DATA_FRAME;
	txHeader.DataLength = FDCAN_DLC_BYTES_8;
	txHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	txHeader.BitRateSwitch = FDCAN_BRS_OFF;
	txHeader.FDFormat = FDCAN_CLASSIC_CAN;
	txHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	txHeader.MessageMarker = 0;

	last_raw_pos = AS5147_Read_Pos();



	while (1)
	{


	}
}

int serial_cnt = 0;

void setSPIMode(int mode)
{
	SPI1->CR1 &= ~SPI_CR1_SPE;
	if(mode == 1){
		// Set CPOL and CPHA bits to configure SPI mode 1
		SPI1->CR1 &= ~SPI_CR1_CPOL;  // Clear CPOL bit (CPOL = 0)
		SPI1->CR1 |= SPI_CR1_CPHA;   // Set CPHA bit  (CPHA = 1)
	}
	else if(mode == 0){
		// Set CPOL and CPHA bits to configure SPI mode 1
		SPI1->CR1 &= ~SPI_CR1_CPOL;  // Clear CPOL bit (CPOL = 0)
		SPI1->CR1 &= ~SPI_CR1_CPHA;   // Set CPHA bit  (CPHA = 0)
	}
	SPI1->CR1 |= SPI_CR1_SPE;
}

void main_1KHz_interrupt()
{

	int multiturn_pos;

	AS5147_Read_Pos();
	AS5147_Read_Pos();
	uint16_t pos = AS5147_Read_Pos();

	int rollover = 0;
	int angle_diff = pos - last_raw_pos;
	if(angle_diff > HALF_ENC_CPR){rollover = -1;}
	else if(angle_diff < -HALF_ENC_CPR){rollover = 1;}
	num_turns += rollover;

	/* Multi-turn position */
	multiturn_pos = pos + 16384*num_turns;

	last_raw_pos = pos;

	AS5147_Read_Vel();
	AS5147_Read_Vel();
	uint16_t velEnc = AS5147_Read_Vel();


	int velInt = twoscomp14bit_to_int(velEnc);


//	FloatToBytes velf2b;
//	velf2b.f = velocity;
	Int32ToBytes int2b;
	int2b.i = velInt;

	// ================================================================================================

	txHeader.DataLength = FDCAN_DLC_BYTES_8;

	encoder_packet[0] = can_id;
	encoder_packet[1] = (pos >> 8) & 0xFF;
	encoder_packet[2] = (pos) & 0xFF;
	encoder_packet[3] = (127+num_turns) &0xFF; // turn tracking only supported up to 127 turns
	encoder_packet[4] = int2b.b[3];
	encoder_packet[5] = int2b.b[2];
	encoder_packet[6] = int2b.b[1];
	encoder_packet[7] = int2b.b[0];

	HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &txHeader, encoder_packet);
	usleep(200);

	char uartTxBuffer[1024];

	if(serial_cnt%100 == 0){
		sprintf(uartTxBuffer, "multiturn: %d,      velocity: %d    \r\n", multiturn_pos, velInt);
		HAL_UART_Transmit(&huart2, (uint8_t *)uartTxBuffer, strlen(uartTxBuffer),1);
	}
	serial_cnt++;


}
