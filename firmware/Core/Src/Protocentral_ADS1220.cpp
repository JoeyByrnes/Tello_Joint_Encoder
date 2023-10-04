//////////////////////////////////////////////////////////////////////////////////////////
//
//    Arduino library for the ADS1220 24-bit ADC breakout board
//
//    Author: Ashwin Whitchurch
//    Copyright (c) 2018 ProtoCentral
//
//    Based on original code from Texas Instruments
//
//    This software is licensed under the MIT License(http://opensource.org/licenses/MIT).
//
//   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT
//   NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
//   IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
//   WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
//   SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
//
//   For information on how to use, visit https://github.com/Protocentral/Protocentral_ADS1220/
//
/////////////////////////////////////////////////////////////////////////////////////////

#include <dwt_delay.hpp>
#include "stm32g4xx_hal.h"
#include <spi.h>
#include <limits.h>
#include <Protocentral_ADS1220.hpp>
#include "main.h"
#include <stdio.h>
#include <string.h>

#ifdef _BV
#undef _BV
#endif

#define _BV(bit) (1<<(bit))

#define SPI_HANDLE hspi1

extern SPI_HandleTypeDef hspi1;

extern UART_HandleTypeDef huart2;


Protocentral_ADS1220::Protocentral_ADS1220() 								// Constructors
{

}

void Protocentral_ADS1220::writeRegister(uint8_t address, uint8_t value)
{
    uint8_t txBuffer[2];
    txBuffer[0] = WREG | (uint8_t)(address << 2);
    txBuffer[1] = value;
    HAL_GPIO_WritePin(m_cs_port, m_cs_pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&SPI_HANDLE, txBuffer, 2, 100);
    HAL_GPIO_WritePin(m_cs_port, m_cs_pin, GPIO_PIN_SET);
}

uint8_t Protocentral_ADS1220::readRegister(uint8_t address)
{
    uint8_t txBuffer[2];
    txBuffer[0] = RREG | (uint8_t)(address << 2);
    txBuffer[1] = 0x00;
    uint8_t rxBuffer[2] = {0x00};
    HAL_GPIO_WritePin(m_cs_port, m_cs_pin, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(&SPI_HANDLE, txBuffer, rxBuffer, 2, 100);
    HAL_GPIO_WritePin(m_cs_port, m_cs_pin, GPIO_PIN_SET);
    return rxBuffer[1];
}

void Protocentral_ADS1220::begin(GPIO_TypeDef* cs_port, uint16_t cs_pin, GPIO_TypeDef* drdy_port, uint16_t drdy_pin)
{
    m_drdy_pin = drdy_pin;
    m_cs_pin = cs_pin;
    m_cs_port = cs_port;
    m_drdy_port = drdy_port;

    // Set CS pin as output and initialize it to high
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = m_cs_pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(m_cs_port, &GPIO_InitStruct);
    HAL_GPIO_WritePin(m_cs_port, m_cs_pin, GPIO_PIN_SET);

    // Set DRDY pin as input with pull-up (assuming DRDY is active low, change to GPIO_PULLDOWN if active high)
    GPIO_InitStruct.Pin = m_drdy_pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(m_drdy_port, &GPIO_InitStruct);

    // Initialize SPI1 peripheral (assuming SPI1 is used)
    if (HAL_SPI_Init(&hspi1) != HAL_OK) {
        // Handle SPI initialization error
    }

    // Perform ADS1220 reset
    ads1220_Reset();
    HAL_Delay(50);

    // Wait for the device to pull DRDY low after initialization
    if (!WaitForData(1)) {
        // Handle timeout error
    }

    // Configure ADS1220 registers
    m_config_reg0 = 0x00; // Default settings: AINP=AIN0, AINN=AIN1, Gain 1, PGA enabled
    m_config_reg1 = 0x04; // Default settings: DR=20 SPS, Mode=Normal, Conv mode=continuous, Temp Sensor disabled, Current Source off
    m_config_reg2 = 0x10; // Default settings: Vref internal, 50/60Hz rejection, power open, IDAC off
    m_config_reg3 = 0x00; // Default settings: IDAC1 disabled, IDAC2 disabled, DRDY pin only

    writeRegister(CONFIG_REG0_ADDRESS, m_config_reg0);
    writeRegister(CONFIG_REG1_ADDRESS, m_config_reg1);
    writeRegister(CONFIG_REG2_ADDRESS, m_config_reg2);
    writeRegister(CONFIG_REG3_ADDRESS, m_config_reg3);
}

void Protocentral_ADS1220::PrintRegisterValues(){
    Config_Reg0 = readRegister(CONFIG_REG0_ADDRESS);
    Config_Reg1 = readRegister(CONFIG_REG1_ADDRESS);
    Config_Reg2 = readRegister(CONFIG_REG2_ADDRESS);
    Config_Reg3 = readRegister(CONFIG_REG3_ADDRESS);

//    printf(""Config_Reg : ");
//    printf(Config_Reg0,HEX);
//    printf(Config_Reg1,HEX);
//    printf(Config_Reg2,HEX);
//    printf(Config_Reg3,HEX);
//    printf(" ");
}

void Protocentral_ADS1220::SPI_Command(uint8_t data_in)
{
    uint8_t rxBuffer;

    HAL_GPIO_WritePin(m_cs_port, m_cs_pin, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(&SPI_HANDLE, &data_in, &rxBuffer, 1, 100);
    HAL_GPIO_WritePin(m_cs_port, m_cs_pin, GPIO_PIN_SET);
}

void Protocentral_ADS1220::ads1220_Reset()
{
    uint8_t txBuffer[1] = {RESET};
    HAL_GPIO_WritePin(m_cs_port, m_cs_pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&SPI_HANDLE, txBuffer, 1, 100);
    HAL_GPIO_WritePin(m_cs_port, m_cs_pin, GPIO_PIN_SET);
}

void Protocentral_ADS1220::Start_Conv()
{
    SPI_Command(START);
}

// control register 0
void Protocentral_ADS1220::select_mux_channels(int channels_conf)
{
    m_config_reg0 &= ~REG_CONFIG0_MUX_MASK;
    m_config_reg0 |= channels_conf;
    writeRegister(CONFIG_REG0_ADDRESS,m_config_reg0);
}

void Protocentral_ADS1220::set_pga_gain(int pgagain)
{
    m_config_reg0 &= ~REG_CONFIG0_PGA_GAIN_MASK;
    m_config_reg0 |= pgagain ;
    writeRegister(CONFIG_REG0_ADDRESS,m_config_reg0);
}

void Protocentral_ADS1220::PGA_ON(void)
{
    m_config_reg0 &= ~_BV(0);
    writeRegister(CONFIG_REG0_ADDRESS,m_config_reg0);
}

void Protocentral_ADS1220::PGA_OFF(void)
{
    m_config_reg0 |= _BV(0);
    writeRegister(CONFIG_REG0_ADDRESS,m_config_reg0);
}

// control register 1
void Protocentral_ADS1220::set_data_rate(int datarate)
{
    m_config_reg1 &= ~REG_CONFIG1_DR_MASK;
    m_config_reg1 |= datarate;
    writeRegister(CONFIG_REG1_ADDRESS,m_config_reg1);
}

void Protocentral_ADS1220::set_OperationMode(int OPmode)
{
    m_config_reg1 &= ~REG_CONFIG1_MODE_MASK;
    m_config_reg1 |= OPmode;
    writeRegister(CONFIG_REG1_ADDRESS,m_config_reg1);
}

void Protocentral_ADS1220::set_conv_mode_single_shot(void)
{
    m_config_reg1 &= ~_BV(2);
    writeRegister(CONFIG_REG1_ADDRESS,m_config_reg1);
}

void Protocentral_ADS1220::set_conv_mode_continuous(void)
{
    m_config_reg1 |= _BV(2);
    writeRegister(CONFIG_REG1_ADDRESS,m_config_reg1);
}

void Protocentral_ADS1220::TemperatureSensorMode_disable(void)
{
    m_config_reg1 &= ~_BV(1);
    writeRegister(CONFIG_REG1_ADDRESS,m_config_reg1);
}

void Protocentral_ADS1220::TemperatureSensorMode_enable(void)
{
    m_config_reg1 |= _BV(1);
    writeRegister(CONFIG_REG1_ADDRESS,m_config_reg1);
}

void Protocentral_ADS1220::CurrentSources_OFF(void)
{
    m_config_reg1 &= ~_BV(0);
    writeRegister(CONFIG_REG1_ADDRESS,m_config_reg1);
}

void Protocentral_ADS1220::CurrentSources_ON(void)
{
    m_config_reg1 |= _BV(0);
    writeRegister(CONFIG_REG1_ADDRESS,m_config_reg1);
}

// control register 2
void Protocentral_ADS1220::set_VREF(int vref)
{
    m_config_reg2 &= ~REG_CONFIG2_VREF_MASK;
    m_config_reg2 |= vref;
    writeRegister(CONFIG_REG2_ADDRESS,m_config_reg2);
}

void Protocentral_ADS1220::set_FIR_Filter(int filter)
{
    m_config_reg2 &= ~REG_CONFIG2_FIR_MASK;
    m_config_reg2 |= filter;
    writeRegister(CONFIG_REG2_ADDRESS,m_config_reg2);
}

void Protocentral_ADS1220::LowSideSwitch_OPEN(void)
{
    m_config_reg2 &= ~_BV(3);
    writeRegister(CONFIG_REG2_ADDRESS,m_config_reg2);
}

void Protocentral_ADS1220::LowSideSwitch_CLOSED(void)
{
    m_config_reg2 |= _BV(3);
    writeRegister(CONFIG_REG2_ADDRESS,m_config_reg2);
}

void Protocentral_ADS1220::set_IDAC_Current(int IDACcurrent)
{
    m_config_reg2 &= ~REG_CONFIG2_IDACcurrent_MASK;
    m_config_reg2 |= IDACcurrent;
    writeRegister(CONFIG_REG2_ADDRESS,m_config_reg2);
}

// control register 3
void Protocentral_ADS1220::set_IDAC1_Route(int IDAC1routing)
{
    m_config_reg3 &= ~REG_CONFIG3_IDAC1routing_MASK;
    m_config_reg3 |= IDAC1routing;
    writeRegister(CONFIG_REG3_ADDRESS,m_config_reg3);
}

void Protocentral_ADS1220::set_IDAC2_Route(int IDAC2routing)
{
    m_config_reg3 &= ~REG_CONFIG3_IDAC2routing_MASK;
    m_config_reg3 |= IDAC2routing;
    writeRegister(CONFIG_REG3_ADDRESS,m_config_reg3);
}

 void Protocentral_ADS1220::DRDYmode_default(void)
 {
     m_config_reg3 &= ~_BV(3);
     writeRegister(CONFIG_REG3_ADDRESS,m_config_reg3);
 }

 void Protocentral_ADS1220::DRDYmode_DOUT(void)
 {
     m_config_reg3 |= _BV(3);
     writeRegister(CONFIG_REG3_ADDRESS,m_config_reg3);
 }
// end control register

uint8_t * Protocentral_ADS1220::get_config_reg()
{
    static uint8_t config_Buff[4];

    m_config_reg0 = readRegister(CONFIG_REG0_ADDRESS);
    m_config_reg1 = readRegister(CONFIG_REG1_ADDRESS);
    m_config_reg2 = readRegister(CONFIG_REG2_ADDRESS);
    m_config_reg3 = readRegister(CONFIG_REG3_ADDRESS);

    config_Buff[0] = m_config_reg0 ;
    config_Buff[1] = m_config_reg1 ;
    config_Buff[2] = m_config_reg2 ;
    config_Buff[3] = m_config_reg3 ;

    return config_Buff;
}

bool Protocentral_ADS1220::WaitForData(unsigned int timeout_ms)
{
//    uint32_t startMillis = HAL_GetTick();
//    while (HAL_GPIO_ReadPin(m_drdy_port, m_drdy_pin)) {
//        if (timeout_ms != 0 && (HAL_GetTick() - startMillis) > timeout_ms) {
//            return false; // Timeout occurred
//        }
//    }
//	int timeout_us = timeout_ms*1000;
//	int us_cnt = 0;
//	while (HAL_GPIO_ReadPin(m_drdy_port, m_drdy_pin))
//	{
//
//		DWT_Delay(1);
//		us_cnt++;
//		if(us_cnt >= timeout_us) return false;
//	}
//	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
	usleep(550);
//	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
    return true;
}

uint8_t* Protocentral_ADS1220::Read_Data()
{
    uint8_t txBuffer[3] = {SPI_MASTER_DUMMY, SPI_MASTER_DUMMY, SPI_MASTER_DUMMY};
    HAL_GPIO_WritePin(m_cs_port, m_cs_pin, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(&SPI_HANDLE, txBuffer, DataReg, 3, 5);
    HAL_GPIO_WritePin(m_cs_port, m_cs_pin, GPIO_PIN_SET);
    return DataReg;
}

int32_t Protocentral_ADS1220::DataToInt(){
    int32_t result = 0;
    result = DataReg[0];
    result = (result << 8) | DataReg[1];
    result = (result << 8) | DataReg[2];

    if (DataReg[0] & (1<<7)) {
        result |= 0xFF000000;
    }

    return result;
}

int32_t Protocentral_ADS1220::Read_WaitForData()
{
    if(!WaitForData(10)){
        return 0;
    }
//    char uartTxBuffer[50];
//    sprintf(uartTxBuffer, "GOT DATA\r\n");
//    	HAL_UART_Transmit(&huart2, (uint8_t *)uartTxBuffer, strlen(uartTxBuffer), 100);
    Read_Data();

    return DataToInt();
}

uint8_t* Protocentral_ADS1220::Read_Data_Immediate()
{
	return Read_Data();
//	return DataToInt();
}

int32_t Protocentral_ADS1220::Read_Data_Samples()
{
    uint8_t txBuffer[3] = {SPI_MASTER_DUMMY, SPI_MASTER_DUMMY, SPI_MASTER_DUMMY};
    HAL_GPIO_WritePin(m_cs_port, m_cs_pin, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(&SPI_HANDLE, txBuffer, DataReg, 3, 100);
    HAL_GPIO_WritePin(m_cs_port, m_cs_pin, GPIO_PIN_SET);

    int32_t mResult32 = 0;
    int32_t bit24 = DataReg[0];
    bit24 = (bit24 << 8) | DataReg[1];
    bit24 = (bit24 << 8) | DataReg[2];
    bit24 = (bit24 << 8);
    mResult32 = (bit24 >> 8);

    return mResult32;
}

int32_t Protocentral_ADS1220::Read_SingleShot_WaitForData(void)
{
    Start_Conv();
    return Read_WaitForData();
}

void Protocentral_ADS1220::Start_SingleShot_Conv(void)
{
    Start_Conv();
}

int32_t Protocentral_ADS1220::Read_SingleShot_SingleEnded_WaitForData(uint8_t channel_no)
{
    select_mux_channels(channel_no);
    return Read_SingleShot_WaitForData();
}

#define VREF_MASK ((1 << 6) | (1<<7))
#define VREF_INT (0 << 6)
#define VREF_EXT (1 << 6)

void Protocentral_ADS1220::internal_reference(){
    m_config_reg2 &= ~VREF_MASK;
    m_config_reg2 |= VREF_INT;
    writeRegister(CONFIG_REG2_ADDRESS,m_config_reg2);
}

void Protocentral_ADS1220::external_reference(){
    m_config_reg2 &= ~VREF_MASK;
    m_config_reg2 |= VREF_EXT;
    writeRegister(CONFIG_REG2_ADDRESS,m_config_reg2);
}
