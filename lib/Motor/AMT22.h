/*
  AMT22.h - STM32 library for ATM22 series absolute encoders by CUI Devices.
  Created by Simone Di Blasi, December 2020.
*/


#ifndef AMT22_H_
#define AMT22_H_

// #ifdef __cplusplus
// extern "C" {
// #endif

#include "stdint.h"

#ifdef USE_TEENSY
#define SPI_HandleTypeDef void  
#define GPIO_TypeDef void
#define GPIO_PinState int
#define TIM_HandleTypeDef void
#define GPIO_PIN_RESET 0
#define GPIO_PIN_SET 1
#endif
/*
 * @brief 	Sets the state of the SPI line. It isn't necessary but makes the code more readable than having HAL_GPIO_Write everywhere.
 * @param	  encoderPort to select the GPIO peripheral for STM32.
 * @param   encoderPin specifies the port bit to be written.
 * @param 	csLine value to be set.
 * @retval	none.
 *
 */
void setCSLine (uint16_t encoderPin, int csLine);


/*
 * @brief	  Does SPI transfer.
 * @param	  hspi pointer to a SPI_HandleTypeDef structure that contains the configuration information for SPI module.
 * @param	  sendByte to be transmitted.
 * @param   encoderPort to select the GPIO peripheral for STM32.
 * @param   encoderPin specifies the port bit to be written.
 * @param 	releaseLine used to let the spiWriteRead function know if it should release the chip select line after transfer.
 * @param   timer is used to make microsecond delays during data exchange.
 * @retval 	data received from encoder.
 *
 */
uint8_t spiWriteRead(uint8_t sendByte, uint16_t encoderPin, uint8_t releaseLine);


/*
 * @brief 	Gets the absolute position from the AMT22 encoder using the SPI bus. The AMT22 position includes 2 checkbits to use
 * 				  for position verification. Both 12-bit and 14-bit encoders transfer position via two bytes, giving 16-bits regardless of resolution.
 * 				  For 12-bit encoders the position is left-shifted two bits, leaving the right two bits as zeros. This gives the impression that the encoder
 * 				  is actually sending 14-bits, when it is actually sending 12-bit values, where every number is multiplied by 4.
 * 				  Error values are returned as 0xFFFF.
 * @param	  hspi pointer to a SPI_HandleTypeDef structure that contains the configuration information for SPI module.
 * @param   encoderPort to select the GPIO peripheral for STM32.
 * @param   encoderPin specifies the port bit to be written.
 * @param	  resolution to properly format position responses.
 * @param   timer is used to make microsecond delays during data exchange.
 * @retval	currentPosition of the encoder. In case of error returned value is 0xFFFF.
 *
 */
uint16_t getPositionSPI(uint16_t encoderPin, uint8_t resolution);

/*  TODO 
 * @brief 	Gets the absolute position from the AMT22 encoder using the SPI bus. The AMT22 position includes 2 checkbits to use
 * 				  for position verification. Both 12-bit and 14-bit encoders transfer position via two bytes, giving 16-bits regardless of resolution.
 * 				  For 12-bit encoders the position is left-shifted two bits, leaving the right two bits as zeros. This gives the impression that the encoder
 * 				  is actually sending 14-bits, when it is actually sending 12-bit values, where every number is multiplied by 4.
 * 				  Error values are returned as 0xFFFF.
 * @param	  hspi pointer to a SPI_HandleTypeDef structure that contains the configuration information for SPI module.
 * @param   encoderPort to select the GPIO peripheral for STM32.
 * @param   encoderPin specifies the port bit to be written.
 * @param	  resolution to properly format position responses.
 * @param   timer is used to make microsecond delays during data exchange.
 * @retval	currentPosition of the encoder. In case of error returned value is 0xFFFF.
 *
 */
uint32_t get_turns_AMT22(uint16_t encoderPin, uint8_t resolution);

/*
 * @brief 	Sets value of the given encoder to ZERO.
 * @param 	hspi pointer to a SPI_HandleTypeDef structure that contains the configuration information for SPI module.
 * @param	  encoderPort to select the GPIO peripheral for STM32.
 * @param   encoderPin specifies the port bit to be written.
 * @param   timer is used to make microsecond delays during data exchange.
 * @retval	none.
 *
 */
void setZeroSPI(uint16_t encoderPin);


/*
 * @brief 	Resets given encoder.
 * @param 	hspi pointer to a SPI_HandleTypeDef structure that contains the configuration information for SPI module.
 * @param	  encoderPort to select the GPIO peripheral for STM32.
 * @param   encoderPin specifies the port bit to be written.
 * @param   timer is used to make microsecond delays during data exchange.
 * @retval	none.
 *
 */
void resetAMT22(uint16_t encoderPin);


int getTurnCounterSPI(int16_t* returnArr, uint16_t encoderPin, uint8_t resolution);

// #ifdef __cplusplus
// }
// #endif

#endif // AMT22_H_