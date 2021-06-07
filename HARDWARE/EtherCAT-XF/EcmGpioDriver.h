/******************************************************************************
 *	File	:	EcmUsrDriver.h
 *	Version :	0.6
 *	Date	:	2020/11/20
 *	Author	:	XFORCE
 *
 *	ECM-XF basic driver example - Header file
 *
 *	Demonstrate how to implement API type user driver
 *
 * @copyright (C) 2020 NEXTW TECHNOLOGY CO., LTD.. All rights reserved.
 *
 ******************************************************************************/

#ifndef _ECMGPIODRIVER_H_
#define _ECMGPIODRIVER_H_
#include <stdint.h>

/******************************************************************************
 * GPIO_MODE Definitions
 ******************************************************************************/
#define ECM_GPIO_MODE_INPUT          0x0UL /* Input Mode */
#define ECM_GPIO_MODE_OUTPUT         0x1UL /* Output Mode */
#define ECM_GPIO_MODE_OPEN_DRAIN     0x2UL /* Open-Drain Mode */
#define ECM_GPIO_MODE_BIDIR          0x3UL /* Quasi-bidirectional Mode */
/******************************************************************************
 * GPIO Interrupt Type Definitions
 ******************************************************************************/
#define ECM_GPIO_INT_DIABLE     	0x0UL /* Disable Interrupt */
#define ECM_GPIO_INT_BOTH_EDGE      0x3UL /* Interrupt enable by both Rising Edge and Falling Edge */
#define ECM_GPIO_INT_FALLING        0x4UL /* Interrupt enable by Input Falling Edge */
#define ECM_GPIO_INT_RISING         0x5UL /* Interrupt enable by Input Rising Edge */

/******************************************************************************
 * GPIO Pull Select Definitions
 ******************************************************************************/
#define ECM_GPIO_PUSEL_DISABLE          0x0UL           /* Pull Select Disable Mode */
#define ECM_GPIO_PUSEL_PULL_UP          0x1UL           /* Pull-up Mode */
#define ECM_GPIO_PUSEL_PULL_DOWN        0x2UL           /* Pull-down Mode */


/**
 * Set GPIO mode
 *
 * @param  ch				: GPIO channel. The valid range is from 0 to 15.
 * @param  direction		: IO direction. See ECM_GPIO_MODE_## definitions.
 * @param  pusel			: Pull select. See ECM_GPIO_PUSEL_## definitions.
 *
 * @return					: 1 for success; 0 for timeout; -1 for invalid input parameters.
 */
int ECM_GpioSetMode( uint8_t ch, uint8_t direction, uint8_t pusel);


/**
 * Enable/Disable Debounce mode
 *
 * @param  ch				: GPIO channel. The valid range is from 0 to 15.
 * @param  enable			: IO direction. 1 for enable debounce; 0 for disable debounce.
 *
 * @return					: 1 for success; 0 for timeout; -1 for invalid input parameters.
 */
int ECM_GpioEnableDebounce( uint8_t ch, uint8_t enable);


/**
 * Set Debounce Clock
 *
 * @param  source			: 1 for 10K clock source; 0 for 192M clock source
 * @param  clock			: Debounce cycle. The sampling cycle is 2^(@clock)*(clock source). The valid range is from 0 to 15.
 *
 * @return					: 1 for success; 0 for timeout; -1 for invalid input parameters.
 */
int ECM_GpioSetDebounceClock( uint8_t source, uint8_t clock);

/**
 * Set GPIO value.
 *
 * @param  val				: GPIO output value. The n-th bit of @val is mapping to the n-th GPIO channel.
 *
 * @return					: 1 for success; 0 for timeout; -1 for invalid input parameters.
 */
int ECM_GpioSetValue( uint16_t val);

/**
 * Get GPIO value.
 *
 * @param  val				: GPIO input value pointer. The n-th bit of *(@val) is mapping to the n-th GPIO channel.
 *
 * @return					: 1 for success; 0 for timeout; -1 for invalid input parameters.
 */
int ECM_GpioGetValue( uint16_t *val);

/**
 * Enable/Disable GPIO Interrupt.
 *
 * @param  ch				: GPIO channel. The valid range is from 0 to 15.
 * @param  inttype			: Interrupt type. See ECM_GPIO_INT_## definitions.
 *
 * @return					: 1 for success; 0 for timeout; -1 for invalid input parameters.
 */
int ECM_GpioIntEnable( uint8_t ch, uint8_t inttype);

/**
 * Clear GPIO Interrupt flag.
 *
 * @param  ch				: Clear the interrupt flag of the GPIO channel. The valid range is from 0 to 15.
 *
 * @return					: 1 for success; 0 for timeout; -1 for invalid input parameters.
 */
int ECM_GpioIntClear( uint8_t ch);

/**
 * Get GPIO Interrupt flag.
 *
 * @param  val				: GPIO Interrupt flag pointer. The n-th bit of *(@val) is mapping to the interrupt flag of the n-th GPIO channel.
 *
 * @return					: 1 for success; 0 for timeout; -1 for invalid input parameters.
 */
int ECM_GpioGetIntFlag( uint16_t *val);


int ECM_GpioExtSetValue( uint32_t val);
int ECM_GpioExtGetValue( uint32_t *val);
int ECM_GpioExtGetIntFlag( uint32_t *val);
#endif /* _ECMGPIODRIVER_H_ */
