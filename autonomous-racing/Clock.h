#ifndef CLOCK_H_
#define CLOCK_H_

/**
 * @file      Clock.h
 * @brief     Provide functions that initialize the MSP432 clock module
 * @details   Reconfigure MSP432 to run at 48 MHz
 * @version   TI-RSLK MAX v1.1
 * @author    Daniel Valvano and Jonathan Valvano
 * @copyright Copyright 2019 by Jonathan W. Valvano, valvano@mail.utexas.edu,
 * @warning   AS-IS
 * @note      For more information see  http://users.ece.utexas.edu/~valvano/
 * @date      June 28, 2019

 ******************************************************************************/

/* This example accompanies the book
   "Embedded Systems: Introduction to Robotics,
   Jonathan W. Valvano, ISBN: 9781074544300, copyright (c) 2019
 For more information about my classes, my research, and my books, see
 http://users.ece.utexas.edu/~valvano/

Simplified BSD License (FreeBSD License)
Copyright (c) 2017, Jonathan Valvano, All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice,
   this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

The views and conclusions contained in the software and documentation are
those of the authors and should not be interpreted as representing official
policies, either expressed or implied, of the FreeBSD Project.
*/
#include <stdint.h>
/*!
 * @defgroup MSP432
 * @brief
 * @{*/
/**
 * Configure the MSP432 clock to run at 48 MHz
 * @param none
 * @return none
 * @note  Since the crystal is used, the bus clock will be very accurate
 * @see Clock_GetFreq()
 * @brief  Initialize clock to 48 MHz
 */
void Clock_Init48MHz(void);



/**
 * Simple delay function which delays about n milliseconds.
 * It is implemented with a nested for-loop and is very approximate.
 * @param  n is the number of msec to wait
 * @return none
 * @note This function assumes a 48 MHz clock.
 * This implementation is not very accurate.
 * To improve accuracy, you could tune this function
 * by adjusting the constant within the implementation
 * found in the <b>Clock.c</b> file.
 * For a more accurate time delay, you could use the SysTick module.
 * @brief  Software implementation of a busy-wait delay
 */
void Clock_Delay1ms(uint32_t n);

/**
 * Simple delay function which delays about n microseconds.
 * It is implemented with a nested for-loop and is very approximate.
 * @param  n is the number of usec to wait
 * @return none
 * @note This function assumes a 48 MHz clock.
 * This implementation is not very accurate.
 * To improve accuracy, you could tune this function
 * by adjusting the constant within the implementation
 * found in the <b>Clock.c</b> file.
 * For a more accurate time delay, you could use the SysTick module.
 * @brief  Software implementation of a busy-wait delay
 */
void Clock_Delay1us(uint32_t n);

#endif
