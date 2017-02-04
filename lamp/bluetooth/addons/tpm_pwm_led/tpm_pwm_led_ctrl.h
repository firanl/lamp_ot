/*!
 * \defgroup tpm_pwm  TPM_PWM led control
 * @{
 * MKW30Z 5 channels control R G B Warm White Cold White using PWM.
 ******************************************************************************/
/*!
* @file tpm_pwm_led_ctrl.h
*
* @author  firanl
*
* @version 1.0
*
* @date Feb-02-2017
*
* @brief PWM TPM led control RGBWWCW
*
********************************************************************************
*/

#ifndef _TPM_PWM_LED_CTRL_H_
#define _TPM_PWM_LED_CTRL_H_

/******************************************************************************
* Header files
******************************************************************************/
// Stack header files
#include "EmbeddedTypes.h"

/******************************************************************************
* User definitions
*******************************************************************************/
#define PWM_factor_MIN 0
#define PWM_factor_MAX 100

#define PWM_factor_Init 0

// Switch pwm outs to off
#define TPM_PWM_WarmWhiteOff()       TPM_PWM_WarmWhite(PWM_factor_MIN)
#define TPM_PWM_ColdWhiteOff()       TPM_PWM_ColdWhite(PWM_factor_MIN)
#define TPM_PWM_RedOff()             TPM_PWM_Red(PWM_factor_MIN)
#define TPM_PWM_GreenOff()           TPM_PWM_Green(PWM_factor_MIN)
#define TPM_PWM_BlueOff()            TPM_PWM_Blue(PWM_factor_MIN)

// Switch pwm outs to on
#define TPM_PWM_WarmWhiteOn()        TPM_PWM_WarmWhite(PWM_factor_MAX)
#define TPM_PWM_ColdWhiteOn()        TPM_PWM_ColdWhite(PWM_factor_MAX)
#define TPM_PWM_RedOn()              TPM_PWM_Red(PWM_factor_MAX)
#define TPM_PWM_GreenOn()            TPM_PWM_Green(PWM_factor_MAX)
#define TPM_PWM_BlueOn()             TPM_PWM_Blue(PWM_factor_MAX)



/******************************************************************************
* Type definitions
******************************************************************************/

/******************************************************************************
* Globals
*******************************************************************************/

/******************************************************************************
* Configuration options
******************************************************************************/

/******************************************************************************
* Public Function prototypes
*******************************************************************************/

/*!**********************************************************************************
* \brief        Init TPM PWM registers. Set all PWM outputs to PWM_factor_Init.
*
* \param[in]    void   
* \return       void
************************************************************************************/
void TPM_PWM_Init(void);

/*!**********************************************************************************
* \brief        Set PWM duty cycle for warm white channel.
*
* \param[in]    uint8_t  pwm duty cycle value 
*
* \return       void
************************************************************************************/
void TPM_PWM_WarmWhite(uint8_t dutyCycleWarmWhite);

/*!**********************************************************************************
* \brief        Set PWM duty cycle for cold white channel.
*
* \param[in]    uint8_t  pwm duty cycle value 
*
* \return       void
************************************************************************************/
void TPM_PWM_ColdWhite(uint8_t dutyCycleColdWhite);

/*!**********************************************************************************
* \brief        Set PWM duty cycle for red channel.
*
* \param[in]    uint8_t  pwm duty cycle value 
*
* \return       void
************************************************************************************/
void TPM_PWM_Red(uint8_t dutyCycleRed);

/*!**********************************************************************************
* \brief        Set PWM duty cycle for green channel.
*
* \param[in]    uint8_t  pwm duty cycle value 
*
* \return       void
************************************************************************************/
void TPM_PWM_Green(uint8_t dutyCycleGreen);

/*!**********************************************************************************
* \brief        Set PWM duty cycle for blue channel.
*
* \param[in]    uint8_t  pwm duty cycle value 
*
* \return       void
************************************************************************************/
void TPM_PWM_Blue(uint8_t dutyCycleBlue);

/*!**********************************************************************************
* \brief        Set PWM duty cycle to PWM_factor_MIN for all channels.
*
* \param[in]    void  
*
* \return       void
************************************************************************************/
void TPM_PWM_Off(void);







/*!
 * @} End of tpm_pwm
 */


#endif
/* End of file */