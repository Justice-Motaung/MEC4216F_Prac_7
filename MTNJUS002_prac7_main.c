//********************************************************************
//*                    MEC4126F C template                           *
//*==================================================================*
//* WRITTEN BY: Jesse Arendse   	                 	             *
//* DATE CREATED: 07/04/2023                                         *
//* MODIFIED: Dylan Fanner                                           *
//* DATE MODIFIED: 23/01/2025                                        *
//*==================================================================*
//* PROGRAMMED IN: Visual Studio Code                                *
//* TARGET:        STM32F0                                           *
//*==================================================================*
//* DESCRIPTION:     Template for MEC4126F C Practicals              *
//*                                                                  *
//Student Number: MTNJUS002 
// Prac 7 attempt
//********************************************************************
// INCLUDE FILES
//====================================================================

#define STM32F051

#include "stm32f0xx.h"											   
#include "stdio.h"
#include "stdint.h"

//====================================================================
// GLOBAL CONSTANTS
//====================================================================


//====================================================================
// GLOBAL VARIABLES
//====================================================================
float e, P, I, Ts = 0.001f;  // error, proportional, integral, sample time
float g_prev = 0.0f, sOp = 0.0f; // previous gain, system output placeholder

// ADC readings
uint16_t real_angle = 0;
uint16_t command_angle = 0;

//====================================================================
// FUNCTION DECLARATIONS
//====================================================================
void set_to_48MHz(void);
void ADC_config(void);

//====================================================================
// MAIN FUNCTION
//====================================================================

void main (void)
{
    set_to_48MHz();
    ADC_config();

    while (1)
    {
     
    }
}							
// End of main

//====================================================================
// ISR DEFINITIONS
//====================================================================

void TIM6_IRQHandler(void)
{
    // Acknowledge interrupt
    TIM6->SR &= ~TIM_SR_UIF;

    // Start ADC conversion for actual angle
    ADC1->CR |= ADC_CR_ADSTART;
    while (!(ADC1->ISR & ADC_ISR_EOC));
    actual_angle = ADC1->DR;

    // Start ADC conversion for command angle
    ADC1->CR |= ADC_CR_ADSTART;
    while (!(ADC1->ISR & ADC_ISR_EOC));
    command_angle = ADC1->DR;

    // Simple error calculation
    e = (float)command_angle - (float)real_angle;

    // Work in progress: PID computation
    float g = e * P;
    float o = ((2 + I * Ts) * g + (I * Ts - 2) * g_prev + sOp) / 2;
}

//====================================================================
// FUNCTION DEFINITIONS
//====================================================================
/*
*   Function to configure system clock to 48 MHz
*/
void set_to_48MHz(void)
{
    if ((RCC->CFGR & RCC_CFGR_SWS) == RCC_CFGR_SWS_PLL)
    {
        RCC->CFGR &= (uint32_t) (~RCC_CFGR_SW);
        while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_HSI);
    }
    RCC->CR &= (uint32_t)(~RCC_CR_PLLON);
    while ((RCC->CR & RCC_CR_PLLRDY) != 0);
    RCC->CFGR = ((RCC->CFGR & (~0x003C0000)) | 0x00280000);
    RCC->CR |= RCC_CR_PLLON;
    while ((RCC->CR & RCC_CR_PLLRDY) == 0);
    RCC->CFGR |= (uint32_t) (RCC_CFGR_SW_PLL);
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);
}

void ADC_config(void)
{
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN; 
    GPIOA->MODER |= GPIO_MODER_MODER5 | GPIO_MODER_MODER6; 
    RCC->APB2ENR |= RCC_APB2ENR_ADCEN;
    ADC1->CFGR1 |= ADC_CFGR1_RES_0 | ADC_CFGR1_CONT | ADC_CFGR1_WAIT;
    ADC1->CHSELR |= ADC_CHSELR_CHSEL5 | ADC_CHSELR_CHSEL6;
    ADC1->CR |= ADC_CR_ADEN;
    while ((ADC1->ISR & ADC_ISR_ADRDY) == 0);
}



//********************************************************************
// END OF PROGRAM
//********************************************************************