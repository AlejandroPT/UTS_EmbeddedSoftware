/*
 * LEDs.c
 *
 *  Created on: 11 Apr 2018
 *      Author: 13113117
 */
#include "LEDs.h"

bool LEDs_Init(void)
{
  PORTA_PCR10 |= PORT_PCR_MUX(1);
  PORTA_PCR11 |= PORT_PCR_MUX(1);
  PORTA_PCR28 |= PORT_PCR_MUX(1);
  PORTA_PCR29 |= PORT_PCR_MUX(1);

  //Set LEDs as output
  GPIOA_PDDR |= LED_ORANGE_MASK;
  GPIOA_PDDR |= LED_YELLOW_MASK;
  GPIOA_PDDR |= LED_GREEN_MASK;
  GPIOA_PDDR |= LED_BLUE_MASK;

  //Turn on PortA
  SIM_SCGC5 |= SIM_SCGC5_PORTA_MASK;

  //Turn off LEDs
  GPIOA_PSOR |= LED_ORANGE_MASK;
  GPIOA_PSOR |= LED_YELLOW_MASK;
  GPIOA_PSOR |= LED_GREEN_MASK;
  GPIOA_PSOR |= LED_BLUE_MASK;

  return true;
}


void LEDs_On(const TLED color)
{
  GPIOA_PCOR |= color;
}


void LEDs_Off(const TLED color)
{
  GPIOA_PSOR |= color;
}

void LEDs_Toggle(const TLED color)
{
  GPIOA_PTOR |= color;
}
