/*! @file
 *
 *  @brief Functions to handle PIT commands
 *
 *  This contains the structure for the functions that handle PIT commands.
 *
 *  @author 11989668, 13113117
 *  @date 2018-05-01
 */

#include "PIT.h"
#include "MK70F12.h"


static uint32_t ModulePeriodNanoSec;

static void* PITUserArguments;

static void (*PITCallbackFunction)(void*);



bool PIT_Init(const uint32_t moduleClk, void (*userFunction)(void*), void* userArguments)
{
  PIT0_Semaphore = OS_SemaphoreCreate(0);
  PIT1_Semaphore = OS_SemaphoreCreate(0);

  //PITCallbackFunction = userFunction;
  //PITUserArguments = userArguments;

  ModulePeriodNanoSec = 1000000000/moduleClk; //Stores the module clock in nanoseconds

  SIM_SCGC6 |= SIM_SCGC6_PIT_MASK;         //Enable PIT clock

  PIT_MCR &= ~PIT_MCR_MDIS_MASK;           //Enable PIT timer (0 to enable)
  PIT_MCR |= PIT_MCR_FRZ_MASK;             //Timers are stopped in Debug Mode

  PIT_TCTRL0 |= PIT_TCTRL_TIE_MASK;        //Enable PIT interrupts
  PIT_TCTRL1 |= PIT_TCTRL_TIE_MASK;        //Enable PIT interrupts

  NVICICPR2 = (1<<(68 % 32));              //Clear any pending interrupts on PIT Channel 0
  NVICISER2 = (1<<(68 % 32));              //Enable PIT Channel 0 interrupts

  NVICICPR2 = (1<<(69 % 32));              //Clear any pending interrupts on PIT Channel 0
  NVICISER2 = (1<<(69 % 32));              //Enable PIT Channel 0 interrupts

  return true;
}

void PIT_Set(const uint8_t channelNb, const uint64_t period, const bool restart)
{
  int64_t temp, temp1, temp2;
  switch(channelNb)
  {
    case 0:
      if (restart)
        PIT_Enable(channelNb, false);      //Disable the timer
      temp = (period/ModulePeriodNanoSec);
      temp1 = PIT_LDVAL0;
      PIT_LDVAL0 = (period/ModulePeriodNanoSec) - 1;
      temp2 = PIT_LDVAL0;

      if (restart)
        PIT_Enable(channelNb, true);      //Re-Enable the timer
      PIT_TCTRL0 |= PIT_TCTRL_TIE_MASK;        //Enable PIT interrupts
      break;
    case 1:
      if (restart)
        PIT_Enable(channelNb, false);      //Disable the timer
      temp = (period/ModulePeriodNanoSec);
      temp1 = PIT_LDVAL1;
      PIT_LDVAL1 = (period/ModulePeriodNanoSec) - 1;
      temp2 = PIT_LDVAL1;

      if (restart)
        PIT_Enable(channelNb, true);      //Re-Enable the timer
      PIT_TCTRL1 |= PIT_TCTRL_TIE_MASK;        //Enable PIT interrupts
      break;
  }
}

void PIT_Enable(const uint8_t channelNb, const bool enable)
{
  switch(channelNb)
  {
    case 0:
      if (enable)
        PIT_TCTRL0 |= PIT_TCTRL_TEN_MASK;       //Enable the timer
      else
        PIT_TCTRL0 &= ~PIT_TCTRL_TEN_MASK;      //Disable the timer
      break;
    case 1:
      if (enable)
        PIT_TCTRL1 |= PIT_TCTRL_TEN_MASK;       //Enable the timer
      else
        PIT_TCTRL1 &= ~PIT_TCTRL_TEN_MASK;      //Disable the timer
      break;
  }
}

void __attribute__ ((interrupt)) PIT0_ISR(void)
{
  OS_ISREnter();

  PIT_TFLG0 |= PIT_TFLG_TIF_MASK;       //Clear interrupt Flag (w1c)
  // if (PITCallbackFunction)
  //     (*PITCallbackFunction)(PITUserArguments);     //Calls the RTC ISR callback function
  OS_SemaphoreSignal(PIT0_Semaphore);

  OS_ISRExit();
}

void __attribute__ ((interrupt)) PIT1_ISR(void)
{
  OS_ISREnter();

  PIT_TFLG1 |= PIT_TFLG_TIF_MASK;       //Clear interrupt Flag (w1c)

  OS_SemaphoreSignal(PIT1_Semaphore);

  //if (PITCallbackFunction)
    //(*PITCallbackFunction)(PITUserArguments);     //Calls the RTC ISR callback function

  OS_ISRExit();
}
