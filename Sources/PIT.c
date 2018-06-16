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
#include "OS.h"
#include "MK70F12.h"


static uint32_t ModulePeriodNanoSec;

static void* PITUserArguments;

static void (*PITCallbackFunction)(void*);

// PIT semaphore
OS_ECB *PITSemaphore;

//Arrays to store the samples, they are also used in main
uint16_t SamplesChA[16];
uint16_t SamplesChB[16];
uint16_t SamplesChC[16];
uint8_t NbSamplesChA, NbSamplesChB, NbSamplesChC = 0;

bool PIT_Init(const uint32_t moduleClk, void (*userFunction)(void*), void* userArguments)
{
  PITSemaphore = OS_SemaphoreCreate(0);

  //PITCallbackFunction = userFunction;
  //PITUserArguments = userArguments;

  ModulePeriodNanoSec = 1000000000/moduleClk; //Stores the module clock in nanoseconds

  SIM_SCGC6 |= SIM_SCGC6_PIT_MASK;         //Enable PIT clock

  PIT_MCR &= ~PIT_MCR_MDIS_MASK;           //Enable PIT timer (0 to enable)
  PIT_MCR |= PIT_MCR_FRZ_MASK;             //Timers are stopped in Debug Mode

  PIT_TCTRL0 |= PIT_TCTRL_TIE_MASK;        //Enable PIT interrupts

  NVICICPR2 = (1<<(68 % 32));              //Clear any pending interrupts on PIT Channel 0
  NVICISER2 = (1<<(68 % 32));              //Enable PIT Channel 0 interrupts

  return true;
}

void PIT_Set(const uint64_t period, const bool restart)
{
    if (restart)
      PIT_Enable(false);      //Disable the timer
    int64_t temp = (period/ModulePeriodNanoSec);
    int64_t temp1 = PIT_LDVAL0;
    PIT_LDVAL0 = (period/ModulePeriodNanoSec) - 1;
    int64_t temp2 = PIT_LDVAL0;

    if (restart)
      PIT_Enable(true);      //Re-Enable the timer
    PIT_TCTRL0 |= PIT_TCTRL_TIE_MASK;        //Enable PIT interrupts
}

void PIT_Enable(const bool enable){
  if (enable)
    PIT_TCTRL0 |= PIT_TCTRL_TEN_MASK;       //Enable the timer
  else
    PIT_TCTRL0 &= ~PIT_TCTRL_TEN_MASK;      //Disable the timer
}


void __attribute__ ((interrupt)) PIT_ISR(void){
  OS_ISREnter();

  PIT_TFLG0 |= PIT_TFLG_TIF_MASK;       //Clear interrupt Flag (w1c)
  if (PITCallbackFunction)
      (*PITCallbackFunction)(PITUserArguments);     //Calls the RTC ISR callback function
  OS_SemaphoreSignal(PITSemaphore);

  //if (PITCallbackFunction)
    //(*PITCallbackFunction)(PITUserArguments);     //Calls the RTC ISR callback function

  OS_ISRExit();
}
