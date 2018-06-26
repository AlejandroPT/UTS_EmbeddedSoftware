/* ###################################################################
 **     Filename    : main.c
 **     Project     : Project
 **     Processor   : MK70FN1M0VMJ12
 **     Version     : Driver 01.01
 **     Compiler    : GNU C Compiler
 **     Date/Time   : 2018-06-10, 21:36, # CodeGen: 0
 **     Abstract    :
 **         Main module.
 **         This module contains user's application code.
 **     Settings    :
 **     Contents    :
 **         No public methods
 **
 ** ###################################################################*/
/*!
 ** @file main.c
 ** @version 6.0
 ** @brief
 **         Main module.
 **         This module contains user's application code.
 */
/*!
 **  @addtogroup main_module main module documentation
 **  @{
 */
/* MODULE main */

#include "Cpu.h"
#include "OS.h"
#include "analog.h"
#include "types.h"

#include "PIT.h"
#include "UART.h"
#include "packet.h"
#include "Flash.h"
#include "LEDs.h"
#include "FFT_UT.h"

#define NB_ANALOG_CHANNELS 3

/*! @brief Data structure used to pass Analog configuration to a user thread
 *
 */
typedef struct AnalogThreadData
{
  OS_ECB* semaphore;
  uint8_t channelNb;
  double rms;
  uint8_t alarm;         //0 for nit triggered, 1 high trigger, 2 for low triggered
  double deviation;      //Deviation from acceptable SetDefaultFlashValues
  int16_t samples[16];   //Deviation from acceptable SetDefaultFlashValues
  uint16_t trigCount;    //Deviation from acceptable SetDefaultFlashValues

} TAnalogThreadData;

/*! @brief Analog thread configuration data
 *
 */
static TAnalogThreadData ChannelData[NB_ANALOG_CHANNELS] =
{
  {  //Channel A in
    .semaphore = NULL,
    .channelNb = 0,
    .rms = 0.0,
    .alarm =0,
    .deviation = 0.0,
    .samples[0] = 0,
    .trigCount = 0
  },
  {  //Channel B in
    .semaphore = NULL,
    .channelNb = 1,
    .rms = 0.0,
    .alarm =0,
    .deviation = 0.0,
    .samples[0] = 0,
    .trigCount = 0
  },
  {  //Channel C in
    .semaphore = NULL,
    .channelNb = 2,
    .rms = 0.0,
    .alarm =0,
    .deviation = 0.0,
    .samples[0] = 0,
    .trigCount = 0
  },
};


// ----------------------------------------
// Thread set up
// ----------------------------------------
// Arbitrary thread stack size - big enough for stacking of interrupts and OS use.
#define THREAD_STACK_SIZE 100


#define BAUD_RATE 115200

#define ALARM 2
#define RAISE 0
#define LOWER 1

#define CHA 0
#define CHB 1
#define CHC 2

const static double LO_TRESHHOLD = 2.00;
const static double HI_TRESHHOLD = 3.00;

//Variables for keeping track of each raise or lower
static bool Raise = false;
static bool Lower = false;
static uint8_t RaiseTimer = 0;          //timing counter to know when to shut off trigger
static uint8_t LowerTimer = 0;
static bool Alarm = false;
static bool Triggered = false;

// Thread stacks
OS_THREAD_STACK(InitModulesThreadStack, THREAD_STACK_SIZE); /*!< The stack for the LED Init thread. */
static uint32_t AnalogThreadStacks[NB_ANALOG_CHANNELS][THREAD_STACK_SIZE] __attribute__ ((aligned(0x08)));
//-------         -----------------       --------------
extern OS_ECB *PIT0_Semaphore;           /*!< Binary semaphore for signaling PIT0 interrupt */
extern OS_ECB *PIT1_Semaphore;           /*!< Binary semaphore for signaling PIT1 interrupt */
OS_ECB *SamplesReadySem;
OS_ECB *AlarmEventSem;


//Stacks
static uint32_t PacketThreadStack[THREAD_STACK_SIZE] __attribute__ ((aligned(0x08)));   /*!< The stack for the packet checking thread. */
static uint32_t PIT0ThreadStack[THREAD_STACK_SIZE] __attribute__ ((aligned(0x08)));   /*!< The stack for the packet checking thread. */
static uint32_t PIT1ThreadStack[THREAD_STACK_SIZE] __attribute__ ((aligned(0x08)));   /*!< The stack for the packet checking thread. */
static uint32_t RxThreadStack[THREAD_STACK_SIZE] __attribute__ ((aligned(0x08)));   /*!< The stack for the packet checking thread. */
static uint32_t TxThreadStack[THREAD_STACK_SIZE] __attribute__ ((aligned(0x08)));   /*!< The stack for the packet checking thread. */


//-------         -----------------       --------------

// ----------------------------------------
// Thread priorities
// 0 = highest priority
// ----------------------------------------
const uint8_t ANALOG_THREAD_PRIORITIES[NB_ANALOG_CHANNELS] = {3, 4, 5};

//Definitions
volatile uint8_t *Timing_Mode;       //1 definitive, 2 inverse
volatile uint8_t *NbRaises;          //Number of raises done
volatile uint8_t *NbLowers;          //Number of lowers done
//TODO Check if it is fine to store NbRaises and NbLowers in s single byte
int16union_t FrequencyInt;    //Frecuency
static float Frequency;
static float PeriodNs;
static float SamplingRate;

const static uint64_t PIT1_RATE = 10000000;  //100Hz

static void PITCallback(void* arg);
double rmsCalc(int16_t samples[16]);
int16_t voltageToRaw(double voltage);
void FrequencyTracking(uint8_t index);
double Spectral_Analysis(unsigned long k);

//Packet Handling Functions
//{
  #define STARTUP_COMMAND 0x04
  #define READ_BYTE_COMMAND 0x08
  #define PROGRAM_BYTE_COMMAND 0x07

  #define TIMING_MODE_COMMAND 0x10
  #define NB_RAISES_COMMAND 0x11
  #define NB_LOWERS_COMMAND 0x12
  #define FREQUENCY_COMMAND 0x17
  #define VOLTAGE_COMMAND 0x18
  #define SPECTRUM_COMMAND 0x19

  static const uint8_t towerNumberHi = 0x31;
  static const uint8_t towerNumberLo = 0x17;

  static const uint8_t towerModeHi = 0x00;  //Check the enumerated type
  static const uint8_t towerModeLo = 0x01;

  volatile uint16union_t *NvTowerNb;
  volatile uint16union_t *NvTowerMode;

  static const uint8_t TOWER_VERSION_HI = 1;
  static const uint8_t TOWER_VERSION_LO = 0;

  static uint8_t PacketCommand,
  	PacketParameter1,
  	PacketParameter2,
  	PacketParameter3;
  //TODO: Remove PacketCommand et al, and change it in the functions

  /*! @brief Tries to get the Mode and Number values from flash, if not there, sets the defaults
   *  @return bool - TRUE if everything if the read (of the stored values) or writes (of defaults) are succesfull
   */
  bool SetDefaultFlashValues()
  {
    if (!Flash_AllocateVar(&Timing_Mode, sizeof(*Timing_Mode)))  //Allocate the flash space for timing mode
      return false;
    if (*Timing_Mode == 0xFF)
      if(!Flash_Write8(Timing_Mode, 0x01))           //If flash is empty, use default value
        return false;

    if (!Flash_AllocateVar(&NbRaises, sizeof(*NbRaises)))        //Allocate the flash space for number of raises
      return false;
    if (*NbRaises == 0xFF)
      if(!Flash_Write8(NbRaises, 0x00))              //If flash is empty, use default value
        return false;

    if (!Flash_AllocateVar(&NbLowers, sizeof(*NbLowers)))        //Allocate the flash space for number of lowers
      return false;
    if (*NbLowers == 0xFF)
      if (!Flash_Write8(NbLowers, 0x00))             //If flash is empty, use default value
        return false;
    return true;
  }

  /*! @brief Sends the startup packet.
   *  @return bool - TRUE if data is successfully sent.
   */
  bool SendStartupPacket()
  {
    PacketCommand = STARTUP_COMMAND;
    PacketParameter1 = 0;
    PacketParameter2 = 0;
    PacketParameter3 = 0;
    Packet_Put(PacketCommand, PacketParameter1, PacketParameter2, PacketParameter3);
  }

  /*! @brief Sends the Read Byte from Flash packet.
   *  @return bool - TRUE if data is successfully sent.
   */
  bool SendReadBytePacket(uint8_t offset)
  {
    PacketCommand = READ_BYTE_COMMAND;
    uint8_t byte;

    Flash_ReadByte(offset, &byte);
    Packet_Put(READ_BYTE_COMMAND, offset, 0, byte);
  }

  /*! @brief Handles a received startup packet.
   *  @return bool - TRUE if data is correct and corresponds to the packet.
   */
  bool HandleStartupPacket()
  {
    if (Packet_Parameter1 != 0 || Packet_Parameter2 != 0 || Packet_Parameter3 != 0) //Check that the values are correct
      return false;
    if (!SendStartupPacket())
      return false;

    return true;
  }

  /*! @brief Handles a received ProgramByte packet.
   *  @return bool - TRUE if data is correct and corresponds to the packet.
   */
  bool HandleProgramBytePacket()
  {
      if (Packet_Parameter1 > 8 || Packet_Parameter1 < 0 || Packet_Parameter2 != '0' || Packet_Parameter3 != 0)  //Check that the values are correct
        return false;

      if (PacketParameter1 == 8)
        return Flash_Erase();
      volatile uint32_t* const address = FLASH_DATA_START + Packet_Parameter1;
      return Flash_Write8(address, Packet_Parameter3);
  }

  /*! @brief Handles a received READ_BYTE_COMMAND packet.
   *  @return bool - TRUE if data is correct and corresponds to the packet.
   */
  bool HandleReadBytePacket()
  {
      if (Packet_Parameter1 > 7 || Packet_Parameter1 < 0 || Packet_Parameter2 != '0' || Packet_Parameter3 != 0)  //Check that the values are correct
        return false;
      return SendReadBytePacket(Packet_Parameter1);
  }

  /*! @brief Handles a received timing mode packet.
   *  @return bool - TRUE if data is correct and corresponds to the packet.
   */
  bool HandleTimingModePacket()
  {
    if (Packet_Parameter1 > 2 ||Packet_Parameter1 < 0 || Packet_Parameter2 != 0 || Packet_Parameter3 != 0) //Check that the values are correct
      return false;
    if (Packet_Parameter1 == 0)
      Packet_Put(TIMING_MODE_COMMAND, *Timing_Mode, 0, 0);
    else if (!Flash_Write8(Timing_Mode, Packet_Parameter1))
      return false;
      //TODO check if this is enough for changing the timing mode

    return true;
  }

  /*! @brief Handles a received number of raises packet.
   *  @return bool - TRUE if data is correct and corresponds to the packet.
   */
  bool HandleNbRaisesPacket()
  {
    if (Packet_Parameter1 > 1 || Packet_Parameter1 < 0 || Packet_Parameter2 != 0 || Packet_Parameter3 != 0) //Check that the values are correct
      return false;
    if (Packet_Parameter1 == 0)
      Packet_Put(NB_RAISES_COMMAND, *NbRaises, 0, 0);
    else if (!Flash_Write8((uint8_t *)NbRaises, 0x00))
      return false;

    return true;
  }

  /*! @brief Handles a received number of lowers packet.
   *  @return bool - TRUE if data is correct and corresponds to the packet.
   */
  bool HandleNbLowersPacket()
  {
    if (Packet_Parameter1 > 1 || Packet_Parameter1 < 0 || Packet_Parameter2 != 0 || Packet_Parameter3 != 0) //Check that the values are correct
      return false;
    if (Packet_Parameter1 == 0)
      Packet_Put(NB_LOWERS_COMMAND, *NbLowers, 0, 0);
    else if (!Flash_Write8((uint8_t *)NbLowers, 0x00))
      return false;

    return true;
  }

  /*! @brief Handles a received voltage packet.
   *  @return bool - TRUE if data is correct and corresponds to the packet.
   */
  bool HandleVoltagePacket()
  {
    if (Packet_Parameter1 < 1 || Packet_Parameter1 > 3 || Packet_Parameter2 != 0 || Packet_Parameter3 != 0) //Check that the values are correct
      return false;

    double voltage = ChannelData[Packet_Parameter1-1].rms;
    uint8_t unit = (uint8_t)voltage;
    uint8_t decimal = (uint8_t)((voltage-unit)*100);

    Packet_Put(VOLTAGE_COMMAND, Packet_Parameter1, unit, decimal);

    return true;
  }

  /*! @brief Handles a received frequency packet.
     *  @return bool - TRUE if data is correct and corresponds to the packet.
     */
  bool HandleFrequencyPacket()
  {
    if (Packet_Parameter1 != 0 || Packet_Parameter2 != 0 || Packet_Parameter3 != 0) //Check that the values are correct
      return false;

    uint8_t unit = (uint8_t)Frequency;
    uint8_t decimal = (uint8_t)((Frequency-unit)*100);

    Packet_Put(FREQUENCY_COMMAND, unit, decimal, 0);

    return true;
  }

  /*! @brief Handles a received spectral packet.
     *  @return bool - TRUE if data is correct and corresponds to the packet.
     */
  bool HandleSpectrumPacket()
  {
    if (Packet_Parameter1 < 0 || Packet_Parameter1 > 7 || Packet_Parameter2 != 0 || Packet_Parameter3 != 0) //Check that the values are correct
      return false;

    double spectrum = Spectral_Analysis(Packet_Parameter1);
    uint8_t unit = (uint8_t)spectrum;
    uint8_t decimal = (uint8_t)((spectrum-unit)*100);

    Packet_Put(SPECTRUM_COMMAND,Packet_Parameter1 ,unit , decimal);

    return true;
  }

  /*! @brief Checks for new packages and handles them depending on the comand.
   *
   *  @return bool - TRUE if data is correct and corresponds to the packet.
   */
  bool HandlePacket()
  {
    bool ErrorStatus = false;

    switch(Packet_Command & ~PACKET_ACK_MASK){
      case STARTUP_COMMAND:
        ErrorStatus = HandleStartupPacket();
        break;

      case PROGRAM_BYTE_COMMAND:
        ErrorStatus = HandleReadBytePacket();
        break;

      case READ_BYTE_COMMAND:
        ErrorStatus = HandleProgramBytePacket();
        break;

      case TIMING_MODE_COMMAND:
        ErrorStatus = HandleTimingModePacket();
        break;

      case NB_RAISES_COMMAND:
        ErrorStatus = HandleNbRaisesPacket();
        break;

      case NB_LOWERS_COMMAND:
        ErrorStatus = HandleNbLowersPacket();
        break;

      case VOLTAGE_COMMAND:
        ErrorStatus = HandleVoltagePacket();
        break;

      case FREQUENCY_COMMAND:
        ErrorStatus = HandleFrequencyPacket();
        break;

      case SPECTRUM_COMMAND:
        ErrorStatus = HandleSpectrumPacket();
        break;

      default:
        break;
    }

    if (ErrorStatus)
    {
      //LEDs_On(LED_BLUE);
      //PacketTimer.ioType.inputDetection = TIMER_OUTPUT_HIGH;
      //FTM_StartTimer(&PacketTimer);
    }

    if (Packet_Command & PACKET_ACK_MASK) {  //Check if an ACK is required, and send it (or the NACK)
      if (ErrorStatus)
        Packet_Put(Packet_Command, Packet_Parameter1, Packet_Parameter2, Packet_Parameter3);
      else
        Packet_Put(Packet_Command & ~PACKET_ACK_MASK, Packet_Parameter1, Packet_Parameter2, Packet_Parameter3);
    }

    return ErrorStatus;
  }
//}
//



/*! @brief Initialises modules.
 *
 */
static void InitModulesThread(void* pData)
{

  Frequency = 50;
  PeriodNs = (1 / Frequency) * 1000000000;
  SamplingRate = PeriodNs / 16;
  // Analog
  (void)Analog_Init(CPU_BUS_CLK_HZ);
  LEDs_Init();
  if(Packet_Init(BAUD_RATE, CPU_BUS_CLK_HZ))
    LEDs_On(LED_ORANGE);
  Flash_Init();
  PIT_Init(CPU_BUS_CLK_HZ, PITCallback, NULL);

  PIT_Set(0, (uint64_t)SamplingRate, true);
  PIT_Set(1, PIT1_RATE, true);
  PIT_Enable(1, false);               //Make sure its not on at start, only when needed


  // Generate the global analog semaphores
  for (uint8_t analogNb = 0; analogNb < NB_ANALOG_CHANNELS; analogNb++)
    ChannelData[analogNb].semaphore = OS_SemaphoreCreate(0);

  // We only do this once - therefore delete this thread
  OS_ThreadDelete(OS_PRIORITY_SELF);
}

//Thread for sampling the channels
void SamplingThread(void* pData){
  #define threadData ((TAnalogThreadData*) pData)
  int16_t inputValue;
  uint8_t nbSamples = 0;
  for (;;){
    OS_SemaphoreWait(threadData->semaphore,0);
                                      //Resets the amount of samples taken
    threadData->rms = rmsCalc(threadData->samples);      //Calculates the RMS value

    if(threadData->rms > HI_TRESHHOLD){
      threadData->deviation = threadData->rms - HI_TRESHHOLD ;
      threadData->alarm = 1;
      PIT_Enable(1, true);
    }
    else if(threadData->rms < LO_TRESHHOLD){
      threadData->deviation = LO_TRESHHOLD - threadData->rms;
      threadData->alarm = 2;
      PIT_Enable(1, true);
    }
    else
    {
      threadData->alarm = 0;
      threadData->trigCount = 0;
    }
    for(uint8_t i = 1; i < 16; i++)
      FrequencyTracking2(i);
  }

}

//Thread to signal when channels should sample
void PIT0Thread(void* data)
{
  uint8_t nbSamples = 0;
  for (;;)
  {
    OS_SemaphoreWait(PIT0_Semaphore, 0);       //Wait on PIT Semaphore
    int16_t inputValue;
    for (uint8_t analogNb = 0; analogNb < NB_ANALOG_CHANNELS; analogNb++){
      Analog_Get(ChannelData[analogNb].channelNb, &inputValue);
      ChannelData[analogNb].samples[nbSamples] = inputValue;
    }
    nbSamples++;
    // Signal the analog channels to take a sample
    if(nbSamples == 16){
      for (uint8_t analogNb = 0; analogNb < NB_ANALOG_CHANNELS; analogNb++)
        OS_SemaphoreSignal(ChannelData[analogNb].semaphore);
      nbSamples = 0;
    }
  }
}

void PIT1Thread(void* data)
{
  for (;;)
  {
    OS_SemaphoreWait(PIT1_Semaphore, 0);       //Wait on PIT Semaphore

    bool alarm = false;
    // Signal the analog channels to take a sample
    for (uint8_t analogNb = 0; analogNb < NB_ANALOG_CHANNELS; analogNb++)
    {
      if(ChannelData[analogNb].alarm != 0)
      {
        if(*Timing_Mode == 2)    //If mode is in inverse, do calculation, else add 5 to counter to make it trigger in 5 seconds
        {
          double tempCount = 25.0 / (0.5 / ChannelData[analogNb].deviation * 5);   //Calculation to see how much to increment in timer considerering a 100Hz interrupt

          if(tempCount > 25.0)   //Adjust if delay is going to be less than 1 second
            tempCount = 25.0;

          if(tempCount < 1)   //Adjust if deviation is so small no increment will be given, max delay is 25 seconds
            tempCount = 1.0;

          ChannelData[analogNb].trigCount += (uint16_t)(tempCount);
        }
        else
          ChannelData[analogNb].trigCount += 5;

        //If elapsed time has occured
        if(ChannelData[analogNb].trigCount >= 2500)
        {
          //If signal was above threshold, trigger a lower
          if(ChannelData[analogNb].alarm == 1)
          {
            Lower = true;

            LowerTimer = 0;
            Analog_Put(LOWER, voltageToRaw(5));
            if(!Triggered)
              Flash_Write8(NbLowers, *NbLowers+1);
          }
          //If signal was below threshold, trigger a raise
          if(ChannelData[analogNb].alarm == 2)
          {
            Raise = true;

            RaiseTimer = 0;
            Analog_Put(RAISE, voltageToRaw(5));
            if(!Triggered)
              Flash_Write8(NbRaises, *NbRaises+1);
          }
          //ChannelData[analogNb].alarm = 0;           //Reset counter
          Triggered = true;
        }
        else
          ChannelData[analogNb].trigCount++;
      }

      alarm += ChannelData[analogNb].alarm;   //result should be 0 (false) if all alarm are off

    }
    Alarm = alarm;
    //If the alarm is triggered, set it on. Else turn it off
    if (alarm)
      Analog_Put(ALARM, 16000);
    else {
      Analog_Put(ALARM, voltageToRaw(0.0));
      Analog_Put(RAISE, voltageToRaw(0.0));
      Analog_Put(LOWER, voltageToRaw(0.0));
      PIT_Enable(1, false);
      Triggered = false;
    }

    //If no alarm, raise of lower is going on, turn of the PIT1 timer
    if (!(alarm || Lower || Raise))
      PIT_Enable(1, false);
  }
}

void PacketThread(void* data)
{
  SendStartupPacket();
  SetDefaultFlashValues();
  for (;;)
  {
    if (Packet_Get()) //Check if there is a packet in the retrieved data
    {
      HandlePacket();
    }
  }
}

/*lint -save  -e970 Disable MISRA rule (6.3) checking. */
int main(void)
/*lint -restore Enable MISRA rule (6.3) checking. */
{
  OS_ERROR error;

  // Initialise low-level clocks etc using Processor Expert code
  PE_low_level_init();

  // Initialize the RTOS
  OS_Init(CPU_CORE_CLK_HZ, true);

  // Create module initialisation thread
  error = OS_ThreadCreate(InitModulesThread,
                          NULL,
                          &InitModulesThreadStack[THREAD_STACK_SIZE - 1],
                          0); // Highest priority

  error = OS_ThreadCreate(RxThread,
                          NULL,
                          &RxThreadStack[THREAD_STACK_SIZE-1],
                          1);

  error = OS_ThreadCreate(TxThread,
                          NULL,
                          &TxThreadStack[THREAD_STACK_SIZE-1],
                          2);

  // Create threads for analog loopback channels
  for (uint8_t threadNb = 0; threadNb < NB_ANALOG_CHANNELS; threadNb++)
  {
    error = OS_ThreadCreate(SamplingThread,
                            &ChannelData[threadNb],
                            &AnalogThreadStacks[threadNb][THREAD_STACK_SIZE - 1],
                            ANALOG_THREAD_PRIORITIES[threadNb]);
  }

  error = OS_ThreadCreate(PIT0Thread,
                          NULL,
                          &PIT0ThreadStack[THREAD_STACK_SIZE-1],
                          6);

  error = OS_ThreadCreate(PIT1Thread,
                          NULL,
                          &PIT1ThreadStack[THREAD_STACK_SIZE-1],
                          7);

  error = OS_ThreadCreate(PacketThread,
                          NULL,
                          &PacketThreadStack[THREAD_STACK_SIZE-1],
                          8);

  // Start multithreading - never returns!
  OS_Start();
}


void PITCallback(void* arg)
{
  /*
  Analog_Get(analogData->0, &SamplesChA[NbSamplesChA]);
  NbSamplesChA = (NbSamplesChA + 1) % 16;
  Analog_Get(analogData->1, &SamplesChB[NbSamplesChB]);
  NbSamplesChB = (NbSamplesChB + 1) % 16;
  Analog_Get(analogData->2, &SamplesChC[NbSamplesChC]);
  NbSamplesChB = (NbSamplesChB + 1) % 16;
  */
}

/*!
 * @brief Converts analogue input into voltage in Volts
 *
 * @return float - voltage value
 */
double rawToVoltage(int16_t raw)
{
  return ((float) raw * 20)/pow(2,16);
}

/*!
 * @brief Converts volts into analog output
 *
 * @return int16_t - voltage value
 */
int16_t voltageToRaw(double voltage)
{
  return (uint16_t) ((voltage * pow(2,16)) / 20);
}

/*!
 * @brief Converts 16 analog samples into RMS voltage
 * @return double - voltage value
 */
double rmsCalc(int16_t samples[16])
{
  double sum = 0;
  for(uint8_t i = 0; i < 16; i++)
    sum += rawToVoltage(samples[i]) * rawToVoltage(samples[i]);
  return (double)sqrt(sum/16);
}

/*!
 * @brief Gets a frequency by interpolating the places where the wave crosses 0, and calculating the error to the current sampling rate
 */
void FrequencyTracking(uint8_t index)
{
  static float offset1 = 0;
  static float offset2 = 0;
  static float spaceBetweenOffsets = 0;

  float sample1, sample2;

  //Check that index -1 will be valid, if not get the previous sample (sample 16)
  if (index == 0)
    sample1 = rawToVoltage(ChannelData[CHA].samples[15]);
  else
    sample1 = rawToVoltage(ChannelData[CHA].samples[index - 1]);

  sample2 = rawToVoltage(ChannelData[CHA].samples[index]);

  //If there is a  positive crossing through 0V
  if (sample1 < 0 && sample2 > 0)
  {
    offset1 = (-sample1) / (sample2 - sample1);      //calculate the offset in fractions between samples
    spaceBetweenOffsets = 0;                         //Reset the space between offsets
  }
  //If there is a  negative crossing through 0V
  else if (sample1 > 0 && sample2 < 0)
  {
    offset2 = (-sample1) / (sample2 - sample1);      //calculate the offset in fractions between samples
    double newPeriodNs = ((spaceBetweenOffsets - offset1 + offset2) / 8 * PeriodNs); // Period of wave in s
    double newFreq = 1.0  / (newPeriodNs / 1000000000);
    if (newFreq >= 47.5 && newFreq <= 52.5)
    {
      Frequency = newFreq;                           //Update global frequency
      PeriodNs = (1 / Frequency) * 1000000000;
      SamplingRate = PeriodNs / 16;
      PIT_Set(0, SamplingRate, true);                //Redefine PIT period and restart
    }
  }
  spaceBetweenOffsets++;
}

float calculateTimeOffset(float sample1, float sample2)
{
  sample1 = rawToVoltage(sample1);
  sample2 = rawToVoltage(sample2);
  float gradient = (sample2 - sample1)/(1);
  float timeOffset = ((-sample1) / gradient);
  return timeOffset;
}

void FrequencyTracking2(uint8_t count)
{
  static uint8_t crossingNb = 1;
  static float offset1;
  static float offset2;
  static float sampleOffset;

  //Avoid count[-1]..
  if (count != 0)
  {

    if (ChannelData[CHA].samples[count] > 0 && ChannelData[CHA].samples[count-1] < 0)
    {
      // Switch between first and second zero crossing
      switch (crossingNb)
      {
        // First zero crossing
        case 1:
          // Calculate time offset (fraction of a sample) between samples[count] and the zero crossing
          offset1 = calculateTimeOffset(ChannelData[CHA].samples[count-1], ChannelData[CHA].samples[count]);
          sampleOffset = 0; // Reset sample offset
          crossingNb = 2; // We've found the first zero crossing, find the next..
          break;

          // Second zero crossing
        case 2:
          // Calculate time offset (fraction of a sample) between samples[count] and the zero crossing
          offset2 = calculateTimeOffset(ChannelData[CHA].samples[count-1], ChannelData[CHA].samples[count]);
          // Number of samples between the first zero crossing and the second zero crossing
          // Minus the time offset of the first zero crossing
          // Plus the time offset of the second zero crossing
          // Multiplied by the sample period..
          //float period_s =  // Convert Period in ns to period in s .. Is there a better way?
          double new_period = (sampleOffset - offset1 + offset2) * ((float) SamplingRate / 1000000000); // Period of wave in s
          double frequency = (1 / (new_period)); // Calculate frequency

          // Filter 'bad' frequencies
          if (frequency >= 47.5 && frequency <= 52.5)
          {
            Frequency = frequency; // Set global frequency
            PeriodNs = ((1 / frequency) / 16 ) * 1000000000; // Period in nanoseconds
            PIT_Set(0, PeriodNs, true); // Redefine PIT period and restart
          }
          crossingNb = 1;
          break;
        default:
          break;
      }
    }
  }
  // Increment sample offset
  sampleOffset++;
}

//Calculates the Spectrum of the samples in Channel A
//k is the harmonic number
double Spectral_Analysis(unsigned long k){
  double data[32];
  for(uint8_t i = 0; i < 32; i+=2){
    data[i] = rawToVoltage(ChannelData[CHA].samples[i]);
    data[i+1] = 0; //Make real part 0
  }

  fft(data, 16);

  double v = fftMagnitude(data,16,k);
  //double dB = fftMagdB(data,16,k,2.0); // largest component is 2V
  double phase = fftPhase(data,16,k);
  double freq = fftFrequency(16,k,Frequency * 16);

  return v;
}

/*!
 ** @}
 */
