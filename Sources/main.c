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
#include "Packet.h"
#include "Flash.h"
#include "LEDs.h"


// ----------------------------------------
// Thread set up
// ----------------------------------------
// Arbitrary thread stack size - big enough for stacking of interrupts and OS use.
#define THREAD_STACK_SIZE 100
#define NB_ANALOG_CHANNELS 3

#define ALARM 3
#define RAISE 1
#define LOWER 2

const static double LO_TRESHHOLD = 2.00;
const static double HI_TRESHHOLD = 3.00;

// Thread stacks
OS_THREAD_STACK(InitModulesThreadStack, THREAD_STACK_SIZE); /*!< The stack for the LED Init thread. */
static uint32_t AnalogThreadStacks[NB_ANALOG_CHANNELS][THREAD_STACK_SIZE] __attribute__ ((aligned(0x08)));
//-------         -----------------       --------------
extern OS_ECB *PIT_Semaphore;           /*!< Binary semaphore for signaling PIT interrupt */
OS_ECB *SamplesReadySem;
OS_ECB *AlarmEventSem;

//Thread declarations
static void PacketCheckerThread(void* pData);

//Stacks
static uint32_t PacketCheckerThreadStack[THREAD_STACK_SIZE] __attribute__ ((aligned(0x08)));   /*!< The stack for the packet checking thread. */

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
int16union_t Frequency;    //Frecuency

static void PITCallback(void* arg);

//Packet Handling Functions
{
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
    if (Timing_Mode == 0xFF)
      if(!Flash_Write16((uint16_t *)Timing_Mode, 0x1))           //If flash is empty, use default value
        return false;

    if (!Flash_AllocateVar(&NbRaises, sizeof(*NbRaises)))        //Allocate the flash space for number of raises
      return false;
    if (NbRaises == 0xFF)
      if(!Flash_Write16((uint16_t *)NbRaises, 0x0))              //If flash is empty, use default value
        return false;

    if (!Flash_AllocateVar(&NbLowers, sizeof(*NbLowers)))        //Allocate the flash space for number of lowers
      return false;
    if (NbLowers == 0xFF)
      if (!Flash_Write16((uint16_t *)NbLowers, 0x0))             //If flash is empty, use default value
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
    return Packet_Put(PacketCommand, PacketParameter1, PacketParameter2, PacketParameter3);
  }

  /*! @brief Sends the Read Byte from Flash packet.
   *  @return bool - TRUE if data is successfully sent.
   */
  bool SendReadBytePacket(uint8_t offset)
  {
    PacketCommand = READ_BYTE_COMMAND;
    uint8_t byte;

    Flash_ReadByte(offset, &byte);
    return Packet_Put(READ_BYTE_COMMAND, offset, 0, byte);
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
      return Packet_Put(TIMING_MODE_COMMAND, &Timing_Mode, 0, 0);
    else if (!Flash_Write8((uint8_t *)Timing_Mode, Packet_Parameter1))
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
      return Packet_Put(NB_RAISES_COMMAND, &NbRaises, 0, 0);
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
      return Packet_Put(NB_LOWERS_COMMAND, &NbLowers, 0, 0);
    else if (!Flash_Write8((uint8_t *)NbLowers, 0x00))
      return false;

    return true;
  }
}
//

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
static ChannelData ChannelData[NB_ANALOG_CHANNELS] =
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

/*! @brief Initialises modules.
 *
 */
static void InitModulesThread(void* pData)
{
  SamplesReadySem = OS_SemaphoreCreate(0);
  AlarmEventSem = OS_SemaphoreCreate(0);

  // Analog
  (void)Analog_Init(CPU_BUS_CLK_HZ);

  // Generate the global analog semaphores
  for (uint8_t analogNb = 0; analogNb < NB_ANALOG_CHANNELS; analogNb++)
    AnalogThreadData[analogNb].semaphore = OS_SemaphoreCreate(0);

  // We only do this once - therefore delete this thread
  OS_ThreadDelete(OS_PRIORITY_SELF);
}

//Thread for sampling the channels
void SamplingThread(void* pData){
  #define threadData ((TAnalogThreadData*) pData)
  int16_t inputValue;
  uint8_t nbSamples = 0
  for(;;){
    OS_SemaphoreWait(threadData->semaphore,0);

    Analog_Get(threadData->channelNb, &inputValue);
    threadData->samples[nbSamples++] = inputValue;

    if(nbSamples == 16){
      threadData->nbSamples = 0;                           //Resets the amount of samples taken
      threadData->rms = rmsCalc(threadData->samples);      //Calculates the RMS value

      if(threadData->rms > HI_TRESHHOLD){
        threadData->alarm = 1;
        threadData->trigCount = 0;
        Analog_Put(ALARM, voltageToRaw(5.00));
      }
      else if(threadData->rms < LO_TRESHHOLD){
        threadData->alarm = 2;
        threadData->trigCount = 0;
        Analog_Put(ALARM, voltageToRaw(5.00));
      }
    }
  }
}

//Thread to signal when channels should sample
void PIT0Thread(void* data)
{
  for (;;)
  {
    OS_SemaphoreWait(PIT0Semaphore, 0);       //Wait on PIT Semaphore

    // Signal the analog channels to take a sample
    for (uint8_t analogNb = 0; analogNb < NB_ANALOG_CHANNELS; analogNb++)
      (void)OS_SemaphoreSignal(ChannelData[analogNb].semaphore);
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

  // Create threads for analog loopback channels
  for (uint8_t threadNb = 0; threadNb < NB_ANALOG_CHANNELS; threadNb++)
  {
    error = OS_ThreadCreate(AnalogLoopbackThread,
                            &AnalogThreadData[threadNb],
                            &AnalogThreadStacks[threadNb][THREAD_STACK_SIZE - 1],
                            ANALOG_THREAD_PRIORITIES[threadNb]);
  }

  // Start multithreading - never returns!
  OS_Start();
}

/*! @brief Checks for new packages and handles them depending on the comand.
 *
 *  @return bool - TRUE if data is correct and corresponds to the packet.
 */
bool HandlePacket()
{
  bool ErrorStatus = false;

  if (Packet_Get()){
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
  }

  return ErrorStatus;
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
 *
 * @return double - voltage value
 */
double rmsCalc(int16_t samples[16])
{
  double sum = 0;
  for(uint8_t i = 0; i < 16; i++)
    sum = rawToVoltage(samples[i]) * rawToVoltage(samples[i]);
  return sqrt(sum/16);
}

/*!
 ** @}
 */
