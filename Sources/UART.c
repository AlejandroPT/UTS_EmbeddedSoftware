/*! @file
 *
 *  @brief I/O routines for UART communications on the TWR-K70F120M.
 *
 *  This contains the functions for operating the UART (serial port).
 */
/*!
 * @addtogroup UART_module UART documentation
 * @{
 */
/* MODULE UART */

/****************************************HEADER FILES****************************************************/
#include "FIFO.h"
#include "types.h"
#include "MK70F12.h"
#include "OS.h"
#include "UART.h"
#include "Cpu.h"
#include "packet.h"

/****************************************GLOBAL VARS*****************************************************/
static TFIFO RxFIFO;
static TFIFO TxFIFO;
OS_ECB *RxSemaphore; //Receive semaphore
OS_ECB *TxSemaphore; //Transmit semaphore

/****************************************PUBLIC FUNCTION DEFINITION***************************************/

/*! @brief Sets up the UART interface before first use.
 *
 *  @param baudRate The desired baud rate in bits/sec.
 *  @param moduleClk The module clock rate in Hz
 *  @return bool - TRUE if the UART was successfully initialized.
 */
bool UART_Init(const uint32_t baudRate, const uint32_t moduleClk)
{
  // Create semaphores for Receive and Transmit threads
  TxSemaphore = OS_SemaphoreCreate(0);
  RxSemaphore = OS_SemaphoreCreate(0);

  FIFO_Init(&RxFIFO);         //Initialize the Receiving FIFO for usage
  FIFO_Init(&TxFIFO);         //Initialize the Transmitting FIFO for usage

  uint8_t brfa;           //Baud rate fine adjustment variable
  uint16union_t sbr;          //Variable used to hold baud rate value

  if (baudRate == 0) return false;    //Check whether the BaudRate is not set to zero to avoid a runtime error

  SIM_SCGC4 |= SIM_SCGC4_UART2_MASK;  //Enable UART module in SIM_SCGC4
  SIM_SCGC5 |= SIM_SCGC5_PORTE_MASK;  //Enable Pin routing for Port E

  PORTE_PCR16 |= PORT_PCR_MUX(3);   //Set Pin16 to MUX mode Alternate 3
  PORTE_PCR17 |= PORT_PCR_MUX(3);   //Set Pin17 to MUX mode Alternate 3

  UART2_C2 &= ~UART_C2_TE_MASK;   //Disable UART transmitter
  UART2_C2 &= ~UART_C2_RE_MASK;   //Disable UART receiver

  sbr.l = moduleClk / (baudRate * 16);//Set the baud rate

  if (sbr.l > 0x1FFF) return false; //Check that the BaudRate is valid

  UART2_BDH = sbr.s.Hi;       //Set the high part of the BaudRate
  UART2_BDL = sbr.s.Lo;       //Set the low part of the BaudRate

  brfa = (uint8_t) ((moduleClk*2) / baudRate) % 32; //Calculate the BaudRate Find Adjust Value
  UART2_C4 |= UART_C4_BRFA_MASK;    //Prepare the register
  UART2_C4 &= brfa;       //Set the BaudRate Fine Adjust value

  UART2_C2 |= UART_C2_TIE_MASK;  //Transmit interrupt Enable
  UART2_C2 |= UART_C2_RIE_MASK;  //Receive interrupt Enable

  UART2_C2 |= UART_C2_TE_MASK;    //Enables UART transmitter
  UART2_C2 |= UART_C2_RE_MASK;    //Enables UART receiver

  //Initialize NVIC
  //SPRING 2017- Mid semester question pg 4
  //pg 97/2275 - K70 Manual
  NVICICPR1 = (1 << (49 % 32));
  NVICISER1 = (1 << (49 % 32));

  return true;
}

/*! @brief Get a character from the receive FIFO if it is not empty.
 *
 *  @param dataPtr A pointer to memory to store the retrieved byte.
 *  @return bool - TRUE if the receive FIFO returned a character.
 *  @note Assumes that UART_Init has been called.
 */
void UART_InChar(uint8_t * const dataPtr)
{
  //Get the data stored in RxFIFO and store it within the address given by dataPtr
  FIFO_Get(&RxFIFO, dataPtr);
}

/*! @brief Put a byte in the transmit FIFO if it is not full.
 *
 *  @param data The byte to be placed in the transmit FIFO.
 *  @return bool - TRUE if the data was placed in the transmit FIFO.
 *  @note Assumes that UART_Init has been called.
 */
void UART_OutChar(const uint8_t data)
{
  FIFO_Put(&TxFIFO, data); //Place the value stored in data into the TxFIFO
}

/*! @brief The thread which handles the receiving of data
 *
 *  @param data
 *  @note Assumes that UART_Init has been called.
 */
void RxThread(void *data)
{
  for(;;)
  {
    OS_SemaphoreWait(RxSemaphore, 0);
    FIFO_Put(&RxFIFO, UART2_D);
    // Something in RxFIFO, Let Packet Know

    UART2_C2 |= UART_C2_RIE_MASK; //Enable receive interrupt
  }
}

/*! @brief The thread which handles the transmission of data
 *
 *  @param data
 *  @note Assumes that UART_Init has been called.
 */
void TxThread(void *data)
{
  uint8_t txData;

  for(;;)
  {
    // Wait on TxSemaphore
    OS_SemaphoreWait(TxSemaphore, 0);
    // Transmit Data
    FIFO_Get(&TxFIFO, &txData);
    UART2_D = txData;

    UART2_C2 |= UART_C2_TIE_MASK; //Enable hardware to tell me it can transmit again
  }
}


/*! @brief Interrupt service routine for the UART.
 *
 *  @note Assumes the transmit and receive FIFOs have been initialized.
 */
void __attribute__ ((interrupt)) UART_ISR(void)
{
  OS_ISREnter();
  static uint8_t txData;

  if (UART2_C2 & UART_C2_RIE_MASK)
  {
    if (UART2_S1 & UART_S1_RDRF_MASK)
    {
      OS_SemaphoreSignal(RxSemaphore);
      UART2_C2 &= ~UART_C2_RIE_MASK;
    }
  }
  if (UART2_C2 & UART_C2_TIE_MASK)
  {
    if (UART2_S1 & UART_S1_TDRE_MASK)
    {
      OS_SemaphoreSignal(TxSemaphore);
      UART2_C2 &= ~UART_C2_TIE_MASK;
    }
  }
  OS_ISRExit();
}

/*!
 * @}
 */
