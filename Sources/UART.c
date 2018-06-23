/*! @file
 *
 *  @brief I/O routines for UART communications on the TWR-K70F120M.
 *
 *  This contains the implementation of functions for operating the UART.
 *
 *  @author 11989668, 13113117
 *  @date 2018-04-04
 */
#include "UART.h"
#include "MK70F12.h"

static TFIFO RxFIFO, TxFIFO;

OS_ECB *TxSemaphore;
OS_ECB *RxSemaphore;


bool UART_Init(const uint32_t baudRate, const uint32_t moduleClk)
{
  TxSemaphore = OS_SemaphoreCreate(0);
  RxSemaphore = OS_SemaphoreCreate(0);

  SIM_SCGC4 |= SIM_SCGC4_UART2_MASK;  //Enabling UART2
  SIM_SCGC5 |= SIM_SCGC5_PORTE_MASK;  //Enabling PortE for pin routing
  PORTE_PCR16 = PORT_PCR_MUX(3);     //Modifying the PortE register for the alt3 multiplex option (UART2_Tx) //only = because of the w1c
  PORTE_PCR17 = PORT_PCR_MUX(3);     //Modifying the PortE register for the alt3 multiplex option (UART2_Rx)

  UART2_C2 &= ~UART_C2_RE_MASK;  //Enabling the Receiver Enable bit
  UART2_C2 &= ~UART_C2_TE_MASK;  //Enabling the Transmitter Enable bit

  // Setting the baud rate fine adjust
  uint8_t fine_adjust = (uint8_t)(moduleClk * 2) / (baudRate) % 32;
  UART2_C4 = (fine_adjust & 0x1F);

  // Requested baud rate setup
  uint16union_t setting;				// Setting the unions to efficiently access high(Hi) and low(Lo) parts of integers and words
  setting.l  = (uint16_t)(moduleClk/(baudRate * 16));	// Setting the baud rate which is synchronized with the module clock
  UART2_BDH |= (uint8_t)(setting.s.Hi & 0x1F);		// Buffers the high half of the new value
  UART2_BDL = (uint8_t)setting.s.Lo;			// Reset to a nonzero value (fraction of 4/32)

  UART2_C2 |= UART_C2_RE_MASK;  //Enabling the Receiver Enable bit
  UART2_C2 |= UART_C2_TE_MASK;  //Enabling the Transmitter Enable bit

  //interrupts
  UART2_C2 |= UART_C2_RIE_MASK; // Enable receive interrupt
  UART2_C2 |= UART_C2_TIE_MASK; // Enable transmit interrupt

  NVICICPR1 = (1<<(49 % 32));   // Clear any pending error status sources interrupts on UART2
  NVICISER1 = (1<<(49 % 32));   // Enable error status sources interrupts from UART2

  FIFO_Init(&RxFIFO);  //Initializing the RxFIFO
  FIFO_Init(&TxFIFO);  //Initializing the TxFIFO

  return (setting.l != 0);
}

bool UART_InChar(uint8_t * const dataPtr)
{
  return FIFO_Get(&RxFIFO, dataPtr);
}

bool UART_OutChar(const uint8_t data)
{
  //UART2_C2 &= ~UART_C2_TIE_MASK;         //Disable the transmit interrupt
  bool result = FIFO_Put(&TxFIFO, data);
  //UART2_C2 |= UART_C2_TIE_MASK;          //Enable the transmit interrupt
  return result;
}

void UART_Poll(void)
{
  if (UART2_S1 & UART_S1_TDRE_MASK)
    FIFO_Get(&TxFIFO, &UART2_D); //sending

  if (UART2_S1 & UART_S1_RDRF_MASK)
    FIFO_Put(&RxFIFO, UART2_D); // receiving
}

/*! @brief UART transmit thread
 *  @return void
 */
void TxThread()
{
  uint8_t txData;
  // FIFO_Get (&TxFIFO, UART2_D);

  for (;;)
  {
    OS_SemaphoreWait (TxSemaphore, 0);       //Semaphore to wait for the transmit

    FIFO_Get(&TxFIFO, &txData);              //Obtains bytes from txFIFO

    UART2_D = txData;                        //Write into UART2_D register

    UART2_C2 |= UART_C2_TIE_MASK;           //Enable TIE MASK
  }
}

/*! @brief UART receive thread
 *  @return void
 */
void RxThread()
{
  for (;;)
  {
    OS_SemaphoreWait(RxSemaphore, 0);       //Semaphore to wait for the receive

    FIFO_Put(&RxFIFO, UART2_D);             //Puts byte from UART2_D into RxFifo

    //OS_SemaphoreSignal(PacketSemaphore);    // Semaphore signals the packet semaphore

    UART2_C2 |= UART_C2_RIE_MASK;           // Enable RIE MASK for receive
  }
}

void __attribute__ ((interrupt)) UART_ISR(void)
{
  OS_ISREnter();

  uint8_t temp = 0;
  // Receive a character
  if (UART2_C2 & UART_C2_RIE_MASK)       //Check if receive flag is set
  {
    if (UART2_S1 & UART_S1_RDRF_MASK)    //Clear RDRF flag by reading the status register
    {
      OS_SemaphoreSignal(RxSemaphore);   //Semaphore signals the receive

      UART2_C2 &= ~UART_C2_RIE_MASK;     //Disable RIE after bytes have been received
    }
  }

  // Transmit a character
  if (UART2_C2 & UART_C2_TIE_MASK)      //Check if tranmit flag is set
  {
    if (UART2_S1 & UART_S1_TDRE_MASK)      //Clear TDRE flag by reading the status register
    {
      OS_SemaphoreSignal(TxSemaphore);   //Semaphore signals the receive

      UART2_C2 &= ~UART_C2_TIE_MASK;    //Disable TCIE after bytes have been transmitted
    }
  }

  OS_ISRExit();

}
