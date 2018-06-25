/*!
 * @file <packet.c>
 *
 * @brief
 *         packet module.
 *         This module contains the code for managing incoming and outgoing packets
 *
 *@author Corey Stidston & Menka Mehta
 * @date 2017-03-29
 */
/*!
 * @addtogroup packet_module packet documentation
 * @{
 */

/****************************************HEADER FILES****************************************************/
#include "packet.h"
#include "UART.h"
#include "MK70F12.h"
#include "types.h"
#include "LEDs.h"
#include "Flash.h"
#include "PE_Types.h"
#include "Cpu.h"

/****************************************GLOBAL VARS*****************************************************/

TPacket Packet;

uint8_t packet_position = 0;  //Used to mark the position of incoming bytes

const uint8_t PACKET_ACK_MASK = 0x80u; //Used to mask out the Acknowledgment bit

/****************************************PRIVATE FUNCTION DECLARATION***********************************/

bool PacketTest(void);

/****************************************PRIVATE FUNCTION DEFINITION***************************************/

/*! @brief Handles the stored packet
 *
 *  @return bool - True if the calculated checksum is equal to the packet checksum
 */
bool PacketTest(void)
{
  uint8_t calculated_checksum = Packet_Command ^ Packet_Parameter1 ^ Packet_Parameter2 ^ Packet_Parameter3;
  return (calculated_checksum == Packet_Checksum);
}

/****************************************PUBLIC FUNCTION DEFINITION***************************************/

/*! @brief Initializes the packets by calling the initialization routines of the supporting software modules.
 *
 *  @param baudRate The desired baud rate in bits/sec.
 *  @param moduleClk The module clock rate in Hz
 *  @return bool - TRUE if the packet module was successfully initialized.
 */
bool Packet_Init(const uint32_t baudRate, const uint32_t moduleClk)
{
  PacketPutSemaphore = OS_SemaphoreCreate(1); //Create Packet Semaphore

  return (UART_Init(baudRate, moduleClk));
}

/*! @brief Attempts to get a packet from the received data.
 *
 *  @return bool - TRUE if a valid packet was received.
 */
bool Packet_Get(void) {
  //EnterCritical();
  uint8_t uartData;

  //Checks whether there is data in the RxFIFO and stores it the address pointed by uartData
  UART_InChar(&uartData);
  switch (packet_position)
  {
    //Command byte
    case 0:
      Packet_Command = uartData;
      packet_position++;
      //ExitCritical();
      return false; //Return false, incomplete packet
      break;

      //Parameter1 byte
    case 1:
      Packet_Parameter1 = uartData;
      packet_position++;
      //ExitCritical();
      return false; //Return false, incomplete packet
      break;

      //Parameter2 byte
    case 2:
      Packet_Parameter2 = uartData;
      packet_position++;
      ExitCritical();
      return false; //Return false, incomplete packet
      break;

      //Parameter3 byte
    case 3:
      Packet_Parameter3 = uartData;
      packet_position++;
      //ExitCritical();
      return false; //Return false, incomplete packet
      break;

      //Checksum byte
    case 4:
      Packet_Checksum = uartData;

      if (PacketTest())
      {
        packet_position = 0;
        //ExitCritical();
        return true; //Return true, complete packet
      }
      //The Checksum doesn't match
      //Shift the packets down
      Packet_Command = Packet_Parameter1;
      Packet_Parameter1 = Packet_Parameter2;
      Packet_Parameter2 = Packet_Parameter3;
      Packet_Parameter3 = Packet_Checksum;
      packet_position = 0;
      //ExitCritical();
      return false;
      break;

    default: //reset
      packet_position = 0;
      //ExitCritical();
      break;
  }

  // ExitCritical();
  return false;
}

/*! @brief Builds a packet and places it in the transmit FIFO buffer.
 *
 *  @return bool - TRUE if a valid packet was sent.
 */
void Packet_Put(const uint8_t command, const uint8_t parameter1, const uint8_t parameter2, const uint8_t parameter3)
{
  OS_SemaphoreWait(PacketPutSemaphore, 0); //Wait on Packet Put Semaphore

  UART_OutChar(command); //Place Command byte in TxFIFO
  UART_OutChar(parameter1); //Place Parameter1 byte in TxFIFO
  UART_OutChar(parameter2); //Place Parameter2 byte in TxFIFO
  UART_OutChar(parameter3); //Place Parameter3 byte in TxFIFO
  UART_OutChar(command ^ parameter1 ^ parameter2 ^ parameter3); //Place Checksum byte in TxFIFO

  OS_SemaphoreSignal(PacketPutSemaphore); //Signal Packet Put Semaphore
}

/*!
 * @}
 */
