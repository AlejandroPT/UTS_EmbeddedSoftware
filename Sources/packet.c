/*! @file
 *
 *  @brief Routines to implement packet encoding and decoding for the serial port.
 *
 *  This contains the implementation of functions for implementing the "Tower to PC Protocol" 5-byte packets.
 *
 *  @author 11989668, 13113117
 *  @date 2018-04-04
 */

#include "packet.h"
#include "OS.h"

const uint8_t PACKET_ACK_MASK = 0x80;

OS_ECB *PacketPutAccessSemaphore;

//returns true if the packet passes the checksum, false otherwise
bool PacketTest(uint8_t checksum)
{
  uint8_t calc_checksum = Packet_Command ^ Packet_Parameter1 ^ Packet_Parameter2 ^ Packet_Parameter3;
  return (calc_checksum == checksum);
}

bool Packet_Init(const uint32_t baudRate, const uint32_t moduleClk)
{
  PacketPutAccessSemaphore = OS_SemaphoreCreate(1);
  return UART_Init(baudRate, moduleClk);
}

bool Packet_Get(void)
{
  EnterCritical();
  static uint8_t Position = 0;
  uint8_t uartData;
  if (!UART_InChar(&uartData)) {
    ExitCritical();
    return false;
  }
  switch (Position) {
    case 0:
      Packet_Command = uartData;
      Position++;
      ExitCritical();
      return false;
    case 1:
      Packet_Parameter1 = uartData;
      Position++;
      ExitCritical();
      return false;
    case 2:
      Packet_Parameter2 = uartData;
      Position++;
      ExitCritical();
      return false;
    case 3:
      Packet_Parameter3 = uartData;
      Position++;
      ExitCritical();
      return false;
    case 4:
      // test the checksum
      if (PacketTest(uartData)) {
        Position = 0;
        ExitCritical();
        return true;
      }
      Packet_Command = Packet_Parameter1;
      Packet_Parameter1 = Packet_Parameter2;
      Packet_Parameter2 = Packet_Parameter3;
      Packet_Parameter3 = uartData;
      ExitCritical();
      return false;
    default:
      //reset the counter
      Position = 0;
      ExitCritical();
    return false;
  }
  ExitCritical();
  return false;
}

bool Packet_Put(const uint8_t command, const uint8_t parameter1, const uint8_t parameter2, const uint8_t parameter3)
{
  //If there is an error (false) in UART_OutChar, return a false
  OS_SemaphoreWait(PacketPutAccessSemaphore, 0);
  if (!UART_OutChar(command))
  {
    OS_SemaphoreSignal(PacketPutAccessSemaphore);
    return false;
  }
  if (!UART_OutChar(parameter1))
  {
    OS_SemaphoreSignal(PacketPutAccessSemaphore);
    return false;
  }
  if (!UART_OutChar(parameter2))
  {
    OS_SemaphoreSignal(PacketPutAccessSemaphore);
    return false;
  }
  if (!UART_OutChar(parameter3))
  {
    OS_SemaphoreSignal(PacketPutAccessSemaphore);
    return false;
  }

  if (!UART_OutChar(command ^ parameter1 ^ parameter2 ^ parameter3))  // Sending the checksum
  {
    OS_SemaphoreSignal(PacketPutAccessSemaphore);
    return false;
  }

  OS_SemaphoreSignal(PacketPutAccessSemaphore);
  return true; // All sent fine
}
