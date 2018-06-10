/*! @file
 *
 *  @brief Routines to implement a FIFO buffer.
 *
 *  This contains the implementation for accessing a byte-wide FIFO.
 *
 *  @author 11989668, 13113117
 *  @date 2018-04-04
 */
#include "FIFO.h"

void FIFO_Init(TFIFO * const FIFO)
{
  FIFO->BufferAccess = OS_SemaphoreCreate(1);
  FIFO->SpaceAvailable = OS_SemaphoreCreate(FIFO_SIZE);
  FIFO->ItemsAvailable = OS_SemaphoreCreate(0);

  FIFO->Start = 0;
  FIFO->End = 0;
  FIFO->NbBytes = 0;
}

bool FIFO_Put(TFIFO * const FIFO, const uint8_t data)
{
  OS_SemaphoreWait(FIFO->SpaceAvailable, 0);          //Wait until there is space available
  OS_SemaphoreWait(FIFO->BufferAccess, 0);            //Wait for exclusive buffer acces

  FIFO->NbBytes++;
  FIFO->Buffer[FIFO->End] = data;
  FIFO->End = (FIFO->End +1) % FIFO_SIZE;             // Cycle to the beginning if we reached the end

  OS_SemaphoreSignal(FIFO->BufferAccess);             //Frees the acces to the buffer
  OS_SemaphoreSignal(FIFO->ItemsAvailable);           //Increments number of available items in buffer

  return true;
}

bool FIFO_Get(TFIFO * const FIFO, uint8_t * const dataPtr)
{
  OS_SemaphoreWait(FIFO->ItemsAvailable, 0);      //Wait until there are items available
  OS_SemaphoreWait(FIFO->BufferAccess, 0);        //Wait for exclusive buffer acces

  *dataPtr = FIFO->Buffer[FIFO->Start];
  FIFO->Start = (FIFO->Start +1) % FIFO_SIZE;     // Cycle to the end if we reached the beginning
  FIFO->NbBytes--;

  OS_SemaphoreSignal(FIFO->BufferAccess);         //Frees the acces to the buffer
  OS_SemaphoreSignal(FIFO->SpaceAvailable);       //Increments number of available spaces in buffer

  return true;
}
