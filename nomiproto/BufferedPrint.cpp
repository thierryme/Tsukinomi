
#include "BufferedPrint.h"
#include <Arduino.h>

ChibiOSBufferedPrint::ChibiOSBufferedPrint()
:Print(), print_dst(&Serial)
{
  //  // fill pool with SerialPoolObject array
  // for (int i = 0; i < MEMPOOL_SIZE; i++) {
  //   chPoolFree(&mempool, &mempool_buffer[i]);
  // }
}

ChibiOSBufferedPrint::ChibiOSBufferedPrint(Print *print_dst)
:Print(), print_dst(print_dst)
{
  //  // fill pool with SerialPoolObject array
  // for (int i = 0; i < MEMPOOL_SIZE; i++) {
  //   chPoolFree(&mempool, &mempool_buffer[i]);
  // }
}

size_t ChibiOSBufferedPrint::write(const char* str) {
    return write(str, strlen(str));
  }

size_t ChibiOSBufferedPrint::write(uint8_t b) {
    return write(&b, 1);
  }


size_t ChibiOSBufferedPrint::write(const void* buf, size_t nbyte)
{

  int nb_bytes_send = 0;
  char *msg, *received_data;
  size_t i;

  received_data = (char *)buf;


  // for(i=0;i<nbyte;i++)
  // {
  //   if(nb_bytes_send==0)
  //   {
  //     // get object from memory pool
  //     msg = (char*)chPoolAlloc(&mempool);
  //     if (!msg) {
  //       return 0;
  //     }
  //   }

  //   //form msg
  //   msg[nb_bytes_send] = received_data[i];
  //   nb_bytes_send++;

  //   if(nb_bytes_send == STRINGS_SIZE-1)
  //   {
  //     msg_t s = chMBPost(&mailbox, (msg_t)msg, TIME_INFINITE);

  //     // if (s != MSG_OK) {
  //     //   return 0;
  //     // }

  //     nb_bytes_send = 0;
  //   }
  // }


  // msg[nb_bytes_send] = '\0';
  // msg_t s = chMBPost(&mailbox, (msg_t)msg, TIME_INFINITE);

  // if (s != MSG_OK) {
  //   return 0;
  // }

  for(i=0;i<nbyte;i++)
  {
    //get a byte
    chSemWait(&fifoSpace);
    fifoArray[fifoHead] = received_data[i];

     // signal new data
    chSemSignal(&fifoData);

    // advance FIFO index
    fifoHead = fifoHead < (FIFO_SIZE - 1) ? fifoHead + 1 : 0;
  }

  return i;
}


void ChibiOSBufferedPrint::runSerialSender()
{
  char *msg;
  char last_char = 'A';
  int i;

  // while(last_char!='\0')
  // {

  //   chMBFetch(&mailbox, (msg_t*)&msg, TIME_INFINITE);

  //   for(i = 0; i<STRINGS_SIZE && msg[i] != '\0';i++)
  //   {
  //     print_dst->write(msg[i]);
  //   }
  //   last_char = msg[i];

  //   // put memory back into pool
  //   chPoolFree(&mempool, msg);
  // }

  while(1)
  {
    // wait for next data point
    chSemWait(&fifoData);

    
    print_dst->write(fifoArray[fifoTail]);

     // release record
    chSemSignal(&fifoSpace);
    
    // advance FIFO index
    fifoTail = fifoTail < (FIFO_SIZE - 1) ? fifoTail + 1 : 0;
  }
}