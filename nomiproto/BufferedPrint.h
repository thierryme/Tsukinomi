
#ifndef _BUFFERED_PRINT_H_
#define _BUFFERED_PRINT_H_

#include <Arduino.h>
#include <ChibiOS_ARM.h>


template<int fifo_size>
class ChibiOSBufferedPrint : public Print 
{
  public:


  ChibiOSBufferedPrint()
  :Print(), print_dst(&Serial)
  {
    //  // fill pool with SerialPoolObject array
    // for (int i = 0; i < MEMPOOL_SIZE; i++) {
    //   chPoolFree(&mempool, &mempool_buffer[i]);
  }

  ChibiOSBufferedPrint(Print *print_dst)
  :Print(), print_dst(print_dst)
  {
    //  // fill pool with SerialPoolObject array
    // for (int i = 0; i < MEMPOOL_SIZE; i++) {
    //   chPoolFree(&mempool, &mempool_buffer[i]);
    // }
  }

  /** @brief Used by the Arduino Print class.
   * @param[in] str Pointer to the string.
   * @return count of characters written for success or -1 for failure.
   */
  size_t write(const char* str)
  {
    return write(str, strlen(str));
  }

  /** @brief  Write a single byte.
   *  @param[in] b The byte to be written.
   *  @return +1 for success or -1 for failure.
   */
  size_t write(uint8_t b)
  {
    return write(&b, 1);
  }

/**
 * @brief     Provides a buffered write
 * @details   Override Print class write method
 * 
 * @param[in] buf     bytes to be send
 * @param[in] nbyte   numbers of bytes
 * 
 * @return        number of bytes send
 * 
 * @note this function use the mailbox in a sub optimal way when used to send just one byte
 */
  size_t write(const void* buf, size_t nbyte)
  {

  int nb_bytes_send = 0;
  char *msg, *received_data;
  size_t i;

  received_data = (char *)buf;

  for(i=0;i<nbyte;i++)
  {
    //get a byte
    chSemWait(&fifoSpace);
    fifoArray[fifoHead] = received_data[i];

     // signal new data
    chSemSignal(&fifoData);

    // advance FIFO index
    fifoHead = fifoHead < (fifo_size - 1) ? fifoHead + 1 : 0;
  }

  return i;
}


  void runSerialSender()
  {
    char *msg;
    char last_char = 'A';
    int i;

    while(1)
    {
      // wait for next data point
      chSemWait(&fifoData);

      
      print_dst->write(fifoArray[fifoTail]);

       // release record
      chSemSignal(&fifoSpace);
      
      // advance FIFO index
      fifoTail = fifoTail < (fifo_size - 1) ? fifoTail + 1 : 0;
    }
  }


  
  private:

    Print *print_dst;

    // static const char STRINGS_SIZE = 200;
    // static const char MEMPOOL_SIZE = 200;
    //static const char FIFO_SIZE = fifo_size;

    // //strings memory pool buffer for UART and SD card
    // char mempool_buffer[MEMPOOL_SIZE][STRINGS_SIZE];

    // // memory pool structure
    // MEMORYPOOL_DECL(mempool, MEMPOOL_SIZE, 0);

    // // slots for mailbox messages
    // msg_t mailbox_storage[MEMPOOL_SIZE];

    // // mailbox structure
    // MAILBOX_DECL(mailbox, &mailbox_storage, MEMPOOL_SIZE);

    // count of data records in fifo
    SEMAPHORE_DECL(fifoData, 0);

    // count of free buffers in fifo
    SEMAPHORE_DECL(fifoSpace, fifo_size);

    int fifoHead = 0;
    int fifoTail = 0;
    char fifoArray[fifo_size];

};

#endif

