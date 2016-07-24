
#ifndef _BUFFERED_PRINT_H_
#define _BUFFERED_PRINT_H_

#include <Arduino.h>
#include <ChibiOS_ARM.h>


class ChibiOSBufferedPrint : public Print 
{
  public:


  ChibiOSBufferedPrint();
  ChibiOSBufferedPrint(Print *print_dst);

  /** @brief Used by the Arduino Print class.
   * @param[in] str Pointer to the string.
   * @return count of characters written for success or -1 for failure.
   */
  size_t write(const char* str);

  /** @brief  Write a single byte.
   *  @param[in] b The byte to be written.
   *  @return +1 for success or -1 for failure.
   */
  size_t write(uint8_t b);

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
  size_t write(const void* buf, size_t nbyte);


  void runSerialSender();
  
  private:

    Print *print_dst;

    static const char STRINGS_SIZE = 200;
    static const char MEMPOOL_SIZE = 200;
    static const char FIFO_SIZE = 200;

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
    SEMAPHORE_DECL(fifoSpace, FIFO_SIZE);

    int fifoHead = 0;
    int fifoTail = 0;
    char fifoArray[FIFO_SIZE];

};

#endif

