
#ifndef _BUFFERED_PRINT_H_
#define _BUFFERED_PRINT_H_

#include <Arduino.h>
#include <ChibiOS_ARM.h>


template<int fifo_size>
class ChBufferedPrint : public Print 
{
  public:

/**
 * @brief Constructor for a buffered print object
 * @details This class provides a buffered printer through a thread that empty a fifo
 * 
 * @param[in] print_dst A print object that will be used by the thread (ex:Serial)
 * @param[in] wa The working area of the thread
 * @param[in] swa The size of the working area
 * @param[in] p The priority of the thread
 * @param[in] name The name of the thread (for debugging)
 * 
 * @note Buffered print must be started via the start() method.
 *       Attention, the write methods are not thread safe !
 */
    ChBufferedPrint(Print *print_dst, void *wa, size_t swa, tprio_t p = NORMALPRIO, const char *name = 0)
    :Print(), print_dst(print_dst)
    {
        chSysLock();
        printer_th = chThdCreateI(wa, swa, p, &threadCallback, this);
        printer_th->p_name = name;
        chSysUnlock();
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

  /**
   * @brief     Start the thread that print the fifo buffer
   * 
   * @return    A pointer over the started thread
   * 
   */
    thread_t *start() {
      chSysLock();
      chSchWakeupS(printer_th, MSG_OK);
      chSysUnlock();

      return printer_th;
    }

  
  private:

    Print *print_dst;

    SEMAPHORE_DECL(fifoData, 0);

    // count of free buffers in fifo
    SEMAPHORE_DECL(fifoSpace, fifo_size);

    int fifoHead = 0;
    int fifoTail = 0;
    char fifoArray[fifo_size];

    thread_t *printer_th;


    static void threadCallback(void *arg) {
      return ((ChBufferedPrint *) arg)->runSerialSender();
    }

    void runSerialSender()
    {
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

};

#endif

