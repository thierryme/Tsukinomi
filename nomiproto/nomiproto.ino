// Data logger based on a FIFO to decouple SD write latency from data
// acquisition timing.
//
// The FIFO uses two semaphores to synchronize between tasks.
#define FLASH_DEBUG

#include <SPI.h>

#include <ChibiOS_ARM.h>
#include <SdFat.h>

// MPU9150 sensor libs
#include <Wire.h>
#include "I2Cdev.h"
#include "RTIMUSettings.h"
#include "RTIMU.h"
#include "RTFusionRTQF.h" 
#include "CalLib.h"


// Logging lib
#include "VcdLog.h"

#include "BufferedPrint.h"



RTIMU *imu;                                           // the IMU object
RTFusionRTQF fusion;                                  // the fusion object
RTIMUSettings settings;                               // the settings object
CALLIB_DATA calData;                                  // the calibration data


//VcdLog logger;
const int ANALOG = 1;
const int DIGITAL = 0;
const char AX_ID = '+';
const char AY_ID = '"';
const char AZ_ID = '*';

const char TH_IMU_ID = 'i';
const char TH_MON_ID = 'm';
const char TH_MAIN_ID = 'M';


//
// interval between points in units of 1000 usec
const uint16_t intervalTicks = 50;


enum DataType {INT};

typedef struct SensorsEntry_t
{
  DataType type;
  uint32_t msec;
  MUTEX_DECL(lock);
  volatile void* val;
}SensorsEntry_t; 

SensorsEntry_t imu_th_data_container;
volatile int imu_val;



// const char STRINGS_SIZE = 30;
// const char STR_MEMPOOL_SIZE = 20;

// //strings memory pool buffer for UART and SD card
// char str_mempool_buffer[STR_MEMPOOL_SIZE][STRINGS_SIZE];

// // memory pool structure
// MEMORYPOOL_DECL(str_mempool, STR_MEMPOOL_SIZE, 0);

// // slots for mailbox messages
// msg_t serial_mailbox_storage[STR_MEMPOOL_SIZE];

// // mailbox structure
// MAILBOX_DECL(serial_mailbox, &serial_mailbox_storage, STR_MEMPOOL_SIZE);

//------------------------------------------------------------------------------
// SD file definitions
// const uint8_t sdChipSelect = SS;
// SdFat sd;
// SdFile file;
//------------------------------------------------------------------------------

ChibiOSBufferedPrint BuffPrint(&Serial);
VcdLog Logger(&BuffPrint);


void systemHaltCallback(const char* reason)
{
  Serial.print("HALT: ");
  Serial.println(reason);
  int offset = ch.dbg.trace_buffer.tb_ptr - ch.dbg.trace_buffer.tb_buffer;
  for(int i=0;i<ch.dbg.trace_buffer.tb_size;i++)
  {
    Serial.print(ch.dbg.trace_buffer.tb_buffer[((ch.dbg.trace_buffer.tb_size-i-1)+offset)%ch.dbg.trace_buffer.tb_size].se_time);
    Serial.print(":");
    Serial.println(ch.dbg.trace_buffer.tb_buffer[((ch.dbg.trace_buffer.tb_size-i-1)+offset)%ch.dbg.trace_buffer.tb_size].se_tp->p_name);
  }
}

void contextSwitchCallback(thread_t *ntp, thread_t *otp)
{
}

void threadInitCallback(thread_t *tp)
{
  //Serial.println("thInit");
}

//------------------------------------------------------------------------------
// 64 byte stack beyond task switch and interrupt needs
static THD_WORKING_AREA(waImuSensTh, 256);

static THD_FUNCTION(imuSensorTh, name) {
  // index of record to be filled
  size_t fifoHead = 0;

  chRegSetThreadName((char*)name);

  imu_th_data_container.val = &imu_val;

  while (1) {

    chThdSleep(intervalTicks);

    
    // // get object from memory pool
    // char* msg = (char*)chPoolAlloc(&str_mempool);
    // if (!msg) {
    //   Serial.println("chPoolAlloc failed");
    //   while(1);
    // }


    // // chSysLock();
    // //imu->IMURead();

    // //form msg
    // strcpy(msg, "Toto et tata\n");

    // (logger.toVcdVal(AX_ID, (float)imu->getAccel().x()) + "\n"\
    //        + logger.toVcdVal(AY_ID, (float)imu->getAccel().y())  + "\n"\
    //        + logger.toVcdVal(AZ_ID, (float)imu->getAccel().z())  + "\n").c_str();

    // // send message

    // msg_t s = chMBPost(&serial_mailbox, (msg_t)msg, TIME_IMMEDIATE);
    // if (s != MSG_OK) {
    //   Serial.println("chMBPost failed");
    //   while(1);  
    // }

    //
    //BuffPrint.println("Toto");
    int tmps = millis();
    imu_th_data_container.msec = tmps;
    *((int*)imu_th_data_container.val) = tmps;


  }
}



static THD_WORKING_AREA(waMonitorTh, 256);

static THD_FUNCTION(monitorTh, name)
{

  chRegSetThreadName((char*)name);


  for(;;)
  {
    //char *msg;

    // int offset = ch.dbg.trace_buffer.tb_ptr - ch.dbg.trace_buffer.tb_buffer;
    // for(int i=0;i<ch.dbg.trace_buffer.tb_size;i++)
    // {
    //   Serial.print(ch.dbg.trace_buffer.tb_buffer[((ch.dbg.trace_buffer.tb_size-i-1)+offset)%ch.dbg.trace_buffer.tb_size].se_time);
    //   Serial.print(":");
    //   Serial.println(ch.dbg.trace_buffer.tb_buffer[((ch.dbg.trace_buffer.tb_size-i-1)+offset)%ch.dbg.trace_buffer.tb_size].se_tp->p_name);
    // }

    // // get mail
    // chMBFetch(&serial_mailbox, (msg_t*)&msg, TIME_INFINITE);

    // // Serial.print(logger.toVcdTime(p->msec));
    // // Serial.print(*(int*)p->val);

    // Serial.print(msg);

    // // put memory back into pool
    // chPoolFree(&str_mempool, msg);
    Logger.printSignal(0,*((int*)imu_th_data_container.val),imu_th_data_container.msec);
    chThdSleep(200);

  }
}

static THD_WORKING_AREA(waSerialOut, 256);

static THD_FUNCTION(serialOutTh, name)
{

  chRegSetThreadName((char*)name);


  for(;;)
  {

    BuffPrint.runSerialSender();

  }
}

//------------------------------------------------------------------------------
void setup() 
{

  // Serial -------------------------------------------------------------------------------
  Serial.begin(115200);
  // wait for USB Serial
  while (!Serial);

  // I2C for IMU sensor
  Wire.begin();

  // // fill pool with SerialPoolObject array
  // for (size_t i = 0; i < STR_MEMPOOL_SIZE; i++) {
  //   chPoolFree(&str_mempool, &str_mempool_buffer[i]);
  // }

  // VCDlogger
  // logger.addSignal(AX_ID, "ax", ANALOG);
  // logger.addSignal(AY_ID, "ay", ANALOG);
  // logger.addSignal(AZ_ID, "az", ANALOG);

  // logger.addSignal(TH_IMU_ID, "imu", DIGITAL);
  // logger.addSignal(TH_MON_ID, "mon", DIGITAL);
  // logger.addSignal(TH_MAIN_ID, "main", DIGITAL);
  Logger.addSignal('"', "th_context", VL_DIGITAL);


  // IMU ----------------------------------------------------------------------------------
  imu = RTIMU::createIMU(&settings);                 // create the imu object

  Serial.println("Good morning");

  // int errcode;
  // if ((errcode = imu->IMUInit()) < 0) {
  //     //Serial.print("Failed to init IMU: "); //Serial.println(errcode);
  // }
  // if (!imu->getCalibrationValid())
  // {
  //   //Serial.print("ArduinoIMU starting using device "); //Serial.println(imu->IMUName());
  //   //Serial.println("No valid compass calibration data");

  //    //IMU compass sensor calibration -----------------------------------------------------
  //   //Serial.println("ArduinoMagCal starting");
  //   //Serial.println("After the calibration started, enter s to save current data to EEPROM");
  //   //Serial.println(F("To start the calibration type any character"));
  //   while(!Serial.available()); 
   
  //   imu->setCalibrationMode(true);                     // make sure we get raw data

  //   calData.magValid = false;
  //   for (int i = 0; i < 3; i++) {
  //     calData.magMin[i] = 10000000;                    // init mag cal data
  //     calData.magMax[i] = -10000000;
  //   }
  //   while(true)
  //   {  
  //     boolean changed;
  //     RTVector3 mag;
      
  //     if (imu->IMURead()) {                                 // get the latest data
  //       changed = false;
  //       mag = imu->getCompass();
  //       for (int i = 0; i < 3; i++) {
  //         if (mag.data(i) < calData.magMin[i]) {
  //           calData.magMin[i] = mag.data(i);
  //           changed = true;
  //         }
  //         if (mag.data(i) > calData.magMax[i]) {
  //           calData.magMax[i] = mag.data(i);
  //           changed = true;
  //         }
  //       }
     
  //       if (changed) {
  //         //Serial.println("-------");
  //         //Serial.print("minX: "); //Serial.print(calData.magMin[0]);
  //         //Serial.print(" maxX: "); //Serial.print(calData.magMax[0]); //Serial.println();
  //         //Serial.print("minY: "); //Serial.print(calData.magMin[1]);
  //         //Serial.print(" maxY: "); //Serial.print(calData.magMax[1]); //Serial.println();
  //         //Serial.print("minZ: "); //Serial.print(calData.magMin[2]);
  //         //Serial.print(" maxZ: "); //Serial.print(calData.magMax[2]); //Serial.println();
  //       }
  //     }
      
  //     if (Serial.available()) {
  //       if (Serial.read() == 's') {                  // save the data
  //         calData.magValid = true;
  //         calLibWrite(0, &calData);
  //         //Serial.print("Mag cal data saved for device "); //Serial.println(imu->IMUName());
  //         break;
  //       }
  //     }
  //   }
  // }


  // throw away input
  while (Serial.available()) {
    Serial.read();
    delay(10);
  }

  //Serial.print(logger.getHeader()); //Print the header of the VCD file
  
  // start kernel
  chHooksInit();
  chHooksSetContextSwitchHook(contextSwitchCallback);
  chHooksSetSystemHaltHook(systemHaltCallback);
  chHooksSetThreadInitHook(threadInitCallback);
  chBegin(mainThread);
  while(1);
}
//------------------------------------------------------------------------------
// main thread runs at NORMALPRIO
void mainThread()
{

  chRegSetThreadName("main");
  // start producer thread
  chThdCreateStatic(waImuSensTh, sizeof(waImuSensTh), NORMALPRIO + 1, imuSensorTh, (void*)"imu");  


  // start consumer thread
  chThdCreateStatic(waMonitorTh, sizeof(waMonitorTh), LOWPRIO + 1, monitorTh, (void*)"mon");
  chThdCreateStatic(waSerialOut, sizeof(waSerialOut), LOWPRIO + 1, serialOutTh, (void*)"serial");

  //chThdSleep(TIME_INFINITE);
  Logger.printHeader();
  chThdSleep(1000);

  while(1)
  {
    chThdSleep(400);
    //Logger.printSignal(0,true,millis());
    //BuffPrint.println("Coucou c'est le main je fais une string lonnnnnnnnnnguuuuuuue !!!");
    chThdSleep(20);
  }
}
//------------------------------------------------------------------------------
void loop() {
  // not used
}
