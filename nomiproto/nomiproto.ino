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


RTIMU *imu;                                           // the IMU object
RTFusionRTQF fusion;                                  // the fusion object
RTIMUSettings settings;                               // the settings object
CALLIB_DATA calData;                                  // the calibration data


VcdLog logger;
const int ANALOG = 1;
const int DIGITAL = 0;
const char AX_ID = '+';
const char AY_ID = '"';
const char AZ_ID = '*';


//
// interval between points in units of 1000 usec
const uint16_t intervalTicks = 50;
//------------------------------------------------------------------------------
// SD file definitions
// const uint8_t sdChipSelect = SS;
// SdFat sd;
// SdFile file;
//------------------------------------------------------------------------------
// Fifo definitions

// size of fifo in records
const size_t FIFO_SIZE = 200;

// count of data records in fifo
SEMAPHORE_DECL(fifoData, 0);

// count of free buffers in fifo
SEMAPHORE_DECL(fifoSpace, FIFO_SIZE);

// data type for fifo item
struct FifoItem_t {
  unsigned long msec;  
  RTVector3 accel;
  //long accel;
  int error;
};
// array of data items
FifoItem_t fifoArray[FIFO_SIZE];

// count of overrun errors
int error = 0;

// dummy data
int count = 0;

//------------------------------------------------------------------------------
// 64 byte stack beyond task switch and interrupt needs
static THD_WORKING_AREA(waThread1, 32);

static THD_FUNCTION(imuSensorTh1, arg) {
  // index of record to be filled
  size_t fifoHead = 0;

  while (1) {

    chThdSleep(intervalTicks);
    // get a buffer
    if (chSemWaitTimeout(&fifoSpace, TIME_IMMEDIATE) != MSG_OK) {
      // fifo full indicate missed point
      error++;
      continue;
    }
    FifoItem_t* p = &fifoArray[fifoHead];
    

    imu->IMURead();
    p->msec = imu->getTimestamp();

    //fusion.newIMUData(imu->getGyro(), imu->getAccel(), imu->getCompass(), imu->getTimestamp());
    p->accel = (RTVector3&)imu->getAccel();

    p->error = error;
    error = 0;

    // signal new data
    chSemSignal(&fifoData);
    
    // advance FIFO index
    fifoHead = fifoHead < (FIFO_SIZE - 1) ? fifoHead + 1 : 0;
      
  }
}

//------------------------------------------------------------------------------
void setup() {


  // Serial -------------------------------------------------------------------------------
  Serial.begin(115200);
  // wait for USB Serial
  while (!Serial) {}

  // I2C for IMU sensor
  Wire.begin();
  

  // VCDlogger
  logger.addSignal(AX_ID, "ax", ANALOG);
  logger.addSignal(AY_ID, "ay", ANALOG);
  logger.addSignal(AZ_ID, "az", ANALOG);

  // IMU ----------------------------------------------------------------------------------
  imu = RTIMU::createIMU(&settings);                 // create the imu object

  int errcode;
  if ((errcode = imu->IMUInit()) < 0) {
      Serial.print("Failed to init IMU: "); Serial.println(errcode);
  }
  if (!imu->getCalibrationValid())
  {
    Serial.print("ArduinoIMU starting using device "); Serial.println(imu->IMUName());
    Serial.println("No valid compass calibration data");

     //IMU compass sensor calibration -----------------------------------------------------
    Serial.println("ArduinoMagCal starting");
    Serial.println("After the calibration started, enter s to save current data to EEPROM");
    Serial.println(F("To start the calibration type any character"));
    while(!Serial.available()); 
   
    imu->setCalibrationMode(true);                     // make sure we get raw data

    calData.magValid = false;
    for (int i = 0; i < 3; i++) {
      calData.magMin[i] = 10000000;                    // init mag cal data
      calData.magMax[i] = -10000000;
    }
    while(true)
    {  
      boolean changed;
      RTVector3 mag;
      
      if (imu->IMURead()) {                                 // get the latest data
        changed = false;
        mag = imu->getCompass();
        for (int i = 0; i < 3; i++) {
          if (mag.data(i) < calData.magMin[i]) {
            calData.magMin[i] = mag.data(i);
            changed = true;
          }
          if (mag.data(i) > calData.magMax[i]) {
            calData.magMax[i] = mag.data(i);
            changed = true;
          }
        }
     
        if (changed) {
          Serial.println("-------");
          Serial.print("minX: "); Serial.print(calData.magMin[0]);
          Serial.print(" maxX: "); Serial.print(calData.magMax[0]); Serial.println();
          Serial.print("minY: "); Serial.print(calData.magMin[1]);
          Serial.print(" maxY: "); Serial.print(calData.magMax[1]); Serial.println();
          Serial.print("minZ: "); Serial.print(calData.magMin[2]);
          Serial.print(" maxZ: "); Serial.print(calData.magMax[2]); Serial.println();
        }
      }
      
      if (Serial.available()) {
        if (Serial.read() == 's') {                  // save the data
          calData.magValid = true;
          calLibWrite(0, &calData);
          Serial.print("Mag cal data saved for device "); Serial.println(imu->IMUName());
          break;
        }
      }
    }
  }


  // throw away input
  while (Serial.available()) {
    Serial.read();
    delay(10);
  }

  Serial.print(logger.getHeader()); //Print the header of the VCD file
  
  // start kernel
  chBegin(mainThread);
  while(1);
}
//------------------------------------------------------------------------------
// main thread runs at NORMALPRIO
void mainThread() {
  // FIFO index for record to be written
  size_t fifoTail = 0;

  // time in micros of last point
  uint32_t last = 0;

  // remember errors
  bool overrunError = false;


  // start producer thread
  chThdCreateStatic(waThread1, sizeof(waThread1), NORMALPRIO + 1, imuSensorTh1, NULL);  

  // start Serial write loop
  while (!Serial.available()) {
    // wait for next data point
    chSemWait(&fifoData);

    FifoItem_t* p = &fifoArray[fifoTail];
    if (fifoTail >= FIFO_SIZE) fifoTail = 0;

    Serial.print(logger.toVcdTime(p->msec));
    Serial.print(logger.toVcdVal(AX_ID, p->accel.x()));
    Serial.print(logger.toVcdVal(AY_ID, p->accel.y()));
    Serial.print(logger.toVcdVal(AZ_ID, p->accel.z()));

    // remember error
    if (p->error) overrunError = true;

    // release record
    chSemSignal(&fifoSpace);
    
    // advance FIFO index
    fifoTail = fifoTail < (FIFO_SIZE - 1) ? fifoTail + 1 : 0;
  }

  Serial.println(F("Done"));
  Serial.print(F("imuSensorTh1 unused stack: "));
  Serial.println(chUnusedStack(waThread1, sizeof(waThread1)));
  Serial.print(F("Heap/Main unused: "));
  Serial.println(chUnusedHeapMain());
  if (overrunError) {
    Serial.println();
    Serial.println(F("** overrun errors **"));
  }
  while(1);
}
//------------------------------------------------------------------------------
void loop() {
  // not used
}
