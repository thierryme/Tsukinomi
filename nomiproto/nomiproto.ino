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



//
// interval between points in units of 1000 usec
const uint16_t intervalTicks = 50;


enum DataType {INT, CHAR};

typedef struct SensorsEntry_t
{
  DataType type;
  uint32_t msec;
  MUTEX_DECL(lock);
  void* val;
}SensorsEntry_t; 

SensorsEntry_t imu_container;
RTVector3 imu_val;



//------------------------------------------------------------------------------
// SD file definitions
const uint8_t sdChipSelect = 4;
SdFat sd;
SdFile file;
//------------------------------------------------------------------------------



// - Loggers -----------------------------------------------------------------------------
int id_serial; 
static THD_WORKING_AREA(waSerialOut, 256);

ChBufferedPrint<20> BuffPrint(&Serial, waSerialOut, sizeof(waSerialOut), LOWPRIO + 1, "th_serial");
VcdLog Logger(__DATE__ " " __TIME__, "VcdLog V0.1 - Thread context switch log", "1 ms", &BuffPrint);

int id_sd;
static THD_WORKING_AREA(waSdOut, 256);

ChBufferedPrint<20> BuffPrintSd(&file, waSdOut, sizeof(waSdOut), LOWPRIO + 1, "th_sd");
VcdLog SdLogger(__DATE__ " " __TIME__, "VcdLog V0.1 - Data flux log", "1 ms", &BuffPrintSd);


//- Threads  -----------------------------------------------------------------------------


int id_imu;
static THD_WORKING_AREA(waImuSensTh, 256);

static THD_FUNCTION(imuSensorTh, name) {
  // index of record to be filled
  size_t fifoHead = 0;

  chRegSetThreadName((char*)name);

  imu_container.val = &imu_val;

  while (1) {

   
    if(imu->IMURead())
    {
      imu_container.msec = imu->getTimestamp();;
      (*(RTVector3*)imu_container.val) = imu->getAccel();
      chThdSleep(intervalTicks);
    }


  }
}


int id_mon;
static THD_WORKING_AREA(waMonitorTh, 256);

static THD_FUNCTION(monitorTh, name)
{

  chRegSetThreadName((char*)name);

  for(;;)
  {
    SdLogger.printSignal(0,(float)((RTVector3*)imu_container.val)->x(),imu_container.msec);
    chThdSleep(200);

  }
}

int id_dbg_mon;
static THD_WORKING_AREA(waDbgMonitorTh, 256);

static THD_FUNCTION(dbgMonitorTh, name)
{

  ch_swc_event_t *last_trace = ch.dbg.trace_buffer.tb_ptr;

  chRegSetThreadName((char*)name);

  for(;;)
  {

    if(last_trace != ch.dbg.trace_buffer.tb_ptr)
    {
      ch_swc_event_t *old_trace = last_trace;
      last_trace = last_trace < (ch.dbg.trace_buffer.tb_buffer + ch.dbg.trace_buffer.tb_size - 1) ? last_trace + 1 : ch.dbg.trace_buffer.tb_buffer;

      Logger.printSignal(old_trace->se_tp->log_id, last_trace->se_state, last_trace->se_time-1);
      Logger.printSignal(last_trace->se_tp->log_id, 1, last_trace->se_time);
      // Logger.printSignal(old_trace->se_tp->log_id, false, last_trace->se_time);
      // Logger.printSignal(last_trace->se_tp->log_id, true, last_trace->se_time);
    }
    else
    {
      chThdSleep(200);
    }
  }
}

int id_idle, id_main;


//- System callbacks -----------------------------------------------------------------------------

void systemHaltCallback(const char* reason)
{
  Serial.print("HALT: ");
  Serial.println(reason);
  
  int offset = ch.dbg.trace_buffer.tb_ptr - ch.dbg.trace_buffer.tb_buffer;
  for(int i=0;i<ch.dbg.trace_buffer.tb_size;i++)
  {
    Serial.print(ch.dbg.trace_buffer.tb_buffer[((ch.dbg.trace_buffer.tb_size-i-1)+offset)%ch.dbg.trace_buffer.tb_size].se_time);
    Serial.print(":");
    Serial.print(ch.dbg.trace_buffer.tb_buffer[((ch.dbg.trace_buffer.tb_size-i-1)+offset)%ch.dbg.trace_buffer.tb_size].se_tp->log_id);
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

void idleLoopHookCallback()
{
  static bool log_idle_init = false;

  if(log_idle_init == false)
  {
    thread_t *tp = chThdGetSelfX();
    tp->log_id = id_idle;
    log_idle_init = true;
  }
}


//------------------------------------------------------------------------------
void setup() 
{

  // Serial init --------------------------------------------------------------------------
  Serial.begin(115200);
  // wait for USB Serial
  while (!Serial);


  //Set the loggers traces
  SdLogger.addSignal('+', "ax", VL_REAL);
  SdLogger.addSignal('"', "ay", VL_REAL);
  SdLogger.addSignal('*', "az", VL_REAL);
  SdLogger.addSignal('%', "sun_angle", VL_REAL);


  id_main     = Logger.addSignal('+', "th_main", VL_INT);
  id_mon      = Logger.addSignal('"', "th_mon", VL_INT);
  id_dbg_mon  = Logger.addSignal('*', "th_dbg_mon", VL_INT);
  id_serial   = Logger.addSignal('%', "th_serial", VL_INT);
  id_sd       = Logger.addSignal('&', "th_sd", VL_INT);
  id_imu      = Logger.addSignal('/', "th_imu", VL_INT);
  id_idle     = Logger.addSignal('(', "th_idle", VL_INT);

  
  // I2C for IMU sensor
  Wire.begin();

  // IMU init -----------------------------------------------------------------------------
  imu = RTIMU::createIMU(&settings);                 // create the imu object


  // SD init ------------------------------------------------------------------------------
  char filename[10] = "log00.vcd";
  int base_name_size = 3;

  // open sd
  if (!sd.begin(sdChipSelect))
  {
    sd.errorHalt();
  }

  while(sd.exists(filename)) 
  {
    if (filename[base_name_size + 1] != '9') 
    {
      filename[base_name_size + 1]++;
    } 
    else if (filename[base_name_size] != '9') 
    {
      filename[base_name_size + 1] = '0';
      filename[base_name_size]++;
    } 
    else 
    {
      chSysHalt("Can't create file name");
    }
  }

  if (!file.open(filename, O_CREAT | O_WRITE | O_TRUNC)) 
  {
    sd.errorHalt();
    chSysHalt("SD problem");
  }


  Serial.println("Good morning");


  int errcode;
  if ((errcode = imu->IMUInit()) < 0) 
  {
    Serial.print("Failed to init IMU: ");
    Serial.println(errcode);
  }
  if (!imu->getCalibrationValid())
  {
    Serial.print("ArduinoIMU starting using device ");
    Serial.println(imu->IMUName());
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
          Serial.print("minX: ");
          Serial.print(calData.magMin[0]);
          Serial.print(" maxX: ");
          Serial.print(calData.magMax[0]);
          Serial.println();

          Serial.print("minY: ");
          Serial.print(calData.magMin[1]);
          Serial.print(" maxY: ");
          Serial.print(calData.magMax[1]);
          Serial.println();

          Serial.print("minZ: ");
          Serial.print(calData.magMin[2]);
          Serial.print(" maxZ: ");
          Serial.print(calData.magMax[2]);
          Serial.println();
        }
      }
      
      if (Serial.available())
      {
        if (Serial.read() == 's')
        {                  // save the data
          calData.magValid = true;
          calLibWrite(0, &calData);
          Serial.print("Mag cal data saved for device ");
          Serial.println(imu->IMUName());
          break;
        }
      }
    }
  }



  Serial.println("Type anything to stop.");
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
  chHooksSetIdleLoopHook(idleLoopHookCallback);

  chBegin(mainThread);
  while(1);
}
//------------------------------------------------------------------------------
// main thread runs at NORMALPRIO
void mainThread()
{
  thread_t *tp;
  char *main_thread_name = "th_main";

  chRegSetThreadName(main_thread_name);
  tp = chThdGetSelfX();
  tp->log_id = id_main;


  tp = BuffPrint.start();
  tp->log_id = id_serial;

  tp = BuffPrintSd.start();
  tp->log_id = id_sd;

  //chThdSleep(TIME_INFINITE);
  Logger.printHeader();
  SdLogger.printHeader();
  chThdSleep(1000);

  // start producer threads
  tp = chThdCreateStatic(waImuSensTh, sizeof(waImuSensTh), NORMALPRIO + 1, imuSensorTh, (void*)"th_imu");
  tp->log_id = id_imu;

  tp = chThdCreateStatic(waMonitorTh, sizeof(waMonitorTh), LOWPRIO + 1, monitorTh, (void*)"th_mon");
  tp->log_id = id_mon;

  tp = chThdCreateStatic(waDbgMonitorTh, sizeof(waDbgMonitorTh), LOWPRIO + 1, dbgMonitorTh, (void*)"th_dbg_mon");
  tp->log_id = id_dbg_mon;

  while(1)
  {
    chThdSleep(200);    

    if (Serial.available()) 
    {
      if(Serial.read())
      {    
        file.close();
        chSysHalt("Exit by user");
      }
    }
  }
}
//------------------------------------------------------------------------------
void loop() {
  // not used
}
