//import nessecary libraries
#include "SparkFunLSM6DS3.h"
#include "Wire.h"
#include "SPI.h"
#include "FS.h"
#include "SD.h"

#include "RTClib.h"
#include <Preferences.h>
Preferences preferences;

//initialize I2C devices
RTC_DS3231 rtc;
LSM6DS3 myIMU(I2C_MODE, 0x6B);

//RTC variables written in Flash memory
RTC_DATA_ATTR int booter = 0;
RTC_DATA_ATTR int reseter = 0;
char fileName[10];
RTC_DATA_ATTR int saver = 0;

uint8_t int1Status = 0;
volatile bool activity_flag = true;
volatile bool reset_once = false;
uint8_t latched = 0;

//timer for reseting the motion flag
hw_timer_t * timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;




void setup( void ) {
  //Over-ride default settings if desired
  myIMU.settings.gyroEnabled = 1;  //Can be 0 or 1
  myIMU.settings.gyroRange = 2000;   //Max deg/s.  Can be: 125, 245, 500, 1000, 2000
  myIMU.settings.gyroSampleRate = 833;   //Hz.  Can be: 13, 26, 52, 104, 208, 416, 833, 1666
  myIMU.settings.gyroBandWidth = 200;  //Hz.  Can be: 50, 100, 200, 400;
  myIMU.settings.gyroFifoEnabled = 1;  //Set to include gyro in FIFO
  myIMU.settings.gyroFifoDecimation = 1;  //set 1 for on /1

  myIMU.settings.accelEnabled = 1;
  myIMU.settings.accelRange = 16;      //Max G force readable.  Can be: 2, 4, 8, 16
  myIMU.settings.accelSampleRate = 833;  //Hz.  Can be: 13, 26, 52, 104, 208, 416, 833, 1666, 3332, 6664, 13330
  myIMU.settings.accelBandWidth = 200;  //Hz.  Can be: 50, 100, 200, 400;
  myIMU.settings.accelFifoEnabled = 1;  //Set to include accelerometer in the FIFO
  myIMU.settings.accelFifoDecimation = 1;  //set 1 for on /1
  myIMU.settings.tempEnabled = 1;
  
    //Non-basic mode settings
  myIMU.settings.commMode = 1;

  //FIFO control settings
  myIMU.settings.fifoThreshold = 1200;  //Can be 0 to 4096 (16 bit bytes)
  myIMU.settings.fifoSampleRate = 200;  //Hz.  Can be: 10, 25, 50, 100, 200, 400, 800, 1600, 3300, 6600
  myIMU.settings.fifoModeWord = 6;  //FIFO mode.

  //  //Error accumulation variable
  uint8_t errorAccumulator = 0;
  uint16_t errorsAndWarnings = 0;
  uint8_t dataToWrite = 0;  //Temporary variable



  Serial.begin(115200);  // start serial for output
  delay(100); //relax...
  Serial.println("Processor came out of reset.\n");

     //get current boot number
     preferences.begin("my-app", false);
     unsigned int counter = preferences.getUInt("counter", 0);
     counter++;
     booter = counter;
     Serial.printf("Current counter value: %u\n", counter); 
     Serial.printf("Current booter value: %u\n", booter); 
     Serial.printf("Current reseter value: %u\n", reseter);
     preferences.putUInt("counter", counter);                   // Store the counter to the Preferences
     preferences.end();                                         // Close the Preferences

//start RTC
  if (! rtc.begin()) {
    Serial.println("Couldn't find RTC");
    }

//set time to the time when the code was uploaded, if it is the first boot
 if(booter == 1 ){
  rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }
    

setupSDCard();
setupIMU();

  //start timer for resetting the motion flag
  timer = timerBegin(0, 80, true);
  timerAttachInterrupt(timer, &reset_activity, true);
  timerAlarmWrite(timer, 5000000, true); //105-210 sec test/  reset activity every 10seconds (double the time because  activiy flag has to be false)
  timerAlarmEnable(timer);

        //set respective register while logging
          myIMU.writeRegister( LSM6DS3_ACC_GYRO_TAP_CFG1, 0x91 );         //if who_am_i=106
        //  myIMU.writeRegister( LSM6DS3_ACC_GYRO_TAP_CFG1, 0x01 );         //if who_am_i=105
          myIMU.writeRegister( LSM6DS3_ACC_GYRO_WAKE_UP_DUR, 0x00 );
          myIMU.writeRegister( LSM6DS3_ACC_GYRO_WAKE_UP_THS, 0x01 );
          myIMU.writeRegister( LSM6DS3_ACC_GYRO_MD1_CFG, 0x20 );
          attachInterrupt(digitalPinToInterrupt(13), int1ISR, RISING); 

}


void setupSDCard(){
  if(!SD.begin()){
      return;
  }
  uint8_t cardType = SD.cardType();
  if(cardType == CARD_NONE){
      return;
  }

}

void setupIMU(){
  Wire.begin();
  Wire.setClock(800000L);
  
        if( myIMU.begin() != 0 ){
          Serial.println("Problem starting the sensor");
          Serial.println(myIMU.begin());
          }
        else{
          Serial.println("IMU started.");
          }
          
        uint8_t dataToWrite = 0;                                          //Temporary variable
        myIMU.readRegister(&dataToWrite, LSM6DS3_ACC_GYRO_WHO_AM_I_REG);  //Read out version of LSM6ds3 from "WHO_AM_I" register
        Serial.println(dataToWrite);                                      //Print version - 106=LSM6ds3-c; 105=LSM6ds3

        
        Serial.println("Configuring FIFO");
        myIMU.fifoBegin();
        Serial.println("fifo Begin done");
        
        Serial.println("Clearing out the FIFO...");
        myIMU.fifoClear();
        Serial.println("Fifo clear");
        }


void int1ISR() //sets and resets the activity flag
{
  int1Status++;
  activity_flag = true;
  reset_once = false;
}

void reset_activity(){ //everytime the timer is finished, activity flag gets set to false
  if(reset_once == true){
  activity_flag = false;
  }
  reset_once = true;
}



void loop(){
          float temp;  //This is to hold read data
          uint16_t tempUnsigned;

  //get the current time        
  DateTime now = rtc.now();
  int day=now.day();int month=now.month(); int year=now.year();int hour=now.hour();int minute=now.minute();int second=now.second();
   
    //create file with boot number as name
    snprintf(fileName, 9, "/%03d.txt", booter);
    File file = SD.open(fileName ,FILE_APPEND); 
    
    file.print(reseter);file.print(". Reset_boot, ");file.print(booter);file.print(". Boot:\n"); 
    file.print("Recording started at:\n"); 
    file.print(day);file.print(".");file.print(month);file.print(".");file.print(year);file.print(" - ");
    file.print(hour);file.print(":");file.print(minute);file.print(","); file.print(second);file.print("\n ");
    file.print("Gx;Gy;Gz;X;Y;Z\n");  
    file.close();




       int counter = millis();
      // digitalWrite(15, HIGH); //if LED is connected to T3
       
       while(activity_flag == true){
        
          myIMU.fifoClear(); 
          while( ( myIMU.fifoGetStatus() & 0x8000 ) == 0 ) {};  //Wait for watermark that FIFO is full
          File file = SD.open(fileName ,FILE_APPEND);
          
          while( ( myIMU.fifoGetStatus() & 0x1000 ) == 0 ) {   //empty FIFO        
 
                temp = myIMU.calcGyro(myIMU.fifoRead());
                file.print(temp);file.print(";");
                
                temp = myIMU.calcGyro(myIMU.fifoRead());
                file.print(temp);file.print(";");

                temp = myIMU.calcGyro(myIMU.fifoRead());
                file.print(temp);file.print(";");
                
                temp = myIMU.calcAccel(myIMU.fifoRead());
                file.print(temp);file.print(";");
                
                temp = myIMU.calcAccel(myIMU.fifoRead());
                file.print(temp);file.print(";");

                temp = myIMU.calcAccel(myIMU.fifoRead()); 
                file.print(temp);file.print("\n ");
          
          }
        
          tempUnsigned = myIMU.fifoGetStatus();
          Serial.print("Outside of Fifo");
          Serial.print("\n");  
          myIMU.readRegister(&latched, LSM6DS3_ACC_GYRO_WAKE_UP_SRC);
       }
                  
                    
myIMU.fifoClear();
myIMU.fifoEnd();

file.close();
Serial.println("Going to sleep");
SD.end();

//settings to configure IMU for wake up interrupts
          timerWrite(timer, 0);
          digitalWrite(15, LOW);
          myIMU.writeRegister( LSM6DS3_ACC_GYRO_TAP_CFG1, 0x90 );         //if who_am_i=106 -> LSM6ds3-c
          //myIMU.writeRegister( LSM6DS3_ACC_GYRO_TAP_CFG1, 0x00 );         //if who_am_i=105 -> LSM6ds3
          myIMU.writeRegister( LSM6DS3_ACC_GYRO_WAKE_UP_DUR, 0x00 );
          myIMU.writeRegister( LSM6DS3_ACC_GYRO_WAKE_UP_THS, 0x01 );
          myIMU.writeRegister( LSM6DS3_ACC_GYRO_MD1_CFG, 0x20 );
          attachInterrupt(digitalPinToInterrupt(13), int1ISR, RISING);     // interrupt to wake the ESP32 when the toothbrush is used
          reseter++;
  esp_sleep_enable_ext0_wakeup(GPIO_NUM_13,1);
  esp_deep_sleep_start();
  
                    
}
