// This #include statement was automatically added by the Particle IDE.
#include "sunset.h"
#include "TimeAlarms.h"
#include "credentials.h"
//#include <application.h>

//For Fabian, open = 0x01 and close = 0x00
//For Jasper, open = 0x00 and close = 0x01
#define OPEN 0x00
#define CLOSE 0x01

PRODUCT_ID(5366); // replace by your product ID
PRODUCT_VERSION(4); // increment each time you upload to the console 

unsigned long currentTime;
unsigned long lastTime = 0UL;

const int eeBase = 0x00;
const int magicNumber = 0xcafecafe;

uint8_t DirPin  = D4;
uint8_t StepPin = D3;
uint8_t SleepPin = D2;
uint32_t Steps = 312500; // For Jasper 312500, For Fabian, 250000
uint16_t StepperDelay_us = 80;
uint8_t StepperDelay = 2;

volatile uint8_t flag_request;
volatile uint8_t flag_status;

struct SETTINGS {
    int  magic;
} settings;


bool CalcFlag;
// Set in semi automatic mode for setting the wifi credentials
STARTUP(WiFi.selectAntenna(ANT_EXTERNAL))
//STARTUP(WiFi.selectAntenna(ANT_INTERNAL))
SYSTEM_MODE(SEMI_AUTOMATIC)
SYSTEM_THREAD(ENABLED)

const uint32_t msRetryDelay = 5*60000; // retry every 5min
const uint32_t msRetryTime  =   30000; // stop trying after 30sec

bool   retryRunning = false;
Timer retryTimer(msRetryDelay, retryConnect);  // timer to retry connecting
Timer stopTimer(msRetryTime, stopConnect);     // timer to stop a long running try


char str[200];

void setup()
{
  setupNetwork();
  enableNetwork();

  Particle.connect();    
  
  pinMode(DirPin, OUTPUT);
  pinMode(StepPin, OUTPUT);
  pinMode(SleepPin, OUTPUT);
  Time.zone(1);
  Serial.begin(9600);
  //Particle.syncTime();
  currentTime = now();
  
  CalcFlag = TRUE;
  //Particle.publish("Check Flag");
  flag_status = EEPROM.read(0);
  if (flag_status != CLOSE){
      flag_status = OPEN;
  }
  flag_request = flag_status;
  
  RGB.control(true);
  if (flag_status == CLOSE){
      RGB.color(RGB_COLOR_RED);
  }
  else{
      RGB.color(RGB_COLOR_GREEN);
  }
  
  // create the alarms 
  Alarm.alarmRepeat(4,0,0, SyncTime);  // 5:00am, alarm is in UTC
  Alarm.alarmRepeat(7,30,0, OpenAlarm);  // 8:30am, alarm is in UTC
  
  //Function to run from the cloud
  Particle.function("DoorControl", DoorControl);
  //particle variable to show signal quality for debug purpose
  Particle.variable("SignalQuality", str);
}


void  loop(){  
    
    if (!retryRunning && !Particle.connected())
  { // if we have not already scheduled a retry and are not connected
    Serial.println("schedule");
    stopTimer.start();         // set timeout for auto-retry by system
    retryRunning = true;
    retryTimer.start();        // schedula a retry
  }

    CalcSunsetTime();
    CheckFlagStatus();
    SetColor();

    GetVitals();

    Alarm.delay(1000);
}



void setupNetwork() {
    EEPROM.get(eeBase, settings);
    WiFi.on();

    if (settings.magic == magicNumber 
    ||  WiFi.hasCredentials()) 
        return;

    WiFi.setCredentials(SSID, PASS, WPA2, WLAN_CIPHER_AES);
    WiFi.setCredentials(AUX1_SSID, AUX1_PASS, WPA2, WLAN_CIPHER_AES_TKIP);
    
    WiFi.off();
    delay(100);

    settings.magic = magicNumber;
    EEPROM.put(eeBase, settings);
}

void enableNetwork() {
    WiFi.on();
    WiFi.connect();
    waitUntil(WiFi.ready);
    Particle.connect();   
}


// Function to get wifi strength and signal quality
void GetVitals(){
    WiFiSignal s = WiFi.RSSI();
    auto rat = s.getAccessTechnology();
    float strengthVal = s.getStrengthValue();
    float strengthPercentage = s.getStrength();
    float qualityVal = s.getQualityValue();
    float qualityPercentage = s.getQuality();
    //Covert the float values to a string
    snprintf(str, sizeof(str), "{\Strength\:\%.2f\dBm\,\:\%.2f\%%\,\Quality\:\%.2f\SNR\,\:\%.2f\%%\}", strengthVal, strengthPercentage, qualityVal, qualityPercentage);

}

void CheckFlagStatus(){
    uint8_t local_flag;
    
    ATOMIC_BLOCK(){ 
        local_flag = flag_request;
    }
        
    if (flag_status != local_flag){
        flag_status = local_flag;
        //Store the open/close flag in EEPROM
        EEPROM.write(0, local_flag);
        StepperMotor(local_flag, Steps);
    }
}

void SetColor(){
    //Particle.publish(String(flag_status));
    if (flag_status == OPEN){
        RGB.color(RGB_COLOR_GREEN);
    }
    else if (flag_status == CLOSE){
        RGB.color(RGB_COLOR_RED);
    }
}

// functions to be called when an alarm triggers:
void OpenAlarm(){
    Particle.publish("Open alarm");
    DoorControl("Open");
}

// functions to be called when an alarm triggers:
void CloseAlarm(){
    Particle.publish("Close Alarm");
    DoorControl("Close");
}

void SyncTime(){
    CalcFlag = TRUE;
            Particle.publish("Check Flag");
}

// Calculate the sunset time for my location
void CalcSunsetTime(){
    if (CalcFlag == TRUE) {
        unsigned int SunSetHours;
        unsigned int SunSetMinutes;
        /*GPS Location @home, LAT/LON*/
        double latitude = 51.77;  
        double longitude = -5.82;
        float JD;
        int SunSet;
    
        currentTime = now();
        Particle.publish("Check sunset time");
        //Particle.publish("time:", (const char*)currentTime);
        //Since time is running in GMT, only add 1 extra hour when DST is on.
        JD = calcJD(Time.year(),Time.month(),Time.day());
        
        SunSet = calcSunsetUTC( JD,  latitude,  longitude) + 90;

        SunSetHours = SunSet/60;
        SunSetMinutes = SunSet%60;
        
        Alarm.alarmRepeat(SunSetHours,SunSetMinutes,0, CloseAlarm);
        Particle.publish("Sunset calculation done!");
        
        Serial.print(SunSetHours);
        Serial.print(":");
        Serial.print(SunSetMinutes);
        CalcFlag = FALSE;
    }
}

//Particle funtion to open or close the door.
int DoorControl(String Command){
    if (Command == "Open"){
        Particle.publish("Start open the door");
        if (flag_request != OPEN){
            flag_request = OPEN;
        }
        else{
            Particle.publish("IS_OPEN");
        }
        return 1;
    }
    else if (Command == "Close"){
        Particle.publish("Start close the door");
        if (flag_request != CLOSE){
            flag_request = CLOSE;
        }
        else{
            Particle.publish("IS_CLOSED");
        }
        return 1;
    } 
    else return -1;
}



//Funtion to control the steppermotor
void StepperMotor(int Direction, int Steps){
    int i;
    //Wake up motor controller
    digitalWrite(SleepPin, HIGH);
    
    if (Direction == 1){
        digitalWrite(DirPin, LOW);
    }
    else{
      digitalWrite(DirPin, HIGH);   
    } 
        
    for (i=0; i < Steps; i++){
        digitalWrite(StepPin, HIGH);
        //delay(StepperDelay);
        delayMicroseconds(StepperDelay_us);
        digitalWrite(StepPin, LOW);
        delayMicroseconds(StepperDelay_us);
        //delay(StepperDelay);
    }
    //Put motorcontroller back in sleep
    digitalWrite(SleepPin, LOW);
}

void retryConnect()
{
  if (!Particle.connected())   // if not connected to cloud
  {
    Serial.println("reconnect");
    stopTimer.start();         // set of the timout time
    WiFi.on();
    Particle.connect();        // start a reconnectino attempt
  }
  else                         // if already connected
  {
    Serial.println("connected");
    retryTimer.stop();         // no further attempts required
    retryRunning = false;
  }
}

void stopConnect()
{
    Serial.println("stopped");

    if (!Particle.connected()) // if after retryTime no connection
      WiFi.off();              // stop trying and swith off WiFi
    stopTimer.stop();
}

/*
//This function checks the Daylight Saving Time
bool isDST()
{ // Central European Summer Timer calculation
  int dayOfMonth = Time.day();
  int month = Time.month();
  int dayOfWeek = Time.weekday() - 1; // make Sunday 0 .. Saturday 6

  if (4 <= month && month <= 9)
  { // month between April and September definetly DST
    return true;
  }
    else if (month < 3 || 10 < month)
  { // before March or after October is definetly normal time
    return false;
  }
   // March and October need deeper examination
  boolean lastSundayOrAfter = (dayOfMonth - dayOfWeek > 24);
  if (!lastSundayOrAfter)
  { // before switching Sunday
    return (month == 10); // October DST will be true, March not
  }
  
  if (dayOfWeek)
  { // AFTER the switching Sunday
    return (month == 3); // for March DST is true, for October not
  }
  
    int secSinceMidnightUTC = now() % 86400;
  boolean dayStartedAs = (month == 10); // DST in October, in March not
                                        // on switching Sunday we need to consider the time
  if (secSinceMidnightUTC >= 1 * 3600)
  { // 1:00 UTC (=2:00 CET/3:00 CEST)
    return !dayStartedAs;
  }

  return dayStartedAs;
}
*/