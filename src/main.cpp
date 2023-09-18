////////////////////////////////////////////////

// These define's must be placed at the beginning before #include "SAMDTimerInterrupt.h"
// _TIMERINTERRUPT_LOGLEVEL_ from 0 to 4
// Don't define _TIMERINTERRUPT_LOGLEVEL_ > 0. Only for special ISR debugging only. Can hang the system.
// Don't define TIMER_INTERRUPT_DEBUG > 2. Only for special ISR debugging only. Can hang the system.
#define TIMER_INTERRUPT_DEBUG         0
#define _TIMERINTERRUPT_LOGLEVEL_     0

// Select only one to be true for SAMD21. Must must be placed at the beginning before #include "SAMDTimerInterrupt.h"
#define USING_TIMER_TC3         true      // Only TC3 can be used for SAMD51
#define USING_TIMER_TC4         false     // Not to use with Servo library
#define USING_TIMER_TC5         false
#define USING_TIMER_TCC         false
#define USING_TIMER_TCC1        false
#define USING_TIMER_TCC2        false     // Don't use this, can crash on some boards

#include <Arduino.h>
#include <MPU6050_light.h>
#include <Wire.h>
#include <Adafruit_NeoPixel.h>
#include <Servo.h>
#include "SAMDTimerInterrupt.h"
#include "SAMD_ISR_Timer.h"
#include <SPI.h>
#include <RadioLib.h>

#define HW_TIMER_INTERVAL_MS      10

///////////////////////////////////////////////

#if (TIMER_INTERRUPT_USING_SAMD21)

  #if USING_TIMER_TC3
    #define SELECTED_TIMER      TIMER_TC3
  #elif USING_TIMER_TC4
    #define SELECTED_TIMER      TIMER_TC4
  #elif USING_TIMER_TC5
    #define SELECTED_TIMER      TIMER_TC5
  #elif USING_TIMER_TCC
    #define SELECTED_TIMER      TIMER_TCC
  #elif USING_TIMER_TCC1
    #define SELECTED_TIMER      TIMER_TCC1
  #elif USING_TIMER_TCC2
    #define SELECTED_TIMER      TIMER_TCC
  #else
    #error You have to select 1 Timer  
  #endif

#else

  #if !(USING_TIMER_TC3)
    #error You must select TC3 for SAMD51
  #endif
  
  #define SELECTED_TIMER      TIMER_TC3

#endif  

// Init selected SAMD timer
SAMDTimer ITimer(SELECTED_TIMER);

SAMD_ISR_Timer ISR_Timer;

#define TIMER_INTERVAL_10ms           10L
#define TIMER_INTERVAL_50ms           50L
#define TIMER_INTERVAL_500ms          500L
#define TIMER_INTERVAL_1S             1000L
#define TIMER_INTERVAL_2S             2000L
#define TIMER_INTERVAL_5S             5000L

void TimerHandler(void)
{
  ISR_Timer.run(); //activate timer
}

////////////////////////////////////////////////

#define ADDR_MPU      0x68
#define USER_LED      13
#define RGB_LED_PIN   6
#define SDA           4
#define SCL           3
#define SERVO_PIN     2
#define LORA_MOSI     10
#define LORA_MISO     9
#define LORA_SDK      8
#define LORA_IO0      1
#define LORA_RST      0
#define LORA_NSS      3
#define SERVO_OPEN    85  
#define SERVO_CLOSE   110
#define FONCTION_ACTIVATE 1    // 1 free-fall // 2 free-fall + angle max 45 

////////////////////////////////////////////////

Adafruit_NeoPixel rgb_led (1,RGB_LED_PIN, NEO_GBR + NEO_KHZ800);
MPU6050 imu (Wire);
Servo PServo ;
SX1278 lora_Rx = new Module(LORA_NSS,LORA_IO0,LORA_RST);

void rgb_led_green (void);
void rgb_led_blue (void);
void rgb_led_red (void);
void rgb_led_down (void);

void Timer_Handler_Security_check (void);
float get_total_accel (float AccX,float AccY,float AccZ);
bool bank_angle_detected (float AngleX,float AngleY);
bool servo_open (unsigned char angle);

void setFlag(void);

unsigned int freefall_check_flag = 0;
unsigned int bankAngel_check_flag = 0;
bool termination_flag = false;
bool parachute_deploy_flag = false;
const float freefall_threshold = 0.003;

////////////////////////////////////////////////

void setup() {

  Wire.begin(); //i2c
  imu.begin(); //Déclanchement connection MPU6050
  Serial.begin(9600); //Déclanchement de la liaison serie
  //while (! Serial); // test purpose

  // Interval in millisecs
  if (ITimer.attachInterruptInterval_MS(HW_TIMER_INTERVAL_MS, TimerHandler))
  {
    Serial.print(F("Starting ITimer OK, millis() = ")); Serial.println(millis());
  }
  else
    Serial.println(F("Can't set ITimer. Select another freq. or timer"));

  ISR_Timer.setInterval(TIMER_INTERVAL_10ms,  Timer_Handler_Security_check); //Activate fonction every 10ms

  ITimer.detachInterrupt(); //disable IRQ for led check

  rgb_led.begin();
  rgb_led.setBrightness(25);
  rgb_led_red();
  delay(1000);
  rgb_led_down();
  rgb_led_blue();
  delay(1000);
  rgb_led_down();
  rgb_led_green();

  PServo.attach(SERVO_PIN);
  PServo.write(SERVO_CLOSE);

 // initialize SX1278 with default settings
  Serial.print(F("[SX1278] Initializing ... "));
  int state = lora_Rx.begin();
  if (state == RADIOLIB_ERR_NONE) {
    Serial.println(F("success!"));
  } else {
    Serial.print(F("failed, code "));
    Serial.println(state);
    while (true);
  }

  // set the function that will be called
  // when new packet is received
  lora_Rx.setPacketReceivedAction(setFlag);

  // start listening for LoRa packets
  Serial.print(F("[SX1278] Starting to listen ... "));
  state = lora_Rx.startReceive();
  if (state == RADIOLIB_ERR_NONE) {
    Serial.println(F("success!"));
  } else {
    Serial.print(F("failed, code "));
    Serial.println(state);
    while (true);
  }
  
  // if needed, 'listen' mode can be disabled by calling
  // any of the following methods:
  //
  // radio.standby()
  // radio.sleep()
  // radio.transmit();
  // radio.receive();
  // radio.scanChannel();


  ITimer.reattachInterrupt(); //reable IRQ

}

// flag to indicate that a packet was received
volatile bool receivedFlag = false;

// this function is called when a complete packet
// is received by the module
// IMPORTANT: this function MUST be 'void' type
//            and MUST NOT have any arguments!
#if defined(ESP8266) || defined(ESP32)
  ICACHE_RAM_ATTR
#endif

////////////////////////////////////////////////
// the loop function runs over and over again forever
void loop() {
  imu.update();
  if(((freefall_check_flag >= 15 || bankAngel_check_flag >= 25 || termination_flag == true)) && parachute_deploy_flag == false){
    ITimer.detachInterrupt(); //disable IRQ for led check
    parachute_deploy_flag = true;
    Serial.print("Parachute deploy !!! \n");
    rgb_led_red();
    PServo.write(SERVO_OPEN);
  }
  if (parachute_deploy_flag == true){
    delay(5000);
    Serial.print("systeme reset \n");
    parachute_deploy_flag = false;
    termination_flag = false;
    freefall_check_flag = 0;
    bankAngel_check_flag = 0;
    PServo.write(SERVO_CLOSE);
    rgb_led_green();
    ITimer.reattachInterrupt(); //reable IRQ
  }

  // check if the flag is set
  if(receivedFlag) {
    // reset flag
    ITimer.detachInterrupt(); //disable IRQ for led checks
    receivedFlag = false;

    // you can read received data as an Arduino String
    String str;
    int state = lora_Rx.readData(str);

    // you can also read received data as byte array
    /*
      byte byteArr[8];
      int state = radio.readData(byteArr, 8);
    */

    if (str == "T"){
      termination_flag = true ; 
    }

    if (state == RADIOLIB_ERR_NONE) {
      // packet was successfully received
      Serial.println(F("[SX1278] Received packet!"));

      // print data of the packet
      Serial.print(F("[SX1278] Data:\t\t"));
      Serial.println(str);

      // print RSSI (Received Signal Strength Indicator)
      Serial.print(F("[SX1278] RSSI:\t\t"));
      Serial.print(lora_Rx.getRSSI());
      Serial.println(F(" dBm"));

      // print SNR (Signal-to-Noise Ratio)
      Serial.print(F("[SX1278] SNR:\t\t"));
      Serial.print(lora_Rx.getSNR());
      Serial.println(F(" dB"));

      // print frequency error
      Serial.print(F("[SX1278] Frequency error:\t"));
      Serial.print(lora_Rx.getFrequencyError(true));
      Serial.println(F(" Hz"));

    } else if (state == RADIOLIB_ERR_CRC_MISMATCH) {
      // packet was received, but is malformed
      Serial.println(F("[SX1278] CRC error!"));

    } else {
      // some other error occurred
      Serial.print(F("[SX1278] Failed, code "));
      Serial.println(state);

    }
    ITimer.reattachInterrupt(); //reable IRQ
  }
}

////////////////////////////////////////////////

void rgb_led_green (void){
  rgb_led.setPixelColor(0,rgb_led.Color(0,255,0));
  rgb_led.show();
}
void rgb_led_blue (void){
  rgb_led.setPixelColor(0,rgb_led.Color(255,0,0));
  rgb_led.show();
}
void rgb_led_red (void){
  rgb_led.setPixelColor(0,rgb_led.Color(0,0,255));
  rgb_led.show();
}
void rgb_led_down (void){
  rgb_led.setPixelColor(0,rgb_led.Color(0,0,0));
  rgb_led.show();
}

float get_total_accel(float AccX,float AccY,float AccZ){
  float total_accel = sqrt(((AccX * AccX) + (AccY * AccY) + (AccZ * AccZ)) / 16384.0);
  return total_accel;
}

void Timer_Handler_Security_check (void){
  float t_accel = 0;
  t_accel = get_total_accel(imu.getAccX(),imu.getAccY(),imu.getAccZ());
  if (t_accel < freefall_threshold){
    freefall_check_flag++;
    Serial.print(freefall_check_flag);
    Serial.print("\n");
  }
  else if (freefall_check_flag > 0) {
    freefall_check_flag = 0;
  }
  if(FONCTION_ACTIVATE > 1){
    bank_angle_detected(imu.getAngleX(),imu.getAngleY());
  }
}

bool bank_angle_detected (float AngleX,float AngleY){
  if (AngleX >= 45 || AngleX <= -45){
    bankAngel_check_flag++;
    return true;
  }
  else if(AngleY >= 45 || AngleY <= -45){
    bankAngel_check_flag++;
    return true;
  }
  else {
    bankAngel_check_flag = 0;
    return false;
  }
}

void setFlag(void) {
  // we got a packet, set the flag
  receivedFlag = true;
}