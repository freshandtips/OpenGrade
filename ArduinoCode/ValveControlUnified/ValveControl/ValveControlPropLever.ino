/*
  AOG Section Control
*/
// Last change: 2022-04-21 by Pat 23h00
// to be used with OpenGrade3D v1.1.xx and OpenGrade v2.2.xx
// test with blade offset
//Only cytron output
//-------------------------------------------------------------------------
//
//the original code is BlinkTheRelay from BrianTee AGopenGPS
//-------------------------------------------------------------------------


//Output in PROPO MODE: BLADE_DIR,PWM_VALUE,CUTVALUE
/* ------------------------------------------------------------
   ピン解説（日本語）
   ------------------------------------------------------------
   [シリアル]
   TX1 / RX1 : OpenGrade 通信

   [デジタル出力]
   D2  : LED_DW（ブレード下げ方向LED）
   D3  : Cytron PWM 出力
   D4  : Cytron DIR 出力
   D5  : LED_UP（ブレード上げ方向LED）
   D9  : LED_AUTO（自動状態LED）
   D13 : LED_BUILTIN（自動時点灯）/ 手動下げボタン入力（manualMoveBtn=true時）

   [デジタル入力]
   D6  : オフセット下げボタン（GNDで有効）
   D7  : Work/Auto スイッチ（GNDで有効）
   D8  : 機能ボタン1（同期/復帰トグル、GNDで有効）
   D10 : 機能ボタン2（ちょい上げ、GNDで有効）
   D11 : 機能ボタン3（ちょい下げ、GNDで有効）
   D12 : センサーモード切替入力（LOW=Nano/DAC側）

   [アナログ]
   A0 : LED_ON（設定受信まで点滅）
   A1 : 手動上下レバー入力
   A2 : オフセットレバー入力（bladeOffsetPropLever=true時）
   A3 : 純正センサー電圧監視入力
   A4 : I2C SDA（MCP4725）
   A5 : I2C SCL（MCP4725）
   A6 : オフセット上げボタン（bladeOffsetBtn=true時、外付けプルアップ推奨）
   A7 : 手動上げボタン（manualMoveBtn=true時、外付けプルアップ推奨）

   [MCP4725]
   OUT : ECUへ入れる疑似センサー電圧（手動切替スイッチ経由）
   VCC : 5V
   GND : Nano/ECUセンサーGNDと共通
   ------------------------------------------------------------ */

#include "EEPROM.h"
#include <Wire.h>
#include <Adafruit_MCP4725.h>
#include <math.h>
//User set variables
//PWM or relay mode
bool proportionalValve = true;
//cytron
#define DIR_ENABLE 4 //PD4 cytron dir
#define PWM_OUT 3 //PD3  cytron pwm

//workswitch or work button
bool workButton = true; // true for momentary button, false for switch(continus)
#define WORKSW_PIN 7  //PD7 this pin must be low (to ground) to activate automode IMP on PCB

//proportional lever
bool manualMovePropLever = true; //if a lever for manual operation is installed
bool invertManMove = false;
bool manualMoveBtn = false;
#define LEVER_UP A1 // first axle
#define BMANUP_PIN A7 //manualMoveBtn=true時のみ使用（外付けプルアップ前提）
#define BMANDW_PIN 13 //manualMoveBtn=true時のみ使用

// blade off set choose betwen lever or btn or none.
bool bladeOffsetPropLever = false;
bool invertBladeOffset = false;
bool bladeOffsetBtn = false; // true if this fonctionality is used
#define BOFFUP_PIN A6 //bladeOffsetBtn=true時のみ使用（外付けプルアップ前提）
#define BOFFDW_PIN 6 //offset down
#define LEVER_SIDE A2 // second axle, if used for blade offset

//leds
#define LED_DW 2 //DO2 led down (if used)
#define LED_UP 5 //DO5 led up (if used)
#define LED_AUTO 9 //DO9 led auto
#define LED_ON A0 //A0 on led 

//DAC + original sensor monitor
Adafruit_MCP4725 dac;
#define DAC_I2C_ADDR 0x62
#define TRACTOR_SENSOR_IN A3 //original tractor sensor voltage monitor
#define SENSOR_MODE_SWITCH_PIN 12 //manual switch status input (LOW = DAC side selected)
#define FUNCTION_BTN_SYNC_PIN 8 // function button 1 (active LOW with INPUT_PULLUP)
#define FUNCTION_BTN_2_PIN 10 // function button 2 (+offset)
#define FUNCTION_BTN_3_PIN 11 // function button 3 (-offset)
const float ADC_REF_V = 5.0;
const float DAC_MIN_V = 0.55;
const float DAC_MAX_V = 2.95;
const float DAC_STEP_GAIN = 0.0020;
bool dacPositiveRaisesVoltage = true;
bool syncDacToOriginalOnAutoEntry = true;
float dacVoltage = 1.50;
bool sendOriginalSensorRawInTelemetry = false; // keep old telemetry layout by default
bool functionButtonEnabled = true;
float functionSyncVoltage = 1.80; // destination voltage on first function-button press
const float FUNCTION_TRANSITION_TIME_SEC = 2.0; // seconds to reach target or return voltage
/* ------------------------------------------------------------
   D10/D11 オフセット機能 設定ガイド（日本語）
   ------------------------------------------------------------
   目的:
   - D7 が HIGH（無効/手動）時のみ、D10/D11 で A3 の監視信号へ
     微小オフセットを加えて DAC 出力します。

   動作:
   - D10 を 1 回押すごとに +オフセットを 1 カウント加算
   - D11 を 1 回押すごとに -オフセットを 1 カウント加算
   - モーメンタリースイッチ前提で、長押ししても 1 回の押下につき 1 カウント
     （立ち下がりエッジ検出）

   変更ポイント:
   - FUNCTION_OFFSET_STEP_V を変更すると、1カウントあたりの電圧を変えられます。
     例）0.20f なら 1 回押下で ±0.2V
   ------------------------------------------------------------ */
float FUNCTION_OFFSET_STEP_V = 0.20f;



//end of user set variables

//loop time variables in milliseconds
const byte LOOP_TIME = 50; //20hz
unsigned long lastTime = LOOP_TIME;
unsigned long currentTime = LOOP_TIME;

//Comm checks
byte watchdogTimer = 0;      //make sure we are talking to AOG
byte serialResetTimer = 0;   //if serial buffer is getting full, empty it
bool settingsRecieved = false;
//EEPROM identifier
byte EEP_Ident = 138;

byte deadband = 5;

//Communication with AgOpenGPS
bool isDataFound = false, isSettingFound = false;
int header = 0, tempHeader = 0, temp;

//The variables used for storage
byte relayHi = 0, relayLo = 0, cutValve = 100;

//workSwitch
byte workSwitch = 1; //high is circuit open, low is switch grounded
byte autoEnable = 0;

//pwm variables
byte pwmDrive = 0, pwmDisplay = 0, pwmGainUp = 5, pwmMinUp = 50, pwmGainDw = 5, pwmMinDw = 50, pwmMaxUp = 255, pwmMaxDw = 255, integralMultiplier = 20;
int pwmValue = 0;
float pwmValueCalc = 0;
//int cutValue = 0;

int plannedValveValue = 0, pwm1ago = 0, pwm2ago = 0, pwm3ago = 0, pwm4ago = 0, pwm5ago = 0;
float pwmHist = 0;

//AutoControl switch button  ***********************************************************************************************************
byte currentState = 1;
byte reading;
byte previous = 0;

//BladeOffset stuff ************************************************************
int bladeOffsetIn = 0, bladeOffsetOut = 0;

byte bOUprevious = 0;
byte bODprevious = 0;


int LeverUpValue = 0;
int LeverSideValue = 0;
int LeverPushValue = 0;
int onLedTime = 0;
int autoLedTime = 0;
int originalSensorRaw = 0;
float originalSensorVoltage = 0.0;
byte prevAutoControlActive = 0;
byte sensorModeSwitchDebounced = HIGH;
byte sensorModeSwitchLastRaw = HIGH;
unsigned long sensorModeSwitchLastChange = 0;
const byte SENSOR_MODE_DEBOUNCE_MS = 25;
bool dacReady = false;
byte functionBtnDebounced = HIGH;
byte functionBtnLastRaw = HIGH;
byte functionBtnPrevDebounced = HIGH;
unsigned long functionBtnLastChange = 0;
const byte FUNCTION_BTN_DEBOUNCE_MS = 25;
byte functionBtn2Debounced = HIGH;
byte functionBtn2LastRaw = HIGH;
byte functionBtn2PrevDebounced = HIGH;
unsigned long functionBtn2LastChange = 0;
byte functionBtn3Debounced = HIGH;
byte functionBtn3LastRaw = HIGH;
byte functionBtn3PrevDebounced = HIGH;
unsigned long functionBtn3LastChange = 0;
const byte FUNCTION_BTN23_DEBOUNCE_MS = 25;
bool functionHoldActive = false;
bool functionReturnActive = false;
float functionStoredVoltage = 1.50;
float functionMoveTargetVoltage = 1.50;
float functionMoveStepVoltage = 0.0;
int functionOffsetCount = 0;

void UpdateDACFromPWM(void);
void WriteDACVoltage(float voltage);
byte IsAutoControlActive(void);
void ReadOriginalSensorVoltage(void);
void UpdateSensorModeSwitchState(void);
float ReadOriginalSensorVoltageAveraged(void);
void UpdateFunctionButtonState(void);
void HandleFunctionButtonPress(byte autoControlActive);
void HandleFunctionOffsetButtons(void);
byte MoveDacToward(float targetVoltage, float stepVoltage);
float CalcTransitionStepVoltage(float fromVoltage, float toVoltage);


void setup()
{

  TCCR2B = TCCR2B & B11111000 | B00000110;    // set timer 2 to 256 for PWM frequency of   122.55 Hz
  TCCR1B = TCCR1B & B11111000 | B00000100;    // set timer 1 to 256 for PWM frequency of   122.55 Hz


  //set the baud rate
  Serial.begin(38400);

  //set pins to output
  pinMode(DIR_ENABLE, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(LED_DW, OUTPUT);
  pinMode(LED_UP, OUTPUT);
  pinMode(LED_AUTO, OUTPUT);
  pinMode(LED_ON, OUTPUT);
  pinMode(SENSOR_MODE_SWITCH_PIN, INPUT_PULLUP);
  pinMode(FUNCTION_BTN_SYNC_PIN, INPUT_PULLUP);
  pinMode(FUNCTION_BTN_2_PIN, INPUT_PULLUP);
  pinMode(FUNCTION_BTN_3_PIN, INPUT_PULLUP);
  sensorModeSwitchLastRaw = digitalRead(SENSOR_MODE_SWITCH_PIN);
  sensorModeSwitchDebounced = sensorModeSwitchLastRaw;
  sensorModeSwitchLastChange = millis();
  functionBtnLastRaw = digitalRead(FUNCTION_BTN_SYNC_PIN);
  functionBtnDebounced = functionBtnLastRaw;
  functionBtnPrevDebounced = functionBtnDebounced;
  functionBtnLastChange = millis();
  functionBtn2LastRaw = digitalRead(FUNCTION_BTN_2_PIN);
  functionBtn2Debounced = functionBtn2LastRaw;
  functionBtn2PrevDebounced = functionBtn2Debounced;
  functionBtn2LastChange = millis();
  functionBtn3LastRaw = digitalRead(FUNCTION_BTN_3_PIN);
  functionBtn3Debounced = functionBtn3LastRaw;
  functionBtn3PrevDebounced = functionBtn3Debounced;
  functionBtn3LastChange = millis();


  //keep pulled high and drag low to activate, noise free safe
  pinMode(WORKSW_PIN, INPUT_PULLUP);

  if (manualMoveBtn)
  {
    pinMode(BMANUP_PIN, INPUT_PULLUP);
    pinMode(BMANDW_PIN, INPUT_PULLUP);
  }

  if (bladeOffsetBtn)
  {
    pinMode(BOFFUP_PIN, INPUT_PULLUP);
    pinMode(BOFFDW_PIN, INPUT_PULLUP);
  }

  ReadFromEEPROM();// read saved settings
  Wire.begin();
  dacReady = dac.begin(DAC_I2C_ADDR);
  if (dacReady) WriteDACVoltage(dacVoltage);

}

void loop()
{
  //Loop triggers every 50 msec (20hz)

  currentTime = millis();
  unsigned int time = currentTime;

  if (currentTime - lastTime >= LOOP_TIME)
  {
    lastTime = currentTime;

    //If connection lost to AgOpenGPS, the watchdog will count up
    if (watchdogTimer++ > 250) watchdogTimer = 32;

    //clean out serial buffer to prevent buffer overflow
    if (serialResetTimer++ > 20)
    {
      while (Serial.available() > 0) char t = Serial.read();
      serialResetTimer = 0;
    }

    // On LED settings
    if (settingsRecieved) {
      digitalWrite(LED_ON, HIGH);
      onLedTime = 0;
    }
    else {
      if (onLedTime > 19) onLedTime = 0;
      if (onLedTime < 11) digitalWrite(LED_ON, HIGH);
      else digitalWrite(LED_ON, LOW);
      onLedTime++;
    }

    // auto LED settings
    if (workSwitch == 0)// Auto mode
    {

      if (autoEnable == 1) {
        digitalWrite(LED_AUTO, HIGH);
        autoLedTime = 0;
      }
      else {
        if (autoLedTime > 7) autoLedTime = 0;
        if (autoLedTime > 3) digitalWrite(LED_AUTO, HIGH);
        else digitalWrite(LED_AUTO, LOW);
        autoLedTime++;
      }
    }
    else {
      digitalWrite(LED_AUTO, LOW);
      autoLedTime = 0;
    }

    //safety - turn off if confused
    if (watchdogTimer > 30) cutValve = 100;

    if (watchdogTimer < 29)
    {
      //read the  work switch
      if (workButton)
      {
        //steer Button momentary

        reading = digitalRead(WORKSW_PIN);
        if (reading == LOW && previous == HIGH)
        {
          if (currentState == 1)
          {
            currentState = 0;
            workSwitch = 0;
          }
          else
          {
            currentState = 1;
            workSwitch = 1;
          }
        }
        previous = reading;

      }
      else workSwitch = digitalRead(WORKSW_PIN);  // read work switch
    }
    else workSwitch = 1;

    //read the inputs for manual blade controls
    if (manualMovePropLever)
    {
      //if a lever for manual operation is installed
      LeverUpValue = analogRead(LEVER_UP); //
      if (invertManMove) LeverUpValue = map(LeverUpValue, 0, 1023, 1023, 0);
    }
    else if (manualMoveBtn)
    {
      if (digitalRead(BMANUP_PIN) == LOW) LeverUpValue = 1;
      else if (digitalRead(BMANDW_PIN) == LOW) LeverUpValue = 1022;
      else LeverUpValue = 512;
    }
    else LeverUpValue = 512;
    //0 lift -- 512 neutral-- 1023 lower



    //BladeOffset ************************************************
    if (bladeOffsetPropLever)
    {
      LeverSideValue = analogRead(LEVER_SIDE);
      LeverSideValue = map(LeverSideValue, 0, 1023, 0, 5);
      if (invertBladeOffset) LeverSideValue = map(LeverSideValue, 0, 5, 5, 0);
      //0 offset down -- 2 neutral -- 4-5 offset up

      if (LeverSideValue >= 4 && bOUprevious == HIGH)
      {
        bladeOffsetOut ++;
      }
      if (LeverSideValue == 0 && bOUprevious == HIGH)
      {
        bladeOffsetOut --;
      }
      if (LeverSideValue >= 1 && LeverSideValue <= 3) bOUprevious = HIGH;
      else bOUprevious = LOW;
    }

    else if (bladeOffsetBtn) {
      reading = digitalRead(BOFFUP_PIN);
      if (reading == LOW && bOUprevious == HIGH)
      {
        bladeOffsetOut ++;
      }
      bOUprevious = reading;

      reading = digitalRead(BOFFDW_PIN);
      if (reading == LOW && bODprevious == HIGH)
      {
        bladeOffsetOut --;
      }
      bODprevious = reading;


    }
    else bladeOffsetOut = 0; // 0 mean not activated

    //section relays
    SetPWM();
    UpdateSensorModeSwitchState();
    UpdateFunctionButtonState();
    ReadOriginalSensorVoltage();
    UpdateDACFromPWM();

    if (pwmValue < 0) {
      digitalWrite(LED_DW, HIGH); // lowering the blade
      digitalWrite(LED_UP, LOW);
    }
    if (pwmValue > 0) {
      digitalWrite(LED_UP, HIGH); // lift the blade
      digitalWrite(LED_DW, LOW);
    }
    if (pwmValue == 0) {
      digitalWrite(LED_UP, LOW);
      digitalWrite(LED_DW, LOW);
    }

  } //end of timed loop

  //****************************************************************************************
  //This runs continuously, outside of the timed loop, keeps checking UART for new data
  // PGN - 32762 - 127.250 0x7FFA
  //public int mdHeaderHi, mdHeaderLo = 1, cutValve = 2
  //Settind PGN - 32760 - 127.248 0x7FF8
  if (Serial.available() > 0 && !isDataFound && !isSettingFound)
  {
    int temp = Serial.read();
    header = tempHeader << 8 | temp;                //high,low bytes to make int
    tempHeader = temp;                              //save for next time
    if (header == 32762) isDataFound = true;        //Do we have a match?
    if (header == 32760) isSettingFound = true;        //Do we have a match?
  }

  //Data Header has been found, so the next 6 bytes are the data
  if (Serial.available() > 5 && isDataFound)
  {
    isDataFound = false;
    cutValve = Serial.read();
    bladeOffsetIn = Serial.read(); //bladeOffset value in opengrade 100 mean 0 offset.
    Serial.read(); //optOut1
    Serial.read(); //optOut2
    Serial.read(); //optOut3
    Serial.read(); //optOut4

    //reset watchdog
    watchdogTimer = 0;

    //Reset serial Watchdog
    serialResetTimer = 0;

    //Print data to openGrade, MUST send 8 bytes!
    //valve direction,pwm value,cutvalve,blade offset,opt,opt,opt,opt

    if (pwmValue < 0) // lowering the blade
    {
      Serial.print("1,");
    }
    else Serial.print("0,");


    Serial.print(String((int)pwmDrive) + ",");

    Serial.print(cutValve);
    Serial.print(",");
    Serial.print(bladeOffsetOut); // 100 mean no movement, 0 mean not active, in mm
    bladeOffsetOut = 100;
    Serial.print(",");
    Serial.print(LeverUpValue); //just for info, not used
    Serial.print(",");
    if (sendOriginalSensorRawInTelemetry) Serial.print(originalSensorRaw); // optional debug data
    else Serial.print(LeverSideValue); // keep old field layout for compatibility
    Serial.print(",");
    Serial.print(bladeOffsetIn); //just for info, not used //(LeverPushValue);
    Serial.print(",");
    Serial.println(pwmHist); //just for info, not used

    Serial.flush();   // flush out buffer


  }

  //Setting Header has been found, so the next 8 bytes are the data
  if (Serial.available() > 7 && isSettingFound)
  {
    isSettingFound = false;
    pwmGainUp = Serial.read();
    pwmGainDw = Serial.read();
    pwmMinUp = Serial.read();
    pwmMinDw = Serial.read();
    pwmMaxUp = Serial.read();
    pwmMaxDw = Serial.read();
    integralMultiplier = Serial.read();
    deadband = Serial.read();

    //reset watchdog
    //watchdogTimer = 0;

    //Reset serial Watchdog
    //serialResetTimer = 0;
    SaveToEEPROM();
    settingsRecieved = true;
  }


}

void SetPWM(void)
{
  if (workSwitch == 1) autoEnable = 1; // if auto switch is tourned off turn on AutoEnable for the next time auto switch will be turned on
  if (LeverUpValue < 480) autoEnable = 0; //turn off automode when lifting the blade
  if (LeverUpValue > 1000) autoEnable = 1; // tur on automode when lever is fully presed for lowering the blade

  pwmValue = 0;

  if (workSwitch == 0 && autoEnable == 1)// Auto mode
  {
    digitalWrite(LED_BUILTIN, HIGH);// led on when automode

    if (cutValve >= (100 + deadband))// then lower the blade
    {
      pwmValue = -((cutValve - 100 - deadband) * pwmGainDw + pwmMinDw); //pwmValue is negative
    }
    if  (cutValve <= (100 - deadband)) // then lift the blade
    {
      pwmValue = -((cutValve - 100 + deadband) * pwmGainUp - pwmMinUp); //pwmValue is positive
    }

    if (cutValve != 100 && pwmValue != 0) //calculate some sort of derivative
    {
      pwmHist = ((((pwm1ago) + pwm2ago + (pwm3ago) + (pwm4ago) + (pwm5ago / 2.000)) * (sq(integralMultiplier) / 100.0000)) / sq(cutValve - 100.0000));

      //put pwmHist to 0 when the blade cross the line.
      if (cutValve > 100 && (pwm1ago + pwm2ago + pwm3ago + pwm4ago + pwm5ago) > 0) pwmHist = 0;
      if (cutValve < 100 && (pwm1ago + pwm2ago + pwm3ago + pwm4ago + pwm5ago) < 0) pwmHist = 0;

      pwmValue = (pwmValue - pwmHist);

    }

    if (cutValve > 100 && pwmValue > 0) pwmValue = 0;

    if (cutValve > 100 && pwmValue < -pwmMaxDw) pwmValue = -pwmMaxDw;

    if (cutValve < 100 && pwmValue < 0) pwmValue = 0;

    if (cutValve < 100 && pwmValue > pwmMaxUp) pwmValue = pwmMaxUp;

    if (pwmValue > 0 && pwmValue < pwmMinUp) pwmValue = 0;

    if (pwmValue < 0 && pwmValue > -pwmMinDw) pwmValue = 0;




    pwmDrive = abs(pwmValue);
    plannedValveValue = cutValve;
  } // end of automode
  else // if manual mode
  {
    digitalWrite(LED_BUILTIN, LOW);// led low when manual mode
    pwmDrive = 0;
    plannedValveValue = 100;

    // now give an output value by the lever
    if (LeverUpValue < 480)// lifting the blade range 480 to 0
    {
      pwmValueCalc = (((480 - LeverUpValue) / 450.000 * (pwmMaxUp - pwmMinUp)) + pwmMinUp); // (1 to 480)/450 *(pwmMaxUp-pwmMinUp)+ pwmMinUp
      pwmValue = pwmValueCalc;
      if (pwmValue > pwmMaxUp) pwmValue = pwmMaxUp;
    }
    if (LeverUpValue > 540) // lovering the blade range 540 to 1024
    {
      pwmValueCalc = ((LeverUpValue - 540) / 450.000 * -(pwmMaxDw - pwmMinDw) - pwmMinDw); // (1 to 484)/450*-(pwmMaxDw-pwmMinDw)- pwmMinDw
      pwmValue = pwmValueCalc;
      if (pwmValue < -pwmMaxDw) pwmValue = -pwmMaxDw;
    }

    pwmDrive = abs(pwmValue);


  }

  if (pwmValue < 0) // lowering the blade
  {
    digitalWrite(DIR_ENABLE, HIGH);
    //Serial.print("1,");
  }
  else
  {
    digitalWrite(DIR_ENABLE, LOW);
    //Serial.print("0,");
  }


  if (proportionalValve) analogWrite(PWM_OUT, pwmDrive);
  else
  {
    if (pwmDrive > 2) analogWrite(PWM_OUT, 255);
    else analogWrite(PWM_OUT, 0);
  }
  //Serial.print(String((int)pwmDrive)+",");


  pwm5ago = pwm4ago;
  pwm4ago = pwm3ago;
  pwm3ago = pwm2ago;
  pwm2ago = pwm1ago;
  pwm1ago = pwmValue;




}

void ReadOriginalSensorVoltage(void)
{
  originalSensorRaw = analogRead(TRACTOR_SENSOR_IN);
  originalSensorVoltage = ((float)originalSensorRaw / 1023.0) * ADC_REF_V;
}

float ReadOriginalSensorVoltageAveraged(void)
{
  long sum = 0;
  const byte sampleCount = 8;
  for (byte i = 0; i < sampleCount; i++) sum += analogRead(TRACTOR_SENSOR_IN);

  int avgRaw = sum / sampleCount;
  originalSensorRaw = avgRaw;
  return ((float)avgRaw / 1023.0) * ADC_REF_V;
}

void UpdateSensorModeSwitchState(void)
{
  byte raw = digitalRead(SENSOR_MODE_SWITCH_PIN);
  unsigned long now = millis();

  if (raw != sensorModeSwitchLastRaw)
  {
    sensorModeSwitchLastRaw = raw;
    sensorModeSwitchLastChange = now;
  }

  if ((now - sensorModeSwitchLastChange) >= SENSOR_MODE_DEBOUNCE_MS)
  {
    sensorModeSwitchDebounced = sensorModeSwitchLastRaw;
  }
}

void UpdateFunctionButtonState(void)
{
  byte raw = digitalRead(FUNCTION_BTN_SYNC_PIN);
  byte raw2 = digitalRead(FUNCTION_BTN_2_PIN);
  byte raw3 = digitalRead(FUNCTION_BTN_3_PIN);
  unsigned long now = millis();

  if (raw != functionBtnLastRaw)
  {
    functionBtnLastRaw = raw;
    functionBtnLastChange = now;
  }

  if ((now - functionBtnLastChange) >= FUNCTION_BTN_DEBOUNCE_MS)
  {
    functionBtnDebounced = functionBtnLastRaw;
  }

  if (raw2 != functionBtn2LastRaw)
  {
    functionBtn2LastRaw = raw2;
    functionBtn2LastChange = now;
  }
  if ((now - functionBtn2LastChange) >= FUNCTION_BTN23_DEBOUNCE_MS)
  {
    functionBtn2Debounced = functionBtn2LastRaw;
  }

  if (raw3 != functionBtn3LastRaw)
  {
    functionBtn3LastRaw = raw3;
    functionBtn3LastChange = now;
  }
  if ((now - functionBtn3LastChange) >= FUNCTION_BTN23_DEBOUNCE_MS)
  {
    functionBtn3Debounced = functionBtn3LastRaw;
  }
}

void HandleFunctionButtonPress(byte autoControlActive)
{
  if (!functionButtonEnabled)
  {
    functionBtnPrevDebounced = functionBtnDebounced;
    return;
  }

  if (functionBtnDebounced == LOW && functionBtnPrevDebounced == HIGH && autoControlActive)
  {
    if (!functionHoldActive && !functionReturnActive)
    {
      functionStoredVoltage = dacVoltage;
      functionMoveTargetVoltage = functionSyncVoltage;
      functionMoveStepVoltage = CalcTransitionStepVoltage(dacVoltage, functionMoveTargetVoltage);
      functionHoldActive = true;
    }
    else if (functionHoldActive && !functionReturnActive)
    {
      functionHoldActive = false;
      functionReturnActive = true;
      functionMoveTargetVoltage = functionStoredVoltage;
      functionMoveStepVoltage = CalcTransitionStepVoltage(dacVoltage, functionMoveTargetVoltage);
    }
  }

  functionBtnPrevDebounced = functionBtnDebounced;
}

void HandleFunctionOffsetButtons(void)
{
  // D7 が HIGH（無効/手動）時のみ有効
  if (digitalRead(WORKSW_PIN) != HIGH)
  {
    functionBtn2PrevDebounced = functionBtn2Debounced;
    functionBtn3PrevDebounced = functionBtn3Debounced;
    return;
  }

  // D10: +1カウント（押下1回で1カウント）
  if (functionBtn2Debounced == LOW && functionBtn2PrevDebounced == HIGH)
  {
    functionOffsetCount++;
  }

  // D11: -1カウント（押下1回で1カウント）
  if (functionBtn3Debounced == LOW && functionBtn3PrevDebounced == HIGH)
  {
    functionOffsetCount--;
  }

  functionBtn2PrevDebounced = functionBtn2Debounced;
  functionBtn3PrevDebounced = functionBtn3Debounced;
}

float CalcTransitionStepVoltage(float fromVoltage, float toVoltage)
{
  const float loopTimeSec = (float)LOOP_TIME / 1000.0;
  float transitionLoops = FUNCTION_TRANSITION_TIME_SEC / loopTimeSec;
  if (transitionLoops < 1.0) transitionLoops = 1.0;

  float step = fabs(toVoltage - fromVoltage) / transitionLoops;
  if (step < 0.0001) step = 0.0001;
  return step;
}

byte MoveDacToward(float targetVoltage, float stepVoltage)
{
  float delta = targetVoltage - dacVoltage;

  if (delta > stepVoltage) dacVoltage += stepVoltage;
  else if (delta < -stepVoltage) dacVoltage -= stepVoltage;
  else dacVoltage = targetVoltage;

  return (dacVoltage == targetVoltage);
}

byte IsAutoControlActive(void)
{
  return (workSwitch == 0 && autoEnable == 1 && sensorModeSwitchDebounced == LOW);
}

void UpdateDACFromPWM(void)
{
  if (!dacReady) return;

  byte autoControlActive = IsAutoControlActive();
  HandleFunctionOffsetButtons();
  HandleFunctionButtonPress(autoControlActive);

  if (autoControlActive && !prevAutoControlActive && syncDacToOriginalOnAutoEntry)
  {
    dacVoltage = ReadOriginalSensorVoltageAveraged();
  }
  prevAutoControlActive = autoControlActive;

  if (!autoControlActive)
  {
    functionHoldActive = false;
    functionReturnActive = false;
    float manualOutVoltage = originalSensorVoltage + ((float)functionOffsetCount * FUNCTION_OFFSET_STEP_V);
    if (manualOutVoltage < DAC_MIN_V) manualOutVoltage = DAC_MIN_V;
    if (manualOutVoltage > DAC_MAX_V) manualOutVoltage = DAC_MAX_V;
    dacVoltage = manualOutVoltage;
    WriteDACVoltage(dacVoltage);
    return;
  }

  if (functionHoldActive || functionReturnActive)
  {
    if (MoveDacToward(functionMoveTargetVoltage, functionMoveStepVoltage) && functionReturnActive)
    {
      functionReturnActive = false;
    }

    if (dacVoltage < DAC_MIN_V) dacVoltage = DAC_MIN_V;
    if (dacVoltage > DAC_MAX_V) dacVoltage = DAC_MAX_V;

    WriteDACVoltage(dacVoltage);
    return;
  }

  float step = (float)pwmValue * DAC_STEP_GAIN;
  if (!dacPositiveRaisesVoltage) step = -step;

  dacVoltage += step;

  if (dacVoltage < DAC_MIN_V) dacVoltage = DAC_MIN_V;
  if (dacVoltage > DAC_MAX_V) dacVoltage = DAC_MAX_V;

  WriteDACVoltage(dacVoltage);
}

void WriteDACVoltage(float voltage)
{
  if (voltage < 0.0) voltage = 0.0;
  if (voltage > ADC_REF_V) voltage = ADC_REF_V;

  uint16_t dacCode = (uint16_t)((voltage / ADC_REF_V) * 4095.0);
  if (dacCode > 4095) dacCode = 4095;
  dac.setVoltage(dacCode, false);
}

void SaveToEEPROM() {
  EEPROM.update(1, pwmGainUp);
  EEPROM.update(3, pwmMinUp);
  EEPROM.update(5, pwmGainDw);
  EEPROM.update(7, pwmMinDw);
  EEPROM.update(9, pwmMaxUp);
  EEPROM.update(11, pwmMaxDw);
  EEPROM.update(13, integralMultiplier);
  EEPROM.update(15, deadband);
  EEPROM.update(17, EEP_Ident);
}

void ReadFromEEPROM() {
  int checkValue;
  checkValue = EEPROM.read(17);
  if (checkValue == EEP_Ident) {
    pwmGainUp = EEPROM.read(1);
    pwmMinUp = EEPROM.read(3);
    pwmGainDw = EEPROM.read(5);
    pwmMinDw = EEPROM.read(7);
    pwmMaxUp = EEPROM.read(9);
    pwmMaxDw = EEPROM.read(11);
    integralMultiplier = EEPROM.read(13);
    deadband = EEPROM.read(15);
  }
}
