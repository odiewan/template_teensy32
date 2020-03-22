#include <stdint.h>
#include <Arduino.h>
#include <String.h>
#include <util.h>
#include <U8g2lib.h>
#include <U8x8lib.h>
#include <SPI.h>
#include <Wire.h>
#include <ioChan.h>
#include "Servo.h"

/*-----------------------------------------------------------------------------
//            USB-C
//  GND              Vin
//  0   PWM          GND
//  1   PWM         3.3v
//  2   PWM     PWM   A9  (23)
//  3   PWM     PWM   A8  (22)
//  4   PWM           A7  (21)
//  5   PWM           A6  (20)
//  6   PWM     PWM   A5  (19)
//  7   PWM     PWM   A4  (18)
//  8   PWM           A3  (17)
//  9   PWM           A2  (16)
//  10  PWM     PWM   A1  (15)
//  11  PWM     PWM   A0  (14)
//  12  PWM     PWM   LED (13)
//
//---------------------------------------------------------------------------*/

#define BTN01_IN  2
#define TC01_OUT  12
#define TC00_OUT  11
#define FAN_OUT   10
#define STAT_OUT_PIN    LED_BUILTIN /* 13 */
#define POT_IN    14
#define LM35_IN   15
#define THRM_IN   16
#define PMP00_OUT 23

#define US_MIN 1000
#define US_MAX 2000

#define TEMP_MAX 35


#define AIN_MAX  1023
#define MIN_CMD     0
#define MAX_CMD   255

#define PMP_OFF_CMD  0
#define MIN_PMP_CMD  22
#define NOM_PMP_CMD  24
#define MAX_PMP_CMD  28
#define PMP_CMD_GAIN .075

typedef struct {
  /* data */
  unsigned int aosPin;
  unsigned int aosCmd;
  float aosInMin;
  float aosInMax;
  float aosOutMin;
  float aosOutMax;
  float aosGain;
  float aosOffset;
  float aosVal;
  float aosShadow;

} analogOutSt;

enum TaskType {
  TASK_STAT_LED,
  TASK_POWER_UP,
  TASK_SAWTOOTH,
  TASK_VOLTAGE_MON,
  TASK_HEAT_CTRL,
  TASK_FAN_CTRL,
  TASK_PMP_CTRL,
  TASK_BTN_READ,
  TASK_OP_MODE,
  NUM_TASKS
};

String taskStr[NUM_TASKS] = {
  "TASK_STAT_LED",
  "TASK_POWER_UP",
  "TASK_SAWTOOTH",
  "TASK_VOLTAGE_MON",
  "TASK_HEAT_CTRL",
  "TASK_FAN_CTRL",
  "TASK_PMP_CTRL",
  "TASK_BTN_READ",
  "TASK_OP_MODE",
};

enum opModes {
  OP_MD_NA,
  OP_MD_BOOT,
  OP_MD_PWRUP,
  OP_MD_IDLE,
  OP_MD_MANUAL,
  OP_MD_CIRC,
  OP_MD_MAX,
  OP_MD_MAX_T,
  OP_MD_SP,
  OP_MD_COOL,
  NUM_NOM_MODES,
  OP_MD_PRIME00,
  OP_MD_PRIME01,
  OP_MD_PRIME02,
  NUM_OP_MODES
};


String opModeStr[NUM_OP_MODES] = {
  "OP_MD_NA",
  "OP_MD_BOOT",
  "OP_MD_PWRUP",
  "OP_MD_IDLE",
  "OP_MD_MANUAL",
  "OP_MD_CIRC",
  "OP_MD_MAX",
  "OP_MD_MAX_T",
  "OP_MD_SP",
  "OP_MD_COOL",
  "NUM_NOM_MODES",
  "OP_MD_PRIME00",
  "OP_MD_PRIME01",
  "OP_MD_PRIME02"
};

String opModeStrS[NUM_OP_MODES] = {
  "NA",
  "BOT",
  "PWU",
  "IDL",
  "MAN",
  "CIR",
  "MAX",
  "MXT",
  "PID",
  "COL",
  "NUM_NOM_MODES",
  "P00",
  "P01",
  "P02"
};


enum powerUpSteps {
  PWRUP_NA,
  PWRUP_0,    //---All LEDs off
  PWRUP_1,    //---LF on
  PWRUP_2,    //---LF, RF on
  PWRUP_3,    //---LF, RF, RR on
  PWRUP_4,    //---LF, RF, RR, LR on
  PWRUP_5,    //---LF, RF, RR on
  PWRUP_6,    //---LF, RF on
  PWRUP_7,    //---LF, on
  PWRUP_DONE, //---All LEDs off
  NUM_PWRUP_STEPS
};

String pwrUpStr[NUM_PWRUP_STEPS] = {
  "PWRUP_NA",
  "PWRUP_0",
  "PWRUP_1",
  "PWRUP_2",
  "PWRUP_3",
  "PWRUP_4",
  "PWRUP_5",
  "PWRUP_6",
  "PWRUP_7",
  "PWRUP_DONE"
};


unsigned long iCount;
unsigned int tmr;
unsigned int taskState;
unsigned int opState;
unsigned int pwrUpStep;
int ledState;
int serialOk;

unsigned int knobIn;
uint16_t gblCmd;
int knobInVar;
int tempInVar;
int tempSP;
ioChannel knobInChan;
ioChannel tempInChan;

unsigned int btn00State;
analogOutSt tc00;
analogOutSt tc01;
analogOutSt fan00;
Servo pmp00;

unsigned int fan00Cmd;
unsigned int tc00Cmd;
unsigned int tc01Cmd;
unsigned int pmp00Cmd;

unsigned int pidHeatCmd;
unsigned int pidPmpCmd;

U8G2_SSD1306_128X32_UNIVISION_F_HW_I2C u8g2(U8G2_R0);

//-----------------------------------------------------------------------------
//
//-----------------------------------------------------------------------------
int icISR(unsigned long interval) {
  return iCount % interval;
}

//-----------------------------------------------------------------------------
//
//-----------------------------------------------------------------------------
void writeU8G2_UL(String nStr) {

  static char tmpBuff[sizeof(nStr)];

  nStr.toCharArray(tmpBuff, sizeof(tmpBuff));

  // clear the internal memory
  u8g2.clearBuffer();

  // choose a suitable font at https://github.com/olikraus/u8g2/wiki/fntlistall
  u8g2.setFont(u8g2_font_logisoso16_tr);

  // write something to the internal memory
  u8g2.drawStr(8, 16, tmpBuff);

  // transfer internal memory to the display
  u8g2.sendBuffer();
}

//-----------------------------------------------------------------------------
//
//-----------------------------------------------------------------------------
void writeU8G2_LL(String nStr) {

  static char tmpBuff[sizeof(nStr)];

  nStr.toCharArray(tmpBuff, sizeof(tmpBuff));

  // clear the internal memory
  u8g2.clearBuffer();

  // choose a suitable font at https://github.com/olikraus/u8g2/wiki/fntlistall
  u8g2.setFont(u8g2_font_logisoso16_tr);

  // write something to the internal memory
  u8g2.drawStr(8, 32, tmpBuff);

  // transfer internal memory to the display
  u8g2.sendBuffer();
}

//-----------------------------------------------------------------------------
//
//-----------------------------------------------------------------------------
void writeU8G2_2W(String nStr1, String nStr2) {

  static char tmpBuff1[sizeof(nStr1)];
  static char tmpBuff2[sizeof(nStr2)];

  nStr1.toCharArray(tmpBuff1, sizeof(tmpBuff1));
  nStr2.toCharArray(tmpBuff2, sizeof(tmpBuff2));


  // clear the internal memory
  u8g2.clearBuffer();

  // choose a suitable font at https://github.com/olikraus/u8g2/wiki/fntlistall
  u8g2.setFont(u8g2_font_logisoso16_tr);

  // write something to the internal memory
  u8g2.drawStr(8, 15, tmpBuff1);

  // write something to the internal memory
  u8g2.drawStr(8, 32, tmpBuff2);

  // transfer internal memory to the display
  u8g2.sendBuffer();
}

//-----------------------------------------------------------------------------
//
//-----------------------------------------------------------------------------
void writeU8G2() {
  static String strUL = "";
  static String strLL = "";

  strUL = "k:" + String(knobInVar);
  // strUL += " p:" + String(pmp00Cmd);
  strUL += " t0:" + String(tc00Cmd);


  strLL = "T:" + String(tempInChan.ioEngVal);
  strLL += " Md:" + opModeStrS[opState];
  writeU8G2_2W(strUL, strLL);
  // writeU8G2_UL(strUL);
  // writeU8G2_LL(strLL);
}





//-----------------------------------------------------------------------------
//
//-----------------------------------------------------------------------------
void dispVersion() {
  String tmpStr = "ThermoElectric Cooler Controller v00.00.06";
  Serial.println(tmpStr);
  writeU8G2_UL(tmpStr);

  delay(1000);

  tmpStr = "Written by William 'Odie' O'Dell";
  Serial.println(tmpStr);
  writeU8G2_UL(tmpStr);

  delay(1000);

  tmpStr = "Date: 03-02-20";
  Serial.println(tmpStr);
  writeU8G2_UL(tmpStr);
}

//-----------------------------------------------------------------------------
//
//-----------------------------------------------------------------------------
void initAOS(analogOutSt* aos) {

  pinMode(aos->aosPin, OUTPUT);

  //---Calculate gain as output range/input range
  aos->aosGain = (aos->aosOutMax - aos->aosOutMin) / (aos->aosInMax - aos->aosInMin);

  //---Calculate offset
  aos->aosOffset = 0 - aos->aosOutMin;
}

//-----------------------------------------------------------------------------
//
//-----------------------------------------------------------------------------
unsigned int anWrMap(analogOutSt* aos) {
  aos->aosVal = (aos->aosCmd * aos->aosGain) + aos->aosOffset;

  analogWrite(aos->aosPin, aos->aosVal);

  return (unsigned int)aos->aosVal;
}

//-----------------------------------------------------------------------------
//
//-----------------------------------------------------------------------------
void setup() {
  knobIn = 0;

  btn00State = 0;

  iCount = 0;
  tmr = 0;
  taskState = 0;
  ledState = 0;
  serialOk = 0;
  fan00Cmd = 0;
  tc00Cmd = 0;
  tc01Cmd = 0;
  pidHeatCmd = 0;
  pidPmpCmd = 0;
  pmp00.attach(PMP00_OUT, US_MIN, US_MAX);
  pmp00.write(0);
  gblCmd = 0;

  tempSP = 0;

  pinMode(BTN01_IN, INPUT_PULLUP);

  pinMode(STAT_OUT_PIN, OUTPUT);

  knobInChan = ioChannel(IO_TYPE_AIN_3V3_255, POT_IN, &knobInVar);
  tempInChan = ioChannel(IO_TYPE_AIN_THERM_STIEN_3V3, THRM_IN, &tempInVar);
  tempInChan.ioFilter = IO_FILT_MOVING_AVG;

  tc00 = { TC00_OUT,0,0,255,0,255 };
  tc01 = { TC01_OUT,0,0,255,0,255 };

  fan00 = { FAN_OUT,0,0,255,0,255 };
  initAOS(&tc00);
  initAOS(&tc01);
  initAOS(&fan00);

  Serial.begin(9600);
  tmr = 4;
  u8g2.begin();
  writeU8G2_UL("Booting...");

  while (tmr > 0 && serialOk == 0) {
    if (Serial)
      serialOk = 1;
    delay(1000);

    ledState = !ledState;

    digitalWrite(STAT_OUT_PIN, ledState);
    tmr--;
  }

  Serial.println("Serial Port OK...");
  delay(1000);
  dispVersion();
  // Serial.print("aosTC00:Gain:");
  // Serial.print(tc00.aosGain);
  // Serial.print("aosTC00:Offset:");
  // Serial.print(tc00.aosOffset);
  // Serial.print("aosTC00:at 127:");
  // tc00.aosCmd = 127;
  // Serial.print(anWrMap(&tc00));
  // Serial.println();
}

//-----------------------------------------------------------------------------
//
//-----------------------------------------------------------------------------
void taskInput() {
  static char btn00StateShadow = 0;

  btn00State = digitalRead(BTN01_IN);

  // Increment the op state when a button rising edge is detected
  if (btn00State != btn00StateShadow && !btn00State)
    opState++;


  if (opState > (NUM_NOM_MODES - 1))
    opState = OP_MD_IDLE;

  btn00StateShadow = btn00State;

  knobInChan.procInChan();
  tempInChan.procInChan();
  gblCmd = knobInVar;


}

//-----------------------------------------------------------------
//
//-----------------------------------------------------------------
int taskPowerUp(void) {
  static int pwrUpCnt = 0;

  pwrUpCnt++;

  switch (pwrUpStep) {
    default:
    case PWRUP_NA:
      pwrUpStep = PWRUP_0;
      break;

    case PWRUP_0:
      pwrUpStep = PWRUP_DONE;
      break;

    case PWRUP_DONE:
      break;

  }

  return pwrUpStep;
}



//-----------------------------------------------------------------
//
//-----------------------------------------------------------------
void taskStatLED(void) {
  static int cnt = 0;
  static int ledOn = 0;

  switch (opState) {
    default:
    case OP_MD_NA:
      break;

    case OP_MD_BOOT:
      ledOn = !ledOn;
      break;

    case OP_MD_PWRUP:
      cnt++;
      if (cnt < 15)
        ledOn = true;
      else if (cnt < 30)
        ledOn = false;
      else
        cnt = 0;
      break;
  }

  digitalWrite(STAT_OUT_PIN, ledOn);
}

//-----------------------------------------------------------------------------
//
//-----------------------------------------------------------------------------
void taskDisp() {
  static String strUL = "";
  static String strLL = "";


  switch (opState) {
    default:
    case OP_MD_NA:
    case OP_MD_BOOT:
    case OP_MD_PWRUP:
    case OP_MD_IDLE:
      strUL = "k:" + String(knobInVar);
      strUL += " tc:" + String(tc00Cmd);
      fan00Cmd = MIN_CMD;
      break;

    case OP_MD_MANUAL:
      strUL = "sp:" + String(tempSP);
      strUL += " tc:" + String(tc00Cmd);
      fan00Cmd = MAX_CMD;
      break;

    case OP_MD_CIRC:
      strUL = "pc:" + String(pmp00Cmd);
      break;

    case OP_MD_MAX:
    case OP_MD_MAX_T:
    case OP_MD_SP:
      strUL = "sp:" + String(tempSP);
      strUL += " tc:" + String(tc00Cmd);
      fan00Cmd = MAX_CMD;
      break;

    case OP_MD_PRIME00:
    case OP_MD_PRIME01:
    case OP_MD_PRIME02:
      strUL = "pc:" + String(pmp00Cmd);
      // strUL += " tc:" + String(tc00Cmd);
      fan00Cmd = MIN_CMD;
      break;
  }



  strLL = "T:" + String(tempInChan.ioEngVal);
  strLL += " Md:" + opModeStrS[opState];
  writeU8G2_2W(strUL, strLL);



}

//-----------------------------------------------------------------------------
//
//-----------------------------------------------------------------------------
void taskOutput() {
  digitalWrite(LED_BUILTIN, ledState);

  tc00.aosCmd = tc00Cmd;
  tc01.aosCmd = tc01Cmd;
  fan00.aosCmd = fan00Cmd;
  anWrMap(&tc00);
  anWrMap(&tc01);
  anWrMap(&fan00);
  pmp00.write(pmp00Cmd);
}




//-----------------------------------------------------------------------------
//
//-----------------------------------------------------------------------------
void taskSerial() {
  static String tmpStr = "";
  static float dT = 0;
  static float tShadow = 0;
  static float t = 0;

  // Serial.print(iCount);

  // printSer("icISR", icISR(iCount));

  printSer("opState", opModeStr[opState]);
  printSer("Btn00", btn00State);

  printSer("gblCmd", gblCmd);

  printSer("fan00Cmd", fan00.aosCmd);
  printSer("tc00Cmd", tc00Cmd);
  // printSer("tc01Cmd", tc01Cmd);
  printSer("pmp00Cmd", pmp00Cmd);

  printSer("tempSP", tempSP);
  printSer("tInVar", tempInChan.ioEngVal);
  // printSer("tInVarF", tempInChan.ioEngValFilt);

  //  Calculate delta T * 10
  dT = (tempInChan.ioEngVal - tShadow);
  tShadow = tempInChan.ioEngVal;

  printSer("dT", dT);
  Serial.print(" deg/s");

  Serial.println();
}

//-----------------------------------------------------------------------------
//
//-----------------------------------------------------------------------------
void taskFan() {
  switch (opState) {
    default:
    case OP_MD_NA:
    case OP_MD_BOOT:
    case OP_MD_PWRUP:
    case OP_MD_IDLE:
      fan00Cmd = MIN_CMD;
      break;

    case OP_MD_MANUAL:
    case OP_MD_CIRC:
    case OP_MD_MAX:
    case OP_MD_MAX_T:
    case OP_MD_SP:
      fan00Cmd = MAX_CMD;
      break;

    case OP_MD_PRIME00:
    case OP_MD_PRIME01:
    case OP_MD_PRIME02:
      fan00Cmd = MIN_CMD;
      break;
  }
  if (fan00Cmd > MAX_CMD)
    fan00Cmd = MAX_CMD;
}
//-----------------------------------------------------------------------------
//
//-----------------------------------------------------------------------------
void taskHeat() {
  switch (opState) {
    default:
    case OP_MD_NA:
    case OP_MD_BOOT:
    case OP_MD_PWRUP:
    case OP_MD_IDLE:
    case OP_MD_CIRC:
    case OP_MD_COOL:
    case OP_MD_PRIME00:
    case OP_MD_PRIME01:
    case OP_MD_PRIME02:
      tc00Cmd = MIN_CMD;
      tc01Cmd = MIN_CMD;
      break;

    case OP_MD_MANUAL:
      tc00Cmd = gblCmd;
      tc01Cmd = gblCmd;
      break;

    case OP_MD_MAX:
    case OP_MD_MAX_T:
      if (tempInVar < tempSP) {
        tc00Cmd = MAX_CMD;
        tc01Cmd = MAX_CMD;
      }
      else {
        tc00Cmd = MIN_CMD;
        tc01Cmd = MIN_CMD;

      }

      break;

    case OP_MD_SP:
      tc00Cmd = MAX_CMD;
      tc01Cmd = MAX_CMD;
      // tc00Cmd = pidHeatCmd;
      // tc01Cmd = pidHeatCmd;
      break;
  }

  if (tc00Cmd > MAX_CMD)
    tc00Cmd = MAX_CMD;

  if (tc01Cmd > MAX_CMD)
    tc01Cmd = MAX_CMD;

}
//-----------------------------------------------------------------------------
//
//-----------------------------------------------------------------------------
void taskPump() {

  switch (opState) {
    default:
    case OP_MD_NA:
    case OP_MD_BOOT:
    case OP_MD_PWRUP:
    case OP_MD_IDLE:
      pmp00Cmd = PMP_OFF_CMD;
      break;

    case OP_MD_MANUAL:
      pmp00Cmd = (tc00Cmd * PMP_CMD_GAIN) + MIN_PMP_CMD;


      break;

    case OP_MD_CIRC:
    case OP_MD_MAX:
    case OP_MD_MAX_T:
    case OP_MD_SP:
    case OP_MD_COOL:
      pmp00Cmd = NOM_PMP_CMD;
      break;

    case OP_MD_PRIME00:
    case OP_MD_PRIME01:
    case OP_MD_PRIME02:
      pmp00Cmd = MAX_PMP_CMD;
      break;
  }
  // pmp00Cmd = (knobInVar * .01) + 22.5;
  if (pmp00Cmd > MAX_PMP_CMD)
    pmp00Cmd = MAX_PMP_CMD;

}

//-----------------------------------------------------------------
//  Operational state machine: Handles transitions between op states
// OP_MD_NA,
// OP_MD_BOOT,
// OP_MD_PWRUP,
// OP_MD_IDLE,
// OP_MD_MANUAL,
// OP_MD_CIRC,
// OP_MD_MAX,
// OP_MD_MAX_T,
// OP_MD_SP,
// OP_MD_COOL,
// NUM_NOM_MODES,
// OP_MD_PRIME00,
// OP_MD_PRIME01,
// OP_MD_PRIME02,
// NUM_OP_MODES
//-----------------------------------------------------------------
void taskOpMode(void) {

  static unsigned int opStateShadow = 0;

  switch (opState) {
    default:
    case OP_MD_NA:
      //--Go directly to BOOT mode---
      opState = OP_MD_BOOT;
      tempSP = 1;
      break;

    case OP_MD_BOOT:
      //---Stay in BOOT mode for 100 ticks---
      if (iCount < 5)
        opState = OP_MD_BOOT;
      else
        opState = OP_MD_PWRUP;

      tempSP = 2;
      break;

    case OP_MD_PWRUP:
      //---Stay in PWRUP mode until it's done---
      if (taskPowerUp() == PWRUP_DONE)
        opState++;
      else {
        opState = OP_MD_PWRUP;

      }
      tempSP = 3;
      break;

    case OP_MD_IDLE:
      tempSP = 4;
      break;

    case OP_MD_MANUAL:
      tempSP = 5;
      break;

    case OP_MD_CIRC:
      tempSP = 6;
      break;


    case OP_MD_MAX:
    case OP_MD_MAX_T:
      tempSP = TEMP_MAX;
      break;

    case OP_MD_SP:
    case OP_MD_COOL:
    case OP_MD_PRIME00:
    case OP_MD_PRIME01:
    case OP_MD_PRIME02:
      tempSP = 7;
      break;
  }

  opStateShadow = opState;
}



//-----------------------------------------------------------------
//
//-----------------------------------------------------------------
void taskManager(void) {

  switch (taskState) {

    default:
      break;

    case TASK_STAT_LED:
      taskStatLED();
      break;

    case TASK_VOLTAGE_MON:

      break;

      //---Op mode state engine task---
    case TASK_OP_MODE:
      taskOpMode();
      break;

    case TASK_HEAT_CTRL:
      taskHeat();
      break;

    case TASK_FAN_CTRL:
      taskFan();
      break;

    case TASK_PMP_CTRL:
      taskPump();
      break;

  }

  taskState++;
  if (taskState > NUM_TASKS)
    taskState = 0;

}


//-----------------------------------------------------------------------------
//
//-----------------------------------------------------------------------------
void loop() {

  taskInput();

  taskManager();

  if (icISR(300) == 0) {
    taskSerial();
  }
  if (icISR(300) == 0) {
    taskDisp();
  }

  taskOutput();

  delay(1);

  iCount++;
}
