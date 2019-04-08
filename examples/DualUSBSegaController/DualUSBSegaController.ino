//
// SegaControllerUSB.ino
//   Adam Booth (adam.booth@gmail.com)
//
// A simplified merger of SegaControllerSerialReader.ino and Arduino-USB-HID-RetroJoystickAdapter
// https://github.com/jonthysell/SegaController
// https://github.com/mcgurk/Arduino-USB-HID-RetroJoystickAdapter
//
// Uses the pinout to suit the "two DB9 connectors solded back to back" on an Arudino Pro Micro (ATmega32u4) clone
// as seen at https://github.com/mcgurk/Arduino-USB-HID-RetroJoystickAdapter/tree/master/megadrive
//
// Copyright notice for SegaController:
//
// Author:
//       Jon Thysell <thysell@gmail.com>
//
// Copyright (c) 2017 Jon Thysell <http://jonthysell.com>
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

#include <SegaController.h>

#define DEBUG

// Controller DB9 pins (looking face-on to the end of the plug):
//
// 5 4 3 2 1
//  9 8 7 6
//
// Connect pin 5 to +5V and pin 8 to GND
// Connect the remaining pins to digital I/O pins (see below)

// Arduino pins that are connected to (two DB9 Connectors Back to back on Pro Micro Board)
//                 DB9 Pin 7,  1,  2,  3,  4,  6,  9
SegaController controller1(8,  2,  3,  4,  5,  7,  9);
SegaController controller2(16, 15, A0, A1, A2, 14, 10);

// Controller states
word currentState1 = 0;
word lastState1 = 0;
word currentState2 = 0;
word lastState2 = 0;


// From RetroJoystickAdapter_MegaDrive

inline void translateState(uint8_t *data, uint8_t *state) {
  state[0] = ~data[0];
  state[1] = 127; 
  state[2] = 127;
  if (!bitRead(data[1], 0)) state[2] = 0; /* up */
  if (!bitRead(data[1], 1)) state[2] = 255; /* down */
  if (!bitRead(data[1], 2)) state[1] = 0; /* left */
  if (!bitRead(data[1], 3)) state[1] = 255; /* right */
}

uint8_t J1BTN6 = 0;
uint8_t J2BTN6 = 0;

uint8_t plugged1 = 0;
uint8_t plugged2 = 0;

// Setup HID for USB
#include "HID.h"

#if ARDUINO < 10606
#error The Joystick2 library requires Arduino IDE 1.6.6 or greater. Please update your IDE.
#endif

#if !defined(USBCON)
#error The Joystick2 library can only be used with a USB MCU (e.g. Arduino Leonardo, Arduino Micro, etc.).
#endif

#if !defined(_USING_HID)
#error "legacy HID core (non pluggable)"
#endif

#define JOYSTICK_REPORT_ID  0x04
#define JOYSTICK2_REPORT_ID 0x05

#define JOYSTICK_DATA_SIZE 2
#define JOYSTICK_STATE_SIZE 3

//================================================================================
//================================================================================
//  Joystick (Gamepad)


#define HIDDESC_MACRO(REPORT_ID) \
    /* Joystick # */ \
    0x05, 0x01,               /* USAGE_PAGE (Generic Desktop) */ \
    0x09, 0x04,               /* USAGE (Joystick) */ \
    0xa1, 0x01,               /* COLLECTION (Application) */ \
    0x85, REPORT_ID,          /* REPORT_ID */ \
    /* 8 Buttons */ \
    0x05, 0x09,               /*   USAGE_PAGE (Button) */ \
    0x19, 0x01,               /*   USAGE_MINIMUM (Button 1) */ \
    0x29, 0x08,               /*   USAGE_MAXIMUM (Button 8) */ \
    0x15, 0x00,               /*   LOGICAL_MINIMUM (0) */ \
    0x25, 0x01,               /*   LOGICAL_MAXIMUM (1) */ \
    0x75, 0x01,               /*   REPORT_SIZE (1) */ \
    0x95, 0x08,               /*   REPORT_COUNT (8) */ \
    0x81, 0x02,               /*   INPUT (Data,Var,Abs) */ \
    /* X and Y Axis */ \
    0x05, 0x01,               /*   USAGE_PAGE (Generic Desktop) */ \
    0x09, 0x01,               /*   USAGE (Pointer) */ \
    0xA1, 0x00,               /*   COLLECTION (Physical) */ \
    0x09, 0x30,               /*     USAGE (x) */ \
    0x09, 0x31,               /*     USAGE (y) */ \
    0x15, 0x00,               /*     LOGICAL_MINIMUM (0) */ \
    0x26, 0xff, 0x00,         /*     LOGICAL_MAXIMUM (255) */ \
    0x75, 0x08,               /*     REPORT_SIZE (8) */ \
    0x95, 0x02,               /*     REPORT_COUNT (2) */ \
    0x81, 0x02,               /*     INPUT (Data,Var,Abs) */ \
    0xc0,                     /*   END_COLLECTION */ \
    0xc0                      /* END_COLLECTION */




static const uint8_t hidReportDescriptor[] PROGMEM = {
  HIDDESC_MACRO(JOYSTICK_REPORT_ID),
  HIDDESC_MACRO(JOYSTICK2_REPORT_ID)
};

class Joystick_ {

private:
  uint8_t joystickId;
  uint8_t reportId;
  uint8_t olddata[JOYSTICK_DATA_SIZE];
  uint8_t state[JOYSTICK_STATE_SIZE];
  uint8_t flag;

public:
  uint8_t type;
  uint8_t data[JOYSTICK_DATA_SIZE];

  Joystick_(uint8_t initJoystickId, uint8_t initReportId) {
    // Setup HID report structure
    static bool usbSetup = false;
  
    if (!usbSetup) {
      static HIDSubDescriptor node(hidReportDescriptor, sizeof(hidReportDescriptor));
      HID().AppendDescriptor(&node);
      usbSetup = true;
    }
    
    // Initalize State
    joystickId = initJoystickId;
    reportId = initReportId;
  
    data[0] = 0;
    data[1] = 0;
    memcpy(olddata, data, JOYSTICK_DATA_SIZE);
    translateState(data, state);
    sendState(1);
  }

  void updateState() {
    if (memcmp(olddata, data, JOYSTICK_DATA_SIZE)) {    
      memcpy(olddata, data, JOYSTICK_DATA_SIZE);
      translateState(data, state);
      flag = 1;
    }
  }

  void sendState(uint8_t force = 0) {
    if (flag || force) {
      // HID().SendReport(Report number, array of values in same order as HID descriptor, length)
      HID().SendReport(reportId, state, JOYSTICK_STATE_SIZE);
      flag = 0;
    }
  }

};


Joystick_ Joystick[2] =
{
    Joystick_(0, JOYSTICK_REPORT_ID),
    Joystick_(1, JOYSTICK2_REPORT_ID)
};


void setup()
{
#ifdef DEBUG
    Serial.begin(9600);
#endif
}

void loop()
{
    currentState1 = controller1.getState();
    currentState2 = controller2.getState();


    // Only report controller1 state if it's changed
    if (currentState1 != lastState1)
    {
        bitWrite(Joystick[0].data[0], 0, !(currentState1 & SC_BTN_A)); //A1
        bitWrite(Joystick[0].data[0], 1, !(currentState1 & SC_BTN_B)); //B1
        bitWrite(Joystick[0].data[0], 2, !(currentState1 & SC_BTN_C)); //C1
        bitWrite(Joystick[0].data[0], 3, !(currentState1 & SC_BTN_START)); //Start1
        bitWrite(Joystick[0].data[0], 4, !(currentState1 & SC_BTN_X)); //X1
        bitWrite(Joystick[0].data[0], 5, !(currentState1 & SC_BTN_Y)); //Y1
        bitWrite(Joystick[0].data[0], 6, !(currentState1 & SC_BTN_Z)); //Z1
        bitWrite(Joystick[0].data[0], 7, !(currentState1 & SC_BTN_MODE)); //MODE1
        bitWrite(Joystick[0].data[1], 0, !(currentState1 & SC_BTN_UP)); //UP1
        bitWrite(Joystick[0].data[1], 1, !(currentState1 & SC_BTN_DOWN)); //DOWN1
        bitWrite(Joystick[0].data[1], 2, !(currentState1 & SC_BTN_LEFT)); //LEFT1
        bitWrite(Joystick[0].data[1], 3, !(currentState1 & SC_BTN_RIGHT)); //RIGHT1
        lastState1 = currentState1;
#ifdef DEBUG
    serdebugState(currentState1, "JP1: ");
#endif
    }

    // Only report controller2 state if it's changed
    if (currentState2 != lastState2)
    {
        bitWrite(Joystick[1].data[0], 0, !(currentState1 & SC_BTN_A)); //A2
        bitWrite(Joystick[1].data[0], 1, !(currentState1 & SC_BTN_B)); //B2
        bitWrite(Joystick[1].data[0], 2, !(currentState1 & SC_BTN_C)); //C2
        bitWrite(Joystick[1].data[0], 3, !(currentState1 & SC_BTN_START)); //Start2
        bitWrite(Joystick[1].data[0], 4, !(currentState1 & SC_BTN_X)); //X2
        bitWrite(Joystick[1].data[0], 5, !(currentState1 & SC_BTN_Y)); //Y2
        bitWrite(Joystick[1].data[0], 6, !(currentState1 & SC_BTN_Z)); //Z2
        bitWrite(Joystick[1].data[0], 7, !(currentState1 & SC_BTN_MODE)); //MODE2
        bitWrite(Joystick[1].data[1], 0, !(currentState1 & SC_BTN_UP)); //UP2
        bitWrite(Joystick[1].data[1], 1, !(currentState1 & SC_BTN_DOWN)); //DOWN2
        bitWrite(Joystick[1].data[1], 2, !(currentState1 & SC_BTN_LEFT)); //LEFT2
        bitWrite(Joystick[1].data[1], 3, !(currentState1 & SC_BTN_RIGHT)); //RIGHT2
        lastState2 = currentState2;
#ifdef DEBUG
    serdebugState(currentState1, "JP2: ");
#endif
    }

    Joystick[0].updateState();
    Joystick[1].updateState();
    Joystick[0].sendState();
    Joystick[1].sendState();  
}

void serdebugState(word currentState, char jp[6])
{
    Serial.print(jp);
    Serial.print((currentState & SC_CTL_ON)    ? "+" : "-");
    Serial.print((currentState & SC_BTN_UP)    ? "U" : "0");
    Serial.print((currentState & SC_BTN_DOWN)  ? "D" : "0");
    Serial.print((currentState & SC_BTN_LEFT)  ? "L" : "0");
    Serial.print((currentState & SC_BTN_RIGHT) ? "R" : "0");
    Serial.print((currentState & SC_BTN_START) ? "S" : "0");
    Serial.print((currentState & SC_BTN_A)     ? "A" : "0");
    Serial.print((currentState & SC_BTN_B)     ? "B" : "0");
    Serial.print((currentState & SC_BTN_C)     ? "C" : "0");
    Serial.print((currentState & SC_BTN_X)     ? "X" : "0");
    Serial.print((currentState & SC_BTN_Y)     ? "Y" : "0");
    Serial.print((currentState & SC_BTN_Z)     ? "Z" : "0");
    Serial.print((currentState & SC_BTN_MODE)  ? "M" : "0");
    Serial.print("\n");
}
