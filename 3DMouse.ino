// Version History
// ===============
// 3DMouse 1.0 by MajicDesigns
// - Refactored code and calculations
// - Included digital inputs for switches (#define INCLUDE_SWITCHES)
// - Rationalized DBEUG coding and information
// - DEBUG code eliminated when not enabled
// 
// -------------------------------------------
// Original version as joy4_tt_v2.ino by TeachingTech
// Hardware and assembly description at 
// https://www.printables.com/model/864950-open-source-spacemouse-space-mushroom-remix?utm_source=pocket_saves
// 
// Original code is the combination of multiple works by others:
// 1. Original code for the Space Mushroom by Shiura on 
//    Thingiverse: https://www.thingiverse.com/thing:5739462
//
// The next two from the comments on the instructables 
// page: https://www.instructables.com/Space-Mushroom-Full-6-DOFs-Controller-for-CAD-Appl/
// and the comments of Thingiverse: https://www.thingiverse.com/thing:5739462/comments
//
// 2. Code to emulate a 3DConnexion Space Mouse by jfedor: https://pastebin.com/gQxUrScV
//
// 3. This code was then remixed by BennyBWalker to include the above two 
//    sketches: https://pastebin.com/erhTgRBH.
// 
// 4. Four joystick remix code by fdmakara: https://www.thingiverse.com/thing:5817728
// 
// This work is a refactoring of all of these by TeachingTech.
// The basis is fdmakara's four joystick movement logic, with jfedor/BennyBWalker's HID 
// SpaceMouse emulation. The four joystick logic sketch was setup for the joystick library 
// instead of HID, so elements of this were omitted where not needed.
// During development debugging code was added to track exactly what was happening.
// On top of this, I have added more control of speed/direction and comments/links to 
// informative resources to try and explain what is happening in each phase.
//
// Spacemouse Emulation
// ====================
// I followed the instructions here from nebhead: 
// https://gist.github.com/nebhead/c92da8f1a8b476f7c36c032a0ac2592a
// with two key differences:
// 1. I changed the word 'DaemonBite' to 'Spacemouse' in all references.
// 2. I changed the VID and PID values as per jfedor's instructions: 
//    vid=0x256f, pid=0xc631 (SpaceMouse Pro Wireless (cabled))
// 
// When compiling and uploading, I select Arduino 
// AVR boards (in Sketchbook) > Spacemouse 
// and then the serial port. 
// 
// You will also need to download and install the 3DConnexion software: 
// https://3dconnexion.com/us/drivers-application/3dxware-10/
// If all goes well, the 3DConnexion software will show a SpaceMouse Pro 
// wireless when the Arduino is connected.
// -------------------------------------------

#include "HID-Project.h" // Arduino HID library by NicoHood: https://github.com/NicoHood/HID 

// Comment out the line to exclude key switches from the build
//#define INCLUDE_SWITCHES

// Helper macros/constexpr
//
static constexpr uint8_t HI8(int16_t v) { return(v >> 8); }
static constexpr uint8_t LO8(int16_t v) { return(v & 0xff); }

// Debugging
// =========
// 0: Debugging off. Set to this once everything is working.
// 1: Output raw joystick values. 0-1023 raw ADC 10-bit values
// 2: Output centered joystick values. Values should be approx -500 to +500, jitter around 0 at idle.
// 3: Output centered joystick values. Filtered for deadzone. Approx -500 to +500, locked to zero at idle.
// 4: Output translation and rotation values. Approx -800 to 800 depending on the parameter.
// 5: Output debug 4 and 5 side by side for direct cause and effect reference.
#define DEBUG 4

// Joystick Hardware
// =================
// Default Assembly when looking from above:
//    C           Y+
//    |           .
// B--+--D   X-...Z+...X+
//    |           .
//    A           Y-
//
// Wiring. Matches the first eight analogue pins of the Arduino Pro Micro (atmega32u4)
// Pin numbers are defined in the J[] structure initializer below.
const uint8_t NUM_ANALOG = 8;   // 4 joysticks * 2 inputs each

// Axes identifiers (array index) are matched to J[] order.
// Range should be 0 .. NUM_ANALOG-1
const uint8_t AX = 0;
const uint8_t AY = 1;
const uint8_t BX = 2;
const uint8_t BY = 3;
const uint8_t CX = 4;
const uint8_t CY = 5;
const uint8_t DX = 6;
const uint8_t DY = 7;

// Define a structure to hold all the data related to a pin
typedef struct
{
  uint8_t pin;      // the analog input pin number 
  char    label[4]; // the label identifier
  
  int16_t centerpoint;  // center point offset adjustment for this input pin
  int16_t raw;          // raw data read from this pin
  int16_t value;        // calculated actual value
} anaData_t;

// Initialize the Joystick data array with static data
anaData_t J[NUM_ANALOG] =
{
  { A1, "AX ", 0, 0, 0 }, // Joy A X-axis
  { A0, "AY ", 0, 0, 0 }, // Joy A Y-axis
  { A3, "BX ", 0, 0, 0 }, // Joy B X-axis
  { A2, "BY ", 0, 0, 0 }, // Joy B Y-axis
  { A7, "CX ", 0, 0, 0 }, // Joy C X-axis
  { A6, "CY ", 0, 0, 0 }, // Joy C Y-axis
  { A9, "DX ", 0, 0, 0 }, // Joy D X-axis
  { A8, "DY ", 0, 0, 0 }, // Joy D Y-axis
};

#ifdef INCLUDE_SWITCHES
// Digital Hardware
// ================
// switchesa are wired so that they will connect the digital input to
// ground. Wire one siode of the switch to ground and the other to the 
// Arduino pin. Switches are 'active' or 'pressed' when the value read 
// is LOW.
// Pin numbers are defined in the static initializer for the S array.
const uint16_t MAX_SWITCHES = 16;
const uint32_t DEBOUNCE_TIME = 30;    // debounce time in milliseconds
const uint8_t  NO_PIN = 0xff;         // no pin defined

typedef enum { IDLE, TRIGGERED, RETRY } state_t;

// Define a structure to hold all the data related to a digital pin
typedef struct
{
  uint8_t pin;    // the digital input pin number (0xff means unused)
  state_t state;  // current state for the debounce FSM

  bool      lastState; // the last digital state (ON = true)
  uint32_t  timeTrig;  // time the digital was triggered
  uint8_t   value;     // current reportable value for the the input (0/1)
} digData_t;

// Initialize the switch data array with static data
// Define the switches as required, use NO_PIN for the switches 
// not used. These will be skipped during processing and sent as 
// logical 0 in the HID report.
digData_t S[MAX_SWITCHES] =
{
  {      2, IDLE, true, 0, 0 }, // Switch  1
  {      3, IDLE, true, 0, 0 }, // Switch  2
  {      5, IDLE, true, 0, 0 }, // Switch  3
  {      7, IDLE, true, 0, 0 }, // Switch  4
  {     16, IDLE, true, 0, 0 }, // Switch  5
  { NO_PIN, IDLE, true, 0, 0 }, // Switch  6
  { NO_PIN, IDLE, true, 0, 0 }, // Switch  7
  { NO_PIN, IDLE, true, 0, 0 }, // Switch  8
  { NO_PIN, IDLE, true, 0, 0 }, // Switch  9
  { NO_PIN, IDLE, true, 0, 0 }, // Switch 10
  { NO_PIN, IDLE, true, 0, 0 }, // Switch 11
  { NO_PIN, IDLE, true, 0, 0 }, // Switch 12
  { NO_PIN, IDLE, true, 0, 0 }, // Switch 13
  { NO_PIN, IDLE, true, 0, 0 }, // Switch 14
  { NO_PIN, IDLE, true, 0, 0 }, // Switch 15
  { NO_PIN, IDLE, true, 0, 0 }  // Switch 16
};

#endif

// Sensitivity Adjustments
// =======================
// Dead Zone
// ---------
// Deadzone to filter out unintended movements due to jitter in 
// the raw data or calculated values.
// Recommended to have this as small as possible to enable smaller 
// knob movements but it depends on potentiometers.
// Increase if the mouse has small movements when it should 
// be idle or the mouse is too sensitive to subtle movements.
const int16_t VALUE_DZ = 3;
const int16_t TRANS_DZ = 10;
const int16_t ROT_DZ = 10;

// Translational Sensitivity
// -------------------------
// The final translation calculation is divided by this factor to 
// decrease the amount of movement. Decrease the value to decrease
// sensitivity. Sensitivity can also be changed in the 3DConnexion 
// driver.
const float SENS_TRANS = 1.0;
// Set the maximum value for the translation value.
// The value will be clamped to the range -MAX_TRANS..MAX_TRANS.
const int16_t MAX_TRANS = 500;

// Rotational Sensitivity
// ----------------------
// The final rotation calculation is divided by this factor to 
// decrease the amount of movement. Decrease the value to decrease
// sensitivity. Sensitivity can also be changed in the 3DConnexion 
// driver.
const float SENS_ROT = 1.0;
// Set the maximum value for the rotation value.
// The value will be clamped to the range -MAX_ROT..MAX_ROT.
const int16_t MAX_ROT = 500;

// Motion Direction Setting
// ------------------------
// Reverse the direction of translation/rotation calculations depending
// on requirements. Set to -1 to reverse, 1 to leave as-is.
// This setting can also be changed in the 3DConnexion software.
const int16_t INV_X = -1;   // Pan left/right
const int16_t INV_Y = -1;   // Pan up/down
const int16_t INV_Z =  1;   // Zoom in/out
const int16_t INV_RX =  1;  // Rotate around X axis (tilt front/back)
const int16_t INV_RY =  1;  // Rotate around Y axis (tilt left/right)
const int16_t INV_RZ = -1;  // Rotate around Z axis (twist left/right)

// USB HID Interface
// =================
// This portion sets up the communication with the 3DConnexion software. 
// The communication protocol is created here.
// hidReportDescriptor webpage can be found here: 
// https://eleccelerator.com/tutorial-about-usb-hid-report-descriptors/ 
static const uint8_t _hidReportDescriptor[] PROGMEM =
{
  0x05, 0x01,         // Usage Page (Generic Desktop)
  0x09, 0x08,         // 0x08: Usage (Multi-Axis)

  0xa1, 0x01,         //  Collection (Application)

  0xa1, 0x00,         //   Collection (Physical)
  0x85, 0x01,         //    Report ID
  //  0x16, 0x00, 0x80,   //    Logical minimum (-500)
  //  0x26, 0xff, 0x7f,   //    Logical maximum (500)
  0x16, LO8(-MAX_TRANS), HI8(-MAX_TRANS),   //    Logical minimum (-MAX_TRANS)
  0x26, LO8(MAX_TRANS), HI8(MAX_TRANS),     //    Logical maximum (MAX_TRANS)
  0x36, 0x00, 0x80,   //    Physical Minimum (-32768)
  0x46, 0xff, 0x7f,   //    Physical Maximum (32767)
  0x09, 0x30,         //    Usage (X)
  0x09, 0x31,         //    Usage (Y)
  0x09, 0x32,         //    Usage (Z)
  0x75, 0x10,         //    Report Size (16)
  0x95, 0x03,         //    Report Count (3)
  0x81, 0x02,         //    Input (variable,absolute)
  0xC0,               //   End Collection

  0xa1, 0x00,         //   Collection (Physical)
  0x85, 0x02,         //    Report ID
  //  0x16, 0x00, 0x80,   //    Logical minimum (-500)
  //  0x26, 0xff, 0x7f,   //    Logical maximum (500)
  0x16, LO8(-MAX_ROT), HI8(-MAX_ROT),   //    Logical minimum (-MAX_ROT)
  0x26, LO8(MAX_ROT), HI8(MAX_ROT),     //    Logical maximum (MAX_ROT)
  0x36, 0x00, 0x80,   //    Physical Minimum (-32768)
  0x46, 0xff, 0x7f,   //    Physical Maximum (32767)
  0x09, 0x33,         //    Usage (RX)
  0x09, 0x34,         //    Usage (RY)
  0x09, 0x35,         //    Usage (RZ)
  0x75, 0x10,         //    Report Size (16)
  0x95, 0x03,         //    Report Count (3)
  0x81, 0x02,         //    Input (variable,absolute)
  0xC0,               //   End Collection

#ifdef INCLUDE_SWITCHES
  0xa1, 0x00,         //   Collection (Physical)
  0x85, 0x03,         //    Report ID
  0x15, 0x00,         //    Logical Minimum (0)
  0x25, 0x01,         //    Logical Maximum (1)
  0x75, 0x01,         //    Report Size (1)
  0x95, MAX_SWITCHES, //    Report Count (MAX_SWITCHES)
  0x05, 0x09,         //    Usage Page (Button)
  0x19, 1,            //    Usage Minimum (Button #1)
  0x29, MAX_SWITCHES, //    Usage Maximum (Button #MAX_SWITCHES)
  0x81, 0x02,         //    Input (variable,absolute)
  0xC0,               //  End Collection
#endif

  0xC0                // End Collection
};

void sendHIDReport(int16_t rx, int16_t ry, int16_t rz, int16_t x, int16_t y, int16_t z)
// Function to send translation, rotation and swicthes data to the 
// 3DConnexion software using the HID protocol defined above. 
{
  const uint8_t DOF = 6;  // degrees of freedom

  // Two sets of analog data are sent - translation and then rotation.
  // For each, a 16 bit integer is split into two using bit shifting. 
  // The first 8 bits is low byte and the second 8 bts is high byte.
  uint8_t trans[DOF] = 
  { 
    LO8(x), HI8(x), HI8(y), LO8(y), HI8(z), LO8(z)
  };
  HID().SendReport(1, trans, DOF);

  uint8_t rot[DOF] =
  {
    LO8(rx), HI8(rx), LO8(ry), HI8(ry), LO8(rz), HI8(rz)
  };

  HID().SendReport(2, rot, DOF);

#ifdef INCLUDE_SWITCHES
  // One set of digital data is sent.
  // only 16 bits can be sent with this code.

  // pack the switches into 16 bits
  uint16_t sw = 0;
  for (uint8_t i = 0; i < MAX_SWITCHES; i++)
    sw |= (S[i].value << i);

  // create the data for transmission
  uint8_t data[2] = { LO8(sw), HI8(sw) };

  HID().SendReport(3, data, DOF);
#endif
}

void readJoysticks(void)
// Read and store analogue values for each joystick axis.
{
  for (uint8_t i = 0; i < NUM_ANALOG; i++)
    J[i].raw = analogRead(J[i].pin);
}

int16_t clamp(int16_t v, int16_t min, int16_t max)
// Ensure the value v is in the range [min, max].
{
  if (v < min) return(min);
  if (v > max) return(max);
  return(v);
}

constexpr int16_t sensAdjust(float v, float s)
{
  return ((int16_t) (v * s));
}

#ifdef INCLUDE_SWITCHES
void readSwitches(void)
// Read the digital switches. Looking for a debounced transition
// from HIGH->LOW or LOW->HIGH to set the currenbt reportable 
// value.
{
  for (uint8_t i = 0; i < MAX_SWITCHES; i++)
  {
    if (S[i].pin != NO_PIN)
    {
      bool v = digitalRead(S[i].pin) == LOW;

      switch (S[i].state)
      {
      case IDLE:    // waiting for the start of a change
        if (v == S[i].lastState)
          break;   // nothing to process as still the same
        S[i].state = TRIGGERED;
        S[i].timeTrig = millis();
        break;

      case TRIGGERED: // detected the start of change, delay debounce time
        if (millis() - S[i].timeTrig >= DEBOUNCE_TIME)
          S[i].state = RETRY;
        break;

      case RETRY:  // retry the digital read
        if (v != S[i].lastState)
        {
          S[i].value = (v ? 1 : 0);
          S[i].lastState = v;
          S[i].state = IDLE;
        }
        break;
      }
    }
  }
}
#endif

#if (DEBUG != 0)
char *fmtValue(uint16_t v)
{
  static char sz[10];

  sprintf(sz, "%4d", v);
  return(sz);
}

void debugInputs(bool showRaw = false)
// Display data currrently held for each analog and digital pin.
// if showRaw is true display the raw data rather than calculated data
{
  Serial.print("\n");
  for (uint8_t i = 0; i < NUM_ANALOG; i++)
  {
    Serial.print(J[i].label);
    Serial.print(fmtValue(showRaw ? J[i].raw : J[i].value));
    Serial.print((i < NUM_ANALOG - 1) ? " " : "");
  }

#ifdef INCLUDE_SWITCHES
  Serial.print(" | SW[1-");
  Serial.print(MAX_SWITCHES);
  Serial.print("]: ");
  for (uint8_t i = 0; i < MAX_SWITCHES; i++)
  {
    if (S[i].pin != NO_PIN)
      Serial.print(S[i].value);
    else
      Serial.print(".");
  }
#endif
}

void debugMovement(int16_t tx, int16_t ty, int16_t tz, int16_t rx, int16_t ry, int16_t rz, bool showCalc = false)
// Display the translation and rotation values supplied. 
// Will also show the ADC calculated values if showCalc is true.
{
  Serial.print("\n");
  if (showCalc)  // show the calculated values
  {
    for (uint8_t i = 0; i < NUM_ANALOG; i++)
    {
      Serial.print(J[i].label);
      Serial.print(fmtValue(J[i].value));
      Serial.print((i < NUM_ANALOG-1) ? " " : ""); 
    }
    Serial.print(" ||");
  }
  Serial.print(" TX "); Serial.print(fmtValue(tx));
  Serial.print(" TY "); Serial.print(fmtValue(ty));
  Serial.print(" TZ "); Serial.print(fmtValue(tz));
  Serial.print(" RX "); Serial.print(fmtValue(rx));
  Serial.print(" RY "); Serial.print(fmtValue(ry));
  Serial.print(" RZ "); Serial.print(fmtValue(rz));
}
#endif

void setup(void) 
{
  // Set HID protocol.
  static HIDSubDescriptor node(_hidReportDescriptor, sizeof(_hidReportDescriptor));
  HID().AppendDescriptor(&node);
  
#if (DEBUG != 0)
  // Begin Serial for debugging
  Serial.begin(57600);
#endif

#ifdef INCLUDE_SWITCHES
  // initialize the swich digital to correct state
  for (uint8_t i = 0; i < MAX_SWITCHES; i++)
    if (S[i].pin != NO_PIN)
      pinMode(S[i].pin, INPUT_PULLUP);
#endif

  // Read idle/center positions for joysticks.
  delay(100);
  readJoysticks();
  delay(100);
  readJoysticks();
  
  // now copy them to remember them
  for (uint8_t i = 0; i < NUM_ANALOG; i++)
    J[i].centerpoint = J[i].raw;
#if (DEBUG != 0)
  debugInputs(true);
#endif
}

void loop(void)
{
  // Read joystick values 0-1023
  readJoysticks();
#ifdef INCLUDE_SWITCHES
  readSwitches();
#endif

#if (DEBUG == 1)
  // Display the raw data 0..1023 from the ADC.
  debugInputs(true);
#endif

  // Subtract center position from measured position shift zero point.
  for (uint8_t i = 0; i < NUM_ANALOG; i++)
    J[i].value = J[i].raw - J[i].centerpoint;

#if (DEBUG == 2)
  // Display calculated joystick values.
  // Values should be approx -500 to +500, jitter around 0 at idle.
  debugInputs();
#endif

  // Filter movement values. 
  // Set to zero if calculated value is less than VALUE_DZ threshold.
  for (uint8_t i = 0; i < NUM_ANALOG; i++)
  {
    if (abs(J[i].value) < VALUE_DZ)
      J[i].value = 0;
  }

#if (DEBUG == 3)
  // Display filtered joystick values.
  // Approx -500 to +500, locked to zero at idle
  debugInputs();
#endif

  // Now derive overall translation and rotation values based on the 
  // joystick positions.
  int16_t transX, transY, transZ;   // translation values
  int16_t rotX, rotY, rotZ;         // rotation values

  // Translations
  transX = transY = transZ = 0;
  if ((abs(J[AX].value) > TRANS_DZ) && (abs(J[BX].value) > TRANS_DZ) &&
      (abs(J[CX].value) > TRANS_DZ) && (abs(J[DX].value) > TRANS_DZ))
  {
    transZ = (J[AX].value + J[BX].value + J[CX].value + J[DX].value) / 4;
  }
  else
  {
    transX = (J[CY].value - J[AY].value) / 2;  
    transY = (J[BY].value - J[DY].value) / 2;
  }

  // Rotations
  rotX = rotY = rotZ = 0;
  if ((abs(J[AY].value) > ROT_DZ) && (abs(J[BY].value) > ROT_DZ) &&
      (abs(J[CY].value) > ROT_DZ) && (abs(J[DY].value) > ROT_DZ))
  {
    rotZ = (J[AY].value + J[BY].value + J[CY].value + J[DY].value) / 4;
  }
  else 
  {
    rotX = (J[AX].value - J[CX].value) / 2;
    rotY = (J[BX].value - J[DX].value) / 2;
  }

  // Apply Sensitivity adjustment
  transX = sensAdjust((float)transX, SENS_TRANS);
  transY = sensAdjust((float)transY, SENS_TRANS);
  transZ = sensAdjust((float)transZ, SENS_TRANS);
  rotX = sensAdjust((float)rotX, SENS_ROT);
  rotY = sensAdjust((float)rotY, SENS_ROT);
  rotZ = sensAdjust((float)rotZ, SENS_ROT);
  
  // Ensure the values are clamped to the max values 
  // and reversed if necessary (see top of file for settings)
  transX = INV_X * clamp(transX, -MAX_TRANS, MAX_TRANS);
  transY = INV_Y * clamp(transY, -MAX_TRANS, MAX_TRANS);
  transZ = INV_Z * clamp(transZ, -MAX_TRANS, MAX_TRANS);
  rotX = INV_RX * clamp(rotX, -MAX_ROT, MAX_ROT);
  rotY = INV_RY * clamp(rotY, -MAX_ROT, MAX_ROT);
  rotZ = INV_RZ * clamp(rotZ, -MAX_ROT, MAX_ROT);

#if (DEBUG == 4)
  // Display translation and rotation values. 
  // Approx -800 to 800 depending on the parameter.
  debugMovement(transX, transY, transZ, rotX, rotY, rotZ, false);
#endif
  

#if (DEBUG == 5)
  // Display Report debug 3 and 4 side by side for direct reference.
  // Very useful if you need to alter which inputs are used in the 
  // arithmetic above.    
  debugMovement(transX, transY, transZ, rotX, rotY, rotZ, true);
#endif

  // Finally, send data to the 3DConnexion software.
  // The correct order for me was determined after trial and error
  sendHIDReport(rotX, rotZ, rotY, transX, transZ, transY);
}
