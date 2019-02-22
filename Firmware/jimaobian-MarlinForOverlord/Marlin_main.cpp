/* -*- c++ -*- */

/*
   Reprap firmware based on Sprinter and grbl.
   Copyright (C) 2011 Camiel Gubbels / Erik van der Zalm

   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
   This firmware is a mashup between Sprinter and grbl.
   (https://github.com/kliment/Sprinter)
   (https://github.com/simen/grbl/tree)

   It has preliminary support for Matthew Roberts advance algorithm
   http://reprap.org/pipermail/reprap-dev/2011-May/003323.html
 */

#include "Marlin.h"

#include "ultralcd.h"
#include "UltiLCD2.h"
#include "planner.h"
#include "stepper.h"
#include "temperature.h"
#include "motion_control.h"
#include "cardreader.h"
#include "watchdog.h"
#include "ConfigurationStore.h"
#include "lifetime_stats.h"
#include "electronics_test.h"
#include "language.h"
#include "pins_arduino.h"

#include "UltiLCD2_low_lib.h"
#include "UltiLCD2_hi_lib.h"
#include "UltiLCD2_menu_print.h"
#include "UltiLCD2_menu_maintenance.h"
#include "fitting_bed.h"

#include <avr/sleep.h>

#include "SDUPS.h"

#include "fitting_bed.h"

#if NUM_SERVOS > 0
  #include "Servo.h"
#endif

#if defined(DIGIPOTSS_PIN) && DIGIPOTSS_PIN > -1
  #include <SPI.h>
#endif

#define VERSION_STRING  "1.0.0"

// look here for descriptions of gcodes: http://linuxcnc.org/handbook/gcode/g-code.html
// http://objects.reprap.org/wiki/Mendel_User_Manual:_RepRapGCodes

//Implemented Codes
//-------------------
// G0  -> G1
// G1  - Coordinated Movement X Y Z E
// G2  - CW ARC
// G3  - CCW ARC
// G4  - Dwell S<seconds> or P<milliseconds>
// G10 - retract filament according to settings of M207
// G11 - retract recover filament according to settings of M208
// G28 - Home all Axis
// G90 - Use Absolute Coordinates
// G91 - Use Relative Coordinates
// G92 - Set current position to coordinates given

//RepRap M Codes
// M0   - Unconditional stop - Wait for user to press a button on the LCD (Only if ULTRA_LCD is enabled)
// M1   - Same as M0
// M104 - Set extruder target temp
// M105 - Read current temp
// M106 - Fan on
// M107 - Fan off
// M109 - Wait for extruder current temp to reach target temp.
// M114 - Display current position

//Custom M Codes
// M17  - Enable/Power all stepper motors
// M18  - Disable all stepper motors; same as M84
// M20  - List SD card
// M21  - Init SD card
// M22  - Release SD card
// M23  - Select SD file (M23 filename.g)
// M24  - Start/resume SD print
// M25  - Pause SD print
// M26  - Set SD position in bytes (M26 S12345)
// M27  - Report SD print status
// M28  - Start SD write (M28 filename.g)
// M29  - Stop SD write
// M30  - Delete file from SD (M30 filename.g)
// M31  - Output time since last M109 or SD card start to serial
// M42  - Change pin status via gcode Use M42 Px Sy to set pin x to value y, when omitting Px the onboard led will be used.
// M80  - Turn on Power Supply
// M81  - Turn off Power Supply
// M82  - Set E codes absolute (default)
// M83  - Set E codes relative while in Absolute Coordinates (G90) mode
// M84  - Disable steppers until next move,
//        or use S<seconds> to specify an inactivity timeout, after which the steppers will be disabled.  S0 to disable the timeout.
// M85  - Set inactivity shutdown timer with parameter S<seconds>. To disable set zero (default)
// M92  - Set axis_steps_per_unit - same syntax as G92
// M114 - Output current position to serial port
// M115 - Capabilities string
// M117 - display message
// M119 - Output Endstop status to serial port
// M126 - Solenoid Air Valve Open (BariCUDA support by jmil)
// M127 - Solenoid Air Valve Closed (BariCUDA vent to atmospheric pressure by jmil)
// M128 - EtoP Open (BariCUDA EtoP = electricity to air pressure transducer by jmil)
// M129 - EtoP Closed (BariCUDA EtoP = electricity to air pressure transducer by jmil)
// M140 - Set bed target temp
// M190 - Wait for bed current temp to reach target temp.
// M200 - Set filament diameter
// M201 - Set max acceleration in units/s^2 for print moves (M201 X1000 Y1000)
// M202 - Set max acceleration in units/s^2 for travel moves (M202 X1000 Y1000) Unused in Marlin!!
// M203 - Set maximum feedrate that your machine can sustain (M203 X200 Y200 Z300 E10000) in mm/sec
// M204 - Set default acceleration: S normal moves T filament only moves (M204 S3000 T7000) im mm/sec^2  also sets minimum segment time in ms (B20000) to prevent buffer underruns and M20 minimum feedrate
// M205 -  advanced settings:  minimum travel speed S=while printing T=travel only,  B=minimum segment time X= maximum xy jerk, Z=maximum Z jerk, E=maximum E jerk
// M206 - set additional homeing offset
// M207 - set retract length S[positive mm] F[feedrate mm/sec] Z[additional zlift/hop]
// M208 - set recover=unretract length S[positive mm surplus to the M207 S*] F[feedrate mm/sec]
// M209 - S<1=true/0=false> enable automatic retract detect if the slicer did not support G10/11: every normal extrude-only move will be classified as retract depending on the direction.
// M218 - set hotend offset (in mm): T<extruder_number> X<offset_on_X> Y<offset_on_Y>
// M220 S<factor in percent>- set speed factor override percentage
// M221 S<factor in percent>- set extrude factor override percentage
// M240 - Trigger a camera to take a photograph
// M280 - set servo position absolute. P: servo index, S: angle or microseconds
// M300 - Play beepsound S<frequency Hz> P<duration ms>
// M301 - Set PID parameters P I and D
// M302 - Allow cold extrudes, or set the minimum extrude S<temperature>.
// M303 - PID relay autotune S<temperature> sets the target temperature. (default target temperature = 150C)
// M304 - Set bed PID parameters P I and D
// M400 - Finish all moves
// M500 - stores paramters in EEPROM
// M501 - reads parameters from EEPROM (if you need reset them after you changed them temporarily).
// M502 - reverts to the default "factory settings".  You still need to store them in EEPROM afterwards if you want to.
// M503 - print the current settings (from memory not from eeprom)
// M540 - Use S[0|1] to enable or disable the stop SD card print on endstop hit (requires ABORT_ON_ENDSTOP_HIT_FEATURE_ENABLED)
// M600 - Pause for filament change X[pos] Y[pos] Z[relative lift] E[initial retract] L[later retract distance for removal]
// M907 - Set digital trimpot motor current using axis codes.
// M908 - Control digital trimpot directly.
// M350 - Set microstepping mode.
// M351 - Toggle MS1 MS2 pins directly.
// M923 - Select file and start printing directly (can be used from other SD file)
// M928 - Start SD logging (M928 filename.g) - ended by M29
// M999 - Restart after being stopped by error

//Stepper Movement Variables

//===========================================================================
//=============================imported variables============================
//===========================================================================


//===========================================================================
//=============================public variables=============================
//===========================================================================
#ifdef SDSUPPORT
CardReader card;
#endif
float homing_feedrate[] = HOMING_FEEDRATE;
bool axis_relative_modes[] = AXIS_RELATIVE_MODES;
int feedmultiply=100; //100->1 200->2
int saved_feedmultiply;
int extrudemultiply[EXTRUDERS]=ARRAY_BY_EXTRUDERS(100, 100, 100); //100->1 200->2
float current_position[NUM_AXIS] = { 0.0, 0.0, 0.0, 0.0 };
float add_homeing[3]={0,0,0};
float min_pos[3] = { 0, 0, 0 };
float max_pos[3] = { 0, 0, 0 };
// Extruder offset, only in XY plane
#if EXTRUDERS > 1
float extruder_offset[2][EXTRUDERS] = {
  #if defined(EXTRUDER_OFFSET_X) && defined(EXTRUDER_OFFSET_Y)
  EXTRUDER_OFFSET_X, EXTRUDER_OFFSET_Y
  #endif
};
#endif
uint8_t active_extruder = 0;
uint8_t fanSpeed=0;
uint8_t fanSpeedPercent=100;
uint8_t targetFanSpeed=0;
int targetFeedmultiply=0;


struct machinesettings {
  machinesettings() : has_saved_settings(0) {
  }
  int feedmultiply;
  int HotendTemperature[EXTRUDERS];
  int BedTemperature;
  uint8_t fanSpeed;
  int extrudemultiply[EXTRUDERS];
  long max_acceleration_units_per_sq_second[NUM_AXIS];
  float max_feedrate[NUM_AXIS];
  float acceleration;
  float minimumfeedrate;
  float mintravelfeedrate;
  long minsegmenttime;
  float max_xy_jerk;
  float max_z_jerk;
  float max_e_jerk;
  uint8_t has_saved_settings;
};
machinesettings machinesettings_tempsave[10];

#ifdef SERVO_ENDSTOPS
int servo_endstops[] = SERVO_ENDSTOPS;
int servo_endstop_angles[] = SERVO_ENDSTOP_ANGLES;
#endif
#ifdef BARICUDA
int ValvePressure=0;
int EtoPPressure=0;
#endif

#ifdef FWRETRACT
bool autoretract_enabled=false;
bool retracted=false;
float retract_length=6, retract_feedrate=80*60, retract_zlift=0.8;
  #if EXTRUDERS > 1
float extruder_swap_retract_length=16.0;
  #endif
float retract_recover_length=0, retract_recover_feedrate=25*60;
#endif

uint8_t printing_state;




uint8_t Device_type = 0;

bool Device_isGate = false;
bool Device_isNewHeater = false;
bool Device_isPro = false;
bool Device_isWifi = false;
bool Device_isBedHeat = false;
bool Device_isLevelSensor = false;
bool Device_isABS = false;
bool Device_isPowerSaving = false;
bool Device_isBattery = false;

bool isBedPreheat = false;

bool Device_isNewPCB = false;

int PID_MAX = 160;

int FILAMENT_FORWARD_LENGTH = FILAMENT_FORWARD_LENGTH_PRO;
int FILAMENT_REVERSAL_LENGTH = FILAMENT_REVERSAL_LENGTH_PRO;

unsigned int dropsegments = DROP_SEGMENTS;


bool storeDevice(uint8_t type)
{
  eeprom_write_byte((uint8_t*)EEPROM_DEVICE_OFFSET , type);
  eeprom_write_byte((uint8_t*)EEPROM_DEVICE_OFFSET + 1 , type);
  eeprom_write_byte((uint8_t*)EEPROM_DEVICE_OFFSET + 2 , type);
  
  return (eeprom_read_byte((uint8_t*)EEPROM_DEVICE_OFFSET) == type
      && eeprom_read_byte((uint8_t*)EEPROM_DEVICE_OFFSET+1) == type
      && eeprom_read_byte((uint8_t*)EEPROM_DEVICE_OFFSET+2) == type);
}

void retrieveDevice()
{
  WRITE(PCB_VERSION_PIN, HIGH);
  SET_INPUT(PCB_VERSION_PIN);
  delay(1);
  if (READ(PCB_VERSION_PIN)) {
    Device_isNewPCB = false; //old
  }
  else{
    Device_isNewPCB = true; //new
  }
  
  Device_type =  eeprom_read_byte((const uint8_t*)EEPROM_DEVICE_OFFSET);
  if (Device_type != eeprom_read_byte((const uint8_t*)EEPROM_DEVICE_OFFSET+1)
      || Device_type != eeprom_read_byte((const uint8_t*)EEPROM_DEVICE_OFFSET+2)) {
    Device_type = 0;
    return;
  }
  
  switch (Device_type) {
    case OVERLORD_TYPE_PNHW:
      Device_isGate = false;
      Device_isNewHeater = true;
      Device_isPro = true;
      Device_isWifi = true;
      Device_isBedHeat = true;
      Device_isLevelSensor = false;
      Device_isABS = true;
      Device_isBattery = true;
      break;
    case OVERLORD_TYPE_PNHL:
      Device_isGate = true;
      Device_isNewHeater = true;
      Device_isPro = true;
      Device_isWifi = false;
      Device_isBedHeat = true;
     Device_isLevelSensor = false;
      Device_isABS = false;
      Device_isBattery = false;
      break;
    case OVERLORD_TYPE_PNH:
      Device_isGate = false;
      Device_isNewHeater = true;
      Device_isPro = true;
      Device_isWifi = false;
      Device_isBedHeat = true;
      Device_isLevelSensor = false;
      Device_isABS = true;
      Device_isBattery = true;
      break;
    case OVERLORD_TYPE_P:
      Device_isGate = false;
      Device_isNewHeater = false;
      Device_isPro = true;
      Device_isWifi = false;
      Device_isBedHeat = true;
      Device_isLevelSensor = false;
      Device_isABS = true;
      Device_isBattery = true;
      break;
    case OVERLORD_TYPE_MBNH:
      Device_isGate = false;
      Device_isNewHeater = true;
      Device_isPro = false;
      Device_isWifi = false;
      Device_isBedHeat = true;
      Device_isLevelSensor = false;
      Device_isABS = true;
      Device_isBattery = false;
      break;
    case OVERLORD_TYPE_MNH:
      Device_isGate = false;
      Device_isNewHeater = true;
      Device_isPro = false;
      Device_isWifi = false;
      Device_isBedHeat = false;
      Device_isLevelSensor = false;
      Device_isABS = false;
      Device_isBattery = false;
      break;
    case OVERLORD_TYPE_MB:
      Device_isGate = false;
      Device_isNewHeater = false;
      Device_isPro = false;
      Device_isWifi = false;
      Device_isBedHeat = true;
      Device_isLevelSensor = false;
      Device_isABS = true;
      Device_isBattery = false;
      break;
    case OVERLORD_TYPE_M:
      Device_isGate = false;
      Device_isNewHeater = false;
      Device_isPro = false;
      Device_isWifi = false;
      Device_isBedHeat = false;
      Device_isLevelSensor = false;
      Device_isABS = false;
      Device_isBattery = false;
      break;
    case OVERLORD_TYPE_PS:
      Device_isGate = false;
      Device_isNewHeater = true;
      Device_isPro = true;
      Device_isWifi = false;
      Device_isBedHeat = true;
      Device_isLevelSensor = true;
      Device_isABS = true;
      Device_isBattery = false;
      break;
    case OVERLORD_TYPE_MS:
      Device_isGate = false;
      Device_isNewHeater = true;
      Device_isPro = false;
      Device_isWifi = false;
      Device_isBedHeat = false;
      Device_isLevelSensor = true;
      Device_isABS = false;
      Device_isBattery = false;
      break;
    case OVERLORD_TYPE_PSD:
      Device_isGate = true;
      Device_isNewHeater = true;
      Device_isPro = true;
      Device_isWifi = false;
      Device_isBedHeat = true;
      Device_isLevelSensor = true;
      Device_isABS = true;
      Device_isBattery = false;
      break;
    default:
      Device_type = 0;
      return;
      break;
  }
  
  if (Device_isNewHeater) {
    PID_MAX = 255;
  }
  else{
    PID_MAX = 160;
  }

  Device_isPowerSaving = Device_isNewPCB;
  
  if (Device_isPro) {
    FILAMENT_FORWARD_LENGTH = FILAMENT_FORWARD_LENGTH_PRO;
    FILAMENT_REVERSAL_LENGTH = FILAMENT_REVERSAL_LENGTH_PRO;
  }
  else{
    FILAMENT_FORWARD_LENGTH = FILAMENT_FORWARD_LENGTH_MINI;
    FILAMENT_REVERSAL_LENGTH = FILAMENT_REVERSAL_LENGTH_MINI;
  }
}



#ifdef PowerOnDemand
uint8_t powerOnDemandState=PowerOnDemandSleeping;
unsigned long powerOnDemandTimer=millis();
unsigned long powerOnDemandEnergyTimer=millis();
#endif

#ifdef SoftwareAutoLevel
float touchPlateOffset;
uint8_t fittingBedTorque;
#endif

void wifiSDInit()
{
  SET_OUTPUT(SD_POWER_PIN);
  SET_OUTPUT(SD_SELECT_PIN);
  WRITE(SD_POWER_PIN, LOW);
  WRITE(SD_SELECT_PIN, LOW);
}


bool isWindowsServerStarted = false;
bool isWindowsPrinting = false;
bool isWindowsSD = false;

  #define WIFI_SD_OVERLORD 1
  #define WIFI_SD_WINDOWS 2


void wifiSDChangeMaster(uint8_t role)
{
  unsigned long timerLocal = millis();
  if (role == WIFI_SD_OVERLORD) {
    WRITE(SD_POWER_PIN, HIGH);
    while (true) {
      manage_heater();
      manage_inactivity();
      if (millis()-timerLocal>=1000UL) {
        break;
      }
    }
    WRITE(SD_SELECT_PIN, LOW);
    WRITE(SD_POWER_PIN, LOW);
    isWindowsSD = false;
  }
  else if (role == WIFI_SD_WINDOWS) {
    isWindowsSD = true;
    WRITE(SD_POWER_PIN, HIGH);
    while (true) {
      manage_heater();
      manage_inactivity();
      if (millis()-timerLocal>=1000UL) {
        break;
      }
    }
    WRITE(SD_SELECT_PIN, HIGH);
    WRITE(SD_POWER_PIN, LOW);
  }
}


//===========================================================================
//=============================private variables=============================
//===========================================================================
const char axis_codes[NUM_AXIS] = {'X', 'Y', 'Z', 'E'};
static float destination[NUM_AXIS] = {  0.0, 0.0, 0.0, 0.0};
static float delta[3] = {0.0, 0.0, 0.0};
static float offset[3] = {0.0, 0.0, 0.0};
static bool home_all_axis = true;
float feedrate = 1500.0, next_feedrate, saved_feedrate;
static long gcode_N, gcode_LastN, Stopped_gcode_LastN = 0;

static bool relative_mode = false;  //Determines Absolute or Relative Coordinates

static char cmdbuffer[BUFSIZE][MAX_CMD_SIZE];

#define FromSerial -2
#define FromFlash -3
//#define FromSDCard 2

static int32_t commandFrom[BUFSIZE];

static int bufindr = 0;
static int bufindw = 0;
static int buflen = 0;
//static int i = 0;
static char serial_char;
static int serial_count = 0;
static boolean comment_mode = false;
static char *strchr_pointer; // just a pointer to find chars in the cmd string like X, Y, Z, E, etc

const int sensitive_pins[] = SENSITIVE_PINS; // Sensitive pin list for M42

//static float tt = 0;
//static float bt = 0;

//Inactivity shutdown variables
unsigned long previous_millis_cmd = 0;
static unsigned long max_inactive_time = 0;
static unsigned long stepper_inactive_time = DEFAULT_STEPPER_DEACTIVE_TIME*1000l;

unsigned long starttime=0;
unsigned long stoptime=0;
unsigned long pausetime=0;


static uint8_t tmp_extruder;


uint8_t Stopped = false;

#if NUM_SERVOS > 0
Servo servos[NUM_SERVOS];
#endif

#define RamCommandBufSize 64

static bool isEnqueueingCommand = false;
static const char *enqueueingFlashCommandBuf[EnqueueingCommandBufSize];
static char *enqueueingRamCommandBuf[EnqueueingCommandBufSize];
static bool isEnqueueingRamCommand[EnqueueingCommandBufSize];
static int enqueueingCommandBufIndex=0;
static int enqueueingCommandLenth=0;
static int enqueueingCommandIndex=0;


volatile uint8_t stepperTorqueX=0;
volatile uint8_t stepperTorqueY=0;
volatile uint8_t stepperTorqueZ=0;

uint8_t languageType;

//===========================================================================
//=============================ROUTINES=============================
//===========================================================================

void get_arc_coordinates();
bool setTargetedHotend(int code);

void serial_echopair_P(const char *s_P, float v)
{
  serialprintPGM(s_P); SERIAL_ECHO(v);
}
void serial_echopair_P(const char *s_P, double v)
{
  serialprintPGM(s_P); SERIAL_ECHO(v);
}
void serial_echopair_P(const char *s_P, unsigned long v)
{
  serialprintPGM(s_P); SERIAL_ECHO(v);
}

extern "C" {
extern unsigned int __bss_end;
extern unsigned int __heap_start;
extern void *__brkval;

int freeMemory() {
  int free_memory;

  if((int)__brkval == 0)
    free_memory = ((int)&free_memory) - ((int)&__bss_end);
  else
    free_memory = ((int)&free_memory) - ((int)__brkval);

  return free_memory;
}
}

//Clear all the commands in the ASCII command buffer, to make sure we have room for abort commands.
void clear_command_queue()
{
  if (buflen > 0)
  {
    bufindw = (bufindr + 1)%BUFSIZE;
    buflen = 1;
  }
}


void enquecommand(const char *cmd)
{
  uint8_t ramCommandBufLenth = strnlen(cmd, RamCommandBufSize-1)+1;

  char *ramCommandBufPtr = (char *)malloc(ramCommandBufLenth);

  if (ramCommandBufPtr==NULL) {
    SERIAL_ECHOLN(freeMemory());
    Stop(STOP_REASON_OUT_OF_MEMORY);
    return;
  }

  strncpy(ramCommandBufPtr, cmd, RamCommandBufSize-1);
  ramCommandBufPtr[ramCommandBufLenth-1]=0;

  if (enqueueingCommandLenth < EnqueueingCommandBufSize) {
    enqueueingRamCommandBuf[(enqueueingCommandIndex + enqueueingCommandLenth) % EnqueueingCommandBufSize] = ramCommandBufPtr;
    isEnqueueingRamCommand[(enqueueingCommandIndex + enqueueingCommandLenth) % EnqueueingCommandBufSize] =true;
    enqueueingCommandLenth += 1;

    SERIAL_DEBUGPGM("enqueing \"");
    SERIAL_DEBUGLN(cmd);
    SERIAL_DEBUGLNPGM("\"");

    isEnqueueingCommand = true;
  }
  else{
    SERIAL_ERROR_START;
    SERIAL_ERROR("enqueing full");
  }
}

void enquecommand_P(const char *cmd)
{
  if (enqueueingCommandLenth < EnqueueingCommandBufSize) {

    enqueueingFlashCommandBuf[(enqueueingCommandIndex + enqueueingCommandLenth) % EnqueueingCommandBufSize] = cmd;
    isEnqueueingRamCommand[(enqueueingCommandIndex + enqueueingCommandLenth) % EnqueueingCommandBufSize] =false;
    enqueueingCommandLenth += 1;

//    SERIAL_DEBUGPGM("enqueing \"");
//    SERIAL_DEBUGPGM(cmd);
//    SERIAL_DEBUGLNPGM("\"");
    isEnqueueingCommand = true;
  }
  else{
    SERIAL_ERROR_START;
    SERIAL_ERROR("enqueing full");
  }
}

bool is_command_queued()
{
  return enqueueingCommandLenth > 0;
}

uint8_t commands_queued()
{
  return enqueueingCommandLenth;
}

void discardEnqueueingCommand()
{
  for (int i=0; i<enqueueingCommandLenth; i++) {
    if (isEnqueueingRamCommand[enqueueingCommandIndex]) {
      free(enqueueingRamCommandBuf[enqueueingCommandIndex]);
    }
    enqueueingCommandIndex = (enqueueingCommandIndex + 1) % EnqueueingCommandBufSize;
  }
  enqueueingCommandLenth=0;
  isEnqueueingCommand=false;
  enqueueingCommandBufIndex =0;
}

void discardCommandInBuffer()
{
  for (int i=0; i<BUFSIZE; i++) {
    cmdbuffer[i][0]='\0';
  }
}

bool isCommandInBuffer()
{
  return buflen > 0;
}

void setup_killpin()
{
#if defined(KILL_PIN) && KILL_PIN > -1
  SET_INPUT(KILL_PIN);
  WRITE(KILL_PIN,HIGH);
#endif
}

void setup_photpin()
{
#if defined(PHOTOGRAPH_PIN) && PHOTOGRAPH_PIN > -1
  SET_OUTPUT(PHOTOGRAPH_PIN);
  WRITE(PHOTOGRAPH_PIN, LOW);
#endif
}

void setup_powerhold()
{
#if defined(SUICIDE_PIN) && SUICIDE_PIN > -1
  SET_OUTPUT(SUICIDE_PIN);
  WRITE(SUICIDE_PIN, HIGH);
#endif
#if defined(PS_ON_PIN) && PS_ON_PIN > -1
  SET_OUTPUT(PS_ON_PIN);
  WRITE(PS_ON_PIN, PS_ON_AWAKE);
#endif
}

void suicide()
{
#if defined(SUICIDE_PIN) && SUICIDE_PIN > -1
  SET_OUTPUT(SUICIDE_PIN);
  WRITE(SUICIDE_PIN, LOW);
#endif
}

void servo_init()
{
#if (NUM_SERVOS >= 1) && defined(SERVO0_PIN) && (SERVO0_PIN > -1)
  servos[0].attach(SERVO0_PIN);
#endif
#if (NUM_SERVOS >= 2) && defined(SERVO1_PIN) && (SERVO1_PIN > -1)
  servos[1].attach(SERVO1_PIN);
#endif
#if (NUM_SERVOS >= 3) && defined(SERVO2_PIN) && (SERVO2_PIN > -1)
  servos[2].attach(SERVO2_PIN);
#endif
#if (NUM_SERVOS >= 4) && defined(SERVO3_PIN) && (SERVO3_PIN > -1)
  servos[3].attach(SERVO3_PIN);
#endif
#if (NUM_SERVOS >= 5)
  #error "TODO: enter initalisation code for more servos"
#endif

  // Set position of Servo Endstops that are defined
#ifdef SERVO_ENDSTOPS
  for(int8_t i = 0; i < 3; i++)
  {
    if(servo_endstops[i] > -1) {
      servos[servo_endstops[i]].write(servo_endstop_angles[i * 2 + 1]);
    }
  }
#endif
}


void newPowerInit()
{
#ifdef BatteryPin
  SET_INPUT(BatteryPin);
  WRITE(BatteryPin, LOW);
#endif

#ifdef PowerCheckPin
  SET_INPUT(PowerCheckPin);
  WRITE(PowerCheckPin, LOW);
#endif

#ifdef SLEEP_PIN
  SET_OUTPUT(SLEEP_PIN);
  WRITE(SLEEP_PIN, LOW);
#endif

#if ENERGE_PIN
  SET_OUTPUT(ENERGE_PIN);
  WRITE(ENERGE_PIN, HIGH);
#endif
}

void newEnergeSleep()
{
  WRITE(ENERGE_PIN, LOW);
}

void newEnergeWakeUp()
{
  WRITE(ENERGE_PIN, HIGH);
}

void newPowerSleep()
{
  WRITE(SLEEP_PIN, LOW);
}

void newPowerWakeUp()
{
  WRITE(SLEEP_PIN, HIGH);
}

void sleepISR(){
  sleep_disable();
  detachInterrupt(4);
  newEnergeWakeUp();
  watchdog_reset();
  watchdog_init();
}

void sleepAll(){
//  SERIAL_DEBUGLN("sleeping");

  lcd_lib_i2c_stop();

  delayMicroseconds(10);
  lcd_lib_RGB_off();
  delayMicroseconds(10);
  lcd_lib_oled_off();
  watchdog_disable();

  newPowerSleep();
  newEnergeSleep();

  delayMicroseconds(200);
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  sleep_enable();
  attachInterrupt(4, sleepISR, LOW);

  sleep_cpu();
  delayMicroseconds(200);
//  SERIAL_DEBUGLN("waking up");

  powerOnDemandTimer=millis();
  powerOnDemandEnergyTimer = millis();

  lcd_clear_cache();
  
  lcd_lib_init();
  lcd_lib_clear();

}




void gateInit()
{
  SET_INPUT(GATE_PIN);
  WRITE(GATE_PIN, HIGH);
}

bool gateOpened()
{
  static bool gateState = false;
  static uint8_t gateStateDelay = 0;
  bool newGateState = READ(GATE_PIN);

  if (gateState != newGateState)
  {
    if (gateStateDelay) {
      gateStateDelay--;
    }
    else{
      gateState = newGateState;
    }
  }else{
    gateStateDelay = 20;
  }

  return gateState;
}

void storeLanguage(uint8_t language)
{
  eeprom_write_byte((uint8_t*)EEPROM_LANGUAGE_OFFSET, language);
  languageType = language;
}

void retriveLanguage()
{
  languageType = eeprom_read_byte((const uint8_t*)EEPROM_LANGUAGE_OFFSET);
  if (languageType > 2) {
    languageType = LANGUAGE_ENGLISH;
  }
}

void setup()
{
  // Check startup - does nothing if bootloader sets MCUSR to 0
  MYSERIAL.begin(BAUDRATE);
  SERIAL_PROTOCOLLNPGM("start");
  SERIAL_ECHO_START;
  
  byte mcu = MCUSR;
  if(mcu & 1) SERIAL_ECHOLNPGM(MSG_POWERUP);
  if(mcu & 2) SERIAL_ECHOLNPGM(MSG_EXTERNAL_RESET);
  if(mcu & 4) SERIAL_ECHOLNPGM(MSG_BROWNOUT_RESET);
  if(mcu & 8) SERIAL_ECHOLNPGM(MSG_WATCHDOG_RESET);
  if(mcu & 32) SERIAL_ECHOLNPGM(MSG_SOFTWARE_RESET);
  
  retrieveDevice();
  newPowerInit();
  if (Device_isPowerSaving && !(mcu & 2) && (mcu & 1)) {
    sleepAll();
  }
  setup_killpin();
  setup_powerhold();
  

  MCUSR=0;
  watchdog_disable();
  SERIAL_ECHOPGM(MSG_MARLIN);
  SERIAL_ECHOLNPGM(VERSION_STRING);
#ifdef STRING_VERSION_CONFIG_H
  #ifdef STRING_CONFIG_H_AUTHOR
  SERIAL_ECHO_START;
  SERIAL_ECHOPGM(MSG_CONFIGURATION_VER);
  SERIAL_ECHOPGM(STRING_VERSION_CONFIG_H);
  SERIAL_ECHOPGM(MSG_AUTHOR);
  SERIAL_ECHOLNPGM(STRING_CONFIG_H_AUTHOR);
  SERIAL_ECHOPGM("Compiled: ");
  SERIAL_ECHOLNPGM(__DATE__);
  #endif
#endif
  SERIAL_ECHO_START;
  SERIAL_ECHOPGM(MSG_FREE_MEMORY);
  SERIAL_ECHO(freeMemory());
  SERIAL_ECHOPGM(MSG_PLANNER_BUFFER_BYTES);
  SERIAL_ECHOLN((int)sizeof(block_t)*BLOCK_BUFFER_SIZE);
  for(int8_t i = 0; i < BUFSIZE; i++)
  {
    commandFrom[i] = FromSerial;
  }

  // loads data from EEPROM if available else uses defaults (and resets step acceleration rate)
  Config_RetrieveSettings();
  lifetime_stats_init();

  SERIAL_DEBUGLNPGM("plainFactor:");
  SERIAL_DEBUGLN(plainFactorA*1000000.0);
  SERIAL_DEBUGLN(plainFactorB*1000000.0);
  SERIAL_DEBUGLN(plainFactorC*1000000.0);

  SERIAL_DEBUGLNPGM("plainFactor:");
  SERIAL_DEBUGLN(plainFactorAAC*1000000.0);
  SERIAL_DEBUGLN(plainFactorBBC*1000000.0);
  SERIAL_DEBUGLN(plainFactorCAC*1000000.0);
  SERIAL_DEBUGLN(plainFactorCBC*1000000.0);

  SERIAL_DEBUGPGM("PCB VERSION:");
  SERIAL_DEBUGLN((int)Device_isNewPCB);

  tp_init(); // Initialize temperature loop
  plan_init(); // Initialize planner;
  watchdog_init();
  st_init(); // Initialize stepper, this enables interrupts!
  setup_photpin();
  servo_init();

  lcd_init();
  newPowerInit();
  wifiSDInit();

  gateInit();

#if defined(CONTROLLERFAN_PIN) && CONTROLLERFAN_PIN > -1
  SET_OUTPUT(CONTROLLERFAN_PIN); //Set pin used for driver cooling fan
#endif


#ifdef FilamentDetection
  SET_INPUT(FilamentDetectionPin);
  WRITE(FilamentDetectionPin, LOW);
#endif

  retriveLanguage();
  SERIAL_DEBUGLN((int)Device_type);
}

void calculate_delta_reverse(float theDelta[3], float theResult[3]);

void printFreeMemory(){
  static unsigned long freeMemoryTimer=millis();
  
  if (millis()-freeMemoryTimer >1000) {
    freeMemoryTimer=millis();
    SERIAL_DEBUGLNPGM("freeMemory");
    SERIAL_DEBUGLN(freeMemory());
  }
}


void loop()
{
  if (Device_isWifi) {
    static uint8_t powerAtomFlag = 0;
    static unsigned long powerAtomTimer = millis();
    
    switch (powerAtomFlag) {
      case 0:
        if (millis()-powerAtomTimer>3000) {
          SET_OUTPUT(15);
          WRITE(15, HIGH);
          powerAtomFlag=1;
        }
        break;
      case 1:
        if (millis()-powerAtomTimer>10000) {
          SET_OUTPUT(15);
          WRITE(15, LOW);
          powerAtomFlag=2;
        }
        break;
        
      default:
        break;
    }
  }

  manualLevelRoutine();

  if(buflen < (BUFSIZE-1))
    get_command();
#ifdef SDSUPPORT
  card.checkautostart(false);
#endif
  if(buflen)
  {
#ifdef SDSUPPORT
    if(card.saving)
    {
      if(strstr_P(cmdbuffer[bufindr], PSTR("M29")) == NULL)
      {
        card.write_command(cmdbuffer[bufindr]);
        if(card.logging)
        {
          process_commands();
        }
        else
        {
          SERIAL_PROTOCOLLNPGM(MSG_OK);
        }
      }
      else
      {
        card.closefile();
        SERIAL_PROTOCOLLNPGM(MSG_FILE_SAVED);
      }
    }
    else
    {
      process_commands();
    }
#else
    process_commands();
#endif //SDSUPPORT
    if (buflen > 0)
    {
      buflen = (buflen-1);
      bufindr = (bufindr + 1)%BUFSIZE;
    }
  }
  //check heater every n milliseconds
  manage_heater();
  manage_inactivity();
  checkHitEndstops();
  lcd_update();
  lifetime_stats_tick();

}

void get_command()
{

  if (MYSERIAL.available()) {

    lastSerialCommandTime=millis();

    while( MYSERIAL.available() > 0  && buflen < BUFSIZE) {
      serial_char = MYSERIAL.read();

      if(serial_char == '\n' ||
         serial_char == '\r' ||
         serial_char == 0    ||
         (serial_char == ':' && comment_mode == false) ||
         serial_count >= (MAX_CMD_SIZE - 1) )
      {
        if(!serial_count) { //if empty line
          comment_mode = false; //for new command
          return;
        }
        cmdbuffer[bufindw][serial_count] = 0; //terminate string
        if(!comment_mode) {
          comment_mode = false; //for new command
          commandFrom[bufindw] = FromSerial;
          if(strchr(cmdbuffer[bufindw], 'N') != NULL)
          {
            strchr_pointer = strchr(cmdbuffer[bufindw], 'N');
            gcode_N = (strtol(&cmdbuffer[bufindw][strchr_pointer - cmdbuffer[bufindw] + 1], NULL, 10));
            if(gcode_N != gcode_LastN+1 && (strstr_P(cmdbuffer[bufindw], PSTR("M110")) == NULL) ) {
              SERIAL_ERROR_START;
              SERIAL_ERRORPGM(MSG_ERR_LINE_NO);
              SERIAL_ERRORLN(gcode_LastN);
              //Serial.println(gcode_N);
              FlushSerialRequestResend();
              serial_count = 0;
              return;
            }

            if(strchr(cmdbuffer[bufindw], '*') != NULL)
            {
              byte checksum = 0;
              byte count = 0;
              while(cmdbuffer[bufindw][count] != '*') checksum = checksum^cmdbuffer[bufindw][count++];
              strchr_pointer = strchr(cmdbuffer[bufindw], '*');

              if( (int)(strtod(&cmdbuffer[bufindw][strchr_pointer - cmdbuffer[bufindw] + 1], NULL)) != checksum) {
                SERIAL_ERROR_START;
                SERIAL_ERRORPGM(MSG_ERR_CHECKSUM_MISMATCH);
                SERIAL_ERRORLN(gcode_LastN);
                FlushSerialRequestResend();
                serial_count = 0;
                return;
              }
              //if no errors, continue parsing
            }
            else
            {
              SERIAL_ERROR_START;
              SERIAL_ERRORPGM(MSG_ERR_NO_CHECKSUM);
              SERIAL_ERRORLN(gcode_LastN);
              FlushSerialRequestResend();
              serial_count = 0;
              return;
            }

            gcode_LastN = gcode_N;
            //if no errors, continue parsing
          }
          else // if we don't receive 'N' but still see '*'
          {
            if((strchr(cmdbuffer[bufindw], '*') != NULL))
            {
              SERIAL_ERROR_START;
              SERIAL_ERRORPGM(MSG_ERR_NO_LINENUMBER_WITH_CHECKSUM);
              SERIAL_ERRORLN(gcode_LastN);
              serial_count = 0;
              return;
            }
          }
          if((strchr(cmdbuffer[bufindw], 'G') != NULL)) {
            strchr_pointer = strchr(cmdbuffer[bufindw], 'G');
            switch((int)((strtod(&cmdbuffer[bufindw][strchr_pointer - cmdbuffer[bufindw] + 1], NULL)))) {
            case 0:
            case 1:
            case 2:
            case 3:
              if(Stopped == false) {             // If printer is stopped by an error the G[0-3] codes are ignored.
#ifdef SDSUPPORT
                if(card.saving)
                  break;
#endif //SDSUPPORT
                SERIAL_PROTOCOLLNPGM(MSG_OK);
              }
              else {
                SERIAL_ERRORLNPGM(MSG_ERR_STOPPED);
                LCD_MESSAGEPGM(MSG_STOPPED);
              }
              break;
            default:
              break;
            }

          }
#ifdef ENABLE_ULTILCD2
          strchr_pointer = strchr(cmdbuffer[bufindw], 'M');
          if (strtol(&cmdbuffer[bufindw][strchr_pointer - cmdbuffer[bufindw] + 1], NULL, 10) != 105)
            lastSerialCommandTime = millis();
#endif



          bufindw = (bufindw + 1)%BUFSIZE;
          buflen += 1;
        }
        serial_count = 0; //clear buffer
      }
      else
      {
        if(serial_char == ';') comment_mode = true;
        if(!comment_mode) cmdbuffer[bufindw][serial_count++] = serial_char;
        //      SERIAL_BLE_PROTOCOL("cmdbuffer");
        //      SERIAL_BLE_PROTOCOL(cmdbuffer[bufindw]);
      }
    }

  }


  if (serial_count!=0)
  {
    if (millis() - lastSerialCommandTime > 1000)
    {
      serial_count = 0;
    }
    else{
      return;
    }
  }

  if (isEnqueueingCommand==true && serial_count==0) {
    //    serialprintPGM(enqueueingFlashCommandBuf[enqueueingCommandIndex]);

    while (buflen < BUFSIZE) {
      if (isEnqueueingRamCommand[enqueueingCommandIndex]) {
        serial_char = *(enqueueingRamCommandBuf[enqueueingCommandIndex]+enqueueingCommandBufIndex);
      }
      else{
        serial_char=pgm_read_byte(enqueueingFlashCommandBuf[enqueueingCommandIndex]+enqueueingCommandBufIndex);
      }
      enqueueingCommandBufIndex++;

      if(serial_char == '\n' ||
         serial_char == '\r' ||
         (serial_char == ':' && comment_mode == false) ||
         serial_count >= (MAX_CMD_SIZE - 1) ||
         serial_char == '\0')
      {
        if(serial_count) {
          cmdbuffer[bufindw][serial_count] = 0; //terminate string
          commandFrom[bufindw] = FromFlash;

//          if (isEnqueueingRamCommand[enqueueingCommandIndex])
//          {
//            SERIAL_DEBUGPGM("Ram:");
//          }
//          else{
//            SERIAL_DEBUGPGM("Flash:");
//          }
//          SERIAL_DEBUGLN(cmdbuffer[bufindw]);

          buflen += 1;
          bufindw = (bufindw + 1)%BUFSIZE;
          comment_mode = false; //for new command
          serial_count = 0; //clear buffer
        }
        else
        {
          comment_mode = false; //for new command
        }
      }
      else
      {
        if(serial_char == ';') comment_mode = true;
        if(!comment_mode) cmdbuffer[bufindw][serial_count++] = serial_char;
      }

      if (serial_char == '\0') {
        enqueueingCommandBufIndex=0;
        if (isEnqueueingRamCommand[enqueueingCommandIndex] == true) {
          free(enqueueingRamCommandBuf[enqueueingCommandIndex]);
          //          SERIAL_ECHOLN(freeMemory());
        }
        if (enqueueingCommandLenth > 0) {
          enqueueingCommandIndex = (enqueueingCommandIndex + 1) % EnqueueingCommandBufSize;
          enqueueingCommandLenth--;
          if (enqueueingCommandLenth == 0) {
            isEnqueueingCommand = false;
          }
        }
        break;
      }
    }
  }


#ifdef SDSUPPORT


  if (card.sdprinting && serial_count==0 && !isEnqueueingCommand && !card.pause) {

    while (buflen < BUFSIZE) {
      if (card.eof())
      {
//        SERIAL_DEBUGLNPGM("card EOF");
        card.sdprinting=false;
        return;
      }

      if (movesplanned() <= (BLOCK_BUFFER_SIZE/2) && buflen) {
        return;
      }

      card.fgets(cmdbuffer[bufindw], MAX_CMD_SIZE);

      //      SERIAL_DEBUGPGM("sdraw:");
      //      SERIAL_DEBUGLN(cmdbuffer[bufindw]);

      if (card.errorCode())
      {
//        SERIAL_ECHOLNPGM("sd error");
        if (!card.sdInserted)
        {
          //            serial_count = 0;
          card.sdprinting = false;
          return;
        }

        //On an error, reset the error, reset the file position and try again.
  #ifdef ClearError
        card.clearError();
  #endif
        if (card.pause == true) {
          card.sdprinting = false;
          return;
        }
        //          serial_count = 0;
        //Screw it, if we are near the end of a file with an error, act if the file is finished. Hopefully preventing the hang at the end.
        if (card.getFilePos() > card.getFileSize() - 1024)
          card.sdprinting = false;
        else
          card.setIndex(card.getFilePos());
        return;
      }

      if (cmdbuffer[bufindw][0]==';') {
        continue;
      }

      strchr_pointer = strchr(cmdbuffer[bufindw], ';');
      if (strchr_pointer != NULL) {
        *strchr_pointer = 0;
      }
      //      SERIAL_DEBUGPGM("sd:");
      //      SERIAL_DEBUGLN(cmdbuffer[bufindw]);

      commandFrom[bufindw] = card.getFilePos();
      buflen += 1;
      bufindw = (bufindw + 1)%BUFSIZE;
    }


  }
#endif //SDSUPPORT
}

float code_value()
{
  return (strtod(strchr_pointer + 1, NULL));
}

long code_value_long()
{
  return (strtol(strchr_pointer + 1, NULL, 10));
}

bool code_seen(char code)
{
  strchr_pointer = strchr(cmdbuffer[bufindr], code);
  return (strchr_pointer != NULL); //Return True if a character was found
}

bool code_first_not(char code)
{
  return (*cmdbuffer[bufindr] != code); //Return True if a character was not found
}

#define DEFINE_PGM_READ_ANY(type, reader)       \
  static inline type pgm_read_any(const type *p)  \
  { return pgm_read_ ## reader ## _near(p); }

DEFINE_PGM_READ_ANY(float,       float);
DEFINE_PGM_READ_ANY(signed char, byte);

#define XYZ_CONSTS_FROM_CONFIG(type, array, CONFIG) \
  static const PROGMEM type array ## _P[3] =        \
  { X_ ## CONFIG, Y_ ## CONFIG, Z_ ## CONFIG };     \
  static inline type array(int axis)          \
  { return pgm_read_any(&array ## _P[axis]); }


XYZ_CONSTS_FROM_CONFIG(float, base_min_pos_GATE,    MIN_POS_GATE);
XYZ_CONSTS_FROM_CONFIG(float, base_max_pos_GATE,    MAX_POS_GATE);
XYZ_CONSTS_FROM_CONFIG(float, base_home_pos_GATE,   HOME_POS_GATE);
XYZ_CONSTS_FROM_CONFIG(float, max_length_GATE,      MAX_LENGTH_GATE);

XYZ_CONSTS_FROM_CONFIG(float, base_min_pos_PRO,    MIN_POS_PRO);
XYZ_CONSTS_FROM_CONFIG(float, base_max_pos_PRO,    MAX_POS_PRO);
XYZ_CONSTS_FROM_CONFIG(float, base_home_pos_PRO,   HOME_POS_PRO);
XYZ_CONSTS_FROM_CONFIG(float, max_length_PRO,      MAX_LENGTH_PRO);

XYZ_CONSTS_FROM_CONFIG(float, base_min_pos_MINI,    MIN_POS_MINI);
XYZ_CONSTS_FROM_CONFIG(float, base_max_pos_MINI,    MAX_POS_MINI);
XYZ_CONSTS_FROM_CONFIG(float, base_home_pos_MINI,   HOME_POS_MINI);
XYZ_CONSTS_FROM_CONFIG(float, max_length_MINI,      MAX_LENGTH_MINI);

XYZ_CONSTS_FROM_CONFIG(float, base_min_pos_PRO_SENSOR,    MIN_POS_PRO_SENSOR);
XYZ_CONSTS_FROM_CONFIG(float, base_max_pos_PRO_SENSOR,    MAX_POS_PRO_SENSOR);
XYZ_CONSTS_FROM_CONFIG(float, base_home_pos_PRO_SENSOR,   HOME_POS_PRO_SENSOR);
XYZ_CONSTS_FROM_CONFIG(float, max_length_PRO_SENSOR,      MAX_LENGTH_PRO_SENSOR);

#define XYZ_CONSTS_READ(type, array)  \
static inline type array(int axis)  \
{ \
  if (Device_isGate) {  \
    return pgm_read_any(&array ## _GATE_P[axis]);  \
  } \
  else if (Device_isPro){ \
    if (Device_isLevelSensor) {\
      return pgm_read_any(&array ## _PRO_SENSOR_P[axis]); \
    }\
    else{\
      return pgm_read_any(&array ## _PRO_P[axis]); \
    }\
  } \
  else {  \
    return pgm_read_any(&array ## _MINI_P[axis]);  \
  } \
}

XYZ_CONSTS_READ(float, base_min_pos);
XYZ_CONSTS_READ(float, base_max_pos);
XYZ_CONSTS_READ(float, base_home_pos);
XYZ_CONSTS_READ(float, max_length);

XYZ_CONSTS_FROM_CONFIG(float, home_retract_mm, HOME_RETRACT_MM);
XYZ_CONSTS_FROM_CONFIG(signed char, home_dir,  HOME_DIR);

static void axis_is_at_home(int axis) {
  current_position[axis] = base_home_pos(axis) + add_homeing[axis];
  min_pos[axis] =          base_min_pos(axis);// + add_homeing[axis];
  max_pos[axis] =          base_max_pos(axis);// + add_homeing[axis];
}

static void homeaxis(int axis) {
#define HOMEAXIS_DO(LETTER) \
  ((LETTER ## _MIN_PIN > -1 && LETTER ## _HOME_DIR==-1) || (LETTER ## _MAX_PIN > -1 && LETTER ## _HOME_DIR==1))
  if (axis==X_AXIS ? HOMEAXIS_DO(X) :
      axis==Y_AXIS ? HOMEAXIS_DO(Y) :
      axis==Z_AXIS ? HOMEAXIS_DO(Z) :
      0) {

    // Engage Servo endstop if enabled
#ifdef SERVO_ENDSTOPS
    if (servo_endstops[axis] > -1) servos[servo_endstops[axis]].write(servo_endstop_angles[axis * 2]);
#endif

    current_position[axis] = 0;
    plan_set_position_old(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);
    current_position[axis] = 1.5 * max_length(axis) * home_dir(axis);
    feedrate = homing_feedrate[axis];
    plan_buffer_line_old(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], feedrate/60, active_extruder);
    st_synchronize();


    if (!isEndstopHit())
    {
      SERIAL_ERROR_START;
      SERIAL_ERRORLNPGM("Endstop not pressed after homing down. Endstop broken?");
      switch (axis) {
      case X_AXIS:
        Stop(STOP_REASON_X_ENDSTOP_BROKEN_ERROR);
        break;
      case Y_AXIS:
        Stop(STOP_REASON_Y_ENDSTOP_BROKEN_ERROR);
        break;
      case Z_AXIS:
        Stop(STOP_REASON_Z_ENDSTOP_BROKEN_ERROR);
        break;
      default:
        Stop(STOP_REASON_X_ENDSTOP_BROKEN_ERROR);
        break;
      }

      return;
    }

    current_position[axis] = 0;
    plan_set_position_old(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);
    current_position[axis] = -home_retract_mm(axis) * home_dir(axis);
    plan_buffer_line_old(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], feedrate/60, active_extruder);
    st_synchronize();

    bool endstop_pressed = false;
    switch(axis)
    {
    case X_AXIS:
#if defined(X_MIN_PIN) && X_MIN_PIN > -1 && X_HOME_DIR == -1
      endstop_pressed = (READ(X_MIN_PIN) != X_ENDSTOPS_INVERTING);
#endif
#if defined(X_MAX_PIN) && X_MAX_PIN > -1 && X_HOME_DIR == 1
      endstop_pressed = (READ(X_MAX_PIN) != X_ENDSTOPS_INVERTING);
#endif
      break;
    case Y_AXIS:
#if defined(Y_MIN_PIN) && Y_MIN_PIN > -1 && Y_HOME_DIR == -1
      endstop_pressed = (READ(Y_MIN_PIN) != Y_ENDSTOPS_INVERTING);
#endif
#if defined(Y_MAX_PIN) && Y_MAX_PIN > -1 && Y_HOME_DIR == 1
      endstop_pressed = (READ(Y_MAX_PIN) != Y_ENDSTOPS_INVERTING);
#endif
      break;
    case Z_AXIS:
#if defined(Z_MIN_PIN) && Z_MIN_PIN > -1 && Z_HOME_DIR == -1
      endstop_pressed = (READ(Z_MIN_PIN) != Z_ENDSTOPS_INVERTING);
#endif
#if defined(Z_MAX_PIN) && Z_MAX_PIN > -1 && Z_HOME_DIR == 1
      endstop_pressed = (READ(Z_MAX_PIN) != Z_ENDSTOPS_INVERTING);
#endif
      break;
    }
    if (endstop_pressed)
    {
      SERIAL_ERROR_START;
      SERIAL_ERRORLNPGM("Endstop still pressed after backing off. Endstop stuck?");
      endstops_hit_on_purpose();

      switch (axis) {
      case X_AXIS:
        Stop(STOP_REASON_X_ENDSTOP_STUCK_ERROR);
        break;
      case Y_AXIS:
        Stop(STOP_REASON_Y_ENDSTOP_STUCK_ERROR);
        break;
      case Z_AXIS:
        Stop(STOP_REASON_Z_ENDSTOP_STUCK_ERROR);
        break;
      default:
        Stop(STOP_REASON_X_ENDSTOP_STUCK_ERROR);
        break;
      }
      return;
    }

    current_position[axis] = 2*home_retract_mm(axis) * home_dir(axis);
    feedrate = homing_feedrate[axis]/4;
    plan_buffer_line_old(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], feedrate/60, active_extruder);
    st_synchronize();



    axis_is_at_home(axis);
    destination[axis] = current_position[axis];
    feedrate = 0.0;
    endstops_hit_on_purpose();

    // Retract Servo endstop if enabled
#ifdef SERVO_ENDSTOPS
    if (servo_endstops[axis] > -1) servos[servo_endstops[axis]].write(servo_endstop_angles[axis * 2 + 1]);
#endif
  }
}
#define HOMEAXIS(LETTER) homeaxis(LETTER ## _AXIS)


ISR(TIMER3_CAPT_vect)
{
  static uint8_t stepperTorqueTimer=0;
  uint8_t stepperTorqueTimerbuf=(stepperTorqueTimer&0x1f);
  if (stepperTorqueTimerbuf==0) {
    enable_x();
    enable_y();
    enable_z();
  }

  if(stepperTorqueTimerbuf==stepperTorqueX) {
    disable_x();
  }

  if (stepperTorqueTimerbuf==stepperTorqueY) {
    disable_y();
  }

  if (stepperTorqueTimerbuf==stepperTorqueZ) {
    disable_z();
  }

  stepperTorqueTimer++;
}

void setStepperTorque(uint8_t theTorqueX, uint8_t theTorqueY, uint8_t theTorqueZ)
{
  stepperTorqueX=theTorqueX;
  stepperTorqueY=theTorqueY;
  stepperTorqueZ=theTorqueZ;

  if (theTorqueX == (uint8_t)-1 && theTorqueY == (uint8_t)-1 && theTorqueZ == (uint8_t)-1) {

    TCCR3B = _BV(WGM33)|_BV(WGM32);
    TCCR3A = 0;

    TIMSK3 = 0;

    TIFR3 = _BV(ICF3);

    enable_x();
    enable_y();
    enable_z();
  }
  else{

    TCNT3 = 0;
    ICR3=8*(2)-1;

    TCCR3B = _BV(WGM33)|_BV(WGM32)|_BV(CS30); //set the ctc mode
    TCCR3A = 0;             //set the ctc mode and no compare output

    TIMSK3 = _BV(ICIE3);    //enable the timer3 overflood
  }
}

void level_abort()
{
  add_homeing[Z_AXIS] = 1.0/plainFactorC;
  add_homeing[Z_AXIS] -= -ADDING_Z_FOR_POSITIVE;
  add_homeing[Z_AXIS] -= touchPlateOffset;
  fittingBedUpdateK();
  
  discardEnqueueingCommand();
  discardCommandInBuffer();
  quickStop();
  doCooldown();
  enquecommand_P(PSTR("G28\nM84"));
}


void process_commands()
{
  unsigned long codenum; //throw away variable
  char *starpos = NULL;

  printing_state = PRINT_STATE_NORMAL;
  if((code_first_not('M')) && code_seen('G'))
  {
    switch((int)code_value())
    {
    case 0:             // G0 -> G1
    case 1:             // G1
      if(Stopped == false) {
        get_coordinates();             // For X Y Z E F
        prepare_move();
        //ClearToSend();
        return;
      }
    //break;
    case 2:             // G2  - CW ARC
      if(Stopped == false) {
        get_arc_coordinates();
        prepare_arc_move(true);
        return;
      }
    case 3:             // G3  - CCW ARC
      if(Stopped == false) {
        get_arc_coordinates();
        prepare_arc_move(false);
        return;
      }
    case 4:             // G4 dwell
      LCD_MESSAGEPGM(MSG_DWELL);
      codenum = 0;
      if(code_seen('P')) codenum = code_value();             // milliseconds to wait
      if(code_seen('S')) codenum = code_value() * 1000;             // seconds to wait

      st_synchronize();
      codenum += millis();             // keep track of when we started waiting
      previous_millis_cmd = millis();
      printing_state = PRINT_STATE_DWELL;
      while(millis()  < codenum ) {
        manage_heater();
        manage_inactivity();
        lcd_update();
        lifetime_stats_tick();
      }
      break;
#ifdef FWRETRACT
    case 10:             // G10 retract
      if(!retracted)
      {
        destination[X_AXIS]=current_position[X_AXIS];
        destination[Y_AXIS]=current_position[Y_AXIS];
        destination[Z_AXIS]=current_position[Z_AXIS];
  #if EXTRUDERS > 1
        if (code_seen('S') && code_value_long() == 1)
          destination[E_AXIS]=current_position[E_AXIS]-extruder_swap_retract_length/volume_to_filament_length[active_extruder];
        else
          destination[E_AXIS]=current_position[E_AXIS]-retract_length/volume_to_filament_length[active_extruder];
  #else
        destination[E_AXIS]=current_position[E_AXIS]-retract_length/volume_to_filament_length[active_extruder];
  #endif
        float oldFeedrate = feedrate;
        feedrate=retract_feedrate;
        retract_recover_length = current_position[E_AXIS]-destination[E_AXIS];            //Set the recover length to whatever distance we retracted so we recover properly.
        retracted=true;
        prepare_move();
        feedrate = oldFeedrate;
      }

      break;
    case 11:             // G11 retract_recover
      if(retracted)
      {
        destination[X_AXIS]=current_position[X_AXIS];
        destination[Y_AXIS]=current_position[Y_AXIS];
        destination[Z_AXIS]=current_position[Z_AXIS];
        destination[E_AXIS]=current_position[E_AXIS]+retract_recover_length;
        float oldFeedrate = feedrate;
        feedrate=retract_recover_feedrate;
        retracted=false;
        prepare_move();
        feedrate = oldFeedrate;
      }
      break;
#endif //FWRETRACT
    case 28:             //G28 Home all Axis one at a time
      fittingBedResetK();
#ifdef FWRETRACT
      retracted=false;
#endif
      printing_state = PRINT_STATE_HOMING;
      saved_feedrate = feedrate;
      saved_feedmultiply = feedmultiply;
      feedmultiply = 100;
      previous_millis_cmd = millis();

      enable_endstops(true);

      for(int8_t i=0; i < NUM_AXIS; i++) {
        destination[i] = current_position[i];
      }
      feedrate = 0.0;

      // A delta can only safely home all axis at the same time
      // all axis have to home at the same time

      // Move all carriages up together until the first endstop is hit.
      current_position[X_AXIS] = 0;
      current_position[Y_AXIS] = 0;
      current_position[Z_AXIS] = 0;
      plan_set_position_old(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);

      destination[X_AXIS] = 2 * max_length(Z_AXIS);
      destination[Y_AXIS] = 2 * max_length(Z_AXIS);
      destination[Z_AXIS] = 2 * max_length(Z_AXIS);
      feedrate = 1.732 * homing_feedrate[X_AXIS];
      plan_buffer_line_old(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS], destination[E_AXIS], feedrate/60, active_extruder);
      st_synchronize();
      endstops_hit_on_purpose();

      current_position[X_AXIS] = destination[X_AXIS];
      current_position[Y_AXIS] = destination[Y_AXIS];
      current_position[Z_AXIS] = destination[Z_AXIS];

      // take care of back off and rehome now we are all at the top
      HOMEAXIS(X);
      HOMEAXIS(Y);
      HOMEAXIS(Z);

      plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);

      fittingBedUpdateK();

#ifdef ENDSTOPS_ONLY_FOR_HOMING
      enable_endstops(false);
#endif

      feedrate = saved_feedrate;
      feedmultiply = saved_feedmultiply;
      previous_millis_cmd = millis();
      endstops_hit_on_purpose();
      break;

    //----------------------------------------------


    case 29:             //G29 autolevel touch bed
    {

      char serialBuffer[30];
      float add_homeing_z_back_up;
      
      long st_get_position_array[3];
      
      unsigned long torqueIndexTimer;

      fittingBedTorque=19;

      fittingBedResetK();
      fittingBedArrayInit();

#ifdef FWRETRACT
      retracted=false;
#endif
      printing_state = PRINT_STATE_HOMING;
      saved_feedrate = feedrate;
      saved_feedmultiply = feedmultiply;
      feedmultiply = 100;
      previous_millis_cmd = millis();

      enable_endstops(true);

      for(int8_t i=0; i < NUM_AXIS; i++) {
        destination[i] = current_position[i];
      }
      feedrate = 0.0;

      // A delta can only safely home all axis at the same time
      // all axis have to home at the same time

      add_homeing_z_back_up = add_homeing[Z_AXIS];

      add_homeing[Z_AXIS]=0;

      // Move all carriages up together until the first endstop is hit.
      current_position[X_AXIS] = 0;
      current_position[Y_AXIS] = 0;
      current_position[Z_AXIS] = 0;
      plan_set_position_old(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);

      destination[X_AXIS] = 2 * max_length(Z_AXIS);
      destination[Y_AXIS] = 2 * max_length(Z_AXIS);
      destination[Z_AXIS] = 2 * max_length(Z_AXIS);
      feedrate = 1.732 * homing_feedrate[X_AXIS];
      plan_buffer_line_old(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS], destination[E_AXIS], feedrate/60, active_extruder);
      st_synchronize();
      endstops_hit_on_purpose();

      if (printing_state==PRINT_STATE_HOMING_ABORT) {
        level_abort();
        return;
      }

      current_position[X_AXIS] = destination[X_AXIS];
      current_position[Y_AXIS] = destination[Y_AXIS];
      current_position[Z_AXIS] = destination[Z_AXIS];

      // take care of back off and rehome now we are all at the top
      HOMEAXIS(X);
      if (printing_state==PRINT_STATE_HOMING_ABORT) {
        level_abort();
        return;
      }

      HOMEAXIS(Y);
      if (printing_state==PRINT_STATE_HOMING_ABORT) {
        level_abort();
        return;
      }

      HOMEAXIS(Z);
      if (printing_state==PRINT_STATE_HOMING_ABORT) {
        level_abort();
        return;
      }

      plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);

      if (Device_isLevelSensor) {
        
        for (int i=0; i<3; i++) {
          st_get_position_array[i]= st_get_position(i);
        }
        
        feedrate=homing_feedrate[X_AXIS];
        destination[X_AXIS]=0;
        destination[Y_AXIS]=0;
        destination[Z_AXIS]=10;
        st_synchronize();
        plan_buffer_line(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS], destination[E_AXIS], feedrate/60, active_extruder);
        st_synchronize();
      }
      
      for (int index=0; index<NodeNum; index++) {
        //key:
        //        index = 5;
        //unlock the z probe

        if (Device_isLevelSensor) {
          feedrate=2000;
          
          destination[Z_AXIS] = 3;
          destination[X_AXIS] = fittingBedArray[index][X_AXIS];
          destination[Y_AXIS] = fittingBedArray[index][Y_AXIS];
          plan_buffer_line(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS], destination[E_AXIS], feedrate/60, active_extruder);
          st_synchronize();
          
          if (printing_state==PRINT_STATE_HOMING_ABORT) {
            level_abort();
            return;
          }
          
          destination[Z_AXIS]=-10;
          
          destination[Y_AXIS]+=20;
          
          feedrate=1000;
          
          plan_buffer_line(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS], destination[E_AXIS], feedrate/60, active_extruder);
          
          destination[Y_AXIS]-=20;
          
          st_synchronize();
          
          feedrate=2000;
          
          if (printing_state==PRINT_STATE_HOMING_ABORT) {
            level_abort();
            setStepperTorque(-1,-1,-1);
            return;
          }
          
          endstops_hit_on_purpose();
          
          for (int axisIndex=0; axisIndex<3; axisIndex++) {
            delta[axisIndex]=base_home_pos(Z_AXIS)+((st_get_position(axisIndex)-st_get_position_array[axisIndex])/STEPS_PER_UNIT_DELTA)+sqrt(sq(DELTA_DIAGONAL_ROD)- square(DELTA_RADIUS))+ADDING_Z_FOR_POSITIVE;
          }
          
          calculate_delta_reverse(delta,fittingBedArray[index]);
          
//          fittingBedArray[index][Z_AXIS] = 10.0 - (st_get_position_array[X_AXIS] - st_get_position(X_AXIS))/STEPS_PER_UNIT_DELTA + ADDING_Z_FOR_POSITIVE;
          destination[Z_AXIS] = fittingBedArray[index][Z_AXIS] - ADDING_Z_FOR_POSITIVE;
          
          plan_set_position(fittingBedArray[index][X_AXIS], fittingBedArray[index][Y_AXIS], destination[Z_AXIS], destination[E_AXIS]);
          destination[Z_AXIS] = 3;
          plan_buffer_line(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS], destination[E_AXIS], feedrate/60, active_extruder);
          st_synchronize();
        }
        else{
          
          feedrate=homing_feedrate[X_AXIS]*1.8;
          destination[X_AXIS]=0;
          destination[Y_AXIS]=0;
          destination[Z_AXIS]=10;
          st_synchronize();
          plan_buffer_line(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS], destination[E_AXIS], feedrate/60, active_extruder);
          st_synchronize();
          
          if (printing_state==PRINT_STATE_HOMING_ABORT) {
            level_abort();
            return;
          }
          
          destination[Z_AXIS] = 2;
          destination[X_AXIS] = fittingBedArray[index][X_AXIS];
          destination[Y_AXIS] = fittingBedArray[index][Y_AXIS];
          plan_buffer_line(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS], destination[E_AXIS], feedrate/60, active_extruder);
          st_synchronize();
          
          if (printing_state==PRINT_STATE_HOMING_ABORT) {
            level_abort();
            return;
          }
          
          feedrate=800;
          switch (index) {
            case 0:
              setStepperTorque(fittingBedTorque,fittingBedTorque,fittingBedTorque-1);
              break;
            case 1:
              setStepperTorque(fittingBedTorque-1,fittingBedTorque,fittingBedTorque-1);
              break;
            case 2:
              setStepperTorque(fittingBedTorque-1,fittingBedTorque,fittingBedTorque);
              break;
            case 3:
              setStepperTorque(fittingBedTorque-1,fittingBedTorque-1,fittingBedTorque);
              break;
            case 4:
              setStepperTorque(fittingBedTorque,fittingBedTorque-1,fittingBedTorque);
              break;
            case 5:
              setStepperTorque(fittingBedTorque,fittingBedTorque-1,fittingBedTorque-1);
              break;
            default:
              break;
          }
          
          destination[Z_AXIS]=-5;
          
          plan_buffer_line(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS], destination[E_AXIS], feedrate/60, active_extruder);
          
          st_synchronize();
          if (printing_state==PRINT_STATE_HOMING_ABORT) {
            level_abort();
            setStepperTorque(-1,-1,-1);
            return;
          }
          
          torqueIndexTimer = millis();
          
          while (1) {
            if (millis()-torqueIndexTimer>200) {             //millis()-torqueIndexTimer<50 may cause the problem
              break;
            }
            manage_heater();
            manage_inactivity();
            lcd_update();
          }
          
          setStepperTorque(0,0,0);
          torqueIndexTimer = millis();
          
          while (1) {
            if (millis()-torqueIndexTimer>500) {             //millis()-torqueIndexTimer<50 may cause the problem
              break;
            }
            manage_heater();
            manage_inactivity();
            lcd_update();
          }
          
          
          for (int torqueIndex=fittingBedTorque+1; torqueIndex>=14; torqueIndex--) {
            
            switch (index) {
              case 0:
                setStepperTorque(torqueIndex,torqueIndex,torqueIndex-1);
                break;
              case 1:
                setStepperTorque(torqueIndex-1,torqueIndex,torqueIndex-1);
                break;
              case 2:
                setStepperTorque(torqueIndex-1,torqueIndex,torqueIndex);
                break;
              case 3:
                setStepperTorque(torqueIndex-1,torqueIndex-1,torqueIndex);
                break;
              case 4:
                setStepperTorque(torqueIndex,torqueIndex-1,torqueIndex);
                break;
              case 5:
                setStepperTorque(torqueIndex,torqueIndex-1,torqueIndex-1);
                break;
              default:
                break;
            }
            
            destination[Z_AXIS]=0;
            plan_set_position(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS], destination[E_AXIS]);
            
            destination[Z_AXIS]=1.9;
            
            plan_buffer_line(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS], destination[E_AXIS], feedrate/60, active_extruder);
            
            st_synchronize();
            if (printing_state==PRINT_STATE_HOMING_ABORT) {
              level_abort();
              setStepperTorque(-1,-1,-1);
              return;
            }
            
            destination[Z_AXIS]=0;
            plan_set_position(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS], destination[E_AXIS]);
            
            destination[Z_AXIS]=-2.0;
            plan_buffer_line(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS], destination[E_AXIS], feedrate/60, active_extruder);
            
            st_synchronize();
            if (printing_state==PRINT_STATE_HOMING_ABORT) {
              level_abort();
              setStepperTorque(-1,-1,-1);
              return;
            }
          }
          
          current_position[X_AXIS]=0;
          current_position[Y_AXIS]=0;
          current_position[Z_AXIS]=0;
          
          plan_set_position_old(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);
          
          for (uint8_t torqueIndex=14; torqueIndex<=22; torqueIndex++) {
            
            switch (index) {
              case 0:
                setStepperTorque(torqueIndex,torqueIndex,torqueIndex-1);
                break;
              case 1:
                setStepperTorque(torqueIndex-1,torqueIndex,torqueIndex-1);
                break;
              case 2:
                setStepperTorque(torqueIndex-1,torqueIndex,torqueIndex);
                break;
              case 3:
                setStepperTorque(torqueIndex-1,torqueIndex-1,torqueIndex);
                break;
              case 4:
                setStepperTorque(torqueIndex,torqueIndex-1,torqueIndex);
                break;
              case 5:
                setStepperTorque(torqueIndex,torqueIndex-1,torqueIndex-1);
                break;
              default:
                break;
            }
            
            
            feedrate=200;
            
            destination[Z_AXIS]=0;
            plan_set_position(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS], destination[E_AXIS]);
            
            destination[Z_AXIS]=0.1;
            
            plan_buffer_line(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS], destination[E_AXIS], feedrate/60, active_extruder);
            
            st_synchronize();
            
            destination[Z_AXIS]=0;
            plan_set_position(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS], destination[E_AXIS]);
            
            if (torqueIndex<=17) {
              destination[Z_AXIS]=-0.18;
            }
            else{
              destination[Z_AXIS]=-0.11;
            }
            
            plan_buffer_line(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS], destination[E_AXIS], feedrate/60, active_extruder);
            
            st_synchronize();
            
            //          setStepperTorque(0,0,0);
            
            torqueIndexTimer=millis();
            
            while (1) {
              if (millis()-torqueIndexTimer>100) {             //millis()-torqueIndexTimer<50 may cause the problem
                break;
              }
              manage_heater();
              manage_inactivity();
              lcd_update();
            }
            
          }
          
          setStepperTorque(-1,-1,-1);
          
          torqueIndexTimer=millis();
          
          while (true) {
            if (millis()-torqueIndexTimer>200) {             //millis()-torqueIndexTimer<50 may cause the problem
              break;
            }
            manage_heater();
            manage_inactivity();
            lcd_update();
          }
          
          //        feedrate=2000;
          //
          //        destination[Z_AXIS]=0;
          //        plan_set_position(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS], destination[E_AXIS]);
          //
          //        destination[Z_AXIS]=10;
          //
          //        plan_buffer_line(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS], destination[E_AXIS], feedrate/60, active_extruder);
          //
          //
          //        goto key;
          
          
          if (printing_state==PRINT_STATE_HOMING_ABORT) {
            level_abort();
            return;
          }
          
          feedrate=homing_feedrate[X_AXIS]*1.8;
          
          current_position[X_AXIS]=0;
          current_position[Y_AXIS]=0;
          current_position[Z_AXIS]=0;
          
          plan_set_position_old(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);
          
          st_get_position_array[X_AXIS]=st_get_position(X_AXIS);
          st_get_position_array[Y_AXIS]=st_get_position(Y_AXIS);
          st_get_position_array[Z_AXIS]=st_get_position(Z_AXIS);
          
          
          destination[X_AXIS] = 1.5 * max_length(Z_AXIS) * home_dir(X_AXIS);
          destination[Y_AXIS] = 1.5 * max_length(Z_AXIS) * home_dir(Y_AXIS);
          destination[Z_AXIS] = 1.5 * max_length(Z_AXIS) * home_dir(Z_AXIS);
          
          plan_buffer_line_old(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS], destination[E_AXIS], feedrate/60, active_extruder);
          st_synchronize();
          
          if (printing_state==PRINT_STATE_HOMING_ABORT) {
            level_abort();
            return;
          }
          
          //            SERIAL_DEBUGPGM("checkHitEndstops");
          //          checkHitEndstops();
          //          endstops_hit_on_purpose();
          
          for (int axisIndex=0; axisIndex<3; axisIndex++) {
            
            feedrate = homing_feedrate[axisIndex];
            destination[axisIndex] += max_length(Z_AXIS) * home_dir(axisIndex);
            plan_buffer_line_old(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS], destination[E_AXIS], feedrate/60, active_extruder);
            
            st_synchronize();
            
            if (printing_state==PRINT_STATE_HOMING_ABORT) {
              level_abort();
              return;
            }
            
            destination[axisIndex] -= home_retract_mm(Z_AXIS) * home_dir(axisIndex);
            plan_buffer_line_old(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS], destination[E_AXIS], feedrate/60, active_extruder);
            
            st_synchronize();
            
            if (printing_state==PRINT_STATE_HOMING_ABORT) {
              level_abort();
              return;
            }
            
            feedrate = homing_feedrate[axisIndex]/4;
            
            destination[axisIndex] += 2*home_retract_mm(Z_AXIS) * home_dir(axisIndex);
            plan_buffer_line_old(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS], destination[E_AXIS], feedrate/60, active_extruder);
            
            st_synchronize();
            
            if (printing_state==PRINT_STATE_HOMING_ABORT) {
              level_abort();
              return;
            }
            
            delta[axisIndex]=base_home_pos(Z_AXIS)-((st_get_position(axisIndex)-st_get_position_array[axisIndex])/STEPS_PER_UNIT_DELTA)+sqrt(sq(DELTA_DIAGONAL_ROD)- square(DELTA_RADIUS))+ADDING_Z_FOR_POSITIVE;
            
            axis_is_at_home(axisIndex);
            endstops_hit_on_purpose();
            
          }
          
          calculate_delta_reverse(delta,fittingBedArray[index]);
          plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);

        }

       

        SERIAL_BED_DEBUGLNPGM("fittingBedArray");
        SERIAL_BED_DEBUG("G1 F4000 X");
        SERIAL_BED_DEBUG(dtostrf(fittingBedArray[index][X_AXIS], 10, 10, serialBuffer));
        SERIAL_BED_DEBUG(" Y");
        SERIAL_BED_DEBUG(dtostrf(fittingBedArray[index][Y_AXIS], 10, 10, serialBuffer));
        SERIAL_BED_DEBUG(" Z");
        SERIAL_BED_DEBUGLN(dtostrf(fittingBedArray[index][Z_AXIS]-ADDING_Z_FOR_POSITIVE, 10, 10, serialBuffer));

      }

      SERIAL_BED_DEBUGLNPGM("add_homeing[Z_AXIS]");
      SERIAL_BED_DEBUGLN(add_homeing[Z_AXIS]);
      SERIAL_BED_DEBUGLN(current_position[Z_AXIS]);

      fittingBed();

      SERIAL_BED_DEBUGLNPGM("fittingBed");
      SERIAL_BED_DEBUGLN(dtostrf(plainFactorA, 10, 15, serialBuffer));
      SERIAL_BED_DEBUGLN(dtostrf(plainFactorB, 10, 15, serialBuffer));
      SERIAL_BED_DEBUGLN(dtostrf(plainFactorC, 10, 15, serialBuffer));

      add_homeing[Z_AXIS] = 1.0/plainFactorC;
      add_homeing[Z_AXIS] -= -ADDING_Z_FOR_POSITIVE;
      add_homeing[Z_AXIS] -= touchPlateOffset;
      SERIAL_BED_DEBUGLNPGM("add_homeing[Z_AXIS]");
      SERIAL_BED_DEBUGLN(add_homeing[Z_AXIS]);
      Config_StoreSettings();

      for(int8_t i=0; i < NUM_AXIS; i++) {
        destination[i] = current_position[i];
      }

      // A delta can only safely home all axis at the same time
      // all axis have to home at the same time

      // Move all carriages up together until the first endstop is hit.
      current_position[X_AXIS] = 0;
      current_position[Y_AXIS] = 0;
      current_position[Z_AXIS] = 0;
      plan_set_position_old(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);

      st_synchronize();

      if (printing_state==PRINT_STATE_HOMING_ABORT) {
        level_abort();
        return;
      }

      destination[X_AXIS] = 2 * max_length(Z_AXIS);
      destination[Y_AXIS] = 2 * max_length(Z_AXIS);
      destination[Z_AXIS] = 2 * max_length(Z_AXIS);
      feedrate = 1.732 * homing_feedrate[X_AXIS];
      plan_buffer_line_old(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS], destination[E_AXIS], feedrate/60, active_extruder);
      st_synchronize();
      endstops_hit_on_purpose();

      if (printing_state==PRINT_STATE_HOMING_ABORT) {
        level_abort();
        return;
      }

      current_position[X_AXIS] = destination[X_AXIS];
      current_position[Y_AXIS] = destination[Y_AXIS];
      current_position[Z_AXIS] = destination[Z_AXIS];

      // take care of back off and rehome now we are all at the top
      HOMEAXIS(X);
      if (printing_state==PRINT_STATE_HOMING_ABORT) {
        level_abort();
        return;
      }
      HOMEAXIS(Y);
      if (printing_state==PRINT_STATE_HOMING_ABORT) {
        level_abort();
        return;
      }
      HOMEAXIS(Z);
      if (printing_state==PRINT_STATE_HOMING_ABORT) {
        level_abort();
        return;
      }


      plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);


      SERIAL_BED_DEBUGLNPGM("fittingBedUpdateKBefore:");
      SERIAL_BED_DEBUGLN(plainFactorAAC*1000000000.0);
      SERIAL_BED_DEBUGLN(plainFactorBBC*1000000000.0);
      SERIAL_BED_DEBUGLN(plainFactorCAC*1000000000.0);
      SERIAL_BED_DEBUGLN(plainFactorCBC*1000000000.0);

      fittingBedUpdateK();

      SERIAL_BED_DEBUGLNPGM("fittingBedUpdateKAfter:");
      SERIAL_BED_DEBUGLN(plainFactorAAC*1000000000.0);
      SERIAL_BED_DEBUGLN(plainFactorBBC*1000000000.0);
      SERIAL_BED_DEBUGLN(plainFactorCAC*1000000000.0);
      SERIAL_BED_DEBUGLN(plainFactorCBC*1000000000.0);


  #ifdef ENDSTOPS_ONLY_FOR_HOMING
      enable_endstops(false);
  #endif

      feedrate = saved_feedrate;
      feedmultiply = saved_feedmultiply;
      previous_millis_cmd = millis();
      endstops_hit_on_purpose();
    }
    break;
        
    
    case 90:             // G90
      relative_mode = false;
      break;
    case 91:             // G91
      relative_mode = true;
      break;
    case 92:             // G92
      if(!code_seen(axis_codes[E_AXIS]))
        st_synchronize();
      for(int8_t i=0; i < NUM_AXIS; i++) {
        if(code_seen(axis_codes[i])) {
          if(i == E_AXIS) {
            current_position[i] = code_value();
            plan_set_e_position(current_position[E_AXIS]);
          }
          else {
            current_position[i] = code_value();
            plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);
          }
        }
      }
      break;
    }
  }

  else if(code_seen('M'))
  {
    switch( (int)code_value() )
    {
  #ifdef ULTIPANEL
    case 0:             // M0 - Unconditional stop - Wait for user button press on LCD
    case 1:             // M1 - Conditional stop - Wait for user button press on LCD
    {
      printing_state = PRINT_STATE_WAIT_USER;
      LCD_MESSAGEPGM(MSG_USERWAIT);
      codenum = 0;
      if(code_seen('P')) codenum = code_value();             // milliseconds to wait
      if(code_seen('S')) codenum = code_value() * 1000;             // seconds to wait

      st_synchronize();
      previous_millis_cmd = millis();
      if (codenum > 0) {
        codenum += millis();             // keep track of when we started waiting
        while(millis()  < codenum && !lcd_clicked()) {
          manage_heater();
          manage_inactivity();
          lcd_update();
          lifetime_stats_tick();
        }
      }else{
        while(!lcd_clicked()) {
          manage_heater();
          manage_inactivity();
          lcd_update();
          lifetime_stats_tick();
        }
      }
      LCD_MESSAGEPGM(MSG_RESUMING);
    }
    break;
  #endif
  #ifdef ENABLE_ULTILCD2
    case 0:             // M0 - Unconditional stop - Wait for user button press on LCD
    case 1:             // M1 - Conditional stop - Wait for user button press on LCD
    {
      printing_state = PRINT_STATE_WAIT_USER;

      codenum = 0;
      if(code_seen('P')) codenum = code_value();             // milliseconds to wait
      if(code_seen('S')) codenum = code_value() * 1000;             // seconds to wait

      st_synchronize();
      previous_millis_cmd = millis();
      if (codenum > 0) {
        codenum += millis();             // keep track of when we started waiting
        while(millis()  < codenum && !lcd_lib_button_down) {
          manage_heater();
          manage_inactivity();
          lcd_update();
          lifetime_stats_tick();
        }
      }else{
        while(!lcd_lib_button_down) {
          manage_heater();
          manage_inactivity();
          lcd_update();
          lifetime_stats_tick();
        }
      }
    }
    break;
  #endif
    case 17:
      LCD_MESSAGEPGM(MSG_NO_MOVE);
      enable_x();
      enable_y();
      enable_z();
      enable_e0();
      enable_e1();
      enable_e2();
      break;

  #ifdef SDSUPPORT
    case 20:             // M20 - list SD card
      SERIAL_PROTOCOLLNPGM(MSG_BEGIN_FILE_LIST);
      card.ls();
      SERIAL_PROTOCOLLNPGM(MSG_END_FILE_LIST);
      break;
    case 21:             // M21 - init SD card

      card.initsd();

      break;
    case 22:             //M22 - release SD card
      card.release();

      break;
    case 23:             //M23 - Select file
      starpos = (strchr(strchr_pointer + 4,'*'));
      if(starpos!=NULL)
        *(starpos-1)='\0';
      card.openFile(strchr_pointer + 4,true);
      break;
    case 24:             //M24 - Start SD print
      //      card.startFileprint();
      //      starttime=millis();
      doPreparePrint();
      break;
    case 25:             //M25 - Pause SD print
      card.pauseSDPrint();
      //      card.sdprinting=false;
      //      card.closefile();
      break;
    case 26:             //M26 - Set SD index
      if(card.isOk() && code_seen('S')) {
        card.setIndex(code_value_long());
      }
      break;
    case 27:             //M27 - Get SD status
      card.getStatus();
      break;
    case 28:             //M28 - Start SD write
      starpos = (strchr(strchr_pointer + 4,'*'));
      if(starpos != NULL) {
        char* npos = strchr(cmdbuffer[bufindr], 'N');
        strchr_pointer = strchr(npos,' ') + 1;
        *(starpos-1) = '\0';
      }
      card.openFile(strchr_pointer+4,false);
      break;
    case 29:             //M29 - Stop SD write
      //processed in write to file routine above
      //card,saving = false;
      break;
    case 30:             //M30 <filename> Delete File
      if (card.isOk()) {
        card.closefile();
        starpos = (strchr(strchr_pointer + 4,'*'));
        if(starpos != NULL) {
          char* npos = strchr(cmdbuffer[bufindr], 'N');
          strchr_pointer = strchr(npos,' ') + 1;
          *(starpos-1) = '\0';
        }
        card.removeFile(strchr_pointer + 4);
      }
      break;
    case 923:             //M923 - Select file and start printing
      starpos = (strchr(strchr_pointer + 4,'*'));
      if(starpos!=NULL)
        *(starpos-1)='\0';
      card.openFile(strchr_pointer + 4,true);
      card.startFileprint();
      starttime=millis();
      break;
    case 928:             //M928 - Start SD write
      starpos = (strchr(strchr_pointer + 5,'*'));
      if(starpos != NULL) {
        char* npos = strchr(cmdbuffer[bufindr], 'N');
        strchr_pointer = strchr(npos,' ') + 1;
        *(starpos-1) = '\0';
      }
      card.openLogFile(strchr_pointer+5);
      break;

  #endif //SDSUPPORT

    case 31:             //M31 take time since the start of the SD print or an M109 command
    {
      stoptime=millis();
      char time[30];
      unsigned long t=(stoptime-starttime)/1000;
      int sec,min;
      min=t/60;
      sec=t%60;
      sprintf_P(time, PSTR("%i min, %i sec"), min, sec);
      SERIAL_ECHO_START;
      SERIAL_ECHOLN(time);
      lcd_setstatus(time);
      autotempShutdown();
    }
    break;
    case 42:             //M42 -Change pin status via gcode
      if (code_seen('S'))
      {
        int pin_status = code_value();
        int pin_number = LED_PIN;
        if (code_seen('P') && pin_status >= 0 && pin_status <= 255)
          pin_number = code_value();
        for(int8_t i = 0; i < (int8_t)sizeof(sensitive_pins); i++)
        {
          if (sensitive_pins[i] == pin_number)
          {
            pin_number = -1;
            break;
          }
        }
  #if defined(FAN_PIN) && FAN_PIN > -1
        if (pin_number == FAN_PIN)
          fanSpeed = pin_status;
  #endif
        if (pin_number > -1)
        {
          pinMode(pin_number, OUTPUT);
          digitalWrite(pin_number, pin_status);
          analogWrite(pin_number, pin_status);
        }
      }
      break;
    case 104:             // M104
      if(setTargetedHotend(104)) {
        break;
      }
      if (code_seen('S')) setTargetHotend(code_value(), tmp_extruder);
      setWatch();
      break;
    case 140:             // M140 set bed temp
      if (code_seen('S')) setTargetBed(code_value());
      break;
    case 105:             // M105
      if(setTargetedHotend(105)) {
        break;
      }
  #if defined(TEMP_0_PIN) && TEMP_0_PIN > -1
      SERIAL_PROTOCOLPGM("ok T:");
      SERIAL_PROTOCOL_F(degHotend(tmp_extruder),1);
      SERIAL_PROTOCOLPGM(" /");
      SERIAL_PROTOCOL_F(degTargetHotend(tmp_extruder),1);
      if (Device_isBedHeat) {
        SERIAL_PROTOCOLPGM(" B:");
        SERIAL_PROTOCOL_F(degBed(),1);
        SERIAL_PROTOCOLPGM(" /");
        SERIAL_PROTOCOL_F(degTargetBed(),1);
      }
  #else
      SERIAL_ERROR_START;
      SERIAL_ERRORLNPGM(MSG_ERR_NO_THERMISTORS);
  #endif

      SERIAL_PROTOCOLPGM(" @:");
      SERIAL_PROTOCOL(getHeaterPower(tmp_extruder));

      SERIAL_PROTOCOLPGM(" B@:");
      SERIAL_PROTOCOL(getHeaterPower(-1));

      SERIAL_PROTOCOLLN("");
      return;
      break;
    case 109:
    {            // M109 - Wait for extruder heater to reach target.
      if(setTargetedHotend(109)) {
        break;
      }
      printing_state = PRINT_STATE_HEATING;

      LCD_MESSAGEPGM(MSG_HEATING);
  #ifdef AUTOTEMP
      autotemp_enabled=false;
  #endif
      if (code_seen('S')) setTargetHotend(code_value(), tmp_extruder);
  #ifdef AUTOTEMP
      if (code_seen('S')) autotemp_min=code_value();
      if (code_seen('B')) autotemp_max=code_value();
      if (code_seen('F'))
      {
        autotemp_factor=code_value();
        autotemp_enabled=true;
      }
  #endif

      eeprom_write_word((uint16_t*)EEPROM_SDUPS_HEATING_OFFSET, degTargetHotend(tmp_extruder));

      setWatch();
      codenum = millis();

      /* See if we are heating up or cooling down */
      bool target_direction = isHeatingHotend(tmp_extruder);             // true if heating, false if cooling

  #ifdef TEMP_RESIDENCY_TIME
      long residencyStart;
      residencyStart = -1;
      /* continue to loop until we have reached the target temp
         _and_ until TEMP_RESIDENCY_TIME hasn't passed since we reached it */
      while((residencyStart == -1) ||
            (residencyStart >= 0 && (((unsigned int) (millis() - residencyStart)) < (TEMP_RESIDENCY_TIME * 1000UL))) )
  #else
      while ( target_direction ? (isHeatingHotend(tmp_extruder)) : (isCoolingHotend(tmp_extruder)&&(CooldownNoWait==false)) )
  #endif //TEMP_RESIDENCY_TIME
      {
        if( (millis() - codenum) > 1000UL )
        {                         //Print Temp Reading and remaining time every 1 second while heating up/cooling down
          SERIAL_PROTOCOLPGM("T:");
          SERIAL_PROTOCOL_F(degHotend(tmp_extruder),1);
          SERIAL_PROTOCOLPGM(" E:");
          SERIAL_PROTOCOL((int)tmp_extruder);
  #ifdef TEMP_RESIDENCY_TIME
          SERIAL_PROTOCOLPGM(" W:");
          if(residencyStart > -1)
          {
            codenum = ((TEMP_RESIDENCY_TIME * 1000UL) - (millis() - residencyStart)) / 1000UL;
            SERIAL_PROTOCOLLN( codenum );
          }
          else
          {
            SERIAL_PROTOCOLLN( "?" );
          }
  #else
          SERIAL_PROTOCOLLN("");
  #endif
          codenum = millis();
        }
        manage_heater();
        manage_inactivity();
        lcd_update();
        lifetime_stats_tick();
  #ifdef TEMP_RESIDENCY_TIME
        /* start/restart the TEMP_RESIDENCY_TIME timer whenever we reach target temp for the first time
           or when current temp falls outside the hysteresis after target temp was reached */
        if ((residencyStart == -1 &&  target_direction && (degHotend(tmp_extruder) >= (degTargetHotend(tmp_extruder)-TEMP_WINDOW))) ||
            (residencyStart == -1 && !target_direction && (degHotend(tmp_extruder) <= (degTargetHotend(tmp_extruder)+TEMP_WINDOW))) ||
            (residencyStart > -1 && labs(degHotend(tmp_extruder) - degTargetHotend(tmp_extruder)) > TEMP_HYSTERESIS && (!target_direction || !CooldownNoWait)) )
        {
          residencyStart = millis();
        }
  #endif //TEMP_RESIDENCY_TIME
      }
      LCD_MESSAGEPGM(MSG_HEATING_COMPLETE);
      starttime=millis();
      previous_millis_cmd = millis();
    }
    break;
    case 190:             // M190 - Wait for bed heater to reach target.
      if (Device_isBedHeat) {

        printing_state = PRINT_STATE_HEATING_BED;
        LCD_MESSAGEPGM(MSG_BED_HEATING);
        if (code_seen('S')) setTargetBed(code_value());

        eeprom_write_word((uint16_t*)EEPROM_SDUPS_HEATING_BED_OFFSET, degTargetBed());

        codenum = millis();
        while(current_temperature_bed < target_temperature_bed - TEMP_WINDOW)
        {
          if(( millis() - codenum) > 1000 )           //Print Temp Reading every 1 second while heating up.
          {
            float tt=degHotend(active_extruder);
            SERIAL_PROTOCOLPGM("T:");
            SERIAL_PROTOCOL(tt);
            SERIAL_PROTOCOLPGM(" E:");
            SERIAL_PROTOCOL((int)active_extruder);
            SERIAL_PROTOCOLPGM(" B:");
            SERIAL_PROTOCOL_F(degBed(),1);
            SERIAL_PROTOCOLLN("");
            codenum = millis();
          }
          manage_heater();
          manage_inactivity();
          lcd_update();
          lifetime_stats_tick();
        }
        LCD_MESSAGEPGM(MSG_BED_DONE);
        previous_millis_cmd = millis();
      }
      break;

  #if defined(FAN_PIN) && FAN_PIN > -1
    case 106:             //M106 Fan On
      if (code_seen('S')) {
        fanSpeed=constrain( lround(code_value() * fanSpeedPercent / 100.0),0,255);
      }
      else {
        fanSpeed = lround(255 * int(fanSpeedPercent) / 100.0);
      }
      break;
    case 107:             //M107 Fan Off
      fanSpeed = 0;
      break;
  #endif //FAN_PIN
  #ifdef BARICUDA
      // PWM for HEATER_1_PIN
    #if defined(HEATER_1_PIN) && HEATER_1_PIN > -1
    case 126:             //M126 valve open
      if (code_seen('S')) {
        ValvePressure=constrain(code_value(),0,255);
      }
      else {
        ValvePressure=255;
      }
      break;
    case 127:             //M127 valve closed
      ValvePressure = 0;
      break;
    #endif //HEATER_1_PIN

      // PWM for HEATER_2_PIN
    #if defined(HEATER_2_PIN) && HEATER_2_PIN > -1
    case 128:             //M128 valve open
      if (code_seen('S')) {
        EtoPPressure=constrain(code_value(),0,255);
      }
      else {
        EtoPPressure=255;
      }
      break;
    case 129:             //M129 valve closed
      EtoPPressure = 0;
      break;
    #endif //HEATER_2_PIN
  #endif

  #if defined(PS_ON_PIN) && PS_ON_PIN > -1
    case 80:             // M80 - ATX Power On
      SET_OUTPUT(PS_ON_PIN);             //GND
      WRITE(PS_ON_PIN, PS_ON_AWAKE);
      break;
  #endif

    case 81:             // M81 - ATX Power Off

  #if defined(SUICIDE_PIN) && SUICIDE_PIN > -1
      st_synchronize();
      suicide();
  #elif defined(PS_ON_PIN) && PS_ON_PIN > -1
      SET_OUTPUT(PS_ON_PIN);
      WRITE(PS_ON_PIN, PS_ON_ASLEEP);
  #endif
      break;

    case 82:
      axis_relative_modes[3] = false;
      break;
    case 83:
      axis_relative_modes[3] = true;
      break;
    case 18:             //compatibility
    case 84:             // M84
      if(code_seen('S')) {
        stepper_inactive_time = code_value() * 1000;
      }
      else
      {
        bool all_axis = !((code_seen(axis_codes[0])) || (code_seen(axis_codes[1])) || (code_seen(axis_codes[2]))|| (code_seen(axis_codes[3])));
        if(all_axis)
        {
          st_synchronize();
          disable_e0();
          disable_e1();
          disable_e2();
          finishAndDisableSteppers();
        }
        else
        {
          st_synchronize();
          if(code_seen('X')) disable_x();
          if(code_seen('Y')) disable_y();
          if(code_seen('Z')) disable_z();
  #if ((E0_ENABLE_PIN != X_ENABLE_PIN) && (E1_ENABLE_PIN != Y_ENABLE_PIN)) // Only enable on boards that have seperate ENABLE_PINS
          if(code_seen('E')) {
            disable_e0();
            disable_e1();
            disable_e2();
          }
  #endif
        }
      }
      break;
    case 85:             // M85
      if (code_seen('S')) max_inactive_time = code_value() * 1000;
      break;
    case 92:             // M92
      for(int8_t i=0; i < NUM_AXIS; i++)
      {
        if(code_seen(axis_codes[i]))
        {
          if(i == 3) {             // E
            float value = code_value();
            if(value < 20.0) {
              float factor = axis_steps_per_unit[i] / value;             // increase e constants if M92 E14 is given for netfab.
              max_e_jerk *= factor;
              max_feedrate[i] *= factor;
              axis_steps_per_sqr_second[i] *= factor;
            }
            axis_steps_per_unit[i] = value;
          }
          else {
            axis_steps_per_unit[i] = code_value();
          }
        }
      }
      break;
    case 115:             // M115
      SERIAL_PROTOCOLPGM(MSG_M115_REPORT);
      break;
    case 117:             // M117 display message
      starpos = (strchr(strchr_pointer + 5,'*'));
      if(starpos!=NULL)
        *(starpos-1)='\0';
      lcd_setstatus(strchr_pointer + 5);
      break;
    case 114:             // M114
      SERIAL_PROTOCOLPGM("X:");
      SERIAL_PROTOCOL(current_position[X_AXIS]);
      SERIAL_PROTOCOLPGM("Y:");
      SERIAL_PROTOCOL(current_position[Y_AXIS]);
      SERIAL_PROTOCOLPGM("Z:");
      SERIAL_PROTOCOL(current_position[Z_AXIS]);
      SERIAL_PROTOCOLPGM("E:");
      SERIAL_PROTOCOL(current_position[E_AXIS]);

      SERIAL_PROTOCOLPGM(MSG_COUNT_X);
      SERIAL_PROTOCOL(float(st_get_position(X_AXIS))/axis_steps_per_unit[X_AXIS]);
      SERIAL_PROTOCOLPGM("Y:");
      SERIAL_PROTOCOL(float(st_get_position(Y_AXIS))/axis_steps_per_unit[Y_AXIS]);
      SERIAL_PROTOCOLPGM("Z:");
      SERIAL_PROTOCOL(float(st_get_position(Z_AXIS))/axis_steps_per_unit[Z_AXIS]);

      SERIAL_PROTOCOLPGM(MSG_COUNT_X);
      SERIAL_PROTOCOL(st_get_position(X_AXIS));
      SERIAL_PROTOCOLPGM("Y:");
      SERIAL_PROTOCOL(st_get_position(Y_AXIS));
      SERIAL_PROTOCOLPGM("Z:");
      SERIAL_PROTOCOL(st_get_position(Z_AXIS));

      SERIAL_PROTOCOLLN("");
      break;
    case 120:             // M120
      enable_endstops(false);
      break;
    case 121:             // M121
      enable_endstops(true);
      break;
    case 119:             // M119
      SERIAL_PROTOCOLLN(MSG_M119_REPORT);
  #if defined(X_MIN_PIN) && X_MIN_PIN > -1
      SERIAL_PROTOCOLPGM(MSG_X_MIN);
      SERIAL_PROTOCOLLN(((READ(X_MIN_PIN)^X_ENDSTOPS_INVERTING) ? MSG_ENDSTOP_HIT : MSG_ENDSTOP_OPEN));
  #endif
  #if defined(X_MAX_PIN) && X_MAX_PIN > -1
      SERIAL_PROTOCOLPGM(MSG_X_MAX);
      SERIAL_PROTOCOLLN(((READ(X_MAX_PIN)^X_ENDSTOPS_INVERTING) ? MSG_ENDSTOP_HIT : MSG_ENDSTOP_OPEN));
  #endif
  #if defined(Y_MIN_PIN) && Y_MIN_PIN > -1
      SERIAL_PROTOCOLPGM(MSG_Y_MIN);
      SERIAL_PROTOCOLLN(((READ(Y_MIN_PIN)^Y_ENDSTOPS_INVERTING) ? MSG_ENDSTOP_HIT : MSG_ENDSTOP_OPEN));
  #endif
  #if defined(Y_MAX_PIN) && Y_MAX_PIN > -1
      SERIAL_PROTOCOLPGM(MSG_Y_MAX);
      SERIAL_PROTOCOLLN(((READ(Y_MAX_PIN)^Y_ENDSTOPS_INVERTING) ? MSG_ENDSTOP_HIT : MSG_ENDSTOP_OPEN));
  #endif
  #if defined(Z_MIN_PIN) && Z_MIN_PIN > -1
      SERIAL_PROTOCOLPGM(MSG_Z_MIN);
      SERIAL_PROTOCOLLN(((READ(Z_MIN_PIN)^Z_ENDSTOPS_INVERTING) ? MSG_ENDSTOP_HIT : MSG_ENDSTOP_OPEN));
  #endif
  #if defined(Z_MAX_PIN) && Z_MAX_PIN > -1
      SERIAL_PROTOCOLPGM(MSG_Z_MAX);
      SERIAL_PROTOCOLLN(((READ(Z_MAX_PIN)^Z_ENDSTOPS_INVERTING) ? MSG_ENDSTOP_HIT : MSG_ENDSTOP_OPEN));
  #endif
      break;
    //TODO: update for all axis, use for loop
    case 201:             // M201
      for(int8_t i=0; i < NUM_AXIS; i++)
      {
        if(code_seen(axis_codes[i]))
        {
          max_acceleration_units_per_sq_second[i] = code_value();
        }
      }
      // steps per sq second need to be updated to agree with the units per sq second (as they are what is used in the planner)
      reset_acceleration_rates();
      break;
  #if 0 // Not used for Sprinter/grbl gen6
    case 202:             // M202
      for(int8_t i=0; i < NUM_AXIS; i++) {
        if(code_seen(axis_codes[i])) axis_travel_steps_per_sqr_second[i] = code_value() * axis_steps_per_unit[i];
      }
      break;
  #endif
    case 203:             // M203 max feedrate mm/sec
      for(int8_t i=0; i < NUM_AXIS; i++) {
        if(code_seen(axis_codes[i])) max_feedrate[i] = code_value();
      }
      break;
    case 204:             // M204 acceleration: S - normal moves;  T - filament only moves
    {
      if(code_seen('S')) acceleration = code_value();
      if(code_seen('T')) retract_acceleration = code_value();
    }
    break;
    case 205:             //M205 advanced settings:  minimum travel speed S=while printing T=travel only,  B=minimum segment time X= maximum xy jerk, Z=maximum Z jerk
    {
      if(code_seen('S')) minimumfeedrate = code_value();
      if(code_seen('T')) mintravelfeedrate = code_value();
      if(code_seen('B')) minsegmenttime = code_value();
      if(code_seen('X')) max_xy_jerk = code_value();
      if(code_seen('Z')) max_z_jerk = code_value();
      if(code_seen('E')) max_e_jerk = code_value();
    }
    break;
    case 206:             // M206 additional homing offset
      for(int8_t i=0; i < 3; i++)
      {
        if(code_seen(axis_codes[i])) add_homeing[i] = code_value();
      }
      break;
  #ifdef FWRETRACT
    case 207:             //M207 - set retract length S[positive mm] F[feedrate mm/min] Z[additional zlift/hop]
    {
      if(code_seen('S'))
      {
        retract_length = code_value();
      }
      if(code_seen('F'))
      {
        retract_feedrate = code_value();
      }
      if(code_seen('Z'))
      {
        retract_zlift = code_value();
      }
    } break;
    case 208:             // M208 - set retract recover length S[positive mm surplus to the M207 S*] F[feedrate mm/min]
    {
      if(code_seen('S'))
      {
        retract_recover_length = code_value();
      }
      if(code_seen('F'))
      {
        retract_recover_feedrate = code_value();
      }
    } break;
    case 209:             // M209 - S<1=true/0=false> enable automatic retract detect if the slicer did not support G10/11: every normal extrude-only move will be classified as retract depending on the direction.
    {
      if(code_seen('S'))
      {
        int t= code_value();
        switch(t)
        {
        case 0: autoretract_enabled=false; retracted=false; break;
        case 1: autoretract_enabled=true; retracted=false; break;
        default:
          SERIAL_ECHO_START;
          SERIAL_ECHOPGM(MSG_UNKNOWN_COMMAND);
          SERIAL_ECHO(cmdbuffer[bufindr]);
          SERIAL_ECHOLNPGM("\"");
        }
      }

    } break;
  #endif // FWRETRACT
  #if EXTRUDERS > 1
    case 218:             // M218 - set hotend offset (in mm), T<extruder_number> X<offset_on_X> Y<offset_on_Y>
    {
      if(setTargetedHotend(218)) {
        break;
      }
      if(code_seen('X'))
      {
        extruder_offset[X_AXIS][tmp_extruder] = code_value();
      }
      if(code_seen('Y'))
      {
        extruder_offset[Y_AXIS][tmp_extruder] = code_value();
      }
      SERIAL_ECHO_START;
      SERIAL_ECHOPGM(MSG_HOTEND_OFFSET);
      for(tmp_extruder = 0; tmp_extruder < EXTRUDERS; tmp_extruder++)
      {
        SERIAL_ECHO(" ");
        SERIAL_ECHO(extruder_offset[X_AXIS][tmp_extruder]);
        SERIAL_ECHO(",");
        SERIAL_ECHO(extruder_offset[Y_AXIS][tmp_extruder]);
      }
      SERIAL_ECHOLN("");
    } break;
  #endif
    case 220:             // M220 S<factor in percent>- set speed factor override percentage
    {
      if(code_seen('S'))
      {
        feedmultiply = code_value();
      }
    }
    break;
    case 221:             // M221 S<factor in percent>- set extrude factor override percentage
    {
      if(code_seen('S'))
      {
        extrudemultiply[active_extruder] = code_value();
      }
    }
    break;

  #if NUM_SERVOS > 0
    case 280:             // M280 - set servo position absolute. P: servo index, S: angle or microseconds
    {
      int servo_index = -1;
      int servo_position = 0;
      if (code_seen('P'))
        servo_index = code_value();
      if (code_seen('S')) {
        servo_position = code_value();
        if ((servo_index >= 0) && (servo_index < NUM_SERVOS)) {
          servos[servo_index].write(servo_position);
        }
        else {
          SERIAL_ECHO_START;
          SERIAL_ECHO("Servo ");
          SERIAL_ECHO(servo_index);
          SERIAL_ECHOLN(" out of range");
        }
      }
      else if (servo_index >= 0) {
        SERIAL_PROTOCOL(MSG_OK);
        SERIAL_PROTOCOL(" Servo ");
        SERIAL_PROTOCOL(servo_index);
        SERIAL_PROTOCOL(": ");
        SERIAL_PROTOCOL(servos[servo_index].read());
        SERIAL_PROTOCOLLN("");
      }
    }
    break;
  #endif // NUM_SERVOS > 0

  #if LARGE_FLASH == true && ( BEEPER > 0 || defined(ULTRALCD) )
    case 300:             // M300
    {
      int beepS = code_seen('S') ? code_value() : 110;
      int beepP = code_seen('P') ? code_value() : 1000;
      if (beepS > 0)
      {
    #if BEEPER > 0
        tone(BEEPER, beepS);
        delay(beepP);
        noTone(BEEPER);
    #elif defined(ULTRALCD)
        lcd_buzz(beepS, beepP);
    #endif
      }
      else
      {
        delay(beepP);
      }
    }
    break;
  #endif // M300

  #ifdef PIDTEMP
    case 301:             // M301
    {
      if(code_seen('P')) Kp = code_value();
      if(code_seen('I')) Ki = scalePID_i(code_value());
      if(code_seen('D')) Kd = scalePID_d(code_value());

    #ifdef PID_ADD_EXTRUSION_RATE
      if(code_seen('C')) Kc = code_value();
    #endif

      updatePID();
      SERIAL_PROTOCOL(MSG_OK);
      SERIAL_PROTOCOL(" p:");
      SERIAL_PROTOCOL(Kp);
      SERIAL_PROTOCOL(" i:");
      SERIAL_PROTOCOL(unscalePID_i(Ki));
      SERIAL_PROTOCOL(" d:");
      SERIAL_PROTOCOL(unscalePID_d(Kd));
    #ifdef PID_ADD_EXTRUSION_RATE
      SERIAL_PROTOCOL(" c:");
      //Kc does not have scaling applied above, or in resetting defaults
      SERIAL_PROTOCOL(Kc);
    #endif
      SERIAL_PROTOCOLLN("");
    }
    break;
  #endif //PIDTEMP
  #ifdef PIDTEMPBED
    case 304:             // M304
    {
      if(code_seen('P')) bedKp = code_value();
      if(code_seen('I')) bedKi = scalePID_i(code_value());
      if(code_seen('D')) bedKd = scalePID_d(code_value());

      updatePID();
      SERIAL_PROTOCOL(MSG_OK);
      SERIAL_PROTOCOL(" p:");
      SERIAL_PROTOCOL(bedKp);
      SERIAL_PROTOCOL(" i:");
      SERIAL_PROTOCOL(unscalePID_i(bedKi));
      SERIAL_PROTOCOL(" d:");
      SERIAL_PROTOCOL(unscalePID_d(bedKd));
      SERIAL_PROTOCOLLN("");
    }
    break;
  #endif //PIDTEMP
    case 240:             // M240  Triggers a camera by emulating a Canon RC-1 : http://www.doc-diy.net/photo/rc-1_hacked/
    {
  #if defined(PHOTOGRAPH_PIN) && PHOTOGRAPH_PIN > -1
      const uint8_t NUM_PULSES=16;
      const float PULSE_LENGTH=0.01524;
      for(int i=0; i < NUM_PULSES; i++) {
        WRITE(PHOTOGRAPH_PIN, HIGH);
        _delay_ms(PULSE_LENGTH);
        WRITE(PHOTOGRAPH_PIN, LOW);
        _delay_ms(PULSE_LENGTH);
      }
      delay(7.33);
      for(int i=0; i < NUM_PULSES; i++) {
        WRITE(PHOTOGRAPH_PIN, HIGH);
        _delay_ms(PULSE_LENGTH);
        WRITE(PHOTOGRAPH_PIN, LOW);
        _delay_ms(PULSE_LENGTH);
      }
  #endif
    }
    break;
  #ifdef PREVENT_DANGEROUS_EXTRUDE
    case 302:             // allow cold extrudes, or set the minimum extrude temperature
    {
      float temp = .0;
      if (code_seen('S')) temp=code_value();
      set_extrude_min_temp(temp);
    }
    break;
  #endif
    case 303:             // M303 PID autotune
    {
      float temp = 150.0;
      int e=0;
      int c=5;
      if (code_seen('E')) e=code_value();
      if (e<0)
        temp=70;
      if (code_seen('S')) temp=code_value();
      if (code_seen('C')) c=code_value();
      PID_autotune(temp, e, c);
    }
    break;
    case 400:             // M400 finish all moves
    {
      st_synchronize();
    }
    break;
    case 500:             // M500 Store settings in EEPROM
    {
      Config_StoreSettings();
    }
    break;
    case 501:             // M501 Read settings from EEPROM
    {
      Config_RetrieveSettings();
    }
    break;
    case 502:             // M502 Revert to default settings
    {
      Config_ResetDefault();
    }
    break;
    case 503:             // M503 print settings currently in memory
    {
      Config_PrintSettings();
    }
    break;
  #ifdef ABORT_ON_ENDSTOP_HIT_FEATURE_ENABLED
    case 540:
    {
      if(code_seen('S')) abort_on_endstop_hit = code_value() > 0;
    }
    break;
  #endif
  #ifdef FILAMENTCHANGEENABLE
    case 600:             //Pause for filament change X[pos] Y[pos] Z[relative lift] E[initial retract] L[later retract distance for removal]
    {
      float target[4];
      float lastpos[4];
      target[X_AXIS]=current_position[X_AXIS];
      target[Y_AXIS]=current_position[Y_AXIS];
      target[Z_AXIS]=current_position[Z_AXIS];
      target[E_AXIS]=current_position[E_AXIS];
      lastpos[X_AXIS]=current_position[X_AXIS];
      lastpos[Y_AXIS]=current_position[Y_AXIS];
      lastpos[Z_AXIS]=current_position[Z_AXIS];
      lastpos[E_AXIS]=current_position[E_AXIS];
      //retract by E
      if(code_seen('E'))
      {
        target[E_AXIS]+= code_value();
      }
      else
      {
    #ifdef FILAMENTCHANGE_FIRSTRETRACT
        target[E_AXIS]+= FILAMENTCHANGE_FIRSTRETRACT;
    #endif
      }
      plan_buffer_line(target[X_AXIS], target[Y_AXIS], target[Z_AXIS], target[E_AXIS], feedrate/60, active_extruder);

      //lift Z
      if(code_seen('Z'))
      {
        target[Z_AXIS]+= code_value();
      }
      else
      {
    #ifdef FILAMENTCHANGE_ZADD
        target[Z_AXIS]+= FILAMENTCHANGE_ZADD;
    #endif
      }
      plan_buffer_line(target[X_AXIS], target[Y_AXIS], target[Z_AXIS], target[E_AXIS], feedrate/60, active_extruder);

      //move xy
      if(code_seen('X'))
      {
        target[X_AXIS]+= code_value();
      }
      else
      {
    #ifdef FILAMENTCHANGE_XPOS
        target[X_AXIS]= FILAMENTCHANGE_XPOS;
    #endif
      }
      if(code_seen('Y'))
      {
        target[Y_AXIS]= code_value();
      }
      else
      {
    #ifdef FILAMENTCHANGE_YPOS
        target[Y_AXIS]= FILAMENTCHANGE_YPOS;
    #endif
      }

      plan_buffer_line(target[X_AXIS], target[Y_AXIS], target[Z_AXIS], target[E_AXIS], feedrate/60, active_extruder);

      if(code_seen('L'))
      {
        target[E_AXIS]+= code_value();
      }
      else
      {
    #ifdef FILAMENTCHANGE_FINALRETRACT
        target[E_AXIS]+= FILAMENTCHANGE_FINALRETRACT;
    #endif
      }

      plan_buffer_line(target[X_AXIS], target[Y_AXIS], target[Z_AXIS], target[E_AXIS], feedrate/60, active_extruder);

      //finish moves
      st_synchronize();
      //disable extruder steppers so filament can be removed
      disable_e0();
      disable_e1();
      disable_e2();
      delay(100);
      LCD_ALERTMESSAGEPGM(MSG_FILAMENTCHANGE);
      uint8_t cnt=0;
      while(!lcd_clicked()) {
        cnt++;
        manage_heater();
        manage_inactivity();
        lcd_update();
        lifetime_stats_tick();
        if(cnt==0)
        {
    #if BEEPER > 0
          SET_OUTPUT(BEEPER);

          WRITE(BEEPER,HIGH);
          delay(3);
          WRITE(BEEPER,LOW);
          delay(3);
    #else
          lcd_buzz(1000/6,100);
    #endif
        }
      }

      //return to normal
      if(code_seen('L'))
      {
        target[E_AXIS]+= -code_value();
      }
      else
      {
    #ifdef FILAMENTCHANGE_FINALRETRACT
        target[E_AXIS]+=(-1)*FILAMENTCHANGE_FINALRETRACT;
    #endif
      }
      current_position[E_AXIS]=target[E_AXIS];             //the long retract of L is compensated by manual filament feeding
      plan_set_e_position(current_position[E_AXIS]);
      plan_buffer_line(target[X_AXIS], target[Y_AXIS], target[Z_AXIS], target[E_AXIS], feedrate/60, active_extruder);             //should do nothing
      plan_buffer_line(lastpos[X_AXIS], lastpos[Y_AXIS], target[Z_AXIS], target[E_AXIS], feedrate/60, active_extruder);             //move xy back
      plan_buffer_line(lastpos[X_AXIS], lastpos[Y_AXIS], lastpos[Z_AXIS], target[E_AXIS], feedrate/60, active_extruder);             //move z back
      plan_buffer_line(lastpos[X_AXIS], lastpos[Y_AXIS], lastpos[Z_AXIS], lastpos[E_AXIS], feedrate/60, active_extruder);             //final untretract
    }
    break;
  #endif //FILAMENTCHANGEENABLE
  #ifdef ENABLE_ULTILCD2
    case 601:             //Pause in UltiLCD2, X[pos] Y[pos] Z[relative lift] L[later retract distance]
    {
      float target[4];
      float lastpos[4];
      target[X_AXIS]=current_position[X_AXIS];
      target[Y_AXIS]=current_position[Y_AXIS];
      target[Z_AXIS]=current_position[Z_AXIS];
      target[E_AXIS]=current_position[E_AXIS];
      lastpos[X_AXIS]=current_position[X_AXIS];
      lastpos[Y_AXIS]=current_position[Y_AXIS];
      lastpos[Z_AXIS]=current_position[Z_AXIS];
      lastpos[E_AXIS]=current_position[E_AXIS];

      target[E_AXIS] -= retract_length/volume_to_filament_length[active_extruder];
      plan_buffer_line(target[X_AXIS], target[Y_AXIS], target[Z_AXIS], target[E_AXIS], retract_feedrate/60, active_extruder);

      //lift Z
      if(code_seen('Z'))
      {
        target[Z_AXIS]+= code_value();
      }
      plan_buffer_line(target[X_AXIS], target[Y_AXIS], target[Z_AXIS], target[E_AXIS], homing_feedrate[Z_AXIS]/60, active_extruder);

      //move xy
      if(code_seen('X'))
      {
        target[X_AXIS] = code_value();
      }
      if(code_seen('Y'))
      {
        target[Y_AXIS] = code_value();
      }
      plan_buffer_line(target[X_AXIS], target[Y_AXIS], target[Z_AXIS], target[E_AXIS], homing_feedrate[X_AXIS]/60, active_extruder);

      if(code_seen('L'))
      {
        target[E_AXIS] -= code_value()/volume_to_filament_length[active_extruder];
      }
      plan_buffer_line(target[X_AXIS], target[Y_AXIS], target[Z_AXIS], target[E_AXIS], retract_feedrate/60, active_extruder);

      //finish moves
      st_synchronize();
      //disable extruder steppers so filament can be removed
      disable_e0();
      disable_e1();
      disable_e2();
      while(card.pause) {
        manage_heater();
        manage_inactivity();
        lcd_update();
        lifetime_stats_tick();
      }

      //return to normal
      if(code_seen('L'))
      {
        target[E_AXIS] += code_value()/volume_to_filament_length[active_extruder];
      }
      plan_buffer_line(target[X_AXIS], target[Y_AXIS], target[Z_AXIS], target[E_AXIS], retract_feedrate/60, active_extruder);             //Move back the L feed.
      plan_buffer_line(lastpos[X_AXIS], lastpos[Y_AXIS], target[Z_AXIS], target[E_AXIS], homing_feedrate[X_AXIS]/60, active_extruder);             //move xy back
      plan_buffer_line(lastpos[X_AXIS], lastpos[Y_AXIS], lastpos[Z_AXIS], target[E_AXIS], homing_feedrate[Z_AXIS]/60, active_extruder);             //move z back
      plan_buffer_line(lastpos[X_AXIS], lastpos[Y_AXIS], lastpos[Z_AXIS], lastpos[E_AXIS], retract_feedrate/60, active_extruder);             //final untretract
    }
    break;

    case 605:             // M605 store current set values
    {
      uint8_t tmp_select;
      if (code_seen('S'))
      {
        tmp_select = code_value();
        if (tmp_select>9) tmp_select=9;
      }
      else
        tmp_select = 0;
      machinesettings_tempsave[tmp_select].feedmultiply = feedmultiply;
      machinesettings_tempsave[tmp_select].BedTemperature = target_temperature_bed;
      machinesettings_tempsave[tmp_select].fanSpeed = fanSpeed;
      for (int i=0; i<EXTRUDERS; i++)
      {
        machinesettings_tempsave[tmp_select].HotendTemperature[i] = target_temperature[i];
        machinesettings_tempsave[tmp_select].extrudemultiply[i] = extrudemultiply[i];
      }
      for (int i=0; i<NUM_AXIS; i++)
      {
        machinesettings_tempsave[tmp_select].max_acceleration_units_per_sq_second[i] = max_acceleration_units_per_sq_second[i];
        machinesettings_tempsave[tmp_select].max_feedrate[i] = max_feedrate[i];
      }
      machinesettings_tempsave[tmp_select].acceleration = acceleration;
      machinesettings_tempsave[tmp_select].minimumfeedrate = minimumfeedrate;
      machinesettings_tempsave[tmp_select].mintravelfeedrate = mintravelfeedrate;
      machinesettings_tempsave[tmp_select].minsegmenttime = minsegmenttime;
      machinesettings_tempsave[tmp_select].max_xy_jerk = max_xy_jerk;
      machinesettings_tempsave[tmp_select].max_z_jerk = max_z_jerk;
      machinesettings_tempsave[tmp_select].max_e_jerk = max_e_jerk;
      machinesettings_tempsave[tmp_select].has_saved_settings = 1;
    }
    break;

    case 606:             // M606 recall saved values
    {
      uint8_t tmp_select;
      if (code_seen('S'))
      {
        tmp_select = code_value();
        if (tmp_select>9) tmp_select=9;
      }
      else
        tmp_select = 0;
      if (machinesettings_tempsave[tmp_select].has_saved_settings > 0)
      {
        feedmultiply = machinesettings_tempsave[tmp_select].feedmultiply;
        target_temperature_bed = machinesettings_tempsave[tmp_select].BedTemperature;
        fanSpeed = machinesettings_tempsave[tmp_select].fanSpeed;
        for (int i=0; i<EXTRUDERS; i++)
        {
          target_temperature[i] = machinesettings_tempsave[tmp_select].HotendTemperature[i];
          extrudemultiply[i] = machinesettings_tempsave[tmp_select].extrudemultiply[i];
        }
        for (int i=0; i<NUM_AXIS; i++)
        {
          max_acceleration_units_per_sq_second[i] = machinesettings_tempsave[tmp_select].max_acceleration_units_per_sq_second[i];
          max_feedrate[i] = machinesettings_tempsave[tmp_select].max_feedrate[i];
        }
        acceleration = machinesettings_tempsave[tmp_select].acceleration;
        minimumfeedrate = machinesettings_tempsave[tmp_select].minimumfeedrate;
        mintravelfeedrate = machinesettings_tempsave[tmp_select].mintravelfeedrate;
        minsegmenttime = machinesettings_tempsave[tmp_select].minsegmenttime;
        max_xy_jerk = machinesettings_tempsave[tmp_select].max_xy_jerk;
        max_z_jerk = machinesettings_tempsave[tmp_select].max_z_jerk;
        max_e_jerk = machinesettings_tempsave[tmp_select].max_e_jerk;
      }
    }
    break;
  #endif//ENABLE_ULTILCD2

    case 907:             // M907 Set digital trimpot motor current using axis codes.
    {
  #if defined(DIGIPOTSS_PIN) && DIGIPOTSS_PIN > -1
      for(int i=0; i<NUM_AXIS; i++) if(code_seen(axis_codes[i])) digipot_current(i,code_value());
      if(code_seen('B')) digipot_current(4,code_value());
      if(code_seen('S')) for(int i=0; i<=4; i++) digipot_current(i,code_value());
  #endif
  #if defined(MOTOR_CURRENT_PWM_XY_PIN) && MOTOR_CURRENT_PWM_XY_PIN > -1
      if(code_seen('X')) digipot_current(0, code_value());
  #endif
  #if defined(MOTOR_CURRENT_PWM_Z_PIN) && MOTOR_CURRENT_PWM_Z_PIN > -1
      if(code_seen('Z')) digipot_current(1, code_value());
  #endif
  #if defined(MOTOR_CURRENT_PWM_E_PIN) && MOTOR_CURRENT_PWM_E_PIN > -1
      if(code_seen('E')) digipot_current(2, code_value());
  #endif
    }
    break;
    case 908:             // M908 Control digital trimpot directly.
    {
  #if defined(DIGIPOTSS_PIN) && DIGIPOTSS_PIN > -1
      uint8_t channel,current;
      if(code_seen('P')) channel=code_value();
      if(code_seen('S')) current=code_value();
      digitalPotWrite(channel, current);
  #endif
    }
    break;
    case 350:             // M350 Set microstepping mode. Warning: Steps per unit remains unchanged. S code sets stepping mode for all drivers.
    {
  #if defined(X_MS1_PIN) && X_MS1_PIN > -1
      if(code_seen('S')) for(int i=0; i<=4; i++) microstep_mode(i,code_value());
      for(int i=0; i<NUM_AXIS; i++) if(code_seen(axis_codes[i])) microstep_mode(i,(uint8_t)code_value());
      if(code_seen('B')) microstep_mode(4,code_value());
      microstep_readings();
  #endif
    }
    break;
    case 351:             // M351 Toggle MS1 MS2 pins directly, S# determines MS1 or MS2, X# sets the pin high/low.
    {
  #if defined(X_MS1_PIN) && X_MS1_PIN > -1
      if(code_seen('S')) switch((int)code_value())
        {
        case 1:
          for(int i=0; i<NUM_AXIS; i++) if(code_seen(axis_codes[i])) microstep_ms(i,code_value(),-1);
          if(code_seen('B')) microstep_ms(4,code_value(),-1);
          break;
        case 2:
          for(int i=0; i<NUM_AXIS; i++) if(code_seen(axis_codes[i])) microstep_ms(i,-1,code_value());
          if(code_seen('B')) microstep_ms(4,-1,code_value());
          break;
        }
      microstep_readings();
  #endif
    }
    break;
    case 999:             // M999: Restart after being stopped
      Stopped = false;
      lcd_reset_alert_level();
      gcode_LastN = Stopped_gcode_LastN;
      FlushSerialRequestResend();
      break;
    case 780:
      if (code_seen('S')) targetFanSpeed=code_value();
      break;
    case 781:
      if (code_seen('S')) targetFeedmultiply=code_value();
      break;
    case 760:
      isWindowsServerStarted = true;
      SERIAL_ECHOLNPGM("M760");
      break;
    case 761:
      currentMenu = lcd_menu_bluetooth;
      SERIAL_ECHOLNPGM("M761");
      break;
    case 762:
      if (isWindowsPrinting) {
        SERIAL_ECHOLNPGM("M762 P1");
      }
      else{
        SERIAL_ECHOLNPGM("M762 P0");
      }
      break;
    case 763:
      if (isWindowsPrinting) {
        SERIAL_ECHOLNPGM("M763 P0");
      }
      else{
        card.release();
        wifiSDChangeMaster(WIFI_SD_WINDOWS);
        SERIAL_ECHOLNPGM("M763 P1");
      }
      break;
    case 764:
      wifiSDChangeMaster(WIFI_SD_OVERLORD);
      delay(1000);
      card.initsd();
      if (card.isOk()) {
        SERIAL_ECHOLNPGM("M764 P1");
      }
      else{
        SERIAL_ECHOLNPGM("M764 P0");
      }
      break;
    case 768:
      if (!isWindowsPrinting) {
        card.setroot();
        card.openFile("WIFI", true);
        if (card.isFileOpen())
        {
          char buffer[64];
          for(uint8_t n=0; n<8; n++)
          {
            card.fgets(buffer, sizeof(buffer));
            buffer[sizeof(buffer)-1] = '\0';
            while (strlen(buffer) > 0 && buffer[strlen(buffer)-1] < ' ') buffer[strlen(buffer)-1] = '\0';
            if (strncmp_P(buffer, PSTR(";TIME:"), 6) == 0)
              LCD_DETAIL_CACHE_TIME() = atol(buffer + 6);
            else if (strncmp_P(buffer, PSTR(";MATERIAL:"), 10) == 0)
              LCD_DETAIL_CACHE_MATERIAL(0) = atol(buffer + 10);
    #if EXTRUDERS > 1
            else if (strncmp_P(buffer, PSTR(";MATERIAL2:"), 11) == 0)
              LCD_DETAIL_CACHE_MATERIAL(1) = atol(buffer + 11);
    #endif
          }
          card.setIndex(0);

          doPreparePrint();
          SERIAL_ECHOLNPGM("M768 P1");
        }
        else{
          SERIAL_ECHOLNPGM("M768 P0");
        }

      }
      break;
    case 765:
      if (isWindowsPrinting) {
        if (card.isFileOpen()) {
          int progress = card.getFilePos() * 100 / card.getFileSize();
          SERIAL_ECHOPGM("M765 P");
          SERIAL_ECHOLN(progress);
        }
        else{
          SERIAL_ECHOLNPGM("M765 P100");
        }
      }
      else{
        SERIAL_ECHOLNPGM("M765 P101");
      }
      break;

    case 767:
      doPausePrint();
      currentMenu =  lcd_menu_print_abort;
      SERIAL_ECHOLNPGM("M767");
      break;

    case 769:
      SERIAL_ECHOPGM("M769 ");
      SERIAL_ECHOLNPGM(STRING_CONFIG_H_AUTHOR);
      break;
    case 790:
      if (code_seen('P')) {
        if (code_value()==123) {
          if (code_seen('S')) {
            strchr_pointer++;
            char eepromBuffer[8];

            eeprom_write_block(strchr_pointer, (uint8_t*)EEPROM_DEVICE_ID, 8);
            eeprom_write_block(strchr_pointer, (uint8_t*)EEPROM_DEVICE_ID+8, 8);
            eeprom_write_block(strchr_pointer, (uint8_t*)EEPROM_DEVICE_ID+16, 8);

            eeprom_read_block(eepromBuffer, (uint8_t*)EEPROM_DEVICE_ID, 8);
            if (!strncmp(eepromBuffer, strchr_pointer, 8)) {
              eeprom_read_block(eepromBuffer, (uint8_t*)EEPROM_DEVICE_ID+8, 8);
              if (!strncmp(eepromBuffer, strchr_pointer, 8)) {
                eeprom_read_block(eepromBuffer, (uint8_t*)EEPROM_DEVICE_ID+16, 8);
                if (!strncmp(eepromBuffer, strchr_pointer, 8)) {
//                  SERIAL_DEBUGLNPGM("Device ID Stored:");
//                  SERIAL_DEBUGLN(strchr_pointer);
                  currentMenu = lcd_menu_advanced_version;
                }
              }
            }
          }
        }
      }
      break;
    case 791:

      SERIAL_DEBUGLN("RECIEVED!");

      break;


  #ifdef ENABLE_ULTILCD2
    case 10000:            //M10000 - Clear the whole LCD
      lcd_lib_clear();
      break;
    case 10001:            //M10001 - Draw text on LCD, M10002 X0 Y0 SText
    {
      uint8_t x = 0, y = 0;
      if (code_seen('X')) x = code_value_long();
      if (code_seen('Y')) y = code_value_long();
      if (code_seen('S')) lcd_lib_draw_string(x, y, &cmdbuffer[bufindr][strchr_pointer - cmdbuffer[bufindr] + 1]);
    }
    break;
    case 10002:            //M10002 - Draw inverted text on LCD, M10002 X0 Y0 SText
    {
      uint8_t x = 0, y = 0;
      if (code_seen('X')) x = code_value_long();
      if (code_seen('Y')) y = code_value_long();
      if (code_seen('S')) lcd_lib_clear_string(x, y, &cmdbuffer[bufindr][strchr_pointer - cmdbuffer[bufindr] + 1]);
    }
    break;
    case 10003:            //M10003 - Draw square on LCD, M10003 X1 Y1 W10 H10
    {
      uint8_t x = 0, y = 0, w = 1, h = 1;
      if (code_seen('X')) x = code_value_long();
      if (code_seen('Y')) y = code_value_long();
      if (code_seen('W')) w = code_value_long();
      if (code_seen('H')) h = code_value_long();
      lcd_lib_set(x, y, x + w, y + h);
    }
    break;
    case 10004:            //M10004 - Draw shaded square on LCD, M10004 X1 Y1 W10 H10
    {
      uint8_t x = 0, y = 0, w = 1, h = 1;
      if (code_seen('X')) x = code_value_long();
      if (code_seen('Y')) y = code_value_long();
      if (code_seen('W')) w = code_value_long();
      if (code_seen('H')) h = code_value_long();
      lcd_lib_draw_shade(x, y, x + w, y + h);
    }
    break;
    case 10005:            //M10005 - Draw shaded square on LCD, M10004 X1 Y1 W10 H10
    {
      uint8_t x = 0, y = 0, w = 1, h = 1;
      if (code_seen('X')) x = code_value_long();
      if (code_seen('Y')) y = code_value_long();
      if (code_seen('W')) w = code_value_long();
      if (code_seen('H')) h = code_value_long();
      lcd_lib_draw_shade(x, y, x + w, y + h);
    }
    break;
    case 10010:            //M10010 - Request LCD screen button info (R:[rotation difference compared to previous request] B:[button down])
    {
      SERIAL_PROTOCOLPGM("ok R:");
      SERIAL_PROTOCOL_F(lcd_lib_encoder_pos, 10);
      lcd_lib_encoder_pos = 0;
      if (lcd_lib_button_down)
        SERIAL_PROTOCOLLNPGM(" B:1");
      else
        SERIAL_PROTOCOLLNPGM(" B:0");
      return;
    }
    break;

    case 20000:
    {
      if(code_seen('F'))
        setStepperTorque((uint8_t)(code_value_long()),(uint8_t)(code_value_long()),(uint8_t)(code_value_long()));
    }
    break;
    case 20001:
    {
      if(code_seen('Z')) {
        touchPlateOffset = code_value();
      }
      else{
        touchPlateOffset=TouchPlateOffset;
      }

    }
    break;

    case 20002:
    {
      add_homeing[Z_AXIS]=0;
      fittingBedReset();
      fittingBedResetK();
      fittingBedResetBackUp();
    }
    break;
  #endif//ENABLE_ULTILCD2
    }
  }

  else if(code_seen('T'))
  {
    tmp_extruder = code_value();
    if(tmp_extruder >= EXTRUDERS) {
      SERIAL_ECHO_START;
      SERIAL_ECHO("T");
      SERIAL_ECHO(tmp_extruder);
      SERIAL_ECHOLN(MSG_INVALID_EXTRUDER);
    }
    else {
      boolean make_move = false;
      if(code_seen('F')) {
        make_move = true;
        next_feedrate = code_value();
        if(next_feedrate > 0.0) {
          feedrate = next_feedrate;
        }
      }
  #if EXTRUDERS > 1
      if(tmp_extruder != active_extruder) {
        // Save current position to return to after applying extruder offset
        memcpy(destination, current_position, sizeof(destination));
        // Offset extruder (only by XY)
        int i;
        for(i = 0; i < 2; i++) {
          current_position[i] = current_position[i] -
                                extruder_offset[i][active_extruder] +
                                extruder_offset[i][tmp_extruder];
        }
        // Set the new active extruder and position
        active_extruder = tmp_extruder;
        plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);
        // Move to the old position if 'F' was in the parameters
        if(make_move && Stopped == false) {
          prepare_move();
        }
      }
  #endif
      SERIAL_ECHO_START;
      SERIAL_ECHO(MSG_ACTIVE_EXTRUDER);
      SERIAL_PROTOCOLLN((int)active_extruder);
    }
  }
  else if (strcmp_P(cmdbuffer[bufindr], PSTR("Electronics_test")) == 0)
  {
    run_electronics_test();
  }
  else
  {
    SERIAL_ECHO_START;
    SERIAL_ECHOPGM(MSG_UNKNOWN_COMMAND);
    SERIAL_ECHO(cmdbuffer[bufindr]);
    SERIAL_ECHOLNPGM("\"");
  }
  printing_state = PRINT_STATE_NORMAL;

  ClearToSend();
}

void FlushSerialRequestResend()
{
  //char cmdbuffer[bufindr][100]="Resend:";
  MYSERIAL.flush();
  SERIAL_PROTOCOLPGM(MSG_RESEND);
  SERIAL_PROTOCOLLN(gcode_LastN + 1);
  ClearToSend();
}

void ClearToSend()
{
  previous_millis_cmd = millis();
  #ifdef SDSUPPORT
  if(commandFrom[bufindr]==FromSerial)
    return;
  #endif //SDSUPPORT
  SERIAL_PROTOCOLLNPGM(MSG_OK);
}

void get_coordinates()
{
  bool seen[4]={false,false,false,false};
  char buffer[64];

  for(int8_t i=0; i < NUM_AXIS; i++)
  {
    if(code_seen(axis_codes[i]))
    {
      destination[i] = (float)code_value() + (axis_relative_modes[i] || relative_mode)*current_position[i];
      seen[i]=true;
    }
    else
    {
      destination[i] = current_position[i]; //Are these else lines really needed?
    }
  }

  if (seen[Z_AXIS] && commandFrom[bufindr] >= 0) {
    if (destination[Z_AXIS]>current_position[Z_AXIS]) {
      if (destination[Z_AXIS] >= SDUPSGetCoordinateLastZ + 0.1) {
        SDUPSStorePosition(commandFrom[bufindr]);
        SDUPSGetCoordinateLastZ = destination[Z_AXIS];
        SDUPSGetCoordinateZ = current_position[Z_AXIS];

        if (targetFanSpeed) {
          int nextFanSpeed=lround(int(fanSpeed)*100/float(fanSpeedPercent)) +64;
          if (nextFanSpeed>=targetFanSpeed) {
            nextFanSpeed=targetFanSpeed;
            targetFanSpeed=0;
          }

          sprintf_P(buffer, PSTR("M106 S%i"), nextFanSpeed);
          enquecommand(buffer);
        }

        if (targetFeedmultiply) {
          int feedmultiplyBuf=feedmultiply+20;

          if (targetFeedmultiply<=feedmultiplyBuf) {
            feedmultiplyBuf=targetFeedmultiply;
            targetFeedmultiply=0;
          }

          sprintf_P(buffer, PSTR("M220 S%i"), feedmultiplyBuf);
          enquecommand(buffer);
        }
      }
    }
    else{
      SDUPSOverlapPosition(commandFrom[bufindr]);
      SDUPSGetCoordinateLastZ = SDUPSGetCoordinateZ;
    }
  }

  if(code_seen('F'))
  {
    next_feedrate = code_value();
    if(next_feedrate > 0.0) {
      if (next_feedrate>4000) {
        feedrate = (next_feedrate-4000)/2 + 4000;
      }
      else{
        feedrate = next_feedrate;
      }
    }
  }
  #ifdef FWRETRACT
  if(autoretract_enabled)
  {
    if( !(seen[X_AXIS] || seen[Y_AXIS] || seen[Z_AXIS]) && seen[E_AXIS])
    {
      float echange=destination[E_AXIS]-current_position[E_AXIS];
      if(echange<-MIN_RETRACT) //retract
      {
        if(!retracted)
        {
          destination[Z_AXIS]+=retract_zlift; //not sure why chaninging current_position negatively does not work.
          //if slicer retracted by echange=-1mm and you want to retract 3mm, corrrectede=-2mm additionally
          float correctede=-echange-retract_length;
          //to generate the additional steps, not the destination is changed, but inversely the current position
          current_position[E_AXIS]+=-correctede;
          feedrate=retract_feedrate;
          retracted=true;
        }
      }
      else if(echange>MIN_RETRACT) //retract_recover
      {
        if(retracted)
        {
          //current_position[Z_AXIS]+=-retract_zlift;
          //if slicer retracted_recovered by echange=+1mm and you want to retract_recover 3mm, corrrectede=2mm additionally
          float correctede=-echange+1*retract_length+retract_recover_length; //total unretract=retract_length+retract_recover_length[surplus]
          current_position[E_AXIS]+=correctede; //to generate the additional steps, not the destination is changed, but inversely the current position
          feedrate=retract_recover_feedrate;
          retracted=false;
        }
      }
    }
  }
  #endif //FWRETRACT
}

void get_arc_coordinates()
{
  #ifdef SF_ARC_FIX
  bool relative_mode_backup = relative_mode;
  relative_mode = true;
  #endif
  get_coordinates();
  #ifdef SF_ARC_FIX
  relative_mode=relative_mode_backup;
  #endif

  if(code_seen('I')) {
    offset[0] = code_value();
  }
  else {
    offset[0] = 0.0;
  }
  if(code_seen('J')) {
    offset[1] = code_value();
  }
  else {
    offset[1] = 0.0;
  }
}

void clamp_to_software_endstops(float target[3])
{
  if (min_software_endstops) {
    if (target[X_AXIS] < min_pos[X_AXIS]) target[X_AXIS] = min_pos[X_AXIS];
    if (target[Y_AXIS] < min_pos[Y_AXIS]) target[Y_AXIS] = min_pos[Y_AXIS];
    if (target[Z_AXIS] < min_pos[Z_AXIS]) target[Z_AXIS] = min_pos[Z_AXIS];
  }

  if (max_software_endstops) {
    if (target[X_AXIS] > max_pos[X_AXIS]) target[X_AXIS] = max_pos[X_AXIS];
    if (target[Y_AXIS] > max_pos[Y_AXIS]) target[Y_AXIS] = max_pos[Y_AXIS];
    if (target[Z_AXIS] > max_pos[Z_AXIS]) target[Z_AXIS] = max_pos[Z_AXIS];
  }
}

void calculate_delta_reverse(float theDelta[3], float theResult[3])
{
  float x,y,z;

  float a1=sq(DELTA_DIAGONAL_ROD)-sq(DELTA_TOWER1_X)-sq(DELTA_TOWER1_Y)-sq(theDelta[X_AXIS]);
  float a2=sq(DELTA_DIAGONAL_ROD)-sq(DELTA_TOWER2_X)-sq(DELTA_TOWER2_Y)-sq(theDelta[Y_AXIS]);
  float a3=sq(DELTA_DIAGONAL_ROD)-sq(DELTA_TOWER3_X)-sq(DELTA_TOWER3_Y)-sq(theDelta[Z_AXIS]);

  float z21=theDelta[Y_AXIS]-theDelta[X_AXIS];
  float z31=theDelta[Z_AXIS]-theDelta[X_AXIS];

  float a21=-(a2-a1)/2;
  float a31=-(a3-a1)/2;

  float d=(DELTA_TOWER2_X-DELTA_TOWER1_X)*(DELTA_TOWER3_Y-DELTA_TOWER1_Y)-(DELTA_TOWER2_Y-DELTA_TOWER1_Y)*(DELTA_TOWER3_X-DELTA_TOWER1_X);
  float b0=(a21*(DELTA_TOWER3_Y-DELTA_TOWER1_Y)-a31*(DELTA_TOWER2_Y-DELTA_TOWER1_Y))/d;
  float b1=((DELTA_TOWER2_Y-DELTA_TOWER1_Y)*z31-(DELTA_TOWER3_Y-DELTA_TOWER1_Y)*z21)/d;
  float c0=(a31*(DELTA_TOWER2_X-DELTA_TOWER1_X)-a21*(DELTA_TOWER3_X-DELTA_TOWER1_X))/d;
  float c1=((DELTA_TOWER3_X-DELTA_TOWER1_X)*z21-(DELTA_TOWER2_X-DELTA_TOWER1_X)*z31)/d;

  float e=sq(b1)+sq(c1)+1;
  float f=b1*(b0-DELTA_TOWER1_X)+c1*(c0-DELTA_TOWER1_Y)-theDelta[X_AXIS];
  float g=sq(b0-DELTA_TOWER1_X)+sq(c0-DELTA_TOWER1_Y)+sq(theDelta[X_AXIS])-sq(DELTA_DIAGONAL_ROD);

  theResult[Z_AXIS]=(-f-sqrt(sq(f)-e*g))/e;
  theResult[X_AXIS]=b0+b1*theResult[Z_AXIS];
  theResult[Y_AXIS]=c0+c1*theResult[Z_AXIS];
}




void prepare_move()
{
  clamp_to_software_endstops(destination);

  previous_millis_cmd = millis();
  if( (current_position[X_AXIS] == destination [X_AXIS]) && (current_position[Y_AXIS] == destination [Y_AXIS])) {
    plan_buffer_line(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS], destination[E_AXIS], feedrate/60, active_extruder);
  }
  else {
    plan_buffer_line(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS], destination[E_AXIS], feedrate*feedmultiply/60/100.0, active_extruder);
  }
  for(int8_t i=0; i < NUM_AXIS; i++) {
    current_position[i] = destination[i];
  }
}

void prepare_arc_move(char isclockwise) {
  float r = hypot(offset[X_AXIS], offset[Y_AXIS]); // Compute arc radius for mc_arc

  // Trace the arc
  mc_arc(current_position, destination, offset, X_AXIS, Y_AXIS, Z_AXIS, feedrate*feedmultiply/60/100.0, r, isclockwise, active_extruder);

  // As far as the parser is concerned, the position is now == target. In reality the
  // motion control system might still be processing the action and the real tool position
  // in any intermediate location.
  for(int8_t i=0; i < NUM_AXIS; i++) {
    current_position[i] = destination[i];
  }
  previous_millis_cmd = millis();
}

  #if defined(CONTROLLERFAN_PIN) && CONTROLLERFAN_PIN > -1

    #if defined(FAN_PIN)
      #if CONTROLLERFAN_PIN == FAN_PIN
        #error "You cannot set CONTROLLERFAN_PIN equal to FAN_PIN"
      #endif
    #endif

unsigned long lastMotor = 0; //Save the time for when a motor was turned on last
unsigned long lastMotorCheck = 0;

void controllerFan()
{
  if ((millis() - lastMotorCheck) >= 2500) //Not a time critical function, so we only check every 2500ms
  {
    lastMotorCheck = millis();

    if(!READ(X_ENABLE_PIN) || !READ(Y_ENABLE_PIN) || !READ(Z_ENABLE_PIN)
    #if EXTRUDERS > 2
       || !READ(E2_ENABLE_PIN)
    #endif
    #if EXTRUDER > 1
       || !READ(E1_ENABLE_PIN)
    #endif
       || !READ(E0_ENABLE_PIN)) //If any of the drivers are enabled...
    {
      lastMotor = millis(); //... set time to NOW so the fan will turn on
    }

    if ((millis() - lastMotor) >= (CONTROLLERFAN_SECS*1000UL) || lastMotor == 0) //If the last time any driver was enabled, is longer since than CONTROLLERSEC...
    {
      digitalWrite(CONTROLLERFAN_PIN, 0);
      analogWrite(CONTROLLERFAN_PIN, 0);
    }
    else
    {
      // allows digital or PWM fan output to be used (see M42 handling)
      digitalWrite(CONTROLLERFAN_PIN, CONTROLLERFAN_SPEED);
      analogWrite(CONTROLLERFAN_PIN, CONTROLLERFAN_SPEED);
    }
  }
}
  #endif

void manage_inactivity()
{

  if(max_inactive_time)
    if( (millis() - previous_millis_cmd) >  max_inactive_time )
      kill();


  #if !(DISABLE_X==false && DISABLE_Y==false && DISABLE_Z==false && DISABLE_E==false)
  if(stepper_inactive_time)  {
    if( (millis() - previous_millis_cmd) >  stepper_inactive_time )
    {
      if(blocks_queued() == false) {

        if(DISABLE_X) disable_x();
        if(DISABLE_Y) disable_y();
        if(DISABLE_Z) disable_z();
        if(DISABLE_E) {
          disable_e0();
          disable_e1();
          disable_e2();
        }
      }
    }
  }
  #endif
  #if defined(KILL_PIN) && KILL_PIN > -1
  if( 0 == READ(KILL_PIN) )
    kill();
  #endif
  #if defined(SAFETY_TRIGGERED_PIN) && SAFETY_TRIGGERED_PIN > -1
  if (READ(SAFETY_TRIGGERED_PIN))
    Stop(STOP_REASON_SAFETY_TRIGGER);
  #endif
  #if defined(CONTROLLERFAN_PIN) && CONTROLLERFAN_PIN > -1
  controllerFan(); //Check if fan should be turned on to cool stepper drivers down
  #endif
  #ifdef EXTRUDER_RUNOUT_PREVENT
  if( (millis() - previous_millis_cmd) >  EXTRUDER_RUNOUT_SECONDS*1000 )
    if(degHotend(active_extruder)>EXTRUDER_RUNOUT_MINTEMP)
    {
      bool oldstatus=READ(E0_ENABLE_PIN);
      enable_e0();
      float oldepos=current_position[E_AXIS];
      float oldedes=destination[E_AXIS];
      plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS],
                       current_position[E_AXIS]+EXTRUDER_RUNOUT_EXTRUDE*EXTRUDER_RUNOUT_ESTEPS/axis_steps_per_unit[E_AXIS],
                       EXTRUDER_RUNOUT_SPEED/60.*EXTRUDER_RUNOUT_ESTEPS/axis_steps_per_unit[E_AXIS], active_extruder);
      current_position[E_AXIS]=oldepos;
      destination[E_AXIS]=oldedes;
      plan_set_e_position(oldepos);
      previous_millis_cmd=millis();
      st_synchronize();
      WRITE(E0_ENABLE_PIN,oldstatus);
    }
  #endif
  check_axes_activity();
}

void kill()
{
  cli(); // Stop interrupts
  disable_heater();

  disable_x();
  disable_y();
  disable_z();
  disable_e0();
  disable_e1();
  disable_e2();

  #if defined(PS_ON_PIN) && PS_ON_PIN > -1
  pinMode(PS_ON_PIN,INPUT);
  #endif
  SERIAL_ERROR_START;
  SERIAL_ERRORLNPGM(MSG_ERR_KILLED);
  LCD_ALERTMESSAGEPGM(MSG_KILLED);
  suicide();
  while(1) { /* Intentionally left empty */ } // Wait for reset
}

void Stop(uint8_t reasonNr)
{
  disable_heater();
  if(Stopped == false) {
    Stopped = reasonNr;
    Stopped_gcode_LastN = gcode_LastN; // Save last g_code for restart
    SERIAL_ERROR_START;
    SERIAL_ERRORLNPGM(MSG_ERR_STOPPED);
    LCD_MESSAGEPGM(MSG_STOPPED);
  }
}

bool IsStopped() {
  return Stopped;
};
uint8_t StoppedReason() {
  return Stopped;
};

  #ifdef FAST_PWM_FAN
void setPwmFrequency(uint8_t pin, int val)
{
  val &= 0x07;
  switch(digitalPinToTimer(pin))
  {

    #if defined(TCCR0A)
  case TIMER0A:
  case TIMER0B:
    //         TCCR0B &= ~(_BV(CS00) | _BV(CS01) | _BV(CS02));
    //         TCCR0B |= val;
    break;
    #endif

    #if defined(TCCR1A)
  case TIMER1A:
  case TIMER1B:
    //         TCCR1B &= ~(_BV(CS10) | _BV(CS11) | _BV(CS12));
    //         TCCR1B |= val;
    break;
    #endif

    #if defined(TCCR2)
  case TIMER2:
  case TIMER2:
    TCCR2 &= ~(_BV(CS10) | _BV(CS11) | _BV(CS12));
    TCCR2 |= val;
    break;
    #endif

    #if defined(TCCR2A)
  case TIMER2A:
  case TIMER2B:
    TCCR2B &= ~(_BV(CS20) | _BV(CS21) | _BV(CS22));
    TCCR2B |= val;
    break;
    #endif

    #if defined(TCCR3A)
  case TIMER3A:
  case TIMER3B:
  case TIMER3C:
    TCCR3B &= ~(_BV(CS30) | _BV(CS31) | _BV(CS32));
    TCCR3B |= val;
    break;
    #endif

    #if defined(TCCR4A)
  case TIMER4A:
  case TIMER4B:
  case TIMER4C:
    TCCR4B &= ~(_BV(CS40) | _BV(CS41) | _BV(CS42));
    TCCR4B |= val;
    break;
    #endif

    #if defined(TCCR5A)
  case TIMER5A:
  case TIMER5B:
  case TIMER5C:
    TCCR5B &= ~(_BV(CS50) | _BV(CS51) | _BV(CS52));
    TCCR5B |= val;
    break;
    #endif

  }
}
  #endif //FAST_PWM_FAN

bool setTargetedHotend(int code){
  tmp_extruder = active_extruder;
  if(code_seen('T')) {
    tmp_extruder = code_value();
    if(tmp_extruder >= EXTRUDERS) {
      SERIAL_ECHO_START;
      switch(code) {
      case 104:
        SERIAL_ECHO(MSG_M104_INVALID_EXTRUDER);
        break;
      case 105:
        SERIAL_ECHO(MSG_M105_INVALID_EXTRUDER);
        break;
      case 109:
        SERIAL_ECHO(MSG_M109_INVALID_EXTRUDER);
        break;
      case 218:
        SERIAL_ECHO(MSG_M218_INVALID_EXTRUDER);
        break;
      }
      SERIAL_ECHOLN(tmp_extruder);
      return true;
    }
  }
  return false;
}

