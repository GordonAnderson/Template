// =============================================================================
//  GAACE APPLICATION TEMPLATE
//  Gordon Anderson Arduino Core Extensions (GAACE) — v1.0.0
// =============================================================================
//
//  PURPOSE
//  -------
//  This file is a starting-point template for building a GAACE-based firmware
//  module that communicates with the MIPS host application over both a USB
//  serial interface and an I2C (TWI) bus.
//
//  HOW TO USE THIS TEMPLATE  (PlatformIO project)
//  -----------------------------------------------
//  1.  Copy both src/template.cpp and include/template.h, then rename both to
//      match your module (e.g. src/MyModule.cpp and include/MyModule.h).
//      Update the #include in Section 1 (below the feature flags) to reference
//      the renamed header.
//  2.  Work through every TODO comment in order — they guide you through each
//      customisation step.
//  3.  Add your own commands to the cmds[] table (see Section 4).
//  4.  Implement your hardware initialisation and periodic logic (see setup()
//      and Update()).
//  5.  Add fields to the Data struct in include/template.h for any parameters
//      you want to persist.
//
//  FEATURE FLAGS  (set to true/false before anything else)
//  -------------------------------------------------------
//  These #defines gate entire blocks of code. Only enable what your hardware
//  actually needs — unused sections compile away to nothing.
//
//    AppName        — String literal identifying your module in GVER replies
//                     and flash storage.
//    UseWireClient  — Enable when this board communicates with MIPS as an I2C
//                     client (slave).  Requires Wire.h and WireHelper.
//    UseThreads     — Enable a cooperative thread scheduler (ArduinoThread).
//                     The Update() function runs on a 25 ms periodic thread.
//    UseSPIflash    — Enable QSPI/SPI flash filesystem (FatFs via
//                     Adafruit_SPIFlash) for storing defaults and calibration.
//
// TODO 1 ── Set AppName and enable/disable the three feature flags below.
//           Only enable features your hardware actually supports.

#define AppName        "Template"   // e.g. "MyHVModule" — appears in GVER
#define UseWireClient  true         // true  = I2C client to MIPS host
#define UseThreads     true         // true  = use 25 ms periodic Update() thread
#define UseSPIflash    true         // true  = SPI/QSPI flash filesystem

// =============================================================================
//  SECTION 1 — DATA STRUCTURES AND CONSTANTS  (see include/template.h)
// =============================================================================
//
//  In PlatformIO the Data struct, SIGNATURE, TWI command bytes, ESC/ENQ
//  control characters, and forward declarations all live in include/template.h.
//  This include must appear after the feature-flag #defines above so that the
//  #if UseWireClient guards inside the header are evaluated correctly.
#include "template.h"

// =============================================================================
//  SECTION 2 — INCLUDES
// =============================================================================
//
//  Core Arduino and platform headers
#include <Arduino.h>
#include <variant.h>          // Board-specific pin and peripheral definitions
#include <wiring_private.h>   // Low-level wiring helpers (pinPeripheral, etc.)

//  GAACE library headers — all of these live in the GAACE/src folder.
//
//  ringBuffer      — Circular character buffer; underpins commandProcessor input.
//                    Key methods: put(c), get(), peek(), getLine(), getToken(),
//                    lines(), count(), clearLine(), tokensInLine().
//
// CHANGE: isToken() / getToken() note added — see RingBuffer.h for the caveat
//         about using isToken() to gate getToken(). Prefer checking lines() > 0
//         directly before calling getToken().
#include <RingBuffer.h>

//  commandProcessor — Serial command dispatcher.  Parses comma-delimited ASCII
//                    commands from one or more Stream objects and routes them to
//                    typed handlers or function callbacks.
//                    Protocol: "COMMAND,arg1,arg2\n"  →  ACK (0x06) or NAK (0x15)
//                    Key methods: registerStream(), registerCommands(),
//                    processStreams(), processCommands(), sendACK(), sendNAK(),
//                    getValue(), userInput(), print()/println().
//
// CHANGE: DoNotprocessStream() renamed to setStreamActive() in the refactored
//         commandProcessor.  See Section 8 (setup) where the call is updated.
#include <commandProcessor.h>

//  charAllocate    — Lightweight arena allocator for short-lived char arrays.
//                    Avoids heap fragmentation on constrained devices.
//                    Key methods: allocate(size), free(ptr), clear(), defrag(),
//                    available().  commandProcessor calls clear() after each command.
#include <charAllocate.h>

//  debug           — Plug-in command group that adds 19 diagnostic serial commands:
//                    Memory: SETADDRESS, SETOFFSET, WRITE, READ, DUMP, RAM
//                    GPIO:   PINMODE, DOUT, DIN
//                    Analog: ADCRES, DACRES, ADC, DAC
//                    System: DEBUG, UPTIME, RESET, UUID, CPUTEMP
//                    Register with: cp.registerCommands(dbg.debugCommands());
#include <debug.h>

//  Errors.h        — Typed ErrorCode enum (ERR_BADCMD=1, ERR_BADARG=2,
//                    ERR_VALUERANGE=101, ERR_TIMEOUT=128, etc.) for use with
//                    cp.sendNAK(ERR_xxx).
//
// CHANGE: Error codes are now a typed enum (ErrorCode) rather than plain
//         #defines.  ERR_TIMEOUT is now 128 (was 127) because ERR_EEPROMWRITE
//         was renumbered from 110 to 111 to resolve a duplicate-value bug,
//         shifting all subsequent codes up by one.  ERR_NAMENOTFOUND now
//         correctly means "name not found" (was a copy-paste of ERR_NAMEINLIST
//         description).  Two misspelled names have compatibility aliases:
//           ERR_ADCNOTAVALIABLE  →  ERR_ADCNOTAVAILABLE
//           ERR_ADCALREARYSETUP  →  ERR_ADCALREADYSETUP
#include <Errors.h>

//  Devices.h       — ADC/DAC calibration helpers and low-level IC drivers.
//                    Calibration: ADCchan{Chan,m,b}, DACchan{Chan,m,b}
//                    Helpers:     Counts2Value(), Value2Counts(), AnalogIn(),
//                                 AnalogOut(), Filter() (IIR low-pass)
//                    IC drivers:  AD5592R (SPI 8-ch), AD5593R (I2C 8-ch),
//                                 DAC8571 (I2C 16-bit), MCP4725 (I2C 12-bit)
//
// CHANGE: AD5592readWord() now returns uint16_t (was int).
//         SendWord() now takes int16_t (was int) — callers passing full int
//         values on 32-bit platforms were silently truncating the upper 2 bytes.
//         SendInt24() now takes int32_t (was int) for the same reason.
//         Value2Counts() now uses int32_t internally to avoid overflow on AVR.
#include <Devices.h>

//  FlashStorage    — Stores a single struct in internal MCU flash (like EEPROM
//                    but for SAMD/SAMD51 which have no true EEPROM).
//                    Usage: FlashStorage(flash_data, Data);
//                           flash_data.write(data);   // save
//                           data = flash_data.read(); // restore
#include <FlashStorage.h>
#include <FlashAsEEPROM.h>

// NOTE: In PlatformIO the project header lives in include/template.h and is
//       already included above after the feature-flag #defines (Section 1).
//       When renaming this module, update that #include to reference the new
//       header filename in include/ — no additional include is needed here.

#if UseThreads
// ArduinoThread — cooperative scheduler. Each Thread calls its onRun() callback
// at the configured interval.  control.run() must be called every loop().
// Interval is set in milliseconds via setInterval().
#include <Thread.h>
#include <ThreadController.h>
ThreadController control = ThreadController();  // Root scheduler
Thread SystemThread = Thread();                 // 25 ms periodic update thread
#endif  // UseThreads


#if UseSPIflash
// Adafruit_SPIFlash + FatFs — FAT filesystem on on-board QSPI/SPI flash.
// flash_config.h auto-selects the correct FlashTransport backend for the
// target board (ESP32, RP2040, QSPI, SPI, or AVR fallback).
//
// FatFs API subset enabled in ffconf.h (FF_FS_MINIMIZE=3):
//   f_open, f_close, f_read, f_write, f_sync, f_mkfs, f_getlabel, f_setlabel
//
// File open flags from SdFat (used by FatVolume/File32):
//   O_READ   — open for reading
//   O_WRITE  — open for writing
//   O_CREAT  — create if not present
//   O_TRUNC  — truncate on open
#include <Adafruit_SPIFlash.h>
#include <FlashFS/flash_config.h>   // defines flashTransport for this board
Adafruit_SPIFlash flash(&flashTransport);
FatVolume fatfs;
File32    file;
#endif  // UseSPIflash


// =============================================================================
//  SECTION 3 — GLOBAL STATE
// =============================================================================

// Active runtime data.  Loaded from flash on boot; written back by SaveSettings.
Data data;

// Default (factory) values written to flash the very first time the firmware
// runs (or whenever the flash signature is invalid).
//
// TODO 4 ── Set sensible defaults for every field you added to the Data struct.
//           Rev should match your hardware revision number.
//           TWIadd should be the I2C address MIPS expects for this module.
Data Rev_1_data =
{
  sizeof(Data),   // Size  — always first so future code can detect struct growth
  AppName,        // Name
  1,              // Rev   — increment when you change hardware
  #if UseWireClient
  0x00,           // TWIadd — TODO: set to the correct I2C address for this module
  #endif
  // Add default values here for any extra fields you added in TODO 2.
  // Example:  12.5,             // hvSetpoint default = 12.5 V
  SIGNATURE       // Signature — always last
};

// FlashStorage macro: reserves one flash page for persistent storage of Data.
// NOTE: this storage is erased every time the sketch is uploaded.  Use the
// SPI flash filesystem (saveDefaults/loadDefaults) for data that should
// survive firmware updates.
FlashStorage(flash_data, Data);

// Version string returned by the GVER command.
// TODO 5 ── Update the date when you release a new firmware build.
const char *Version = AppName ", version 1.0 Sept 9, 2025";


// =============================================================================
//  SECTION 4 — COMMAND PROCESSOR SETUP
// =============================================================================
//
//  commandProcessor — global instance.  All command registration, stream
//  management and output printing goes through this object.
//
//  debug — attaches the 19-command debug group.  Always register its
//  CommandList (dbg.debugCommands()) so hardware-level diagnostics are
//  available during development.
commandProcessor cp;
debug dbg(&cp);

// ── Command table ─────────────────────────────────────────────────────────────
//
//  Each entry is:  { "CMD", CmdType, nargs, pointer, options, "help string" }
//
//  CmdType values:
//    CMDstr      — pointer is char*;  G reads the string, S writes it.
//    CMDint      — pointer is int*;   G reads, S writes; options = int[2]{min,max}
//    CMDfloat    — pointer is float*; G reads, S writes; options = float[2]{min,max}
//    CMDbool     — pointer is bool*;  G reads (TRUE/FALSE), S writes.
//    CMDbyte     — pointer is uint8_t*.
//    CMDfunction — pointer is void(*)(void); called on receive; uses getValue().
//    CMDdouble   — pointer is double*.
//
//  Command name prefix conventions:
//    "?XXX"  — bidirectional: GXXX reads, SXXX writes (one table entry)
//    "GXXX"  — explicit get-only function command
//    "SXXX"  — explicit set-only function command
//
//  nargs:
//    -1  — don't check argument count (useful for functions that validate
//           internally via getNumArgs() or checkExpectedArgs())
//     0  — command takes no arguments
//     N  — command takes exactly N arguments
//
//  options:
//    NULL          — no constraint
//    int[2]{lo,hi} — range check for CMDint
//    float[2]      — range check for CMDfloat
//    "A,B,C"       — enum check for CMDstr (strstr match)
//
//  ACK / NAK protocol:
//    Every command handler MUST call either cp.sendACK() or cp.sendNAK().
//    For responses that include data, call cp.sendACK(false) (no newline)
//    then immediately print the value, then cp.print() for the newline.
//    Example:
//        cp.sendACK(false);
//        cp.println(data.hvSetpoint);
//
// TODO 6 ── Add your module's commands here.  Follow the patterns shown.
//           Remove any template entries that your module does not need.
//           Keep help strings concise — they are returned verbatim by GCMDS.

Command cmds[] =
{
  // ── Identification & persistence ──────────────────────────────────────────
  // GVER — returns the firmware version string (read-only, CMDstr with 0 args).
  {"GVER",    CMDstr,      -1, (void *)Version,        NULL, "Firmware version"},

  // ?NAME — GNAME returns the device name; SNAME,<str> sets it.
  {"?NAME",   CMDstr,      -1, (void *)&data.Name,     NULL, "Device name"},

  // SAVE / RESTORE — write or reload the Data struct from internal flash.
  {"SAVE",    CMDfunction,  0, (void *)SaveSettings,   NULL, "Save system settings to internal flash"},
  {"RESTORE", CMDfunction,  0, (void *)RestoreSettings,NULL, "Restore system settings from internal flash"},

  // BLOAD — jump to the SAM-BA / UF2 bootloader for firmware updates.
  {"BLOAD",   CMDfunction,  0, (void *)bootloader,     NULL, "Jump to bootloader for firmware update"},

  // ── I2C address management (only compiled when UseWireClient is true) ─────
  #if UseWireClient
  // STWIADD,<hex> — set the I2C address this board listens on (stored in data).
  // GTWIADD       — returns the current I2C address in hex.
  //
  // NOTE: changing the I2C address takes effect only after the next reboot.
  //       Call SAVE after STWIADD to make it permanent.
  {"STWIADD", CMDfunction,  1, (void *)setTWIaddress,  NULL, "Set TWI address (hex); reboot to apply"},
  {"GTWIADD", CMDfunction,  0, (void *)getTWIaddress,  NULL, "Return TWI address (hex)"},
  #endif

  // ── SPI flash filesystem commands (only compiled when UseSPIflash is true) ─
  #if UseSPIflash
  // SAVEF  / LOADF   — save or load the full Data struct to default.dat on flash.
  // SAVECAL / LOADCAL — save or load calibration data only to cal.dat on flash.
  //
  // Use SAVEF/LOADF for settings that should survive firmware uploads.
  // Use SAVECAL/LOADCAL to persist ADCchan/DACchan calibration structs
  // without overwriting the rest of the Data block.
  {"SAVEF",   CMDfunction,  0, (void *)saveDefaults,    NULL, "Save settings to SPI flash (default.dat)"},
  {"LOADF",   CMDfunction,  0, (void *)loadDefaults,    NULL, "Load settings from SPI flash (default.dat)"},
  {"SAVECAL", CMDfunction,  0, (void *)saveCalibrations,NULL, "Save calibration data to SPI flash (cal.dat)"},
  {"LOADCAL", CMDfunction,  0, (void *)loadCalibrations,NULL, "Load calibration data from SPI flash (cal.dat)"},
  #endif

  // TODO 7 ── Add your module-specific commands here.
  //
  //  Pattern A — get/set a variable with range check:
  //    static float hvLimits[] = {0.0, 500.0};
  //    {"?HV", CMDfloat, -1, (void *)&data.hvSetpoint, hvLimits, "HV setpoint (0-500 V)"},
  //
  //  Pattern B — function command with manual argument parsing:
  //    {"SETGAIN", CMDfunction, 2, (void *)setGain, NULL, "Set gain: channel(0-7), value(float)"},
  //
  //  Pattern C — read-only sensor value:
  //    {"GTEMP", CMDfunction, 0, (void *)getTemperature, NULL, "Read board temperature in C"},

  {NULL}  // Sentinel — must remain as the last entry
};
static CommandList cmdList = {cmds, NULL};


// =============================================================================
//  SECTION 5 — DEBUG CALLBACK
// =============================================================================
//
//  The DEBUG serial command (registered by debug::debugCommands()) calls this
//  function.  Use it for ad-hoc hardware tests during development — toggle a
//  pin, read a raw ADC channel, dump a register, etc.  Leave empty in production.
//
// TODO 8 ── Add temporary debug code here during bring-up.
//           Remove or leave empty before shipping.
void Debug(void)
{
  // Example: cp.println(analogRead(A0));
  // Example: digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
}


// =============================================================================
//  SECTION 6 — PERIODIC UPDATE THREAD  (only compiled when UseThreads is true)
// =============================================================================
//
//  Update() is called every 25 ms by the cooperative scheduler.  Use it for:
//    - Reading ADC channels and applying the IIR filter (see Filter() in Devices.h)
//    - Driving DAC outputs to track setpoints
//    - Updating status LEDs
//    - Implementing soft timers (compare millis() against a saved timestamp)
//
//  IMPORTANT: Update() must return quickly.  Do not call delay() or any
//  blocking function inside it.  The commandProcessor runs in loop() between
//  thread ticks, so long Update() bodies stall command processing.
//
//  Devices.h helpers you can use here:
//    float v = AnalogIn(myReadADC, &data.monChan);  // averaged, calibrated read
//    v = Filter(lastV, v, 0.05);                    // 5 % IIR weight
//    AnalogOut(myWriteDAC, &data.dacChan, setpoint); // calibrated write
//
//  NOTE: Filter() uses -1.0f as a "not yet initialised" sentinel.  If your
//  signal can legitimately reach -1.0, manage a separate bool initialised flag
//  and seed lastV on the first real reading rather than relying on the sentinel.
//
// TODO 9 ── Implement your periodic hardware control logic here.
#if UseThreads
void Update(void)
{
  // Example: read a calibrated ADC channel
  // monVoltage = Filter(monVoltage, AnalogIn(readMyADC, &data.monChan), 0.1);

  // Example: drive a DAC output to match a setpoint
  // AnalogOut(writeMyDAC, &data.dacChan, data.hvSetpoint);
}
#endif  // UseThreads


// =============================================================================
//  SECTION 7 — I2C (TWI) CLIENT  (only compiled when UseWireClient is true)
// =============================================================================
//
//  MIPS communicates with this board as an I2C master.  This board is the
//  client (slave).  Two ISR callbacks are registered in setup():
//
//    Wire.onReceive(receiveEvent) — called when MIPS sends bytes to this board.
//    Wire.onRequest(requestEvent) — called when MIPS reads bytes from this board.
//
//  WireHelper (wh) buffers the response in wh.sb (a SerialBuffer) so that
//  requestEvent just calls wh.requestEventProcessor() — no manual byte
//  assembly needed.
//
//  WireHelper typed readers (use inside receiveEvent after reading cmd):
//    wh.ReadUnsignedWord()        → int      (16-bit, LE), -1 if unavailable
//    wh.ReadInt(int32_t *i)       → bool     (32-bit, LE)  [param is int32_t*]
//    wh.ReadUnsignedByte()        → int      (8-bit), -1 if unavailable
//    wh.ReadByte(int8_t *b)       → bool
//    wh.ReadBool(bool *b)         → bool
//    wh.Read16bitInt(int16_t *s)  → bool
//    wh.ReadFloat(float *f)       → bool     (32-bit IEEE 754, LE)
//
//  WireHelper typed senders (queue data for requestEvent):
//    wh.SendBool(bool)
//    wh.SendByte(uint8_t)         [was: byte — updated to uint8_t]
//    wh.SendWord(int16_t)         [was: int  — CHANGED to int16_t; see note below]
//    wh.SendInt24(int32_t)        [was: int  — CHANGED to int32_t]
//    wh.SendFloat(float)          — 32-bit IEEE 754 LE
//
//  IMPORTANT SendWord / SendInt24 type change:
//    The original WireHelper took plain `int` for SendWord and SendInt24.
//    On 32-bit platforms (SAMD, Teensy) `int` is 32 bits, so the upper
//    2 bytes of SendWord were silently dropped.  The refactored signatures
//    are int16_t and int32_t respectively — update any call sites that pass
//    a local `int` variable: cast it explicitly, e.g. wh.SendWord((int16_t)v).
//
//  The requestEventProcessor() sends up to WIRE_MAX_SEND_BYTES (30) queued
//  bytes per I2C read transaction.  Call wh.setReturnAvailable() before the
//  read to make the next requestEvent return the 2-byte count of bytes
//  waiting in wh.sb instead of payload data.
//
//  CHANGE: wh.ReturnAvalible (public bool) replaced by wh.setReturnAvailable()
//          method.  The old spelling TWI_READ_AVALIBLE still works via the
//          compatibility alias defined in Section 1.

#if UseWireClient
#include <WireHelper.h>

WireHelper wh;

// requestEvent — ISR called by the Wire library when MIPS issues an I2C read.
// Do NOT add application logic here.  Just forward to WireHelper.
void requestEvent(void)
{
  wh.requestEventProcessor();
}

// receiveEvent — ISR called when MIPS sends data to this board.
// The first byte is always a command byte (TWI_SERIAL, TWI_CMD, etc.) that
// selects the operation.  Subsequent bytes are the payload.
//
// TWI_SERIAL mode lets MIPS stream multiple ASCII commands without the
// per-command TWI_CMD framing overhead.  Bytes accumulate in the
// commandProcessor ring buffer until an ESC byte closes the session.
//
// TWI_CMD mode is the normal single-command path: MIPS writes one ASCII
// command line (e.g. "GVER\n"), this ISR pushes it into the ring buffer,
// processCommands() executes it, and the response waits in wh.sb.
//
// TODO 10 ── Add cases to the switch statement for any custom I2C command bytes
//            your MIPS module expects beyond the three standard ones.
//            Use wh.ReadFloat(), wh.ReadInt(), etc. to deserialise payloads.
//            Use wh.SendFloat(), wh.SendWord(), etc. to queue responses.
void receiveEvent(int howMany)
{
  uint8_t cmd;
  int     i;
  static  bool TWITALKmode = false;

  while (Wire.available() != 0)
  {
    cmd = Wire.read();

    if (TWITALKmode)
    {
      // Serial-tunnel mode: forward every byte to the command processor until ESC.
      if (cmd == ESC)
      {
        TWITALKmode = false;          // Close the tunnel session
      }
      else
      {
        cp.selectStream(&wh.sb);      // Responses go back via I2C, not USB serial
        cp.rb->put(cmd);              // Feed byte into the command ring buffer
      }
    }
    else switch (cmd)
    {
      case TWI_SERIAL:
        // Open serial-tunnel mode — subsequent bytes are ASCII command data.
        cp.selectStream(&wh.sb);
        TWITALKmode = true;
        break;

      case TWI_CMD:
        // Single-shot command: read the ASCII string, execute it, buffer response.
        wh.sb.clear();                // Discard any leftover response data
        for (i = 0; i < 100; i++)    // Read up to 100 bytes (or until '\n')
        {
          if (Wire.available() != 0)
          {
            cmd = Wire.read();
            cp.rb->put(cmd);          // Push into ring buffer
            if (cmd == '\n') break;   // '\n' terminates the command
          }
          delayMicroseconds(100);     // Give MIPS time to clock out the next byte
        }
        cp.selectStream(&wh.sb);      // Route response to I2C buffer
        cp.processCommands();         // Execute the command now (synchronous)
        break;

      case TWI_READ_AVAILABLE:
        // On next I2C read, return the number of bytes waiting in wh.sb.
        // CHANGE: was `wh.ReturnAvalible = true` (direct public member access).
        // Updated to use the new accessor method, which also corrects the spelling.
        wh.setReturnAvailable();
        break;

      default:
        // TODO 10 (continued) — Handle custom command bytes here.
        // Example:
        //   case MY_CMD_SET_VOLTAGE:
        //   {
        //     float v;
        //     if (wh.ReadFloat(&v)) data.hvSetpoint = v;
        //     break;
        //   }
        break;
    }
  }
}
#endif  // UseWireClient


// =============================================================================
//  SECTION 8 — SETUP
// =============================================================================
//
//  Initialisation order matters:
//    1. Load (or default) persistent data from flash.
//    2. Start USB serial and register it with commandProcessor.
//    3. Register command tables (application commands first, then debug).
//    4. Configure I2C client if UseWireClient is true.
//    5. Start the thread scheduler if UseThreads is true.
//    6. Initialise your hardware peripherals (SPI, DAC, ADC, etc.).
//
// TODO 11 ── Add hardware peripheral initialisation after the GAACE setup block.
//            For example:
//              SPI.begin();
//              pinMode(MY_CS_PIN, OUTPUT);
//              AD5592write(MY_CS_PIN, 0x0B, 0x0000); // power-up all channels
void setup()
{
  // ── 1. Restore persistent data ──────────────────────────────────────────────
  //  Read from internal flash; fall back to factory defaults if the signature
  //  is invalid (first boot, or flash was erased by an upload).
  data = flash_data.read();
  if (data.Signature != SIGNATURE) data = Rev_1_data;

  // ── 2. USB serial ───────────────────────────────────────────────────────────
  //  Serial.begin(0) on SAMD targets means "native USB at full speed" — the
  //  baud rate argument is ignored for USB CDC.  For UART-based boards, pass
  //  the actual baud rate (e.g. 115200).
  Serial.begin(0);
  cp.registerStream(&Serial);   // Register USB serial as a command source/sink

  // ── 3. Command registration ─────────────────────────────────────────────────
  //  Register your application commands first so they appear first in GCMDS.
  cp.registerCommands(&cmdList);
  cp.registerCommands(dbg.debugCommands()); // Always register debug commands
  dbg.registerDebugFunction(Debug);         // Wire up the DEBUG callback

  // ── 4. I2C client ───────────────────────────────────────────────────────────
  #if UseWireClient
  //  Join the I2C bus as a client at the address stored in data.TWIadd.
  //  Wire.begin() must be called before registering the callbacks.
  Wire.begin(data.TWIadd);
  Wire.onReceive(receiveEvent);
  Wire.onRequest(requestEvent);

  //  Register wh.sb as a second commandProcessor stream so that commands
  //  arriving over I2C can be processed and their responses sent back over I2C.
  //  setStreamActive(false) prevents loop() from polling wh.sb — it is driven
  //  only by the TWI_CMD handler inside receiveEvent().
  //
  // CHANGE: DoNotprocessStream(&wh.sb, true) replaced with setStreamActive(&wh.sb, false).
  //         The new method name is consistent in casing and the bool sense is
  //         inverted for clarity: pass false to stop processing, true to resume.
  cp.registerStream(&wh.sb);
  cp.setStreamActive(&wh.sb, false);
  #endif  // UseWireClient

  // ── 5. Thread scheduler ─────────────────────────────────────────────────────
  #if UseThreads
  SystemThread.setName((char *)"Update");
  SystemThread.onRun(Update);
  SystemThread.setInterval(25);   // ms between Update() calls
  control.add(&SystemThread);
  #endif  // UseThreads

  // ── 6. Hardware peripheral initialisation ───────────────────────────────────
  // TODO 11 — Add your hardware setup code here.
}


// =============================================================================
//  SECTION 9 — MAIN LOOP
// =============================================================================
//
//  The loop body is intentionally minimal.  All work is dispatched by the
//  three calls below:
//
//    processStreams()  — reads available bytes from all registered streams
//                        (USB serial, and optionally wh.sb) into the ring buffer.
//    processCommands() — parses one complete command from the ring buffer and
//                        calls the appropriate handler.  Returns true if a
//                        command was processed, false if the buffer had no
//                        complete command.  Called once per loop iteration so
//                        that long commands do not block the thread scheduler.
//    control.run()     — checks each registered thread and calls its onRun()
//                        callback if its interval has elapsed.
//
//  Do NOT add blocking code (delay, while-loops waiting for hardware) here.
//  Use the Update() thread or soft-timer patterns (millis() deltas) instead.
void loop()
{
  cp.processStreams();    // Drain serial/I2C bytes into ring buffer
  cp.processCommands();  // Dispatch one command if a complete line is ready
  #if UseThreads
  control.run();         // Run due thread callbacks (e.g. Update() every 25 ms)
  #endif
}


// =============================================================================
//  SECTION 10 — SYSTEM COMMAND IMPLEMENTATIONS
// =============================================================================

// SaveSettings — write the current Data struct to internal flash.
// The Signature is set here (not in setup) so that a partial write caused by
// power loss during setup does not create an apparently-valid but incomplete record.
void SaveSettings(void)
{
  data.Signature = SIGNATURE;
  flash_data.write(data);
  cp.sendACK();
}

// RestoreSettings — reload Data from internal flash without rebooting.
// Validates the signature before overwriting the live data struct.
// Sends NAK if the flash contents are invalid (never been saved).
void RestoreSettings(void)
{
  static Data td;
  td = flash_data.read();
  if (td.Signature == SIGNATURE)
  {
    data = td;
    cp.sendACK();
  }
  else
  {
    // CHANGE: ERR_NOSDCARD re-used here as "no valid flash data" — this is a
    // deliberate re-purposing noted in the original code.  Consider adding a
    // dedicated ERR_INVALIDFLASH code to Errors.h for clarity.
    cp.sendNAK(ERR_NOSDCARD);
  }
}


// =============================================================================
//  SECTION 11 — SPI FLASH FILESYSTEM  (only compiled when UseSPIflash is true)
// =============================================================================
//
//  Provides persistent file storage on on-board QSPI/SPI flash using FatFs.
//  Unlike FlashStorage (which is erased on every firmware upload), files on the
//  SPI flash survive firmware updates as long as the partition is not reformatted.
//
//  FSsetup() — mounts the filesystem.  Call before any file operation.
//              Returns true on success, prints diagnostic messages on failure.
//
//  saveDefaults / loadDefaults — write/read the full Data struct to default.dat.
//  saveCalibrations / loadCalibrations — write/read calibration data to cal.dat.
//
//  TODO 12 ── In saveCalibrations / loadCalibrations, replace the commented-out
//             file.write / file.read calls with your actual calibration structs.
//             Example:
//               size_t num = file.write((void *)&data.monChan, sizeof(ADCchan));
//             Add one write/read call per calibration channel.

#if UseSPIflash
// FSsetup — initialise the SPI flash chip and mount the FatFs volume.
// Returns true if the filesystem is ready for file I/O.
bool FSsetup(void)
{
  if (!flash.begin())
  {
    cp.println("Error: failed to initialise flash chip!");
    return false;
  }
  cp.println("Flash chip initialised.");

  if (!fatfs.begin(&flash))
  {
    cp.println("Error: failed to mount filesystem!");
    cp.println("Tip: format the flash once using f_mkfs if this is a new chip.");
    return false;
  }
  cp.println("Filesystem mounted.");
  return true;
}

// saveDefaults — serialise the entire Data struct to default.dat.
void saveDefaults(void)
{
  if (!FSsetup()) { cp.sendNAK(); return; }

  if ((file = fatfs.open("default.dat", O_WRITE | O_CREAT)) == 0)
  {
    cp.println("Error: cannot create default.dat");
    cp.sendNAK(ERR_CANTCREATEFILE);
    return;
  }
  size_t num = file.write((void *)&data, sizeof(Data));
  file.close();
  cp.sendACK(false);
  cp.print("default.dat written, bytes = ");
  cp.println((int)num);
}

// loadDefaults — deserialise Data from default.dat, validate signature.
void loadDefaults(void)
{
  Data h;
  if (!FSsetup()) { cp.sendNAK(); return; }

  if ((file = fatfs.open("default.dat", O_READ)) == 0)
  {
    cp.println("Error: cannot open default.dat");
    cp.sendNAK(ERR_CANTOPENFILE);
    return;
  }
  size_t num = file.read((void *)&h, sizeof(Data));
  file.close();

  if ((num != sizeof(Data)) || (h.Signature != SIGNATURE))
  {
    cp.println("Error: default.dat is corrupt or wrong size.");
    cp.sendNAK(ERR_READINGSD);
    return;
  }
  data = h;
  cp.sendACK(false);
  cp.print("default.dat loaded, bytes = ");
  cp.println((int)num);
}

// saveCalibrations — write calibration structs only to cal.dat.
// TODO 12 ── Replace num=0 with actual file.write() calls for your ADCchan/DACchan fields.
void saveCalibrations(void)
{
  if (!FSsetup()) { cp.sendNAK(); return; }

  if ((file = fatfs.open("cal.dat", O_WRITE | O_CREAT)) == 0)
  {
    cp.println("Error: cannot create cal.dat");
    cp.sendNAK(ERR_CANTCREATEFILE);
    return;
  }
  // TODO 12 — replace the lines below with your calibration writes, e.g.:
  //   size_t num = file.write((void *)&data.monChan, sizeof(ADCchan));
  //   num += file.write((void *)&data.dacChan, sizeof(DACchan));
  size_t num = 0;
  file.close();
  cp.sendACK(false);
  cp.print("cal.dat written, bytes = ");
  cp.println((int)num);
}

// loadCalibrations — read calibration structs from cal.dat.
// TODO 12 ── Replace num=0 with actual file.read() calls matching saveCalibrations().
void loadCalibrations(void)
{
  if (!FSsetup()) { cp.sendNAK(); return; }

  if ((file = fatfs.open("cal.dat", O_READ)) == 0)
  {
    cp.println("Error: cannot open cal.dat");
    cp.sendNAK(ERR_CANTOPENFILE);
    return;
  }
  // TODO 12 — replace the lines below with your calibration reads, e.g.:
  //   size_t num = file.read((void *)&data.monChan, sizeof(ADCchan));
  //   num += file.read((void *)&data.dacChan, sizeof(DACchan));
  size_t num = 0;
  file.close();
  cp.sendACK(false);
  cp.print("cal.dat loaded, bytes = ");
  cp.println((int)num);
}
#endif  // UseSPIflash


// =============================================================================
//  SECTION 12 — BOOTLOADER JUMP
// =============================================================================
//
//  Writes the double-tap magic value to the end of SRAM and triggers a soft
//  reset.  The SAM-BA / UF2 bootloader detects this magic value and stays in
//  DFU mode instead of jumping to the application.
//
//  Supported targets: SAMD21, SAMD51.
//  Other targets: add your platform's bootloader entry mechanism here.
//
// CHANGE: BOOT_DOUBLE_TAP_ADDRESS and DBL_TAP_MAGIC were #defined inside the
//         function body in the original, polluting the macro namespace for the
//         rest of the translation unit.  Replaced with static const locals,
//         consistent with the fix applied in debug.cpp::softwareReset().
void bootloader(void)
{
  cp.sendACK();   // Acknowledge before reset so MIPS knows the command was received

  #if SAMD21_SERIES
  __disable_irq();
  static volatile uint32_t * const bootAddr21  = (volatile uint32_t *)(HMCRAMC0_ADDR + HMCRAMC0_SIZE - 4);
  static const    uint32_t         tapMagic21  = 0xf01669efUL;
  *bootAddr21 = tapMagic21;
  NVIC_SystemReset();
  while (true);
  #endif

  #if SAMD51_SERIES
  __disable_irq();
  static volatile uint32_t * const bootAddr51  = (volatile uint32_t *)(HSRAM_ADDR + HSRAM_SIZE - 4);
  static const    uint32_t         tapMagic51  = 0xf01669efUL;
  *bootAddr51 = tapMagic51;
  NVIC_SystemReset();
  while (true);
  #endif

  // TODO 13 ── Add bootloader entry for other platforms (ESP32, RP2040, Teensy)
  //            if needed.  For platforms without a DFU bootloader, you may
  //            simply trigger a software reset (NVIC_SystemReset / SCB_AIRCR).
}


// =============================================================================
//  SECTION 13 — TWI ADDRESS COMMANDS  (only compiled when UseWireClient is true)
// =============================================================================
//
//  setTWIaddress — parses a hex byte argument and stores it in data.TWIadd.
//                  The change takes effect on the next reboot.  Call SAVE after
//                  STWIADD to persist the new address.
//
//  getTWIaddress — returns the current TWI address in hex.
//
//  getValue(int*, ll, ul, HEX) reads the next command token, validates it
//  within [ll, ul], and stores it.  Passing HEX as the fmt argument parses
//  the token as a hexadecimal integer.

#if UseWireClient
void setTWIaddress(void)
{
  int i;
  // Read one hex argument in the range 0x00–0xFF
  if (cp.getValue(&i, 0, 255, HEX))
  {
    data.TWIadd = i;
    cp.sendACK();
    // Reminder: SAVE must be called separately to persist the new address.
  }
  else
  {
    cp.sendNAK(ERR_BADARG);
  }
}

void getTWIaddress(void)
{
  cp.sendACK(false);              // ACK without newline — data follows immediately
  cp.println(data.TWIadd, HEX);  // Print address as hex, then newline
}
#endif  // UseWireClient


// =============================================================================
//  QUICK REFERENCE — commandProcessor getValue() overloads
// =============================================================================
//
//  Use these inside CMDfunction handlers to extract and validate arguments.
//  All overloads return false and leave *val unchanged if the token is missing
//  or fails validation; call cp.sendNAK() in that case.
//
//  bool getValue(char **val, const char *options = NULL)
//    — Pops next token as a char* (arena-allocated; release with cp.ca->free()).
//      options = "A,B,C" restricts to enumerated values (case-sensitive by default).
//      Example: char *mode; if (!cp.getValue(&mode, "ON,OFF")) { cp.sendNAK(); return; }
//
//  bool getValue(int *val, int ll=0, int ul=0, int fmt=DEC)
//    — Parses as int; validates within [ll, ul] (skipped when both are 0).
//      fmt = HEX parses hexadecimal.
//      Example: int ch; if (!cp.getValue(&ch, 0, 7)) { cp.sendNAK(); return; }
//
//  bool getValue(uint32_t *val, uint32_t ll=0, uint32_t ul=0, int fmt=DEC)
//    — Same as int overload but for unsigned 32-bit values.
//
//  bool getValue(float *val, float ll=0, float ul=0)
//    — Parses as float; validates within [ll, ul] (skipped when both are 0.0).
//      Example: float v; if (!cp.getValue(&v, 0.0, 500.0)) { cp.sendNAK(); return; }
//
//  Checking argument count before calling getValue():
//    if (!cp.checkExpectedArgs(2)) return;   // sends NAK automatically
//    — or —
//    if (cp.getNumArgs() != 2) { cp.sendNAK(); return; }
//
//  Example handler with two typed arguments:
//
//    static void setChannelGain(void)
//    {
//      int ch; float gain;
//      if (!cp.checkExpectedArgs(2)) return;
//      if (cp.getValue(&ch, 0, 7) && cp.getValue(&gain, 0.1, 100.0))
//      {
//        applyGain(ch, gain);
//        cp.sendACK();
//      }
//      else cp.sendNAK(ERR_BADARG);
//    }
//
// =============================================================================
//  QUICK REFERENCE — Devices.h calibration workflow
// =============================================================================
//
//  1. Define a DACchan or ADCchan in your Data struct:
//       ADCchan monChan;    // {Chan, m, b}
//       DACchan dacChan;    // {Chan, m, b}
//
//  2. Set the channel index and calibration constants in Rev_1_data:
//       {0, 3276.7, 0.0},  // monChan: Chan=0, m=counts/unit, b=offset
//
//  3. Read a calibrated ADC value:
//       float volts = AnalogIn(myReadADC, &data.monChan, 4);  // average 4 samples
//       volts = Filter(lastVolts, volts, 0.1);                 // 10 % IIR
//
//  4. Write a calibrated DAC value:
//       AnalogOut(myWriteDAC, &data.dacChan, data.hvSetpoint);
//
//  5. Expose calibration constants over serial (bidirectional float commands):
//       static float mLimits[] = {-1e6, 1e6};
//       {"?ADCM", CMDfloat, -1, (void *)&data.monChan.m, mLimits, "ADC slope (counts/unit)"},
//       {"?ADCB", CMDfloat, -1, (void *)&data.monChan.b, mLimits, "ADC offset (counts)"},
// =============================================================================
