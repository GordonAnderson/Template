// =============================================================================
//  GAACE APPLICATION TEMPLATE — MODULE HEADER
//  Gordon Anderson Arduino Core Extensions (GAACE) — v1.0.0
// =============================================================================
//
//  PlatformIO placement: include/template.h
//  Included by:          src/template.cpp  (after the feature-flag #defines)
//
//  Contains: SIGNATURE constant, ESC/ENQ control characters, TWI command byte
//  definitions, the persistent Data struct, and forward declarations for all
//  functions implemented in src/template.cpp.
// =============================================================================
#pragma once
#include <Arduino.h>

// SIGNATURE is written into flash so we can detect un-initialised storage.
// Change this value if you ever change the layout of the Data struct — doing
// so forces a reset to defaults on the next boot rather than reading garbage.
#define SIGNATURE  0xAA55A5A5

// Control characters used in the TWI serial-tunnel protocol.
#define ESC   27    // Ends a TWI_SERIAL streaming session
#define ENQ    5    // Enquiry — reserved for future handshaking

#if UseWireClient
// ── TWI (I2C) command byte definitions ───────────────────────────────────────
//  These bytes are sent by the MIPS host as the first byte of an I2C write to
//  select the operation this client should perform.
//
//  TWI_SERIAL         — Puts the I2C port into "serial tunnel" mode.  All
//                       subsequent bytes are forwarded to the commandProcessor
//                       ring buffer until an ESC byte is received.  Allows MIPS
//                       to send multi-command bursts without per-command framing.
//
//  TWI_CMD            — Single-shot command tunnel.  MIPS sends an ASCII command
//                       string (terminated with '\n') immediately after this byte.
//                       The commandProcessor executes it and the response is
//                       buffered in wh.sb, ready to be read back by MIPS.
//
//  TWI_READ_AVAILABLE — Asks this client to respond on the next I2C read with a
//                       16-bit unsigned count of bytes waiting in wh.sb.  MIPS
//                       uses this to know how many bytes to clock out.
//
// CHANGE: TWI_READ_AVALIBLE renamed to TWI_READ_AVAILABLE (spelling correction).
//         A backward-compatibility alias is provided below for any existing
//         callers still using the old name.
#define TWI_SERIAL          0x27
#define TWI_CMD             0x7F
#define TWI_READ_AVAILABLE  0x82

// Backward-compatibility alias — deprecated, use TWI_READ_AVAILABLE.
#define TWI_READ_AVALIBLE  TWI_READ_AVAILABLE
#endif  // UseWireClient

// ── Persistent data structure ─────────────────────────────────────────────────
//  Every field in Data{} is saved to and restored from internal flash (via
//  FlashStorage).  The Signature field is always written last so that a power-
//  loss during a write does not cause partially-updated data to be accepted on
//  the next boot.
//
// TODO 2 ── Add a field here for every parameter your module needs to persist
//           across power cycles (voltages, calibration constants, I2C address,
//           mode flags, etc.).  Keep the struct small — FlashStorage reserves a
//           whole flash page (~256 bytes minimum).  For larger data use the SPI
//           flash filesystem (see saveDefaults / loadDefaults below).
typedef struct
{
  int16_t       Size;           // Byte count of this struct — used for version detection
  char          Name[20];       // Human-readable board name, settable via SNAME
  int8_t        Rev;            // Firmware/hardware revision number
  #if UseWireClient
  int           TWIadd;         // I2C address this board listens on (set via STWIADD)
  #endif
  // ── Add your own persistent parameters below ──────────────────────────────
  // Example:  float  hvSetpoint;   // High-voltage setpoint in volts
  // Example:  ADCchan monChan;     // ADC calibration (m, b) for a monitor input
  // ──────────────────────────────────────────────────────────────────────────
  unsigned int  Signature;      // Must equal SIGNATURE for the struct to be considered valid
} Data;

// Forward declarations — implementations are in src/template.cpp.
void SaveSettings(void);
void RestoreSettings(void);
void bootloader(void);

void saveDefaults(void);
void loadDefaults(void);
void saveCalibrations(void);
void loadCalibrations(void);

#if UseWireClient
void setTWIaddress(void);
void getTWIaddress(void);
#endif
