# GAACE Application Template

A PlatformIO starter template for building firmware modules using the GAACE_Core framework. Modules developed from this template communicate with the MIPS host application over USB serial and optional TCP/IP connections.

## Target Hardware

| Setting | Value |
|---|---|
| Board | Adafruit ItsyBitsy M0 (SAMD21) |
| Framework | Arduino |
| Build system | PlatformIO |

## Project Structure

```
Template/
├── include/
│   └── template.h          # Data struct, constants, forward declarations
├── lib/
│   ├── GAACE/              # Core framework (commandProcessor, WireHelper, Devices, etc.)
│   ├── ArduinoThread/      # Cooperative thread scheduler
│   ├── FlashStorage/       # Internal MCU flash storage
│   ├── Adafruit_SPIFlash/  # SPI/QSPI flash chip driver
│   └── SdFat_-_Adafruit_Fork/  # FAT filesystem (FatFs)
├── src/
│   └── template.cpp        # Main firmware source
└── platformio.ini          # Build configuration
```

## Feature Flags

Four `#defines` at the top of `src/template.cpp` gate entire blocks of code. Set each to `true` or `false` before doing anything else. Disabled sections compile away completely.

| Flag | Default | Purpose |
|---|---|---|
| `AppName` | `"Template"` | Module name — returned by `GVER`, stored in flash |
| `UseWireClient` | `true` | I2C client (slave) to MIPS host; enables Wire callbacks and TWI commands |
| `UseThreads` | `true` | Cooperative scheduler; runs `Update()` every 25 ms |
| `UseSPIflash` | `true` | FatFs filesystem on on-board QSPI/SPI flash |

## Serial Command Protocol

Commands are ASCII, comma-delimited, newline-terminated:

```
COMMAND,arg1,arg2\n
```

Every command replies with `ACK` (0x06) or `NAK` (0x15). Data responses follow immediately after an ACK.

### Built-in Commands

| Command | Args | Description |
|---|---|---|
| `GVER` | — | Return firmware version string |
| `GNAME` | — | Return device name |
| `SNAME,<str>` | string | Set device name |
| `SAVE` | — | Write Data struct to internal MCU flash |
| `RESTORE` | — | Reload Data struct from internal MCU flash |
| `BLOAD` | — | Jump to SAM-BA / UF2 bootloader (SAMD21/51) |
| `STWIADD,<hex>` | hex byte | Set I2C address (takes effect after reboot + SAVE) |
| `GTWIADD` | — | Return current I2C address in hex |
| `SAVEF` | — | Save full Data struct to `default.dat` on SPI flash |
| `LOADF` | — | Load Data struct from `default.dat` on SPI flash |
| `SAVECAL` | — | Save calibration structs to `cal.dat` on SPI flash |
| `LOADCAL` | — | Load calibration structs from `cal.dat` on SPI flash |

> TWI address commands require `UseWireClient true`.
> SPI flash commands require `UseSPIflash true`.

### Debug Commands (always available)

The `debug` library registers 19 additional diagnostic commands:

| Group | Commands |
|---|---|
| Memory | `SETADDRESS`, `SETOFFSET`, `WRITE`, `READ`, `DUMP`, `RAM` |
| GPIO | `PINMODE`, `DOUT`, `DIN` |
| Analog | `ADCRES`, `DACRES`, `ADC`, `DAC` |
| System | `DEBUG`, `UPTIME`, `RESET`, `UUID`, `CPUTEMP` |

`GCMDS` returns the full command list at runtime.

## Flash Persistence

Two independent persistence mechanisms are provided:

| Mechanism | Header | Survives firmware upload? | Use for |
|---|---|---|---|
| `FlashStorage` (internal) | `FlashStorage.h` | No | Settings during development |
| FatFs on SPI flash | `Adafruit_SPIFlash` + `FlashFS` | Yes | Settings and calibration in production |

`FlashStorage` is erased every time a new sketch is uploaded. Use `SAVEF`/`LOADF` (SPI flash) for data that must survive firmware updates.

## Creating a New Module from This Template

1. Copy `src/template.cpp` → `src/MyModule.cpp` and `include/template.h` → `include/MyModule.h`.
2. Update the `#include` in the **Section 1** block of `MyModule.cpp` to reference `MyModule.h`.
3. Work through the numbered TODO comments in order (TODO 1 – TODO 13).

| TODO | Location | Task |
|---|---|---|
| 1 | `src/template.cpp` | Set `AppName`; enable/disable feature flags |
| 2 | `include/template.h` | Add persistent fields to the `Data` struct |
| 3 | `src/template.cpp` | Header include path — already in place, update when renaming |
| 4 | `src/template.cpp` | Set factory defaults in `Rev_1_data` |
| 5 | `src/template.cpp` | Update firmware version date string |
| 6 | `src/template.cpp` | Add module commands to the `cmds[]` table |
| 7 | `src/template.cpp` | Add additional module-specific commands |
| 8 | `src/template.cpp` | Add temporary bring-up code to `Debug()` |
| 9 | `src/template.cpp` | Implement periodic hardware logic in `Update()` |
| 10 | `src/template.cpp` | Add custom I2C command bytes to `receiveEvent()` |
| 11 | `src/template.cpp` | Add hardware peripheral initialisation to `setup()` |
| 12 | `src/template.cpp` | Implement `saveCalibrations()` / `loadCalibrations()` |
| 13 | `src/template.cpp` | Add bootloader entry for non-SAMD platforms (optional) |

## I2C (TWI) Client Protocol

When `UseWireClient` is enabled, the board listens as an I2C client. MIPS sends one of three command bytes as the first byte of every I2C write:

| Byte | Constant | Meaning |
|---|---|---|
| `0x27` | `TWI_SERIAL` | Open serial-tunnel mode; subsequent bytes are forwarded to the command processor until an ESC (0x1B) byte closes the session |
| `0x7F` | `TWI_CMD` | Single-shot command: MIPS sends an ASCII command string (`\n`-terminated); the response is buffered and returned on the next I2C read |
| `0x82` | `TWI_READ_AVAILABLE` | On the next I2C read, return the 2-byte count of bytes waiting in the response buffer |

## Bundled Libraries

All dependencies are vendored in `lib/` and require no separate installation.

| Library | Version | Purpose |
|---|---|---|
| GAACE | 1.0.0 | Core framework: commandProcessor, RingBuffer, WireHelper, Devices, debug, charAllocate, Errors, FlashFS |
| ArduinoThread | — | Cooperative thread scheduler (`Thread`, `ThreadController`) |
| FlashStorage | — | Single-struct storage in internal MCU flash (SAMD EEPROM emulation) |
| Adafruit_SPIFlash | — | SPI/QSPI flash chip driver |
| SdFat — Adafruit Fork | — | FAT filesystem layer used by Adafruit_SPIFlash |

## Author

Gordon Anderson — [gaa@owt.com](mailto:gaa@owt.com)
