# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

Python library for interfacing with generic (AliExpress) TPMS (Tyre Pressure Monitoring System) USB devices. Tested on TY06 hardware, should work with TY05. Published to PyPI as `tpms`.

## Commands

```bash
# Install dependencies
pip install -r requirements.txt

# Install package in development mode
pip install -e .

# Run the example CLI application
tpms-monitor
# or
python -m tpms_lib.example
```

## Architecture

### Core Library (`tpms_lib/__init__.py`)

The library uses a serial protocol to communicate with TPMS USB receivers. Key components:

- **TPMSDevice**: Main interface class. Handles connection, threading for async reads, and provides the public API.
- **TPMSProtocol**: Low-level protocol handling for frame creation and parsing.
- **TyreState**: Data class holding tyre metrics (pressure in kPa, temperature in Celsius, status flags).
- **TyrePosition**: Enum for positions (FRONT_LEFT, FRONT_RIGHT, REAR_LEFT, REAR_RIGHT, SPARE_TYRE).

### Communication Pattern

- Serial connection at 19200 baud (falls back to 9600, 115200, 38400, 57600)
- Background thread continuously reads data and parses frames
- Callbacks for state updates, pairing completion, and tyre exchanges
- 60ms delay between commands (from Android app)

## Protocol Details

Protocol reverse-engineered from Android app source (`src/java/com/tpms/`).

### Frame Structure

```
[0x55] [0xAA] [length] [cmd/data] [data...] [checksum]
  ^      ^       ^         ^                     ^
header  header  total    varies by           XOR checksum
                frame    message type
                size
```

### Checksum Calculation

XOR of bytes from index 0 to (length - 2):
```python
def calc_checksum(frame):
    length = frame[2]
    checksum = frame[0]
    for i in range(1, length - 1):
        checksum ^= frame[i]
    return checksum & 0xFF
```

### Message Types (determined by length byte)

| Length | Type | frame[3] meaning |
|--------|------|------------------|
| 6 | Command messages | Sub-command (0x01=pair, 0x06=stop, 0x07=query, 0x18=pair success, 0x19=heartbeat, 0x58=reset) |
| 7 | Exchange tyres | Command 0x03, response has 0x30 |
| 8 | Tyre state | Position code (0, 1, 16, 17, 5) |
| 9 | Query ID response | Position code (1-5) |

### Position Codes

| Position | Pairing/State Code | Query ID Code |
|----------|-------------------|---------------|
| Front Left | 0 | 1 |
| Front Right | 1 | 2 |
| Rear Left | 16 (0x10) | 3 |
| Rear Right | 17 (0x11) | 4 |
| Spare | 5 | 5 |

### Command Frames (sent to device)

| Command | Frame (before checksum) |
|---------|------------------------|
| Heartbeat | `[55 AA 06 19 00 ??]` |
| Query IDs | `[55 AA 06 07 00 ??]` |
| Pair FL | `[55 AA 06 01 00 ??]` |
| Pair FR | `[55 AA 06 01 01 ??]` |
| Pair RL | `[55 AA 06 01 10 ??]` |
| Pair RR | `[55 AA 06 01 11 ??]` |
| Pair Spare | `[55 AA 06 01 05 ??]` |
| Stop Pair | `[55 AA 06 06 00 ??]` |
| Exchange | `[55 AA 07 03 pos1 pos2 ??]` |
| Reset | `[55 AA 06 58 55 ??]` |

### Response Frames (from device)

**Tyre State** (length=8): `[55 AA 08 position pressure temp flags checksum]`
- pressure: raw * 3.44 = kPa (subtract 20 if > 50 kPa per Android app)
- temp: raw - 50 = Celsius
- flags: bit 5 = no signal, bit 3 = leaking, bit 4 = low battery

**Pairing Success** (length=6): `[55 AA 06 18 position checksum]`

**Query ID Response** (length=9): `[55 AA 09 position id0 id1 id2 id3 checksum]`

**Exchange Success** (length=7): `[55 AA 07 30 pos1 pos2 checksum]`

### Unit Conversions

- Pressure: `raw * 3.44` = kPa, then `kPa * 0.145038` = PSI
- Temperature: `raw - 50` = Celsius

## Android App Source

Decompiled source is in `src/java/com/tpms/`:
- `encode/FrameEncode3.java` - Command encoding
- `decode/FrameDecode3.java` - Response parsing
- `encode/PackBufferFrameEn3.java` - Checksum calculation
- `view/PaireIDActivity.java` - Pairing UI flow (120s timeout)
