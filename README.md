# voltcraft-sem6000

Perl script for controlling a **Voltcraft SEM-6000** smart wall socket via Bluetooth Low Energy (BLE), directly through the Linux ATT/L2CAP socket layer — no `gatttool`, no external BLE library required.

## Features

- Read switch state (ON/OFF)
- Read live meter data: voltage, current, power, frequency, power factor, total energy
- Turn socket on/off
- Toggle socket state
- PIN authentication
- Verbose/debug hex dump mode

## Requirements

- Linux with BlueZ kernel support (`AF_BLUETOOTH`/`BTPROTO_L2CAP`)
- Perl 5 (no non-core modules beyond `Time::HiRes` and `Getopt::Long`)
- BLE adapter visible to the kernel (e.g. `hciconfig hci0 up`)
- The BLE address and PIN of your SEM-6000 device (default PIN: `0000`)

## Usage

```
./voltcraft.pl -d AA:BB:CC:DD:EE:FF [--pin NNNN] [actions] [options]
```

If no action is given, `--status --meter` is the default.

### Actions

| Flag | Description |
|---|---|
| `--status` | Print switch state (ON/OFF) |
| `--meter` | Print voltage, current, power, frequency, power factor |
| `--on` | Turn socket on |
| `--off` | Turn socket off |
| `--toggle` | Read current state and switch to opposite |

### Options

| Flag | Default | Description |
|---|---|---|
| `-d`, `--device ADDR` | *(required)* | BLE MAC address (`AA:BB:CC:DD:EE:FF`) |
| `--pin NNNN` | `0000` | 4-digit device PIN |
| `--addr-type TYPE` | `public` | BLE address type: `public` or `random` |
| `--connect-timeout SEC` | `5` | Connection timeout in seconds |
| `--wait-notify-ms N` | `3000` | Max wait for a notification response (ms) |
| `-v`, `--debug` | off | Verbose output; repeat (`-vv`) for raw hex dumps |
| `-h`, `--help` | | Show help |

### GATT UUID overrides

Only needed if your device firmware differs from the standard SEM-6000 profile.

| Flag | Default | Purpose |
|---|---|---|
| `--service-uuid` | `fff0` | Primary service UUID |
| `--command-char-uuid` | `fff3` | Write-command characteristic |
| `--notify-char-uuid` | `fff4` | Notification characteristic |

## Examples

```bash
# Read switch state and meter (default)
./voltcraft.pl -d B3:00:00:00:73:C0

# Read only switch state
./voltcraft.pl -d B3:00:00:00:73:C0 --status

# Read only meter data
./voltcraft.pl -d B3:00:00:00:73:C0 --meter

# Turn on
./voltcraft.pl -d B3:00:00:00:73:C0 --on

# Turn off with non-default PIN
./voltcraft.pl -d B3:00:00:00:73:C0 --pin 1234 --off

# Toggle with debug output
./voltcraft.pl -d B3:00:00:00:73:C0 --toggle -v
```

## Protocol notes

All communication uses BLE ATT over a raw Linux L2CAP socket (CID 4). Responses arrive as GATT notifications — no polling.

**Packet framing** (both request and response):
```
[0x0F][LEN][payload...][CHK][0xFF][0xFF]
```
- `LEN` = `len(payload) + 1`
- `CHK` = `(1 + sum(payload)) & 0xFF`

**Command payloads:**

| Command | Payload bytes |
|---|---|
| Auth | `0x17 0x00 0x00 <p0 p1 p2 p3> 0x00 0x00 0x00 0x00` |
| Measurement | `0x04 0x00 0x00 0x00` |
| Switch ON | `0x03 0x00 0x01 0x00 0x00` |
| Switch OFF | `0x03 0x00 0x00 0x00 0x00` |

PIN digits are sent as integers `0`–`9`, not ASCII.

**Measurement response fields** (14 or 16 bytes):

| Bytes | Field |
|---|---|
| `[2]` | Power state (1 = ON) |
| `[3–5]` | Watts × 1000 (24-bit big-endian) |
| `[6]` | Voltage (V) |
| `[7–8]` | Ampere × 1000 (16-bit big-endian) |
| `[9]` | Frequency (Hz) |
| `[12–15]` | Total kWh × 1000 (32-bit big-endian, hardware < v3 only) |

Protocol reference: <https://github.com/Heckie75/voltcraft-sem-6000/blob/master/sem-6000.exp>

## License

MIT
