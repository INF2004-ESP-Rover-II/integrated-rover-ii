# Camera UART receiver: first onboard migration step

Receiver-only diagnostic build restored after the repository pull on 6 November
2026. B3's S3 UART sender is now present. This test needs neither the Sensor Pico
nor the dashboard and never initialises motors. The normal `mecanum_robot` target
now links the camera/DFS modules after restoration; this standalone test still
does not run navigation. Actual UART reception has been confirmed by the user.
Use ONBOARD_DFS.md for the next stationary normal-firmware test and geometry setup.

## Quick test with the actual S3

1. Turn off motor power. With the S3 off, enter the Robo Pico's BOOTSEL USB mode
   and copy `build/firmware/robo_pico/camera_uart_diagnostic.uf2` to `RPI-RP2`.
   This temporarily replaces the rover application with the USB diagnostic.
2. Connect S3 GPIO17 TX to Robo Pico GP21 RX and common GND (3.3 V logic).
   Prefer a 1 kOhm series resistor on the signal. Reserve GP21's button and do
   not press it during this test. Keep both boards powered while the signal is
   connected. The Pico's USB data cable goes to the PC for text output.
3. Run B3's S3 firmware with its webcam connected. Keep its configured Wi-Fi
   router on: the current S3 startup waits for Wi-Fi before it starts UART vision.
   Leave the video-stream viewer closed for the first timing check.
4. From this repository root in PowerShell, find the Pico's USB COM port and open
   it. COM11 below is an example; identify the Robo Pico by unplugging/replugging
   its USB cable and comparing the port list. Close other monitors using it.

   ```powershell
   .\.venv\Scripts\python.exe -m serial.tools.list_ports
   .\.venv\Scripts\python.exe -m serial.tools.miniterm COM11 115200 --dtr 1
   ```

   The local virtual environment now has pyserial installed. On another PC use
   `python -m pip install pyserial` in the desired environment first. Exit the
   console with Ctrl+]. The COM port carries decoded USB text; GPIO17 -> GP21
   carries the separate binary UART stream.
5. Present a clear printed marker ID 1..36 (B3's current filter). Example output:

   ```text
   seq=41 status=0 reported_age=190 effective_age=192ms count=1 | id=7 range=850mm bearing=190cdeg quality=0
   [RX] bytes=650 valid=50 crc=0 format=0 timeout=0 hw_errors=0 overwritten=0
   ```

   Move the marker closer/farther and check range in mm. Move it right/left and
   check bearing sign; 190 cdeg is +1.90 degrees. Use quality=0 observations for
   this basic check. Valid/byte counters should keep increasing.
6. Remove only the marker: expect status=1, count=0 and advancing sequences.
   Unplug only the webcam while keeping S3 powered: expect status=2 then status=3,
   count=0, repeated last sequence and increasing age. Turning off the entire S3
   produces silence, not a CAMERA_LOST packet.

If no text appears, check the USB cable/COM port; the diagnostic prints counters
even without a sender. If bytes stays zero, check TX/RX/GND and that the S3 reached
its vision task. Rising CRC/hardware errors suggest the link or baud rate. If
format errors rise, check ID/count/status/bearing constraints and packet alignment.
Flagged pose failures (quality=1/range=0) are now accepted as diagnostic data. This console test does not certify motor guidance.

## What map_cmd will eventually do

Jeslyn's proposed module owns the map and A* state. `mapfsm_start` begins mapping,
confirmed arrival/exit calls update it, `mapfsm_goto` plans a route, and hop-query/
completion calls coordinate execution. The module is not implemented in this
checkout yet. The UART receiver does not depend on it.

Agreeing or "freezing" an interface means agreeing bytes/arguments, units,
ownership and result meanings before writing both ends. It does not mean every
teammate must finish their module first. The following wire assumptions implement
the proposed packet and should be shared with B3.

## Wire v1 used by this implementation

115200 8N1, little-endian multibyte fields:

```text
A5 | seq:u8 | status:u8 | age_ms:u16 | N:u8 |
N * (id:u8 | lens_range_mm:u16 | bearing_cdeg:i16 | quality:u8) |
crc:u8
```

- Size: 7 + 6N bytes; N=0..4 (7/13/31 bytes for zero/one/four markers).
- Status: 0=OK, 1=NO_MARKER, 2=STALLED, 3=CAMERA_LOST.
- OK requires 1..4 markers; every other status requires N=0.
- IDs 1..255 (0 reserved), unique within one image; range >0; bearing -18000..18000
  hundredths of a degree, positive right.
- B3 now defines quality: 0=OK, 1=pose failed, 2=bad shape, 3=too small,
  4=too far. The diagnostic preserves the byte. A zero range with nonzero quality
  is accepted; a zero range with quality=0 is rejected. Nonzero qualities never
  guide movement, place nodes or confirm arrival. Fresh flagged images keep
  camera health alive without refreshing position.
- B3 measures age_ms from frame arrival at the S3 to transmission, saturated at
  65535. It excludes camera/USB delay. The helper adds rounded-up wire time and
  elapsed local receive time; it cannot reconstruct the missing source latency.
- CRC-8/SMBUS: polynomial 0x07, initial 0, non-reflected, final XOR 0.
  Covers seq through final payload byte, excluding A5 and CRC.
- CRC check for ASCII `123456789` is F4. Empty frame example, sequence=0,
  status=NO_MARKER, age=0: `A5 00 01 00 00 00 16`.
- A sync-valued payload byte is legal. The bounded parser handles bad CRC,
  invalid count/status/marker data, noise and truncated frames. Its partial-frame
  inter-byte timeout is 25 ms; this is NOT the navigation camera-loss timeout.

The parser exposes repeated sequences and all health packets. Downstream camera/
navigation state must still distinguish new images, normal modulo-256 wrap,
duplicate images, old images and health-only reports. Receiving a valid CRC packet
does not itself establish fresh position or confirmed arrival. The packet has no
boot identifier, so later resynchronisation/relocalisation must account for this.

## Files

- `firmware/common/vision_protocol.h/.c`: portable wire types, CRC/parser and age helper.
- `firmware/robo_pico/include/camera_uart.h` and `src/camera_uart.c`:
  UART1 adapter with hardware error counters and a latest-packet mailbox.
- `firmware/robo_pico/src/camera_uart_diagnostic.c`: dedicated receiver-only firmware.
- `firmware/robo_pico/test/test_vision_protocol.c`: portable protocol/failure tests.
- `tools/simulate_camera_uart.py`: binary fixture generator or optional serial sender.
- Normal `main.c`: UART1/camera and UART0/sensors feed local navigation; the
  default viewer receives the rover map. Geometry is required before motion.

The IRQ only parses bounded packets and updates the mailbox. Consumers copy the
latest packet under a short interrupt guard. Intermediate packets can be coalesced;
`mailbox_overwrites` counts this. This is not a reliable mission-event queue. Parsing
preserves the receive time of salvaged packets rather than making them younger
when the main loop recovers them.

## Tests without either board

Run from the repository root, using an available native C compiler and Python:

```powershell
gcc -std=c11 -Wall -Wextra -Wconversion -Werror -Ifirmware/common firmware/robo_pico/test/test_vision_protocol.c firmware/common/vision_protocol.c -o build/test_vision_protocol.exe
.\build\test_vision_protocol.exe
python tools/simulate_camera_uart.py --output build/camera_four.bin --frames 300 --markers 4 --start-seq 254
.\build\test_vision_protocol.exe build/camera_four.bin 300
python tools/simulate_camera_uart.py --output build/camera_empty.bin --status no_marker --frames 20
.\build\test_vision_protocol.exe build/camera_empty.bin 20
```

The existing test Makefile also has `check_vision`. It needs a working native C
compiler; the Arm cross compiler produces firmware, not runnable Windows tests.

Coverage includes golden CRC/empty frame, signed bearing, two-byte ages, repeated
IDs across frames, multiple simultaneous markers, duplicate IDs within one image,
sequence wrap, frozen/health packets, corrupt/truncated input, embedded sync,
resynchronisation timestamps, receiver-clock wrap and saturated ages. These do not
establish electrical UART reception or physical navigation performance.

## Build and hardware bring-up

```powershell
cmake --build build --target camera_uart_diagnostic
```

Output: `build/firmware/robo_pico/camera_uart_diagnostic.uf2`. This target does not
link/init motor control, Wi-Fi or Sensor UART. It is separate from the normal
`mecanum_robot.uf2`; flash the diagnostic explicitly for receiver bring-up. No board
has been flashed by this implementation task. Keep motor power off during this
UART-only hardware test.

Connect a 3.3 V USB-to-UART adapter TX (or later S3 GPIO17) through a 1 kOhm series
resistor to Robo Pico GP21, plus common GND. GP21 shares an onboard button; reserve
it for UART and do not press the button. The resistor limits contention if pressed,
but does not prevent corrupted data. Do not use RS-232 signal levels. No return TX
wire or hardware flow control is required for this first receiver.

Open the Pico's USB serial console to see packet values and parser counters. If
using a PC adapter, install pyserial in the chosen Python environment and run:

```powershell
python tools/simulate_camera_uart.py --port COM7 --frames 120 --markers 2
python tools/simulate_camera_uart.py --port COM7 --status no_marker --frames 30
python tools/simulate_camera_uart.py --port COM7 --status stalled --frames 30
```

COM7 is an example: select the USB-to-UART adapter's port, not the Pico USB console.
The sender generates fabricated approach ranges with repeated IDs. It is not the
production S3 detector. Stalled/lost packets repeat a sequence with increasing age.
Use fabricated serial input only with the receiver-only diagnostic and motor
power off. Do not use it to guide a physical rover running motor-enabled DFS.

## Integration status

Normal production integration is restored: camera reception, dynamic-map DFS,
local hop execution, bounded recovery, executor lease and a receiver-only default
dashboard. B3 quality flags and zero-range failed poses are handled. Missing
camera geometry blocks autonomous movement independently in firmware. Real UART
reception passed per the user; physical navigation/calibration acceptance remains
pending. See ONBOARD_DFS.md for the next stationary test. No A* is implemented.
