# CLAUDE.md — project brief & session handoff

Auto-loaded by Claude Code. Read this first, then `git log` for change rationale.

## What this is
Passive 2.4 GHz **promiscuous-mode WiFi detector for Flock Safety surveillance
gear**, ported to the **Lonely Binary ESP32 "Gold Edition" — classic
ESP32-WROOM-32E** (dual-core Xtensa, 4 MB flash, **no PSRAM**, USB-UART bridge).
Single source file: `main.cpp`. Detection logic (OUI list, addr1/addr2/addr3,
wildcard-probe signature) is upstream research — leave it alone unless asked.

## Hardware / pin map (all defines at top of `main.cpp`)
| GPIO | Function |
|------|----------|
| 2  | Onboard WS2812 RGB LED (Adafruit NeoPixel; color-coded per detection class; idle teal breathing) |
| 4  | External piezo buzzer |
| 16 | GPS NMEA in — Serial1 RX (← GPS TX) |
| 17 | GPS out — Serial1 TX (→ GPS RX, optional) |

GPS = Adafruit Ultimate GPS V3, 9600 NMEA, parsed by TinyGPS++. VIN→3V3, GND→GND.

## Build / flash / monitor (PlatformIO in VSCode)
- Board profile `esp32dev` (the `-n16r8` S3 IDs do NOT exist in espressif32 6.x).
- `platformio.ini` env: `lonelybinary_esp32_gold`. 4 MB `partitions.csv`.
- `lib_deps`: Adafruit NeoPixel, TinyGPSPlus. (NimBLE only if `USE_BLE 1`.)
- **After changing board/memory settings, delete `.pio/` and full-clean** — stale
  include paths cause `sdkconfig.h: No such file` errors.
- Serial monitor 115200. Optional: add `monitor_filters = esp32_exception_decoder`.

## Key facts / gotchas learned this session
- **BLE is disabled (`USE_BLE 0`) on purpose.** On the single-radio classic
  ESP32, BLE coexistence starves the promiscuous sniffer → many missed
  detections (field-confirmed). Code stays behind the guard; don't re-enable for
  detection work. (The BLE "5 copies on iPhone" issue was an app-side quirk.)
- **Persistence:** detection table is a binary snapshot `/fy_sess.bin`
  (header: magic 'FLK2', version, recSize, count, CRC32 + raw FYDetection array),
  **reloaded into the live table on boot** so counts survive power loss. Format
  changed from the old `/session.json`, so the first boot after flashing starts
  fresh (expected). `recSize` guard rejects mismatched-layout files.
- **Log export:** in the serial monitor, `d` dumps the stored table as CSV, `j`
  as JSON, `o` streams the legacy `/session.json`+`/prev_session.json` raw
  (`serialCommandTick()` in loop). The live per-detection JSON stream also works
  for capture (`monitor_filters = log2file`).
- **Flashing preserves SPIFFS:** a firmware upload writes only the app
  partition; the data partition/files survive (partition table unchanged, no
  reformat). The binary format only reads/writes `/fy_sess.bin`, so the old
  `/session.json` is left intact and recoverable via `o` — but the new firmware
  does NOT auto-load it (different format). Back up with esptool `read_flash
  0x2E0000 0x120000` + mkspiffs if you want a zero-trust copy before flashing.
- **GPS** stamps each detection's first sighting with lat/lon + UTC epoch:
  embedded in the JSON (`gps{}`+`utc`, Flask-compatible), in the human DETECT
  lines, and in the persisted record. No fix → still recorded, no geodata.
- The Serial1 debug mirror was retired to free the UART for GPS.
- `Serial.setTxTimeoutMs()` is guarded behind `ARDUINO_USB_CDC_ON_BOOT` (native
  USB only — classic ESP32 HardwareSerial lacks it).
- The file was clang-formatted (Allman braces) by the user's tooling — match it.

## Status — compiles clean; runtime validation pending
Commits `4885560` (BLE off), `2133dd3` (persistence), `1f4f1ec` (GPS) were
authored in the web sandbox, which cannot compile or flash. **First local build
(2026-07-14) passed with no errors or warnings** — the feared TinyGPS++ API and
struct file-I/O breakages did not materialize. RAM 20.0% (65,592 B), flash 27.2%
(802,965 B).

Never flashed to hardware, so all three features are unverified on-device.
Validate in order: (1) detection rate back to normal with BLE off;
(2) `restored N detections` on reboot; (3) GPS `gps=fix` in the status line +
geotags after an outdoor lock.

## Open / possible next steps
- **Piezo loudness** — full implementation plan in
  [`docs/PIEZO_LOUDNESS_PLAN.md`](docs/PIEZO_LOUDNESS_PLAN.md): BTL/anti-phase
  drive (2nd output on **GPIO18**, ~6.6 V p-p, +6 dB) via two phase-offset LEDC
  channels, plus a `PIEZO_RESONANCE_HZ` (~4 kHz) retune. Ready to execute; needs
  a second wire to the piezo and a local compile/test.
- Optional **per-hit GPS track log** (every re-sighting as its own point) vs the
  current one-geotag-per-unique-device model.
- `docs/BLE_DUP_PENDING.md` was removed (issue resolved).

## Branch
Work is on `claude/esp32-gold-edition-optimize-nxb5ir` (not `main`).
