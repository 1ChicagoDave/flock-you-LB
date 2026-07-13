# Flock-You: Promiscuous WiFi Edition (`promiscious-dev` branch)

<img src="flock.png" alt="Flock You" width="300px">

**Passive 2.4 GHz promiscuous-mode detector for Flock Safety surveillance infrastructure. Runs standalone or feeds the Flask dashboard over USB for live GPS-tagged wardriving.**

> **Dev note:** This is the `promiscious-dev` branch — adds the
> DeFlockJoplin wildcard-probe tightening and a 31st OUI on top of the
> `promiscious` baseline. See "Further research" below.

---

## Credit

All WiFi promiscuous detection research — the **30-OUI target list**, the **promiscuous-mode strategy**, and the **addr1-receiver detection technique** — is the work of **ØяĐöØцяöЪöяцฐ / @NitekryDPaul**. The firmware here is a mod of his original firmware with added SPIFFS persistence and Flask-dashboard integration. Full research writeup: [`datasets/NitekryDPaul_wifi_ouis.md`](datasets/NitekryDPaul_wifi_ouis.md).

Additional research credit to **Michael / DeFlockJoplin** for the **wildcard-probe-request signature** and the 31st OUI (`82:6b:f2`). Field-tested to 11/12 cameras caught with only 2 false positives in Joplin. Source: [DeflockJoplin/flock-you](https://github.com/DeflockJoplin/flock-you).

---

## What this branch does

Turns a Lonely Binary ESP32 "Gold Edition" (classic ESP32-WROOM-32E) into a passive WiFi receiver that watches 2.4 GHz management and data frames for Flock Safety MAC OUIs. No AP, no transmit — the radio stays dedicated to sniffing while the device hops channels 1 / 6 / 11 at 350 ms dwell.

Every detection is:

- beeped (external piezo on GPIO4) and flashed on the onboard WS2812 RGB LED (GPIO2), color-coded by detection class
- written to on-device SPIFFS in an atomic CRC-envelope format, surviving power loss
- emitted as one JSON line over USB CDC in the schema `api/flockyou.py` expects, so the Flask dashboard auto-ingests it with GPS temporal matching

The device works standalone (no USB host needed) and plugged in (live dashboard) without any mode switch.

---

## Why promiscuous mode, and why `addr1`

Most WiFi sniffers only check the transmitter address (`addr2`). Flock infrastructure spends most of its duty cycle **asleep** — it wakes briefly in bursts, uploads, then sleeps again. During the silence it may never transmit a single frame in your capture window.

But it may still appear on the air as the **destination** (`addr1`) of probe responses or data frames from nearby APs.

Checking `addr1` in addition to `addr2` picks those silent stations up. It requires two guards to avoid false positives:

- `addr1` is broadcast (`ff:ff:ff:ff:ff:ff`) in beacons and broadcasts — **multicast filter**
- Modern devices use randomised (locally-administered) MACs that can't be fingerprinted by OUI — **randomised-MAC filter** on byte 0 bit 1

Both are applied before the OUI match. This whole approach, including the 30-OUI list, is **@NitekryDPaul's research**.

---

## Further research — the wildcard-probe signature (DeFlockJoplin)

Michael / DeFlockJoplin used the OUI + addr1/addr2/addr3 work above as a starting point and characterised what Flock cameras actually do on the air. His finding:

> The cameras are hopping channels and sending out a wildcard WiFi probe request on every channel. This specific type of request combined with OUI matching has created what seems to be a fairly unique signature.

His drive-test in Joplin caught **11 of 12 cameras** with only **2 false positives**. The 12th camera was doing the same wildcard-probe behaviour but with an OUI (`82:6b:f2`) that wasn't in @NitekryDPaul's original 30 — it's now the 31st entry in our list, credited to him.

The tightened signature that's active on this branch:

1. Frame is 802.11 Management, type=0 subtype=4 (**Probe Request**)
2. SSID Information Element (tag 0) is present with **length 0** (wildcard)
3. `addr2` (transmitter) matches the known-OUI list

When all three hit, we emit `detection_method: wifi_wildcard_probe` — the high-precision class. Non-probe frames from the same OUIs still emit `wifi_oui_addr2`, and the `addr1` receiver-side sleeper-catch still runs independently.

His proof-of-concept firmware (different enough we're not just pulling it in wholesale, but the core idea carried over cleanly): [DeflockJoplin/flock-you](https://github.com/DeflockJoplin/flock-you). The wildcard-probe analysis is his; we ported the detection into this firmware and kept our SPIFFS persistence, Flask JSON emission, and audio/LED feedback on top.

---

## Detection pipeline

```
  [2.4GHz air]
       │
       ▼
  wifiSniffer()                 ← IRAM promiscuous callback (WiFi task)
       │                          fast match only, no Serial / no malloc
       ▼
  alertQueue[32]                ← lock-free ring buffer (ISR-safe mux)
       │
       ▼
  drainAlertQueue()             ← loop() context, per-iteration drain
       │
       ├─► fyAddDetection()           ← always, every hit
       │        │
       │        ▼
       │   fyDet[200]                 ← unique-by-MAC on-device table
       │        │
       │        ▼
       │   autosaveTick()             ← every 60s when dirty
       │        │
       │        ▼
       │   fySaveSession()            ← atomic CRC-envelope write to SPIFFS
       │
       ├─► shouldSuppressDuplicate()  ← 5s per-MAC serial-emit rate limit
       │
       └─► emitDetectionJSON()        ← USB CDC line for Flask
            buzzerBeep() + ledFlash()
```

The split between callback and loop is deliberate: the WiFi task has hard real-time constraints and cannot call `Serial.print` or `malloc` safely. The callback writes only to the lock-free ring buffer; `loop()` does all the heavy work.

---

## OUI target list (@NitekryDPaul research)

All lowercase, colon-separated. 31 Flock Safety infrastructure prefixes:

```
70:c9:4e   3c:91:80   d8:f3:bc   80:30:49   b8:35:32
14:5a:fc   74:4c:a1   08:3a:88   9c:2f:9d   c0:35:32
94:08:53   e4:aa:ea   f4:6a:dd   f8:a2:d6   24:b2:b9
00:f4:8d   d0:39:57   e8:d0:fc   e0:4f:43   b8:1e:a4
70:08:94   58:8e:81   ec:1b:bd   3c:71:bf   58:00:e3
90:35:ea   5c:93:a2   64:6e:69   48:27:ea   a4:cf:12
82:6b:f2   ← contributed by Michael / DeFlockJoplin
```

Pre-compiled into a byte table in `setup()` so the matcher stays entirely in IRAM with no flash-resident lookups during callback execution.

Full dataset and methodology: [`datasets/NitekryDPaul_wifi_ouis.md`](datasets/NitekryDPaul_wifi_ouis.md).

---

## SPIFFS persistence (reloads on boot)

The detection table is saved to SPIFFS as a compact **binary snapshot** and — crucially — **reloaded into the live table on boot**, so the count accumulates and survives power loss instead of resetting to 0. On-disk layout:

```
Header:  magic 'FLK2' | version | recSize=sizeof(FYDetection) | count | CRC32(records)
Records: count × FYDetection structs (raw)
```

Save procedure (autosaves every 60 s when dirty):

1. CRC32 the records region
2. Write header + records to `/fy_sess.tmp`
3. Atomic rename `/fy_sess.tmp` → `/fy_sess.bin` (copy+delete fallback)

Boot:

1. Read `/fy_sess.bin` (fallback `/fy_sess.tmp` from an interrupted save)
2. Validate magic / version / `recSize` / CRC32 — a mismatch (e.g. a firmware struct change) is ignored and the table starts fresh
3. Load the records into the live table; scanning continues from the saved count

`recSize` in the header makes the format self-guarding across firmware versions, and CRC32 uses the standard `0xEDB88320` polynomial.

---

## Flask dashboard integration

The firmware emits one JSON line per detection in the same schema the BLE detector uses, so `api/flockyou.py` picks it up with zero changes:

```json
{"event":"detection","detection_method":"wifi_oui_addr2","protocol":"wifi_2_4ghz","mac_address":"aa:bb:cc:dd:ee:ff","oui":"aa:bb:cc","device_name":"","rssi":-62,"channel":6,"frequency":2437,"gps":{"latitude":37.421998,"longitude":-122.084000,"accuracy":6.2},"utc":1752345600,"ssid":""}
```

The `gps` object and `utc` (unix epoch, UTC) are included only when the on-board GPS has a current fix; without one, both are omitted and the rest of the line is unchanged.

`detection_method` values:

- `wifi_wildcard_probe` — **Probe Request + wildcard SSID from a known OUI** (the DeFlockJoplin high-precision signature). When this fires, the `addr2` broad alert is suppressed for the same frame to avoid double-counting.
- `wifi_oui_addr2` — transmitter-side OUI match on any non-probe frame
- `wifi_oui_addr1` — **receiver-side OUI match** (the @NitekryDPaul technique)
- `wifi_oui_addr3` — BSSID OUI match (mgmt frames only; disabled by default)
- `wifi_ssid` — SSID keyword match (disabled by default)

### GPS wardriving

GPS is now handled **on-device** by the Adafruit Ultimate GPS V3 (see Hardware). Each detection is geotagged with lat/lon and a UTC timestamp at the moment of first sighting, embedded directly in the JSON line and saved into the persistent detection record — no Flask-side GPS puck or browser geolocation required. Flask ingests the `gps`/`utc` fields directly for map export (JSON / CSV / KML for Google Earth).

(The older Flask-side options — a USB NMEA puck or the dashboard's browser Geolocation — still work if you build without GPS, `USE_GPS 0`.)

### Running Flask

```bash
cd api
pip install -r requirements.txt
python flockyou.py
```

Open `http://localhost:5000`, pick your serial port from the UI, detections start showing up live.

---

## Hardware

**Board:** Lonely Binary ESP32 "Gold Edition" — classic ESP32-WROOM-32E module (dual-core Xtensa LX6, 4 MB flash, no PSRAM, onboard USB-UART bridge). Builds on the generic `esp32dev` PlatformIO profile.

| Pin | Function |
|-----|----------|
| GPIO 4 | External piezo buzzer |
| GPIO 2 | Onboard WS2812 RGB LED (addressable NeoPixel) |
| GPIO 16 | GPS NMEA in — Serial1 RX (← GPS TX) |
| GPIO 17 | GPS out — Serial1 TX (→ GPS RX, optional) |

**GPS (Adafruit Ultimate GPS V3):** VIN → 3V3 (or 5V), GND → GND, GPS **TX → GPIO 16**, GPS **RX → GPIO 17** (optional). Default 9600-baud NMEA, parsed on-device with TinyGPS++. Every detection is stamped with the current fix (lat/lon + UTC epoch); with no fix, detections are still recorded, just without geodata. The Serial1 debug mirror was retired to free the UART for GPS — read logs over USB.

The RGB LED encodes the detection class as color:

| Color | Detection |
|-------|-----------|
| 🔴 Red | Wildcard probe (high-precision DeFlockJoplin signature) |
| 🟠 Amber | Transmitter-side OUI (`addr2`) |
| 🔵 Blue | Receiver-side sleeper catch (`addr1`) |
| 🟦 Cyan | BSSID fallback (`addr3`) |
| 🟣 Magenta | SSID keyword |
| 🟢 Green | Boot / startup |
| 🩵 Dim teal (slow breathing) | Idle — powered and scanning, no current hit |

Boot sound: first 6 notes of Super Mario Bros. World 1-2 (underground).

---

## Build and flash

Requires [PlatformIO](https://platformio.org/).

```bash
pio run                     # build
pio run -t upload           # flash
pio device monitor          # serial output
```

`platformio.ini` and `partitions.csv` are at the root (4 MB flash layout: ~2.8 MB app, ~1.1 MB SPIFFS). The one library dependency — Adafruit NeoPixel, for the onboard WS2812 — is declared in `platformio.ini` under `lib_deps`, so PlatformIO fetches it automatically on first build.

---

## Config cheatsheet (top of `main.cpp`)

| Define | Default | Notes |
|---|---|---|
| `CHANNEL_MODE` | `CHANNEL_MODE_CUSTOM` | `CUSTOM` (1/6/11), `FULL_HOP` (1-11), or `SINGLE` |
| `CHANNEL_DWELL_MS` | 350 | Time on each channel before hop |
| `RSSI_MIN` | -95 | Drop frames weaker than this |
| `ALERT_COOLDOWN_MS` | 5000 | Per-MAC serial-emit rate limit |
| `CHECK_ADDR1` | 1 | The @NitekryDPaul receiver-side technique |
| `CHECK_ADDR3` | 0 | BSSID fallback (mgmt frames only) |
| `ENABLE_SSID_MATCH` | 0 | Substring match against `target_ssid_keywords[]` |
| `PROCESS_MGMT_FRAMES` | 1 | Beacons, probe req/resp, etc. |
| `PROCESS_DATA_FRAMES` | 1 | Data frames (where addr1 catch shines) |
| `MAX_DETECTIONS` | 200 | On-device table cap |
| `AUTOSAVE_INTERVAL_MS` | 60000 | SPIFFS save cadence |
| `LED_PIN` | 2 | Onboard WS2812 RGB LED |
| `LED_BRIGHTNESS` | 64 | Per-channel ceiling (0-255) for the WS2812 |
| `BUZZER_PIN` | 4 | External piezo |

---

## Standalone vs connected

**Without USB:** device boots, reloads its saved detection table from SPIFFS (count picks up where it left off), plays the SMB 1-2 intro, starts scanning, stores every unique detection with GPS geotag, flashes the onboard LED on each hit.

**With USB + Flask running:** same thing, plus every detection streams live to the dashboard as a JSON line. Flask adds GPS (if configured) and deduplicates across MAC, building the wardriving map as you move.

Both modes work simultaneously — the SPIFFS write path doesn't care if a host is listening.

---

## Live BLE readout (iPhone / iOS)

A BLE serial mirror (Nordic UART Service, advertises as `FlockYou`) is implemented but **disabled by default (`USE_BLE 0`)**. On the single-radio classic ESP32, BLE coexistence steals airtime from the promiscuous sniffer and — confirmed in field testing — causes many missed detections, which defeats the purpose of the device. The code stays behind the `USE_BLE` guard (with a lazy-connection + WiFi-priority coexistence design) so it can be re-enabled for casual monitoring, but for detection work leave it off and read over USB/Flask.

> **iOS note:** if you do re-enable it, iPhones can't use Bluetooth *Classic* SPP (`BluetoothSerial`) — only BLE. This is a BLE (Nordic UART) implementation for exactly that reason. Enabling `USE_BLE` also re-adds the NimBLE-Arduino dependency in `platformio.ini`.

---

## BLE companion firmware

The BLE-only sibling of this firmware lives on the [`main` branch](https://github.com/colonelpanichacks/flock-you/tree/main). It detects Flock and Raven gear via BLE advertisements (OUI prefix, device name, manufacturer ID `0x09C8`, Raven service UUIDs), runs its own WiFi AP with a phone-facing dashboard at `192.168.4.1`, and emits the same Flask JSON schema. Flash both on separate boards for overlapping BLE + WiFi coverage feeding one Flask dashboard.

---

## Acknowledgments

- **ØяĐöØцяöЪöяцฐ (@NitekryDPaul)** — **WiFi promiscuous detection research**: the 30-OUI Flock Safety target list and the addr1-receiver detection technique that are the baseline of this firmware. The code here is a mod of his original work.
- **Michael / DeFlockJoplin** ([DeflockJoplin/flock-you](https://github.com/DeflockJoplin/flock-you), [deflockjoplin.today](https://deflockjoplin.today)) — **wildcard-probe-request signature** + the 31st OUI (`82:6b:f2`). Drive-tested in Joplin to 11/12 cameras caught with only 2 false positives.
- **Will Greenberg** ([@wgreenberg](https://github.com/wgreenberg)) — BLE manufacturer company ID detection (`0x09C8` XUNTONG) sourced from his [flock-you](https://github.com/wgreenberg/flock-you) fork (used by the BLE companion on `main`)
- **[DeFlock](https://deflock.me)** ([FoggedLens/deflock](https://github.com/FoggedLens/deflock)) — crowdsourced ALPR location data and detection methodologies. Datasets included in `datasets/`
- **[GainSec](https://github.com/GainSec)** — Raven BLE service UUID dataset (`raven_configurations.json`) used by the BLE companion

---

## OUI-SPY Firmware Ecosystem

Flock-You is part of the OUI-SPY firmware family:

| Firmware | Description | Board |
|----------|-------------|-------|
| **[OUI-SPY Unified](https://github.com/colonelpanichacks/oui-spy-unified-blue)** | Multi-mode BLE + WiFi detector | ESP32-S3 / ESP32-C5 |
| **[OUI-SPY Detector](https://github.com/colonelpanichacks/ouispy-detector)** | Targeted BLE scanner with OUI filtering | ESP32-S3 |
| **[OUI-SPY Foxhunter](https://github.com/colonelpanichacks/ouispy-foxhunter)** | RSSI-based proximity tracker | ESP32-S3 |
| **[Flock You](https://github.com/colonelpanichacks/flock-you)** | Flock Safety / Raven surveillance detection (this project) | ESP32-S3 |
| **[Sky-Spy](https://github.com/colonelpanichacks/Sky-Spy)** | Drone Remote ID detection | ESP32-S3 / ESP32-C5 |
| **[Remote-ID-Spoofer](https://github.com/colonelpanichacks/Remote-ID-Spoofer)** | WiFi Remote ID spoofer & simulator with swarm mode | ESP32-S3 |
| **[OUI-SPY UniPwn](https://github.com/colonelpanichacks/Oui-Spy-UniPwn)** | Unitree robot exploitation system | ESP32-S3 |

---

## Author

**colonelpanichacks**

**Oui-Spy devices available at [colonelpanic.tech](https://colonelpanic.tech)**

---

## Disclaimer

Passive reception of publicly-broadcast 802.11 frames for security research, privacy auditing, and education. The device does not transmit and does not authenticate to any network. Detecting the presence of surveillance hardware in public spaces is legal in most jurisdictions; always comply with local laws regarding wireless reception.
