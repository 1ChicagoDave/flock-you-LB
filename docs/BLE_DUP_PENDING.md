# PENDING: BLE "5 copies of each message" — diagnosis + ready fixes

**Status:** open, waiting on one field observation. **Firmware NOT yet changed for this.**
Working note — delete this file once the issue is resolved.

## Symptom
On the Circuit Magic BLE Controller iOS app, each serial line arrives **~5×**.
Content is now correct/complete (MTU fix landed in commit `6b22944`); the problem
is a delivery/rendering multiplier. USB serial (VSCode) shows each line once, so
the multiplication is specific to the BLE path.

## The deciding observation (ask user)
Compare the short `[flockyou] scanning (ch=… det=…)` heartbeat line (every 30 s)
against a long detection / JSON line:

- **Count VARIES with length** (short = 2–3, long = 5, JSON = more)
  → **Cause A: fragmentation rendering.** MTU didn't negotiate up; the app renders
  each BLE packet as its own line, so a ~90-byte line = ~5 packets.
- **Constant 5× on every line regardless of length**
  → **Cause B: duplicate connections/subscriptions.** `notify()` broadcasts to all
  subscribed clients; ~5 stale connections piled up from app reconnects during testing.

A clean-state test also distinguishes them: fully quit app, toggle iPhone BT off/on,
power-cycle ESP32, connect once. If copies drop to 1 → it was Cause B.

---

## Fix A — fragmentation rendering (count varies with length)
Root issue: negotiated ATT MTU is small (~23 → 20-byte notifies), so long lines
fragment and the app shows each packet.

1. Confirm the MTU. Add an MTU-change log so we can see what iOS actually granted:
   ```cpp
   class FYServerCallbacks : public NimBLEServerCallbacks {
     void onMTUChange(uint16_t mtu, ble_gap_conn_desc*) override {
       dualPrintf("[flockyou] BLE MTU negotiated: %u\n", mtu);
     }
     // ...existing onConnect/onDisconnect...
   };
   ```
2. If the app never negotiates MTU (stays 23), we cannot enlarge it unilaterally.
   Mitigate by making BLE lines short — send only the human-readable alert over BLE,
   NOT the ~200-byte JSON (JSON is for USB/Flask only):
   - Add `#define BLE_SEND_JSON 0`.
   - Split the output path: `emitDetectionJSON()` should write to USB/Serial1 only
     (a serial-only printf), bypassing `blePrint()`, when `BLE_SEND_JSON == 0`.
     Simplest: add a `static bool _bleMute` flag, set it true around the
     `emitDetectionJSON()` call in `drainAlertQueue()`, and have `blePrint()` early-return
     when `_bleMute`. Then only the `DETECT-…` human line goes to the phone.
3. Also worth trying app-side: a "append / newline-delimited" or "coalesce" display
   option in Circuit Magic, if it has one.

## Fix B — duplicate connections (constant 5×)
Root issue: multiple stale BLE connections, each subscribed; `notify()` hits all.

Enforce a single connection and stop new ones forming while connected:
```cpp
void onConnect(NimBLEServer* s, ble_gap_conn_desc* desc) override {
  // Drop any earlier connection so only the newest phone remains subscribed.
  for (auto& d : s->getPeerDevices()) {          // NimBLE 1.4.x: vector of conn ids
    if (d != desc->conn_handle) s->disconnect(d);
  }
  bleConnHandle = desc->conn_handle;
  bleConnected  = true;
  s->updateConnParams(desc->conn_handle,
                      BLE_CONN_MIN_INTERVAL, BLE_CONN_MAX_INTERVAL,
                      BLE_CONN_LATENCY, BLE_CONN_TIMEOUT);
  NimBLEDevice::getAdvertising()->stop();         // no 2nd/stale connection while connected
}
void onDisconnect(NimBLEServer*) override {
  bleConnected = false;
  NimBLEDevice::startAdvertising();               // allow a fresh reconnect
}
```
Notes:
- Verify the exact `getPeerDevices()` return type/API in the pinned NimBLE-Arduino
  1.4.x before using; adjust the disconnect loop accordingly.
- Consider also lowering `CONFIG_BT_NIMBLE_MAX_CONNECTIONS` to 1 if easy, as a belt.

---

## Context / current state (as of note)
- Branch: `claude/esp32-gold-edition-optimize-nxb5ir`.
- BLE = NimBLE (Nordic UART Service), advertises as `FlockYou`, lazy connection
  (long interval + slave latency) + `ESP_COEX_PREFER_WIFI`.
- `blePrint()` chunks to negotiated `getPeerMTU()-3`, fallback `BLE_CHUNK` (20).
- Also still open from earlier: optional **piezo resonance retune** (~4 kHz) for
  loudness, and an optional **on-demand BLE** (radio fully off between hits) mode.
