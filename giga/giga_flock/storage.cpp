// ============================================================
// storage.cpp — internal QSPI flash CSV event log (mbed FATFileSystem)
// ============================================================
//
// The GIGA R1 has 16 MB of internal QSPI NOR flash.  Arduino_POSIXStorage can
// NOT reach it — that library only exposes DEV_SDCARD and DEV_USB, there is no
// DEV_QSPI.  So we talk to the flash directly through the mbed block-device /
// filesystem API that ships inside the mbed_giga core.
//
// The QSPI is partitioned (MBR) into two areas: partition 1 holds the Wi-Fi /
// BLE firmware blob used by the onboard module, and partition 2 is a FAT
// user-data area.  We mount partition 2 read/write and keep our CSV there, at
// /fs/detections.csv (see LOG_PATH in config.h).
//
// *** ONE-TIME FORMAT ***
// The user-data FAT partition is created ONCE by the official Arduino example
// sketch:  File > Examples > STM32H747_System > QSPIFormat  (pick the option
// that keeps the Wi-Fi firmware and adds a FAT user-data partition).  Until that
// is done fs.mount() returns non-zero; we then run WITHOUT logging — the
// detector, UI, audio and GPS all keep working, only the CSV is skipped.  We
// DELIBERATELY never reformat/mkfs here: doing so could wipe the Wi-Fi firmware
// partition.

#include "storage.h"
#include "config.h"
#include "flock_types.h"      // g_dev / g_devCount / g_totalEvents for the snapshot

#include "BlockDevice.h"
#include "MBRBlockDevice.h"
#include "FATFileSystem.h"

#include <stdio.h>

// Internal QSPI flash + its FAT user-data partition (MBR partition 2).
static mbed::BlockDevice   *s_root = nullptr;
static mbed::MBRBlockDevice *s_userbd = nullptr;
static mbed::FATFileSystem  s_fs("fs");     // mounts under "/fs"

static FILE *s_log   = nullptr;
static bool  s_ready = false;

bool storage_ready() { return s_ready; }

bool storage_init()
{
  s_ready = false;
  s_log   = nullptr;

  // Default block device on the GIGA is the 16 MB QSPI NOR flash.
  s_root = mbed::BlockDevice::get_default_instance();
  if (!s_root) return false;
  if (s_root->init() != 0) return false;

  // Partition 2 = the FAT user-data partition (partition 1 = Wi-Fi firmware).
  static mbed::MBRBlockDevice userbd(s_root, 2);
  s_userbd = &userbd;
  if (s_userbd->init() != 0)
  {
    // No MBR / partition 2 present — QSPI not formatted for user data yet.
    return false;
  }

  // Mount the existing FAT.  Non-zero => not formatted; run without logging.
  int err = s_fs.mount(s_userbd);
  if (err != 0)
  {
    return false;
  }

  // Fresh file?  Treat "missing" OR "exists but 0 bytes" as new so the header is
  // (re)written — a 0-byte file is what a prior non-durable run left behind.
  bool isNew = false;
  {
    FILE *probe = fopen(LOG_PATH, "r");
    if (probe)
    {
      fseek(probe, 0, SEEK_END);
      if (ftell(probe) == 0) isNew = true;
      fclose(probe);
    }
    else isNew = true;
  }

  s_log = fopen(LOG_PATH, "a");
  if (!s_log)
  {
    s_fs.unmount();
    return false;
  }

  if (isNew)
  {
    fprintf(s_log, "%s\n", LOG_HEADER);
    fflush(s_log);
    // Commit the header (and its FAT directory entry) to flash — see the note in
    // storage_append_event about why fflush alone is not durable.
    fclose(s_log);
    s_log = fopen(LOG_PATH, "a");
    if (!s_log) { s_fs.unmount(); return false; }
  }

  s_ready = true;
  return true;
}

bool storage_append_mark(const char *tag, const char *kind, double lat, double lon,
                         uint32_t utc, uint32_t sats, double hdop)
{
  if (!s_ready || !s_log) return false;

  // Same column count as a detection row; rssi/channel are 0 and the method
  // column carries `kind` (breadcrumb type or reset cause).
  fprintf(s_log, "%s,%s,0,0,%.6f,%.6f,%lu,%lu,%.2f,\"\"\n",
          tag ? tag : "#?", kind ? kind : "mark", lat, lon,
          (unsigned long)utc, (unsigned long)sats, hdop);

  fflush(s_log);
  fclose(s_log);                       // commit (see storage_append_event)
  s_log = fopen(LOG_PATH, "a");
  return (s_log != nullptr);
}

// ---- Device-table snapshot (survives reboot so hit counts persist) ----------
// Binary blob at /fs/fy_table.bin: header + raw DeviceEntry array.  recSize guards
// against a struct-layout change (a mismatched file is ignored, starts fresh).
//
// WRITTEN ATOMICALLY.  The old code opened SNAP_PATH with "wb", which truncates
// the file the instant it opens — so losing power mid-save (ignition off at the
// wrong moment) left a truncated snapshot, the next boot failed to load it, and
// the counts silently restarted at zero.  That happened in the field ~2026-09-10.
// Now we write a temp file, commit it, and only then swap it into place, so an
// interruption at ANY point leaves at least one loadable snapshot:
//   during temp write        -> SNAP_PATH still intact
//   after temp, before swap  -> SNAP_PATH still intact
//   between remove + rename  -> SNAP_TMP is valid; load falls back to it
#define SNAP_PATH  "/fs/fy_table.bin"
#define SNAP_TMP   "/fs/fy_table.tmp"
struct SnapHeader { char magic[4]; uint16_t ver; uint16_t recSize; uint32_t count; uint32_t totalEvents; };

void storage_save_table()
{
  if (!s_ready) return;

  FILE *f = fopen(SNAP_TMP, "wb");
  if (!f) return;

  SnapHeader h;
  memcpy(h.magic, "FYG1", 4);
  h.ver = 1;
  h.recSize = (uint16_t)sizeof(DeviceEntry);
  h.count = (uint32_t)g_devCount;
  h.totalEvents = g_totalEvents;

  bool ok = (fwrite(&h, sizeof(h), 1, f) == 1);
  if (ok && g_devCount > 0)
    ok = (fwrite(g_dev, sizeof(DeviceEntry), g_devCount, f) == (size_t)g_devCount);
  fflush(f);
  if (fclose(f) != 0) ok = false;          // close = commit to flash

  if (!ok)
  {
    remove(SNAP_TMP);                      // keep the previous good snapshot
    return;
  }

  // Swap into place.  If the rename fails the temp file is still a complete
  // snapshot and storage_load_table() will fall back to it.
  remove(SNAP_PATH);
  rename(SNAP_TMP, SNAP_PATH);
}

// Load one snapshot file.  Returns false if absent, truncated or mismatched.
static bool loadSnapshotFrom(const char *path)
{
  FILE *f = fopen(path, "rb");
  if (!f) return false;
  SnapHeader h;
  if (fread(&h, sizeof(h), 1, f) != 1 ||
      memcmp(h.magic, "FYG1", 4) != 0 ||
      h.recSize != sizeof(DeviceEntry) ||
      h.count > MAX_DEVICES)
  {
    fclose(f);
    return false;
  }
  size_t n = fread(g_dev, sizeof(DeviceEntry), h.count, f);
  fclose(f);
  if (n == 0) return false;
  g_devCount    = (int)n;
  g_totalEvents = h.totalEvents;

  // millis() restarts at 0 on boot, so stored timestamps are meaningless now —
  // zero them so restored devices read as "old" (not falsely just-seen) and the
  // keepalive re-logs them cleanly when next sighted.
  for (int i = 0; i < g_devCount; i++)
  {
    g_dev[i].firstSeenMs = 0;
    g_dev[i].lastSeenMs  = 0;
    g_dev[i].lastEventMs = 0;
  }
  return true;
}

bool storage_load_table()
{
  if (!s_ready) return false;
  if (loadSnapshotFrom(SNAP_PATH)) return true;
  // Primary missing/corrupt: recover from an interrupted save.
  if (loadSnapshotFrom(SNAP_TMP))
  {
    Serial.println("[giga] snapshot recovered from interrupted save");
    return true;
  }
  return false;
}

void storage_dump_csv()
{
  if (!s_ready)
  {
    Serial.println("[log DISABLED — QSPI user partition not formatted; run QSPIFormat]");
    return;
  }
  // Close the append handle so we read a fully-committed on-disk view (mbed FAT
  // doesn't always expose an open write handle's bytes to a second read handle).
  if (s_log) { fclose(s_log); s_log = nullptr; }

  FILE *f = fopen(LOG_PATH, "r");
  if (!f)
  {
    Serial.println("[no CSV file yet — nothing has been logged]");
    s_log = fopen(LOG_PATH, "a");      // reopen for continued logging
    return;
  }
  fseek(f, 0, SEEK_END);
  long sz = ftell(f);
  fseek(f, 0, SEEK_SET);
  Serial.print("----- BEGIN detections.csv ("); Serial.print(sz); Serial.println(" bytes) -----");
  char line[256];
  while (fgets(line, sizeof(line), f)) Serial.print(line);
  fclose(f);
  Serial.println("----- END detections.csv -----");

  s_log = fopen(LOG_PATH, "a");        // reopen for continued logging
}

bool storage_append_event(const char *mac, const char *method, int rssi,
                          int channel, double lat, double lon, uint32_t utc,
                          uint32_t sats, double hdop, const char *ssid)
{
  if (!s_ready || !s_log) return false;

  // Columns: mac,method,rssi,channel,lat,lon,utc,sats,hdop,ssid
  // ssid is quoted so commas/spaces in it never break the CSV.
  fprintf(s_log, "%s,%s,%d,%d,%.6f,%.6f,%lu,%lu,%.2f,\"%s\"\n",
          mac ? mac : "",
          method ? method : "",
          rssi, channel, lat, lon,
          (unsigned long)utc, (unsigned long)sats, hdop,
          ssid ? ssid : "");

  // Durability: fflush pushes the stdio buffer to the file layer but does NOT
  // update the FAT directory entry, so after a reboot/power-loss the file would
  // read as 0 bytes.  close+reopen commits the row (data + dir entry) to flash.
  // Detections are sparse, so the per-append open/close cost is negligible.
  fflush(s_log);
  fclose(s_log);
  s_log = fopen(LOG_PATH, "a");
  return (s_log != nullptr);
}
