// ============================================================
// storage.cpp — QSPI flash CSV event log (Arduino_POSIXStorage)
// ============================================================
//
// We use Arduino's official Arduino_POSIXStorage library: it mounts the GIGA's
// 16 MB QSPI flash and exposes plain POSIX stdio (fopen/fprintf/fflush).  The
// QSPI FAT partition mounts at "/qspi", so our file lives at /qspi/detections.csv
// (see LOG_PATH in config.h).
//
// *** ONE-TIME FORMAT ***
// A brand-new GIGA (or one whose QSPI holds a non-FAT filesystem) will FAIL to
// mount until the QSPI is formatted FAT once.  Do it with the Arduino example
// sketch:  File > Examples > STM32H747_System > QSPIFormat  (choose the option
// that creates a FAT partition for user data).  After that this code mounts
// cleanly on every boot.  If the mount fails we simply run without logging —
// the detector, UI, audio and GPS all keep working; only the CSV is skipped.
//
// KNOWN-UNCERTAIN: the exact POSIXStorage enum names (DEV_QSPI / FS_FAT /
// MNT_DEFAULT) and the "/qspi" mount root are per the current library; verify
// against your installed Arduino_POSIXStorage version if mount() won't compile.

#include "storage.h"
#include "config.h"

#include <Arduino_POSIXStorage.h>
#include <stdio.h>

static FILE *s_log   = nullptr;
static bool  s_ready = false;

bool storage_ready() { return s_ready; }

bool storage_init()
{
  s_ready = false;
  s_log   = nullptr;

  // mount() returns 0 on success, non-zero (with errno set) on failure.
  int err = mount(DEV_QSPI, FS_FAT, MNT_DEFAULT);
  if (err != 0)
  {
    // Almost always "not formatted" — run without logging (see header note).
    return false;
  }

  // Is this a fresh file?  (No access()/stat() dependency — just probe read.)
  bool isNew = false;
  {
    FILE *probe = fopen(LOG_PATH, "r");
    if (probe) fclose(probe);
    else       isNew = true;
  }

  s_log = fopen(LOG_PATH, "a");
  if (!s_log)
  {
    umount(DEV_QSPI);
    return false;
  }

  if (isNew)
  {
    fprintf(s_log, "%s\n", LOG_HEADER);
    fflush(s_log);
  }

  s_ready = true;
  return true;
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

  // Flush every append so a power loss keeps the log intact.  If throughput
  // ever matters this can be batched (flush every N rows or on a timer).
  fflush(s_log);
  return true;
}
