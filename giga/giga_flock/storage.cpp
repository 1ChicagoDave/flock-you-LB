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
    s_fs.unmount();
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
