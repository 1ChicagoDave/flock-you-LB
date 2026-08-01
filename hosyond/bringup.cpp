// ============================================================================
// LCDWIKI E32R40T — hardware bring-up / pin-map validation
// ============================================================================
// NOT the detector. This is a throwaway self-test that exercises every onboard
// peripheral so we can confirm the (verified-on-paper) pin map against the real
// board BEFORE porting the Flock detector. Flash with:
//     pio run -e hosyond_e32r40t -t upload
// then open the serial monitor at 115200 and watch the screen.
//
// Serial keys:  c = re-run touch calibration   s = re-test the SD card
//
// Board: LCDWIKI E32R40T (ESP32-32E / ST7796S 320x480 / XPT2046 resistive /
// microSD / SC8002B speaker amp / RGB LED / Li-battery via TP4054).
// Pin map: official spec §4.2 + schematic. See docs / memory for the table.
// ----------------------------------------------------------------------------

#include <Arduino.h>
#include <Preferences.h> // persist touch calibration in NVS
#include <SPI.h>
#include <SD.h>
#include <TFT_eSPI.h> // display + XPT2046 touch (pins come from build_flags)
#include <TinyGPSPlus.h>

// Screen rotation: 0/2 = portrait (320x480), 1/3 = landscape (480x320).
#define SCREEN_ROTATION 0

// ---- Pins NOT owned by TFT_eSPI (those are set via build_flags) -------------
#define PIN_LED_R 22 // RGB LED, common anode: drive LOW to light
#define PIN_LED_G 16
#define PIN_LED_B 17

#define PIN_SD_CS 5 // microSD on VSPI (18 sck / 23 mosi / 19 miso)
#define PIN_SD_SCK 18
#define PIN_SD_MISO 19
#define PIN_SD_MOSI 23

#define PIN_AUDIO_DAC 26 // ESP32 DAC2 -> SC8002B AUDIO_IN
#define PIN_AUDIO_EN 4   // SC8002B shutdown/enable (pulled high = enabled)

#define PIN_BAT_ADC 34 // battery voltage via 100K/100K divider (x2), input-only

#define PIN_GPS_RX 25 // GPS on the I2C 4-pin connector, remapped as UART2.
#define PIN_GPS_TX 32 // GPS TX -> ESP32 IO25 (RX); ESP32 IO32 (TX) -> GPS RX

static TFT_eSPI tft = TFT_eSPI();
static TinyGPSPlus gps;
static SPIClass sdSPI(VSPI);
static Preferences prefs;

static bool sdOK = false;
static uint32_t gpsChars = 0;
static unsigned long lastStatus = 0;
static int screenW = 320, screenH = 480;

// Light one RGB color (common anode -> LOW = on). Pass 0/1 per channel.
static void rgb(bool r, bool g, bool b)
{
  digitalWrite(PIN_LED_R, r ? LOW : HIGH);
  digitalWrite(PIN_LED_G, g ? LOW : HIGH);
  digitalWrite(PIN_LED_B, b ? LOW : HIGH);
}

// Crude DAC tone through the SC8002B amp — proves the audio path + speaker.
static void beep(unsigned freqHz, unsigned ms)
{
  const unsigned halfUs = 500000UL / freqHz;
  const unsigned long end = millis() + ms;
  while (millis() < end)
  {
    dacWrite(PIN_AUDIO_DAC, 230);
    delayMicroseconds(halfUs);
    dacWrite(PIN_AUDIO_DAC, 25);
    delayMicroseconds(halfUs);
  }
  dacWrite(PIN_AUDIO_DAC, 0);
}

static float readBatteryVolts()
{
  // analogReadMilliVolts applies the eFuse ADC calibration; x2 for the divider.
  return (analogReadMilliVolts(PIN_BAT_ADC) * 2.0f) / 1000.0f;
}

static const char *sdTypeName(uint8_t t)
{
  switch (t)
  {
  case CARD_NONE:
    return "NONE (no card / not detected)";
  case CARD_MMC:
    return "MMC";
  case CARD_SD:
    return "SDSC";
  case CARD_SDHC:
    return "SDHC/SDXC";
  default:
    return "UNKNOWN";
  }
}

// Mount the SD on its own VSPI bus. The ESP32 SD lib only mounts FAT16/FAT32;
// a 64GB SDXC card is exFAT out of the box and WON'T mount (and a failed mount
// de-inits the card, so cardType then reads NONE — misleading). allowFormat
// reformats the card FAT32 in place (ERASES it) via format_if_mount_failed.
static void sdMount(bool allowFormat)
{
  tft.fillRect(0, 96, screenW, 44, TFT_BLACK);
  tft.setTextFont(2);
  tft.setCursor(6, 96);

  sdOK = false;
  if (allowFormat)
  {
    Serial.println("[SD] mount+format FAT32 @4MHz (ERASES the card)...");
    tft.setTextColor(TFT_ORANGE, TFT_BLACK);
    tft.print("SD: formatting FAT32...");
    sdOK = SD.begin(PIN_SD_CS, sdSPI, 4000000, "/sd", 5, true);
    tft.fillRect(0, 96, screenW, 44, TFT_BLACK);
    tft.setCursor(6, 96);
  }
  else
  {
    const uint32_t freqs[] = {20000000, 4000000, 1000000, 400000};
    for (uint8_t i = 0; i < 4 && !sdOK; i++)
      sdOK = SD.begin(PIN_SD_CS, sdSPI, freqs[i], "/sd", 5, false);
  }

  uint8_t ct = SD.cardType();
  if (sdOK)
  {
    uint64_t mb = SD.cardSize() / (1024ULL * 1024ULL);
    Serial.printf("[SD] mounted: %s, %llu MB\n", sdTypeName(ct), mb);
    tft.setTextColor(TFT_CYAN, TFT_BLACK);
    tft.printf("SD: OK  %s  %llu MB", sdTypeName(ct), mb);
  }
  else
  {
    Serial.printf("[SD] mount FAILED. cardType=%s\n", sdTypeName(ct));
    Serial.println("[SD]  64GB is exFAT by default -> press 'f' to format FAT32,");
    Serial.println("[SD]  or reformat FAT32 on PC (guiformat/Rufus, or <=32GB).");
    tft.setTextColor(TFT_RED, TFT_BLACK);
    tft.printf("SD: FAIL (%s)\n", sdTypeName(ct));
    tft.setTextColor(TFT_ORANGE, TFT_BLACK);
    tft.setCursor(6, 116);
    tft.print("press 'f' to format FAT32");
  }
}

// Touch calibration. Stored in NVS so it only runs once; press 'c' to redo.
static void touchCalibrate(bool force)
{
  uint16_t cal[5];
  prefs.begin("bringup", false);
  bool have = !force && prefs.getBytesLength("touchcal") == sizeof(cal);
  if (have)
  {
    prefs.getBytes("touchcal", cal, sizeof(cal));
    tft.setTouch(cal);
    Serial.printf("[touch] loaded cal: %u %u %u %u %u\n",
                  cal[0], cal[1], cal[2], cal[3], cal[4]);
  }
  else
  {
    tft.fillScreen(TFT_BLACK);
    tft.setTextFont(2);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.setCursor(10, 10);
    tft.println("Touch the arrow corners");
    tft.calibrateTouch(cal, TFT_MAGENTA, TFT_BLACK, 15);
    prefs.putBytes("touchcal", cal, sizeof(cal));
    Serial.printf("[touch] NEW cal (baked into app later): %u %u %u %u %u\n",
                  cal[0], cal[1], cal[2], cal[3], cal[4]);
  }
  prefs.end();
}

// Redraw the static header after a full-screen clear (calibration etc.).
static void drawHeader()
{
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_GREEN, TFT_BLACK);
  tft.setTextFont(4);
  tft.setCursor(6, 6);
  tft.println("E32R40T bring-up");
  tft.setTextFont(2);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setCursor(6, 40);
  tft.printf("ST7796S %dx%d  rot %d\n", screenW, screenH, SCREEN_ROTATION);
  tft.setTextColor(TFT_YELLOW, TFT_BLACK);
  tft.setCursor(6, 64);
  tft.println("touch=dots  keys: c=cal s=sd f=fmt");
}

void setup()
{
  Serial.begin(115200);
  delay(200);
  Serial.println("\n[E32R40T] bring-up self-test");

  // RGB LED — off (HIGH) then a quick R/G/B sweep so we can eyeball it.
  pinMode(PIN_LED_R, OUTPUT);
  pinMode(PIN_LED_G, OUTPUT);
  pinMode(PIN_LED_B, OUTPUT);
  rgb(1, 0, 0);
  delay(300);
  rgb(0, 1, 0);
  delay(300);
  rgb(0, 0, 1);
  delay(300);
  rgb(0, 0, 0);

  // Display
  tft.init();
  tft.setRotation(SCREEN_ROTATION);
  screenW = tft.width();
  screenH = tft.height();

  touchCalibrate(false); // may draw a full-screen calibration the first time
  drawHeader();

  // microSD on its own VSPI bus
  sdSPI.begin(PIN_SD_SCK, PIN_SD_MISO, PIN_SD_MOSI, PIN_SD_CS);
  sdMount(false);

  // GPS on a remapped UART (I2C connector pins 25/32)
  Serial2.begin(9600, SERIAL_8N1, PIN_GPS_RX, PIN_GPS_TX);
  Serial.println("[GPS] UART2 @ 9600 on RX=25 TX=32");

  // Audio: enable the amp and chirp twice
  pinMode(PIN_AUDIO_EN, OUTPUT);
  digitalWrite(PIN_AUDIO_EN, HIGH); // SC8002B: high = enabled (pulled up by R11)
  beep(2000, 120);
  delay(60);
  beep(2600, 120);

  Serial.println("[setup] done. Touch the screen; status prints below.");
}

void loop()
{
  // Serial commands: c=recalibrate touch  s=re-test SD  f=format SD FAT32
  if (Serial.available())
  {
    int c = Serial.read();
    if (c == 'c' || c == 'C')
    {
      touchCalibrate(true);
      drawHeader();
      sdMount(false);
    }
    else if (c == 's' || c == 'S')
      sdMount(false);
    else if (c == 'f' || c == 'F')
      sdMount(true); // ERASES the card, reformats FAT32
  }

  // Touch — draw a dot where pressed, print raw coords.
  uint16_t tx, ty;
  if (tft.getTouch(&tx, &ty))
  {
    tft.fillCircle(tx, ty, 3, TFT_MAGENTA);
    Serial.printf("[touch] x=%u y=%u\n", tx, ty);
    rgb(1, 1, 1); // white flash on touch
  }
  else
  {
    rgb(0, 0, 0);
  }

  // GPS — feed the parser and count bytes (proves the UART is wired right).
  while (Serial2.available())
  {
    char c = Serial2.read();
    gps.encode(c);
    gpsChars++;
  }

  // Once a second: battery + GPS summary to serial and a status band on screen.
  if (millis() - lastStatus > 1000)
  {
    lastStatus = millis();
    float vbat = readBatteryVolts();
    int sats = gps.satellites.isValid() ? (int)gps.satellites.value() : -1;

    Serial.printf("[status] Vbat=%.2fV  gpsChars=%lu  fix=%s  sats=%d\n",
                  vbat, gpsChars, gps.location.isValid() ? "yes" : "no", sats);

    tft.fillRect(0, screenH - 28, screenW, 28, TFT_NAVY);
    tft.setTextColor(TFT_WHITE, TFT_NAVY);
    tft.setTextFont(2);
    tft.setCursor(6, screenH - 24);
    tft.printf("Vbat %.2fV  GPS bytes %lu  sats %d", vbat, gpsChars, sats);
  }
}
