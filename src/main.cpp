/*
 * ============================================================
 *  M5StickC Plus2  +  SST LuminOx LOX-02 Oxygen Sensor
 * ============================================================
 *
 *  WIRING – Grove / HY2.0-4P port on the M5StickC Plus2:
 *  ┌─────────────────────────────────────────────────────────┐
 *  │  Grove Connector   MCU GPIO    Sensor Pin   Signal       │
 *  │  Black  (GND)   -> GND      -> Pin 2        GND (0V)     │
 *  │  Red    (5V)    -> 5V       -> Pin 1        Vs (+5V)     │
 *  │  Yellow         -> G32 (RX) -> Pin 3        Sensor TX    │
 *  │  White          -> G33 (TX) -> Pin 4        Sensor RX    │
 *  └─────────────────────────────────────────────────────────┘
 *  IMPORTANT: Power (pins 1 & 2) must be applied BEFORE
 *             communicating on pins 3 & 4 (per datasheet).
 *
 *  UART settings (LOX-02):  9600 baud, 8N1, no flow control
 *
 *  Stream format from sensor (default on power-up, ~1 Hz):
 *    "O xxxx.x T yxx.x P xxxx % xxx.xx e xxxx\r\n"
 *     O = ppO2 (mbar), T = temperature (°C),
 *     P = pressure (mbar), % = O2 concentration (%),
 *     e = sensor status (0000 = good)
 *
 *  Display: 135 x 240 ST7789V2, landscape (240 x 135).
 *  Rendering uses a full-screen M5Canvas sprite pushed in one
 *  shot → zero flicker.
 * ============================================================
 */

#include <Arduino.h>
#include <M5Unified.h>

// ── Grove / Serial2 pin assignments ──────────────────────────
#define GROVE_RX_PIN  32   // Yellow wire: Sensor TX  → ESP32 RX
#define GROVE_TX_PIN  33   // White  wire: ESP32 TX   → Sensor RX
#define SENSOR_BAUD   9600

// ── Sensor UART ───────────────────────────────────────────────
#define SensorSerial  Serial2

// ── Display geometry (landscape) ─────────────────────────────
static const int DISP_W = 240;
static const int DISP_H = 135;

// ── Full-screen sprite (canvas) for flicker-free drawing ─────
M5Canvas canvas(&M5.Display);

// ── Sensor data struct ────────────────────────────────────────
struct SensorData {
  float ppO2_mbar   = 0.0f;   // Partial O2 pressure (mbar)
  float O2_pct      = 0.0f;   // O2 concentration (%)
  float temp_C      = 0.0f;   // Temperature (°C)
  float pressure_mb = 0.0f;   // Barometric pressure (mbar)
  int   status      = -1;     // Sensor status (-1 = no data yet)
  bool  valid       = false;  // True once first reading received
};

SensorData sensorData;

// ── Serial receive buffer ─────────────────────────────────────
static char rxBuf[128];
static int  rxIdx = 0;

// ── Colours (RGB565) ─────────────────────────────────────────
static const uint32_t COL_BG       = 0x0A0A1A;   // Near-black blue
static const uint32_t COL_ACCENT   = 0x00E5FF;   // Cyan accent
static const uint32_t COL_O2_GOOD  = 0x00FF88;   // Green  (>18 %)
static const uint32_t COL_O2_WARN  = 0xFFAA00;   // Amber  (16–18 %)
static const uint32_t COL_O2_ALERT = 0xFF3030;   // Red    (<16 %)
static const uint32_t COL_LABEL    = 0x8899BB;   // Muted blue-grey
static const uint32_t COL_VALUE    = 0xFFFFFF;   // White
static const uint32_t COL_UNIT     = 0xAABBCC;   // Light grey
static const uint32_t COL_STATUS_OK   = 0x00FF88;
static const uint32_t COL_STATUS_ERR  = 0xFF3030;
static const uint32_t COL_DIVIDER  = 0x223355;

// ── Helpers ───────────────────────────────────────────────────
// Return an RGB565 colour for O2 percentage level
static uint32_t o2Colour(float pct) {
  if (pct < 16.0f) return COL_O2_ALERT;
  if (pct < 18.0f) return COL_O2_WARN;
  return COL_O2_GOOD;
}

// Draw a horizontal divider line
static void drawDivider(int y) {
  canvas.drawFastHLine(8, y, DISP_W - 16, COL_DIVIDER);
}

// Draw a labelled value row:
//   [label]    [value][unit]
static void drawRow(int y, const char* label, const char* value,
                    const char* unit, uint32_t valueColour = COL_VALUE)
{
  canvas.setTextColor(COL_LABEL);
  canvas.setTextSize(1);
  canvas.drawString(label, 10, y);

  canvas.setTextColor(valueColour);
  canvas.setTextSize(2);
  canvas.drawString(value, 120, y - 4);

  canvas.setTextColor(COL_UNIT);
  canvas.setTextSize(1);
  canvas.drawString(unit, 210, y + 2);
}

// ── Parse one complete stream line from LOX-02 ───────────────
//  Format: "O xxxx.x T yxx.x P xxxx % xxx.xx e xxxx\r\n"
static bool parseLine(const char* line) {
  float ppO2 = 0, temp = 0, pres = 0, pct = 0;
  int   status = 0;

  // sscanf expects exactly the sensor format
  // 'T' field uses sign character before value: "T +25.3" or "T -05.1"
  // We parse the whole line with individual sscanf calls to be robust.

  const char* p;

  // ppO2  – 'O' field
  p = strstr(line, "O ");
  if (!p) return false;
  if (sscanf(p, "O %f", &ppO2) != 1) return false;

  // temperature – 'T' field (sign character included in float parsing)
  p = strstr(line, " T ");
  if (!p) return false;
  if (sscanf(p, " T %f", &temp) != 1) return false;

  // pressure – 'P' field (may be "----" if no barometer → skip)
  p = strstr(line, " P ");
  if (p) {
    if (sscanf(p, " P %f", &pres) != 1) pres = 0.0f;
  }

  // O2 % – '%' field (may be "------" if no barometer → skip)
  p = strstr(line, " % ");
  if (p) {
    if (sscanf(p, " %% %f", &pct) != 1) pct = 0.0f;
  }

  // status – 'e' field
  p = strstr(line, " e ");
  if (p) {
    if (sscanf(p, " e %d", &status) != 1) status = -1;
  }

  sensorData.ppO2_mbar   = ppO2;
  sensorData.temp_C      = temp;
  sensorData.pressure_mb = pres;
  sensorData.O2_pct      = pct;
  sensorData.status      = status;
  sensorData.valid       = true;
  return true;
}

// ── Read sensor serial, accumulate chars, parse on '\n' ──────
static void readSensor() {
  while (SensorSerial.available()) {
    char c = (char)SensorSerial.read();
    if (c == '\n') {
      rxBuf[rxIdx] = '\0';
      parseLine(rxBuf);
      rxIdx = 0;
    } else if (c != '\r') {
      if (rxIdx < (int)(sizeof(rxBuf) - 1)) {
        rxBuf[rxIdx++] = c;
      }
    }
  }
}

// ── Draw the full UI onto the canvas, then push to display ───
static void drawDisplay() {
  canvas.fillSprite(COL_BG);

  // ── Title bar ────────────────────────────────────────────────
  //canvas.fillRect(0, 0, DISP_W, 22, 0x112244);
  //canvas.setTextColor(COL_ACCENT);
  //canvas.setTextSize(1);
  //canvas.setFont(&fonts::FreeSans9pt7b);
  //canvas.drawString("LuminOx LOX-02", 8, 5);

  // Status dot + text (top-right)
  bool statusOK = (sensorData.status == 0);
  uint32_t stColour = sensorData.valid
                        ? (statusOK ? COL_STATUS_OK : COL_STATUS_ERR)
                        : COL_LABEL;
  canvas.fillCircle(DISP_W - 14, 11, 5, stColour);
  canvas.setTextColor(stColour);
  canvas.setFont(&fonts::Font0);
  canvas.setTextSize(1);
  const char* stText = !sensorData.valid ? "WAIT"
                        : (statusOK       ? "OK"  : "ERR");
  canvas.drawString(stText, DISP_W - 44, 7); // 3 chars wide, right-aligned with status dot

  // ── If no data yet, show waiting message ─────────────────────
  if (!sensorData.valid) {
    canvas.setFont(&fonts::FreeSans9pt7b);
    canvas.setTextColor(COL_LABEL);
    canvas.setTextDatum(middle_center);
    canvas.drawString("Waiting for sensor...", DISP_W / 2, DISP_H / 2);
    canvas.setTextDatum(top_left);
    canvas.pushSprite(0, 0);
    return;
  }

  canvas.setTextDatum(top_left); // reset text datum in case it was changed for waiting message

  // ── Big O2 percentage display (hero value) ───────────────────
  uint32_t o2Col = o2Colour(sensorData.O2_pct);
  canvas.setFont(&fonts::FreeSansBold24pt7b);
  canvas.setTextColor(o2Col);
  char buf[32];
  snprintf(buf, sizeof(buf), "%.2f", sensorData.O2_pct);
  canvas.drawString(buf, 8, 6); // Value (8, 26)

  // "%" unit label next to big number
  canvas.setFont(&fonts::FreeSans9pt7b);
  canvas.setTextColor(o2Col);
  canvas.drawString("%", 130, 30);

  // O2 label below
  canvas.setFont(&fonts::Font0);
  canvas.setTextColor(COL_LABEL);
  canvas.setTextSize(1);
  canvas.drawString("O2 Concentration", 8, 52); // Label below big number 68

  drawDivider(65); // horizontal divider below O2 concentration 78

  // ── Three data columns below the divider ─────────────────────
  //   ppO2 | Pressure | Temperature
  int rowY = 70; // Y coordinate of the first row (ppO2), subsequent rows are offset by +40 84
  canvas.setFont(&fonts::FreeSans9pt7b);

  // ppO2 (mbar)
  canvas.setTextColor(COL_LABEL);
  canvas.drawString("ppO2", 8, rowY);
  canvas.setTextColor(COL_VALUE);
  snprintf(buf, sizeof(buf), "%.1f", sensorData.ppO2_mbar);
  canvas.drawString(buf, 8, rowY + 24); // Value below label 74 +14
  canvas.setTextColor(COL_UNIT);
  canvas.setFont(&fonts::Font0);
  canvas.setTextSize(1);
  canvas.drawString("mbar", 8, rowY + 48);

  // Vertical separator
  canvas.drawFastVLine(88, rowY, 40, COL_DIVIDER);

  // Pressure (mbar)
  canvas.setFont(&fonts::FreeSans9pt7b);
  canvas.setTextColor(COL_LABEL);
  canvas.drawString("Pres.", 96, rowY);
  canvas.setTextColor(COL_VALUE);
  snprintf(buf, sizeof(buf), "%4.0f", sensorData.pressure_mb);
  canvas.drawString(buf, 96, rowY + 24);
  canvas.setFont(&fonts::Font0);
  canvas.setTextColor(COL_UNIT);
  canvas.setTextSize(1);
  canvas.drawString("mbar", 96, rowY + 48);

  // Vertical separator
  canvas.drawFastVLine(168, rowY, 40, COL_DIVIDER);

  // Temperature
  canvas.setFont(&fonts::FreeSans9pt7b);
  canvas.setTextColor(COL_LABEL);
  canvas.drawString("Temp", 176, rowY);
  canvas.setTextColor(COL_VALUE);
  snprintf(buf, sizeof(buf), "%+.1f", sensorData.temp_C);
  canvas.drawString(buf, 176, rowY + 24);
  canvas.setFont(&fonts::Font0);
  canvas.setTextColor(COL_UNIT);
  canvas.setTextSize(1);
  canvas.drawString("\xB0" "C", 176, rowY + 48);   // °C

  // ── Push entire canvas to physical display in one shot ───────
  canvas.pushSprite(0, 0);
}

// ── Setup ─────────────────────────────────────────────────────
void setup() {
  // Initialise M5Unified (handles power-hold, IMU, display, etc.)
  auto cfg = M5.config();
  M5.begin(cfg);

  // Landscape orientation, brightness
  M5.Display.setRotation(1);        // 0 = portrait, 1 = landscape
  M5.Display.setBrightness(180);
  M5.Display.fillScreen(COL_BG);

  // Create the full-screen sprite
  canvas.setColorDepth(16);
  canvas.createSprite(DISP_W, DISP_H);
  canvas.setTextDatum(top_left);

  // Initialise sensor UART (Serial2)
  // Power is provided by the Grove 5 V rail before UART is used,
  // so this order is safe (power-up is instantaneous with Grove wiring).
  SensorSerial.begin(SENSOR_BAUD, SERIAL_8N1, GROVE_RX_PIN, GROVE_TX_PIN);

  // Send "M 0\r\n" to ensure stream mode is active
  // (sensor defaults to streaming on power-up, but be explicit)
  delay(500);   // give sensor a moment to power up and start streaming
  SensorSerial.print("M 0\r\n");

  // Initial "waiting" screen
  drawDisplay();
}

// ── Loop ──────────────────────────────────────────────────────
void loop() {
  M5.update();   // process button events, keep power-hold active

  readSensor();  // collect chars from Serial2 and parse complete lines
  drawDisplay(); // redraw canvas and push to display every loop iteration

  // No delay() – we poll as fast as possible.
  // The sensor only sends a new line ~once per second, so the display
  // effectively updates at sensor cadence without busy-blocking.
}