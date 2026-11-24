#include "SensorMesh.h"

#ifdef DISPLAY_CLASS
  #include "UITask.h"
  static UITask ui_task(display);
#endif

/* ----------------------------------------------------------------------------
 * Optional MSA311 tamper (movement) detector, on the sensor I2C bus (Wire1).
 *
 * The accelerometer's own 'active' (slope) interrupt engine watches for movement,
 * so nothing is missed between reads and the MCU can stay in light sleep. Its INT
 * pin is configured latched + active-high and wired to MSA311_INT_PIN, which is
 * also registered as a light-sleep wake source.
 *
 * Once triggered, the tamper alert stays latched until cleared with the
 * 'tamper.clear' command (from the phone, via the admin CLI).
 * ------------------------------------------------------------------------- */
#if ENV_INCLUDE_MSA311
  #include <Adafruit_MSA301.h>

  #ifndef MSA311_INT_PIN
    #error "MSA311_INT_PIN must be defined (GPIO wired to the MSA311 INT pin)"
  #endif
  #ifndef TAMPER_DEFAULT_THRESH_G
    #define TAMPER_DEFAULT_THRESH_G   0.25f   // movement threshold in g (change between samples)
  #endif

  // MSA311 registers not exposed by the Adafruit library (see MSA311 datasheet)
  #define MSA311_REG_INT_CONFIG   0x20   // bit1: INT1 open-drain, bit0: INT1 active-high
  #define MSA311_REG_INT_LATCH    0x21   // bit7: reset latched INTs, bits[3:0]: latch mode
  #define MSA311_REG_ACTIVE_DUR   0x27   // bits[1:0]: active duration = (n+1) ms
  #define MSA311_REG_ACTIVE_TH    0x28   // threshold, LSB = 3.91 mg at +/-2g range
  #define MSA311_INT_LATCHED      0x07
  #define MSA311_INT_RESET        0x80
  #define MSA311_TH_LSB_G_2G      0.00391f

  static Adafruit_MSA311 msa;
  static bool msa_ok = false;
  static float tamper_thresh_g = TAMPER_DEFAULT_THRESH_G;

  static void msaWriteReg(uint8_t reg, uint8_t val) {
    Wire1.beginTransmission(MSA311_I2CADDR_DEFAULT);
    Wire1.write(reg);
    Wire1.write(val);
    Wire1.endTransmission();
  }

  static uint8_t msaReadReg(uint8_t reg) {
    Wire1.beginTransmission(MSA311_I2CADDR_DEFAULT);
    Wire1.write(reg);
    if (Wire1.endTransmission(false) != 0) return 0xFF;
    if (Wire1.requestFrom((uint8_t)MSA311_I2CADDR_DEFAULT, (uint8_t)1) != 1) return 0xFF;
    return Wire1.read();
  }

  static void msaSetThreshold(float g) {
    int th = (int)(g / MSA311_TH_LSB_G_2G + 0.5f);
    msaWriteReg(MSA311_REG_ACTIVE_TH, (uint8_t)constrain(th, 1, 255));
  }

  static void msaResetInt() {
    msaWriteReg(MSA311_REG_INT_LATCH, MSA311_INT_RESET | MSA311_INT_LATCHED);
  }

  static bool msaInit() {
    pinMode(MSA311_INT_PIN, INPUT_PULLDOWN);
    if (!msa.begin(MSA311_I2CADDR_DEFAULT, &Wire1)) return false;

    msa.setPowerMode(MSA301_NORMALMODE);
    msa.setDataRate(MSA301_DATARATE_62_5_HZ);    // plenty for a knock/lift, far less power than 500Hz
    msa.setBandwidth(MSA301_BANDWIDTH_31_25_HZ);
    msa.setRange(MSA301_RANGE_2_G);              // finest threshold resolution

    msaWriteReg(MSA311_REG_INT_CONFIG, 0x01);    // INT1 push-pull, active-high
    msaWriteReg(MSA311_REG_ACTIVE_DUR, 0x00);
    msaSetThreshold(tamper_thresh_g);
    msa.enableInterrupts(false, false, true, true, true);   // active (motion) on X, Y, Z
    msa.mapInterruptPin(false, false, true);                // route 'active' to INT1
    msaResetInt();                                          // latched mode, clear anything pending
    return true;
  }
#endif

class MyMesh : public SensorMesh {
public:
  MyMesh(mesh::MainBoard& board, mesh::Radio& radio, mesh::MillisecondClock& ms, mesh::RNG& rng, mesh::RTCClock& rtc, mesh::MeshTables& tables)
     : SensorMesh(board, radio, ms, rng, rtc, tables),
       battery_data(12*24, 5*60)    // 24 hours worth of battery data, every 5 minutes
  {
  }

#if ENV_INCLUDE_MSA311
  void onTamperDetected() {
    if (!tamper_latched) {
      tamper_latched = true;
      Serial.println("Tamper detected! Sending alert");
      alertIf(true, tamper_alert, HIGH_PRI_ALERT, "Hive tamper: movement detected!");   // send now, don't wait for next sensor read
    }
  }
#endif

protected:
  /* ========================== custom logic here ========================== */
  Trigger low_batt, critical_batt;
  TimeSeriesData  battery_data;
  bool low_batt_on = false, critical_batt_on = false;

#if ENV_INCLUDE_MSA311
  Trigger tamper_alert;
  bool tamper_latched = false;
#endif

  void onSensorDataRead() override {
    float batt_voltage = getVoltage(TELEM_CHANNEL_SELF);

    battery_data.recordData(getRTCClock(), batt_voltage);   // record battery

    // hysteresis, so readings hovering around a threshold don't repeatedly raise/cancel alerts
    critical_batt_on = batt_voltage < (critical_batt_on ? 3.50f : 3.40f);
    low_batt_on      = batt_voltage < (low_batt_on      ? 3.70f : 3.60f);
    alertIf(critical_batt_on, critical_batt, HIGH_PRI_ALERT, "Battery is critical!");
    alertIf(low_batt_on, low_batt, LOW_PRI_ALERT, "Battery is low");

#if ENV_INCLUDE_MSA311
    alertIf(tamper_latched, tamper_alert, HIGH_PRI_ALERT, "Hive tamper: movement detected!");
#endif
  }

  int querySeriesData(uint32_t start_secs_ago, uint32_t end_secs_ago, MinMaxAvg dest[], int max_num) override {
    battery_data.calcMinMaxAvg(getRTCClock(), start_secs_ago, end_secs_ago, &dest[0], TELEM_CHANNEL_SELF, LPP_VOLTAGE);
    return 1;
  }

  bool handleCustomCommand(uint32_t sender_timestamp, char* command, char* reply) override {
    if (strcmp(command, "magic") == 0) {    // example 'custom' command handling
      strcpy(reply, "**Magic now done**");
      return true;   // handled
    }
#if ENV_INCLUDE_MSA311
    if (strcmp(command, "tamper") == 0) {
      sprintf(reply, "sensor:%s state:%s thresh:%.2fg", msa_ok ? "ok" : "MISSING",
              tamper_latched ? "TRIGGERED" : "armed", tamper_thresh_g);
      return true;
    }
    if (strcmp(command, "tamper.debug") == 0) {   // dump INT pin + key MSA311 registers + current accel
      if (!msa_ok) { strcpy(reply, "Err - sensor missing"); return true; }
      msa.read();
      sprintf(reply, "pin:%d st:%02X set0:%02X map0:%02X cfg:%02X lat:%02X th:%02X pwr:%02X odr:%02X g:%.2f,%.2f,%.2f",
              digitalRead(MSA311_INT_PIN), msaReadReg(MSA301_REG_MOTIONINT), msaReadReg(MSA301_REG_INTSET0),
              msaReadReg(MSA301_REG_INTMAP0), msaReadReg(MSA311_REG_INT_CONFIG), msaReadReg(MSA311_REG_INT_LATCH),
              msaReadReg(MSA311_REG_ACTIVE_TH), msaReadReg(MSA301_REG_POWERMODE), msaReadReg(MSA301_REG_ODR),
              msa.x_g, msa.y_g, msa.z_g);
      return true;
    }
    if (strcmp(command, "tamper.watch") == 0) {   // SERIAL ONLY: 10 sec live trace, shake the board while it runs
      if (sender_timestamp != 0) { strcpy(reply, "Err - serial console only"); return true; }
      if (!msa_ok) { strcpy(reply, "Err - sensor missing"); return true; }
      msaResetInt();
      float prev_mag = -1, max_delta = 0;
      uint8_t st_seen = 0;
      for (int i = 0; i < 200; i++) {
        msa.read();
        float mag = sqrtf(msa.x_g*msa.x_g + msa.y_g*msa.y_g + msa.z_g*msa.z_g);
        if (prev_mag >= 0 && fabsf(mag - prev_mag) > max_delta) max_delta = fabsf(mag - prev_mag);
        prev_mag = mag;
        uint8_t st = msaReadReg(MSA301_REG_MOTIONINT);
        st_seen |= st;
        if (i % 4 == 0) {   // print at 5 Hz, sample at 20 Hz
          Serial.printf("pin:%d st:%02X g:%6.2f %6.2f %6.2f |g|:%.2f\n",
                        digitalRead(MSA311_INT_PIN), st, msa.x_g, msa.y_g, msa.z_g, mag);
        }
        delay(50);
      }
      msaResetInt();   // don't let the test leave a latched INT behind
      sprintf(reply, "done - status bits seen:%02X  max |g| change:%.2f", st_seen, max_delta);
      return true;
    }
    if (strcmp(command, "tamper.clear") == 0) {
      tamper_latched = false;
      alertIf(false, tamper_alert, HIGH_PRI_ALERT, "");   // cancel/re-arm the alert
      if (msa_ok) msaResetInt();
      strcpy(reply, "OK - tamper re-armed");
      return true;
    }
    if (memcmp(command, "tamper.thresh ", 14) == 0) {    // format: tamper.thresh {g}  eg. tamper.thresh 0.3
      float g = atof(&command[14]);
      if (g < 0.01f || g > 0.99f) {
        strcpy(reply, "Err - range is 0.01 to 0.99 (g)");
      } else {
        tamper_thresh_g = g;
        if (msa_ok) msaSetThreshold(g);
        sprintf(reply, "OK - thresh %.2fg (until reboot)", g);
      }
      return true;
    }
#endif
    return false;  // not handled
  }
  /* ======================================================================= */
};

StdRNG fast_rng;
SimpleMeshTables tables;

MyMesh the_mesh(board, radio_driver, *new ArduinoMillis(), fast_rng, rtc_clock, tables);

#ifndef POWERSAVING_FIRSTSLEEP_SECS
  #define POWERSAVING_FIRSTSLEEP_SECS  120   // stay awake after boot, so serial CLI is usable
#endif

void halt() {
  while (1) ;
}

static char command[160];

void setup() {
  Serial.begin(115200);
  delay(1000);

  board.begin();

#ifdef HAS_EXTERNAL_WATCHDOG
  external_watchdog.begin();
#endif

#ifdef DISPLAY_CLASS
  if (display.begin()) {
    display.startFrame();
    display.print("Please wait...");
    display.endFrame();
  }
#endif

  if (!radio_init()) { halt(); }

  fast_rng.begin(radio_driver.getRngSeed());

  FILESYSTEM* fs;
#if defined(NRF52_PLATFORM) || defined(STM32_PLATFORM)
  InternalFS.begin();
  fs = &InternalFS;
  IdentityStore store(InternalFS, "");
#elif defined(ESP32)
  SPIFFS.begin(true);
  fs = &SPIFFS;
  IdentityStore store(SPIFFS, "/identity");
#elif defined(RP2040_PLATFORM)
  LittleFS.begin();
  fs = &LittleFS;
  IdentityStore store(LittleFS, "/identity");
  store.begin();
#else
  #error "need to define filesystem"
#endif
  if (!store.load("_main", the_mesh.self_id)) {
    MESH_DEBUG_PRINTLN("Generating new keypair");
    the_mesh.self_id = radio_new_identity();   // create new random identity
    int count = 0;
    while (count < 10 && (the_mesh.self_id.pub_key[0] == 0x00 || the_mesh.self_id.pub_key[0] == 0xFF)) {  // reserved id hashes
      the_mesh.self_id = radio_new_identity(); count++;
    }
    store.save("_main", the_mesh.self_id);
  }

  Serial.print("Sensor ID: ");
  mesh::Utils::printHex(Serial, the_mesh.self_id.pub_key, PUB_KEY_SIZE); Serial.println();

  command[0] = 0;

  sensors.begin();   // NOTE: also initialises Wire1 (ENV_PIN_SDA/SCL)

  the_mesh.begin(fs);

#if ENV_INCLUDE_MSA311
  msa_ok = msaInit();   // must be after sensors.begin()
  if (msa_ok) {
    Serial.println("MSA311 tamper sensor armed");
  #ifdef ESP32
    board.setExtraWakePin(MSA311_INT_PIN);
  #endif
  } else {
    Serial.println("MSA311 not found - tamper detection DISABLED");   // keep running: battery/telemetry still useful
  }
#endif

#ifdef DISPLAY_CLASS
  ui_task.begin(the_mesh.getNodePrefs(), FIRMWARE_BUILD_DATE, FIRMWARE_VERSION);
#endif

  // send out initial zero hop Advertisement to the mesh
#if ENABLE_ADVERT_ON_BOOT == 1
  the_mesh.sendSelfAdvertisement(16000, false);
#endif
}

void loop() {
  int len = strlen(command);
  // `command` must stay NUL-terminated within its bounds. If it ever isn't,
  // strlen() above can return >= sizeof(command) and the loop below would then
  // index past the buffer, so clamp defensively.
  if (len >= (int)sizeof(command)) {
    command[0] = 0;
    len = 0;
  }
  while (Serial.available() && len < sizeof(command)-1) {
    char c = Serial.read();
    if (c != '\n') {
      command[len++] = c;
      command[len] = 0;
    }
    Serial.print(c);
  }
  if (len == sizeof(command)-1) {  // buffer full: treat as a completed line
    command[sizeof(command)-2] = '\r';  // place end-of-line marker inside the buffer
    command[sizeof(command)-1] = 0;     // keep the buffer NUL-terminated
  }

  if (len > 0 && command[len - 1] == '\r') {  // received complete line
    command[len - 1] = 0;  // replace newline with C string null terminator
    char reply[160];
    the_mesh.handleCommand(NULL, 0, command, reply);  // NOTE: there is no sender_timestamp via serial!
    if (reply[0]) {
      Serial.print("  -> "); Serial.println(reply);
    }

    command[0] = 0;  // reset command buffer
  }

#if ENV_INCLUDE_MSA311
  // Latched INT from accelerometer. Only the 'active' (motion) interrupt is mapped to INT1, so the pin
  // itself is the evidence: the MOTION_INT status register is NOT latched (reads 00 most of the time).
  if (msa_ok && digitalRead(MSA311_INT_PIN) == HIGH) {
    msaResetInt();   // release INT pin, so we can sleep again
    the_mesh.onTamperDetected();
  }
#endif

  board.loop();   // let the board feed its watchdog, run periodic housekeeping

  the_mesh.loop();
  sensors.loop();
#ifdef DISPLAY_CLASS
  ui_task.loop();
#endif
  rtc_clock.tick();
#ifdef HAS_EXTERNAL_WATCHDOG
  external_watchdog.loop();
#endif

  if (the_mesh.getNodePrefs()->powersaving_enabled && !the_mesh.hasPendingWork()) {
#if defined(NRF52_PLATFORM)
    board.sleep(0); // nrf ignores seconds param, sleeps whenever possible
#else
    if (the_mesh.millisHasNowPassed(POWERSAVING_FIRSTSLEEP_SECS * 1000)) {
      board.sleep(30); // light sleep; wakes on LoRa packet, tamper INT, or timer (for scheduled jobs)
    }
#endif
  }
}
