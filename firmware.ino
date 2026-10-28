/*
  firmware.ino - open-loop switched-capacitor converter (x1/2, x1, x2)

  - Plain Arduino API only: pinMode, digitalWrite, micros, delayMicroseconds, Serial.
  - No #include, no libraries. Written for ESP32 (Arduino-ESP32 core).
  - Open loop: the output voltage is set by the ratio (mode), NOT regulated.
    freq / duty / dead-time only change output impedance, ripple and losses.

  Serial: 115200 baud, newline terminated. Type "help".
*/

// ============================== USER SETTINGS ==============================
#define SERIAL_BAUD 115200

// PNP switches (S1, S2A, S2B, S3) are driven through a BJT push-pull driver
// that needs a complementary pair per switch:
//   IN  = HIGH -> switch ON      INB = always the inverse of IN
// Index: 0 = S1, 1 = S2A, 2 = S2B, 3 = S3
static const uint8_t PIN_IN [4] = {16, 18, 21, 23};
static const uint8_t PIN_INB[4] = {17, 19, 22, 25};
// S4 (2N3904, low side) is driven from the GPIO through a 1k base resistor (HIGH = ON)
static const uint8_t PIN_S4 = 26;

// Converter modes
static const uint8_t MODE_HALF   = 0;  // x1/2  step-down
static const uint8_t MODE_ONE    = 1;  // x1    bypass
static const uint8_t MODE_DOUBLE = 2;  // x2    step-up

// Defaults at boot (all can be changed at run time over Serial)
static const uint8_t  DEFAULT_MODE     = MODE_DOUBLE;
static const uint32_t DEFAULT_FREQ_HZ  = 1000;  // switching frequency
static const uint32_t DEFAULT_DEAD_US  = 10;    // dead time between phases (BJT storage time)
static const uint8_t  DEFAULT_DUTY_PCT = 50;    // share of phase 1 in the active time
static const bool     AUTO_START       = false; // false = wait for "on" command

// Limits enforced by the Serial parser
static const uint32_t FREQ_MIN_HZ  = 100;
static const uint32_t FREQ_MAX_HZ  = 5000;
static const uint32_t DEAD_MAX_US  = 100;
static const uint8_t  DUTY_MIN_PCT = 10;
static const uint8_t  DUTY_MAX_PCT = 90;
// ===========================================================================

// Switch masks (bit i of the P-type switches == index in PIN_IN)
static const uint8_t M_S1 = 1, M_S2A = 2, M_S2B = 4, M_S3 = 8, M_S4 = 16;

// Run-time state
static uint8_t  curMode  = DEFAULT_MODE;
static uint32_t freqHz   = DEFAULT_FREQ_HZ;
static uint32_t deadUs   = DEFAULT_DEAD_US;
static uint8_t  dutyPct  = DEFAULT_DUTY_PCT;
static bool     running  = false;

static uint32_t periodUs = 0, t1Us = 0, t2Us = 0;
static uint8_t  phase = 0;                 // 0=P1, 1=dead, 2=P2, 3=dead
static uint32_t phaseStartUs = 0, phaseLenUs = 0;

// ------------------------------ switch drivers ------------------------------
// Never drive IN=1 and INB=1 together: both push-pull halves would fight.
static void setPSwitch(uint8_t i, bool on) {
  if (on) {
    digitalWrite(PIN_INB[i], LOW);
    digitalWrite(PIN_IN[i], HIGH);
  } else {
    digitalWrite(PIN_IN[i], LOW);
    digitalWrite(PIN_INB[i], HIGH);
  }
}

// Break-before-make: first switch off what is not wanted, then switch on.
static void applySwitches(uint8_t mask) {
  for (uint8_t i = 0; i < 4; i++) {
    if (!(mask & (1 << i))) setPSwitch(i, false);
  }
  if (!(mask & M_S4)) digitalWrite(PIN_S4, LOW);
  for (uint8_t i = 0; i < 4; i++) {
    if (mask & (1 << i)) setPSwitch(i, true);
  }
  if (mask & M_S4) digitalWrite(PIN_S4, HIGH);
}

static uint8_t phaseMask(uint8_t m, uint8_t ph) {
  switch (m) {
    case MODE_DOUBLE: return ph == 0 ? (M_S1 | M_S4)  : (M_S3 | M_S2A);
    case MODE_HALF:   return ph == 0 ? (M_S1 | M_S2B) : (M_S2A | M_S4);
    case MODE_ONE:    return M_S1 | M_S2A;
    default:          return 0;
  }
}

// ------------------------------- timing engine ------------------------------
static bool timingOk(uint32_t f, uint32_t d) {
  return (2UL * d) <= (1000000UL / f) / 2UL;  // dead time <= period/4
}

static void recalcTiming() {
  periodUs = 1000000UL / freqHz;
  uint32_t active = periodUs - 2UL * deadUs;
  t1Us = (active * dutyPct) / 100UL;
  t2Us = active - t1Us;
}

static void startEngine() {
  applySwitches(0);
  delayMicroseconds(deadUs > 5 ? deadUs : 5);
  if (curMode == MODE_ONE) {            // static bypass, no switching
    applySwitches(phaseMask(MODE_ONE, 0));
    return;
  }
  phase = 3;
  phaseLenUs = 0;
  phaseStartUs = micros();
}

static void stopEngine() {
  running = false;
  applySwitches(0);
}

static void engineStep() {
  if (!running || curMode == MODE_ONE) return;
  uint32_t now = micros();
  if ((uint32_t)(now - phaseStartUs) < phaseLenUs) return;

  phaseStartUs += phaseLenUs;
  if ((uint32_t)(now - phaseStartUs) > periodUs) phaseStartUs = now;  // fell behind

  phase = (phase + 1) & 3;
  switch (phase) {
    case 0:  applySwitches(phaseMask(curMode, 0)); phaseLenUs = t1Us;   break;
    case 1:  applySwitches(0);                     phaseLenUs = deadUs; break;
    case 2:  applySwitches(phaseMask(curMode, 1)); phaseLenUs = t2Us;   break;
    default: applySwitches(0);                     phaseLenUs = deadUs; break;
  }
}

// --------------------------------- Serial UI --------------------------------
static const char *modeName(uint8_t m) {
  return m == MODE_HALF ? "x1/2 (step-down)"
       : m == MODE_ONE  ? "x1 (bypass)"
                        : "x2 (step-up)";
}

static void printStatus() {
  Serial.print("state : "); Serial.println(running ? "RUNNING" : "STOPPED");
  Serial.print("mode  : "); Serial.println(modeName(curMode));
  Serial.print("freq  : "); Serial.print(freqHz);  Serial.println(" Hz");
  Serial.print("dead  : "); Serial.print(deadUs);  Serial.println(" us");
  Serial.print("duty  : "); Serial.print(dutyPct); Serial.println(" % (phase 1 share)");
  Serial.print("timing: period="); Serial.print(periodUs);
  Serial.print(" us, P1="); Serial.print(t1Us);
  Serial.print(" us, P2="); Serial.print(t2Us); Serial.println(" us");
}

static void printHelp() {
  Serial.println("Commands:");
  Serial.println("  mode half|one|double   (also 0.5 | 1 | 2)");
  Serial.println("  freq <Hz>              switching frequency");
  Serial.println("  dead <us>              dead time between phases");
  Serial.println("  duty <pct>             phase 1 share of active time");
  Serial.println("  on | off               start / stop (off = all switches open)");
  Serial.println("  status | help");
}

static void lowerInPlace(char *s) {
  for (; *s; s++) if (*s >= 'A' && *s <= 'Z') *s = (char)(*s + 32);
}

static void handleLine(char *line) {
  char *cmd = strtok(line, " \t");
  char *arg = strtok(NULL, " \t");
  if (!cmd) return;
  lowerInPlace(cmd);
  if (arg) lowerInPlace(arg);

  if (!strcmp(cmd, "help")) {
    printHelp();
  } else if (!strcmp(cmd, "status")) {
    printStatus();
  } else if (!strcmp(cmd, "on")) {
    running = true;
    startEngine();
    Serial.println("OK running");
  } else if (!strcmp(cmd, "off")) {
    stopEngine();
    Serial.println("OK stopped, all switches open");
  } else if (!strcmp(cmd, "mode")) {
    uint8_t m = 255;
    if (arg) {
      if (!strcmp(arg, "half") || !strcmp(arg, "0.5")) m = MODE_HALF;
      else if (!strcmp(arg, "one") || !strcmp(arg, "1")) m = MODE_ONE;
      else if (!strcmp(arg, "double") || !strcmp(arg, "2")) m = MODE_DOUBLE;
    }
    if (m == 255) {
      Serial.println("ERR usage: mode half|one|double");
    } else {
      curMode = m;
      if (running) startEngine();
      Serial.print("OK mode "); Serial.println(modeName(curMode));
    }
  } else if (!strcmp(cmd, "freq")) {
    long v = arg ? atol(arg) : 0;
    if (v < (long)FREQ_MIN_HZ || v > (long)FREQ_MAX_HZ) {
      Serial.print("ERR freq range "); Serial.print(FREQ_MIN_HZ);
      Serial.print(".."); Serial.println(FREQ_MAX_HZ);
    } else if (!timingOk((uint32_t)v, deadUs)) {
      Serial.println("ERR dead time too long for this frequency");
    } else {
      freqHz = (uint32_t)v;
      recalcTiming();
      Serial.print("OK freq "); Serial.println(freqHz);
    }
  } else if (!strcmp(cmd, "dead")) {
    long v = arg ? atol(arg) : -1;
    if (v < 0 || v > (long)DEAD_MAX_US) {
      Serial.print("ERR dead range 0.."); Serial.println(DEAD_MAX_US);
    } else if (!timingOk(freqHz, (uint32_t)v)) {
      Serial.println("ERR dead time too long for this frequency");
    } else {
      deadUs = (uint32_t)v;
      recalcTiming();
      Serial.print("OK dead "); Serial.println(deadUs);
    }
  } else if (!strcmp(cmd, "duty")) {
    long v = arg ? atol(arg) : 0;
    if (v < (long)DUTY_MIN_PCT || v > (long)DUTY_MAX_PCT) {
      Serial.print("ERR duty range "); Serial.print(DUTY_MIN_PCT);
      Serial.print(".."); Serial.println(DUTY_MAX_PCT);
    } else {
      dutyPct = (uint8_t)v;
      recalcTiming();
      Serial.print("OK duty "); Serial.println(dutyPct);
    }
  } else {
    Serial.println("ERR unknown command, type help");
  }
}

static char    lineBuf[32];
static uint8_t lineLen = 0;

static void pollSerial() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (lineLen > 0) {
        lineBuf[lineLen] = 0;
        handleLine(lineBuf);
        lineLen = 0;
      }
    } else if (lineLen < sizeof(lineBuf) - 1) {
      lineBuf[lineLen++] = c;
    }
  }
}

// ----------------------------------- main -----------------------------------
void setup() {
  // Safe state first: every switch OFF
  for (uint8_t i = 0; i < 4; i++) {
    pinMode(PIN_IN[i], OUTPUT);
    pinMode(PIN_INB[i], OUTPUT);
  }
  pinMode(PIN_S4, OUTPUT);
  applySwitches(0);

  Serial.begin(SERIAL_BAUD);
  recalcTiming();
  if (AUTO_START) {
    running = true;
    startEngine();
  }
  Serial.println();
  Serial.println("Switched-capacitor x1/2 | x1 | x2, open loop");
  printHelp();
  printStatus();
}

void loop() {
  engineStep();
  pollSerial();
}
