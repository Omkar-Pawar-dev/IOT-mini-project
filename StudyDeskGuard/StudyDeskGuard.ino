/*
  Smart Study Desk Guard  -  ESP8266 (NodeMCU) + HC-SR04 + Blynk IoT
  ---------------------------------------------------------------------
  Wiring (as built):
    HC-SR04 TRIG -> D2 (GPIO4)      HC-SR04 ECHO -> 1k/2k divider -> D1 (GPIO5)
    Red LED  -> D5 (GPIO14)         Green LED -> D6 (GPIO12)
    Active buzzer -> D7 (GPIO13)    Onboard FLASH button = D3 (GPIO0)

  Libraries: "Blynk" by Volodymyr Shymanskyy (ESP8266 core already has LittleFS)
  Board:     NodeMCU 1.0 (ESP-12E), Flash size "4MB (FS:2MB OTA:~1019KB)"

  Blynk datastreams (create in your template):
    V0 Distance cm (Double)        V1 Status (String)
    V2 Continuous study min (Dbl)  V3 Session study min (Dbl)
    V4 Total study min (Dbl)       V5 Total sessions (Int)
    V6 Too-close events (Int)      V7 Normal distance cm (Double, 15-200)  [input]
    V8 "Set current as normal" (Int, push button)                          [input]
    V9 Study limit minutes (Int, 1-600)                                    [input]
    V10 Table widget (session log)
    V11 Connection info (String)
  Optional Blynk events: session_end, break_due
*/

// ======================= 1. USER CONFIG =======================
#define BLYNK_TEMPLATE_ID "TMPL3Mvu-tny0"
#define BLYNK_TEMPLATE_NAME "Smart Study Desk Guard"
#define BLYNK_AUTH_TOKEN "PEMhUR55uP7qhUYMwhx1sVY5d_WCd_RO"
#define BLYNK_PRINT Serial

#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <DNSServer.h>
#include <LittleFS.h>
#include <time.h>
#include <BlynkSimpleEsp8266.h>

// Fallback Wi-Fi (used if nothing was saved through the setup page; it is also
// tried alternately with the saved one). Leave SSID "" to skip.
const char* DEFAULT_SSID = "TECNO POVA 7 5G";
const char* DEFAULT_PASS = "@Omkarp1";

// Setup hotspot (password must be >= 8 chars, or "" for an open network)
const char* AP_SSID = "StudyDesk-Setup";
const char* AP_PASS = "studydesk";

const long TZ_OFFSET_S = 19800;               // IST (UTC+5:30)

// ---- Behaviour tuning ----
const float    CLOSE_FACTOR   = 2.0f / 3.0f;  // "too close" below 2/3 of normal
const float    GOOD_FACTOR    = 0.85f;        // "good" from 85 % of normal and farther
const float    AWAY_FACTOR    = 1.8f;         // farther than 1.8 x normal = nobody there
const uint32_t START_CONFIRM_MS  = 10000UL;   // must be present this long to start a session
const uint32_t RETURN_CONFIRM_MS = 5000UL;    // ...to come back from a break
const uint32_t AWAY_CONFIRM_MS   = 30000UL;   // away this long = break begins
const uint32_t ZONE_CONFIRM_MS   = 3000UL;    // good/close change must hold this long
const uint32_t SESSION_GAP_MS    = 600000UL;  // break < 10 min = same session
const uint32_t BREAK_RESET_MS    = 300000UL;  // a break >= 5 min resets the "continuous study" clock
const uint32_t GREEN_EVERY_MS    = 120000UL;  // green pulse every 2 min in good position
const uint32_t CLOSE_EVERY_MS    = 30000UL;   // too-close double alert every 30 s
const uint32_t SOLID_BEEP_MS     = 60000UL;   // beep while red is solid (0 = silent)
const uint32_t MIN_LOG_STUDY_S   = 60;        // sessions with less study time are not logged
const uint32_t OFFLINE_GRACE_MS  = 60000UL;   // offline this long before offline beeps start
const uint32_t OFFLINE_BEEP_MS   = 300000UL;  // one beep every 5 min while offline
const bool     OFFLINE_BEEP_ONLY_WHEN_PRESENT = true;
const bool     BUZZER_ACTIVE_HIGH = true;
const float    STATIONARY_BAND_CM = 0.8f;      // total spread allowed while "frozen"
const uint32_t STATIONARY_MS      = 150000UL;  // frozen this long = object, not a person (5 min)
const float    UNBLOCK_BAND_CM    = 2.0f;      // reading moves this far from the frozen value = cleared
const uint32_t REPLAY_COUNT = 10;               // sessions re-sent when you press the button

// ======================= 2. PINS =======================
const uint8_t PIN_TRIG = 4, PIN_ECHO = 5, PIN_RED = 14, PIN_GREEN = 12, PIN_BUZZ = 13, PIN_BTN = 0;

#define VP_DIST    V0
#define VP_STATUS  V1
#define VP_CONT    V2
#define VP_SESS    V3
#define VP_TOTAL   V4
#define VP_COUNT   V5
#define VP_CLOSE   V6
#define VP_NORMAL  V7
#define VP_SETBTN  V8
#define VP_LIMIT   V9
#define VP_TABLE   V10
#define VP_INFO    V11
#define VP_REPLAY  V12

// ======================= 3. STATE =======================
enum Zone : uint8_t { Z_NONE, Z_CLOSE, Z_MID, Z_GOOD };

float    normalCm = 60.0f;
uint16_t limitMin = 45;
float    closeLimit, goodMin, awayLimit;
String   cfgSsid = "", cfgPass = "";
bool     fsOk = false;

struct Totals { uint32_t studyS = 0, sessions = 0, closeCnt = 0, lastId = 0, lastSynced = 0; } tot;

struct SessionRec {
  uint32_t id = 0, startEpoch = 0, endEpoch = 0, studyS = 0, breakS = 0, breaks = 0, closeCnt = 0, overS = 0;
};

// sensor
const uint8_t  FILTER_N = 7;
const uint32_t SAMPLE_MS = 100;
float    win[FILTER_N];
uint8_t  winIdx = 0, winCount = 0;
float    distFilt = 400.0f;
uint32_t lastSample = 0;

// zone debounce
Zone     stableZone = Z_NONE, candZone = Z_NONE, rawZone = Z_NONE;
uint32_t candSince = 0;

// session
bool     sessActive = false, onBreak = false;
uint32_t sessStartMs = 0, segStartMs = 0, breakStartMs = 0;
uint32_t studyAccMs = 0, breakAccMs = 0, contBaseMs = 0, overAccMs = 0;
uint32_t breaksCnt = 0, sessClose = 0;
bool     blocked = false;
uint32_t nextBlockAlert = 0;
float    wMin = 0, wMax = 0, blockAnchor = 0;
uint32_t wStart = 0, blockedSince = 0;
bool     wActive = false;

// alerts
bool     solidRed = false, overNotified = false;
bool     haveGreen = false, haveRed = false, haveClose = false;
uint32_t lastGreenAt = 0, lastRedAt = 0, lastCloseAt = 0;
uint32_t nextOfflineBeep = 0;
bool     cloudUp = false;

// output patterns
struct Step { uint8_t r, g, b; uint16_t ms; };
const Step PAT_GREEN[]    = {{0,1,0,1000}};
const Step PAT_RED[]      = {{1,0,1,150},{1,0,0,250}};
const Step PAT_RED_BEEP[] = {{1,0,1,150}};
const Step PAT_DOUBLE[]   = {{1,1,1,150},{0,0,0,120},{1,1,1,150}};
const Step PAT_BEEP[]     = {{0,0,1,120}};
const Step PAT_ACK[]      = {{0,1,1,120},{0,0,0,100},{0,1,1,120}};
const Step PAT_BOOT[]     = {{1,1,0,250},{0,0,1,80}};
const Step PAT_BLOCKED[] = {{1,0,1,5000}};
#define PLAY(p) playPattern(p, sizeof(p) / sizeof(p[0]))
const Step* patSteps = nullptr;
uint8_t  patLen = 0, patIdx = 0;
uint32_t patStepStart = 0;
bool     patPlaying = false;

// networking
ESP8266WebServer server(80);
DNSServer dns;
bool     apOn = false, wifiWasUp = false, forceConnect = true;
uint32_t apLastClientMs = 0, lastWifiTry = 0, wifiDownSince = 0, lastBlynkTry = 0;
uint8_t  wifiTryIdx = 0;
bool     blynkJustConnected = false, haveTriedBlynk = false;
bool replaying = false;
uint32_t lastPush = 0, lastSync = 0, lastCheckpoint = 0, lastTick = 0;
String   lastStatusSent = "";
const uint32_t LOG_MAX_BYTES = 60000UL;

// ======================= 4. OUTPUTS =======================
void setOut(bool r, bool g, bool b) {
  digitalWrite(PIN_RED, r);
  digitalWrite(PIN_GREEN, g);
  digitalWrite(PIN_BUZZ, BUZZER_ACTIVE_HIGH ? b : !b);
}
void playPattern(const Step* p, uint8_t n) {
  patSteps = p; patLen = n; patIdx = 0; patStepStart = millis(); patPlaying = true;
  setOut(p[0].r, p[0].g, p[0].b);
}
void updatePattern(uint32_t now) {
  if (!patPlaying) { setOut(solidRed, false, false); return; }
  if (now - patStepStart >= patSteps[patIdx].ms) {
    patIdx++;
    if (patIdx >= patLen) { patPlaying = false; setOut(solidRed, false, false); return; }
    patStepStart = now;
    setOut(patSteps[patIdx].r, patSteps[patIdx].g, patSteps[patIdx].b);
  }
}

// ======================= 5. PERSISTENCE =======================
void recalcThresholds() {
  closeLimit = normalCm * CLOSE_FACTOR;
  goodMin    = normalCm * GOOD_FACTOR;
  awayLimit  = normalCm * AWAY_FACTOR;
  if (awayLimit < normalCm + 30.0f) awayLimit = normalCm + 30.0f;
  if (awayLimit > 300.0f) awayLimit = 300.0f;
}
void saveCfg() {
  if (!fsOk) return;
  File f = LittleFS.open("/cfg.txt", "w");
  if (!f) return;
  f.printf("normal=%.1f\nlimit=%u\nssid=%s\npass=%s\n", normalCm, (unsigned)limitMin, cfgSsid.c_str(), cfgPass.c_str());
  f.close();
}
void loadCfg() {
  if (!fsOk || !LittleFS.exists("/cfg.txt")) return;
  File f = LittleFS.open("/cfg.txt", "r");
  if (!f) return;
  while (f.available()) {
    String l = f.readStringUntil('\n'); l.replace("\r", "");
    int eq = l.indexOf('=');
    if (eq < 1) continue;
    String k = l.substring(0, eq), v = l.substring(eq + 1);
    if (k == "normal") { float x = v.toFloat(); if (x >= 15 && x <= 200) normalCm = x; }
    else if (k == "limit") { int x = v.toInt(); if (x >= 1 && x <= 600) limitMin = x; }
    else if (k == "ssid") cfgSsid = v;
    else if (k == "pass") cfgPass = v;
  }
  f.close();
}
void saveTotals() {
  if (!fsOk) return;
  File f = LittleFS.open("/totals.txt", "w");
  if (!f) return;
  f.printf("%lu,%lu,%lu,%lu,%lu\n", (unsigned long)tot.studyS, (unsigned long)tot.sessions,
           (unsigned long)tot.closeCnt, (unsigned long)tot.lastId, (unsigned long)tot.lastSynced);
  f.close();
}
void loadTotals() {
  if (!fsOk || !LittleFS.exists("/totals.txt")) return;
  File f = LittleFS.open("/totals.txt", "r");
  if (!f) return;
  String l = f.readStringUntil('\n'); f.close();
  unsigned long a, b, c, d, e;
  if (sscanf(l.c_str(), "%lu,%lu,%lu,%lu,%lu", &a, &b, &c, &d, &e) == 5) {
    tot.studyS = a; tot.sessions = b; tot.closeCnt = c; tot.lastId = d; tot.lastSynced = e;
  }
}
String recToLine(const SessionRec& r) {
  char buf[128];
  snprintf(buf, sizeof(buf), "%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu",
           (unsigned long)r.id, (unsigned long)r.startEpoch, (unsigned long)r.endEpoch,
           (unsigned long)r.studyS, (unsigned long)r.breakS, (unsigned long)r.breaks,
           (unsigned long)r.closeCnt, (unsigned long)r.overS);
  return String(buf);
}
bool parseRec(const String& line, SessionRec& r) {
  unsigned long a[8];
  if (sscanf(line.c_str(), "%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu",
             &a[0], &a[1], &a[2], &a[3], &a[4], &a[5], &a[6], &a[7]) != 8) return false;
  r.id = a[0]; r.startEpoch = a[1]; r.endEpoch = a[2]; r.studyS = a[3];
  r.breakS = a[4]; r.breaks = a[5]; r.closeCnt = a[6]; r.overS = a[7];
  return true;
}
void trimLog() {
  File f = LittleFS.open("/sessions.csv", "r");
  if (!f) return;
  if (f.size() < LOG_MAX_BYTES) { f.close(); return; }
  f.seek(f.size() / 2);
  f.readStringUntil('\n');                       // drop partial line
  File o = LittleFS.open("/s_tmp.csv", "w");
  if (!o) { f.close(); return; }
  while (f.available()) {
    String l = f.readStringUntil('\n'); l.trim();
    if (l.length()) { o.print(l); o.print('\n'); }
  }
  o.close(); f.close();
  LittleFS.remove("/sessions.csv");
  LittleFS.rename("/s_tmp.csv", "/sessions.csv");
}
void appendLog(const SessionRec& r) {
  if (!fsOk) return;
  File f = LittleFS.open("/sessions.csv", "a");
  if (!f) return;
  f.print(recToLine(r)); f.print('\n');
  f.close();
  trimLog();
}

// ======================= 6. TIME HELPERS =======================
bool timeValid() { return time(nullptr) > 1700000000L; }
uint32_t epochAtMs(uint32_t atMs, uint32_t now) {           // 0 when the clock is unknown
  if (!timeValid()) return 0;
  return (uint32_t)time(nullptr) - (now - atMs) / 1000UL;
}
String fmtEpoch(uint32_t e) {
  if (e == 0) return "no clock";
  time_t t = e; char b[24];
  strftime(b, sizeof(b), "%d %b %H:%M", localtime(&t));
  return String(b);
}
String describeName(const SessionRec& r) { return "#" + String(r.id) + "  " + fmtEpoch(r.startEpoch); }
String describeValue(const SessionRec& r) {
  return "Study " + String(r.studyS / 60) + "m | Breaks " + String(r.breaks) + " (" + String(r.breakS / 60) +
         "m) | Too close " + String(r.closeCnt) + " | Over limit " + String(r.overS / 60) + "m";
}

// ======================= 7. SENSOR =======================
float readDistanceCm() {
  digitalWrite(PIN_TRIG, LOW);  delayMicroseconds(3);
  digitalWrite(PIN_TRIG, HIGH); delayMicroseconds(10);
  digitalWrite(PIN_TRIG, LOW);
  unsigned long us = pulseIn(PIN_ECHO, HIGH, 25000UL);
  if (us == 0) return -1.0f;
  float d = us * 0.0343f / 2.0f;
  if (d < 2.0f || d > 400.0f) return -1.0f;
  return d;
}
void sampleSensor() {
  float d = readDistanceCm();
  win[winIdx] = (d < 0) ? 400.0f : d;               // no echo = far away
  winIdx = (winIdx + 1) % FILTER_N;
  if (winCount < FILTER_N) winCount++;
  float t[FILTER_N];
  for (uint8_t i = 0; i < winCount; i++) t[i] = win[i];
  for (uint8_t i = 1; i < winCount; i++) {           // insertion sort
    float k = t[i]; int j = i - 1;
    while (j >= 0 && t[j] > k) { t[j + 1] = t[j]; j--; }
    t[j + 1] = k;
  }
  distFilt = t[winCount / 2];                        // median: kills single erratic readings
}
Zone classify(float d) {
  if (d >= 400.0f || d > awayLimit) return Z_NONE;
  if (d < closeLimit) return Z_CLOSE;
  if (d < goodMin)    return Z_MID;
  return Z_GOOD;
}

// ======================= 8. SESSION LOGIC =======================
uint32_t currentStudyMs(uint32_t now) {
  if (!sessActive) return 0;
  return studyAccMs + (onBreak ? 0 : (now - segStartMs));
}
uint32_t continuousMs(uint32_t now) { return currentStudyMs(now) - contBaseMs; }

void startSession(uint32_t t) {
  sessActive = true; onBreak = false;
  sessStartMs = segStartMs = t;
  studyAccMs = breakAccMs = contBaseMs = overAccMs = 0;
  breaksCnt = sessClose = 0;
  haveGreen = haveRed = false; overNotified = false;
  Serial.println("[session] started");
}
void beginBreak(uint32_t t) {
  if ((int32_t)(t - segStartMs) > 0) studyAccMs += t - segStartMs;
  onBreak = true; breakStartMs = t; solidRed = false;
  Serial.println("[session] break begins");
}
void endBreak(uint32_t t) {
  uint32_t dur = ((int32_t)(t - breakStartMs) > 0) ? t - breakStartMs : 0;
  breakAccMs += dur; breaksCnt++;
  onBreak = false; segStartMs = t;
  if (dur >= BREAK_RESET_MS) {                       // long enough to count as a real rest
    contBaseMs = studyAccMs;
    haveRed = false; overNotified = false; haveGreen = false;
  }
  Serial.printf("[session] back after %lus\n", (unsigned long)(dur / 1000));
}
void checkpointSession(uint32_t now) {
  if (!fsOk || !sessActive) return;
  SessionRec r;
  uint32_t studyMs = currentStudyMs(now);
  r.startEpoch = epochAtMs(sessStartMs, now);
  r.endEpoch = timeValid() ? (uint32_t)time(nullptr) : 0;
  r.studyS = studyMs / 1000; r.breakS = breakAccMs / 1000; r.breaks = breaksCnt;
  r.closeCnt = sessClose; r.overS = overAccMs / 1000;
  File f = LittleFS.open("/active.txt", "w");
  if (!f) return;
  f.print(recToLine(r)); f.print('\n');
  f.close();
}
void storeFinished(SessionRec r) {
  if (r.studyS < MIN_LOG_STUDY_S) return;
  r.id = ++tot.lastId;
  tot.studyS += r.studyS; tot.sessions++; tot.closeCnt += r.closeCnt;
  appendLog(r);
  saveTotals();
  Serial.printf("[log] session #%lu saved (%lus study)\n", (unsigned long)r.id, (unsigned long)r.studyS);
}
void finalizeSession(uint32_t now) {                 // called when a break outlasts SESSION_GAP_MS
  SessionRec r;
  r.startEpoch = epochAtMs(sessStartMs, now);
  r.endEpoch   = epochAtMs(breakStartMs, now);
  r.studyS = studyAccMs / 1000; r.breakS = breakAccMs / 1000; r.breaks = breaksCnt;
  r.closeCnt = sessClose; r.overS = overAccMs / 1000;
  storeFinished(r);
  sessActive = false; onBreak = false; solidRed = false;
  if (fsOk) LittleFS.remove("/active.txt");
}
void recoverInterruptedSession() {                   // power was cut mid-session
  if (!fsOk || !LittleFS.exists("/active.txt")) return;
  File f = LittleFS.open("/active.txt", "r");
  if (!f) return;
  String l = f.readStringUntil('\n'); f.close();
  SessionRec r;
  if (parseRec(l, r)) { storeFinished(r); Serial.println("[log] recovered session cut by power-off"); }
  LittleFS.remove("/active.txt");
}

int confirmNeeded(Zone from, Zone to) {
  if (to == Z_NONE) return AWAY_CONFIRM_MS;
  if (from == Z_NONE) return sessActive ? RETURN_CONFIRM_MS : START_CONFIRM_MS;
  return ZONE_CONFIRM_MS;
}
void onZoneChange(Zone from, Zone to, uint32_t now) {
  bool fromP = (from != Z_NONE), toP = (to != Z_NONE);
  if (!fromP && toP) {
    if (!sessActive) startSession(candSince);
    else if (onBreak) endBreak(candSince);
  } else if (fromP && !toP) {
    if (sessActive && !onBreak) beginBreak(candSince);
  }
  if (to == Z_CLOSE) { sessClose++; haveClose = false; }
  Serial.printf("[zone] %d -> %d (%.1f cm)\n", from, to, distFilt);
}
void updateStationary(uint32_t now) {
  bool inRange = (distFilt < 400.0f && distFilt <= awayLimit);

  if (blocked) {                                     // wait until the reading changes
    if (fabsf(distFilt - blockAnchor) > UNBLOCK_BAND_CM) {
      blocked = false; wActive = false;
      Serial.println("[sensor] obstruction cleared");
    }
    return;
  }
  if (!inRange) { wActive = false; return; }         // nobody/nothing in range: restart watching
  if (!wActive) { wActive = true; wMin = wMax = distFilt; wStart = now; return; }

  if (distFilt < wMin) wMin = distFilt;
  if (distFilt > wMax) wMax = distFilt;
  if (wMax - wMin > STATIONARY_BAND_CM) {            // there is movement: restart the window
    wMin = wMax = distFilt; wStart = now;
    return;
  }
  if (now - wStart >= STATIONARY_MS) {               // frozen long enough: it's an object
    blocked = true; blockAnchor = distFilt; blockedSince = wStart;
    Serial.printf("[sensor] BLOCKED at %.1f cm\n", distFilt);
    if (sessActive && !onBreak) {                    // the person left; end their study time when it froze
      uint32_t t = ((int32_t)(blockedSince - segStartMs) > 0) ? blockedSince : segStartMs;
      beginBreak(t);
    }
    stableZone = candZone = Z_NONE; candSince = now;
      nextBlockAlert = now;                            // first alarm fires immediately
    if (Blynk.connected()) Blynk.logEvent("sensor_blocked", "Something is blocking the distance sensor");
  }
}
void updateState(uint32_t now) {
  rawZone = blocked ? Z_NONE : classify(distFilt);
  if (rawZone != candZone) { candZone = rawZone; candSince = now; }
  if (candZone != stableZone && (now - candSince) >= (uint32_t)confirmNeeded(stableZone, candZone)) {
    Zone old = stableZone;
    stableZone = candZone;
    onZoneChange(old, stableZone, now);
  }
  if (sessActive && onBreak && (now - breakStartMs) >= SESSION_GAP_MS) finalizeSession(now);
}

// ======================= 9. ALERTS =======================
uint32_t redIntervalMs(uint32_t overMs) {            // 0 = solid
  uint32_t m = overMs / 60000UL;
  if (m < 10) return 120000UL;
  if (m < 20) return 60000UL;
  if (m < 25) return 30000UL;
  if (m < 28) return 15000UL;
  if (m < 30) return 5000UL;
  return 0;
}
void runAlerts(uint32_t now) {
  uint32_t dt = now - lastTick; lastTick = now;
  bool present = sessActive && !onBreak && stableZone != Z_NONE && rawZone != Z_NONE;
  uint32_t limitMs = (uint32_t)limitMin * 60000UL;
  bool over = false; uint32_t overMs = 0, iv = 120000UL;
  if (present) {
    uint32_t c = continuousMs(now);
    if (c > limitMs) { over = true; overMs = c - limitMs; iv = redIntervalMs(overMs); overAccMs += dt; }
  }
  solidRed = over && iv == 0;
  if (!over) overNotified = false;
  if (over && !overNotified) {
    overNotified = true;
    if (Blynk.connected()) Blynk.logEvent("break_due", "Time for a break - study limit exceeded");
  }
  if (patPlaying) return;                            // one pattern at a time
  if (blocked) {                                     // sensor obstructed: 5 s alarm, repeat 10 s after it ends
    if ((int32_t)(now - nextBlockAlert) >= 0) {
      PLAY(PAT_BLOCKED);
      nextBlockAlert = now + 5000UL + 10000UL;       // 5 s on + 10 s gap
    }
    return;                                          // nothing else alerts while blocked
  }
  if (present && stableZone == Z_CLOSE && (!haveClose || now - lastCloseAt >= CLOSE_EVERY_MS)) {
    PLAY(PAT_DOUBLE); haveClose = true; lastCloseAt = now; return;        // highest priority
  }
  if (over) {
    if (iv == 0) {
      if (SOLID_BEEP_MS && (!haveRed || now - lastRedAt >= SOLID_BEEP_MS)) { PLAY(PAT_RED_BEEP); haveRed = true; lastRedAt = now; return; }
    } else if (!haveRed || now - lastRedAt >= iv) {
      PLAY(PAT_RED); haveRed = true; lastRedAt = now; return;
    }
  } else if (present && stableZone == Z_GOOD && (!haveGreen || now - lastGreenAt >= GREEN_EVERY_MS)) {
    PLAY(PAT_GREEN); haveGreen = true; lastGreenAt = now; return;
  }
  // offline reminder (never blocks anything above)
  bool up = Blynk.connected();
  if (up != cloudUp) { cloudUp = up; if (!up) nextOfflineBeep = now + OFFLINE_GRACE_MS; }
  if (!cloudUp && (int32_t)(now - nextOfflineBeep) >= 0 && (!OFFLINE_BEEP_ONLY_WHEN_PRESENT || present)) {
    PLAY(PAT_BEEP); nextOfflineBeep = now + OFFLINE_BEEP_MS;
  }
}

// ======================= 10. CALIBRATION =======================
void setNormal(float v) {
  normalCm = v; recalcThresholds(); saveCfg();
  Serial.printf("[cfg] normal distance = %.1f cm\n", v);
}
bool calibrateFromCurrent() {
  if (winCount < FILTER_N || distFilt >= 200.0f || distFilt < 15.0f) {
    Serial.println("[cfg] calibration refused: no stable reading");
    return false;
  }
  setNormal(distFilt);
  if (Blynk.connected()) Blynk.virtualWrite(VP_NORMAL, normalCm);
  PLAY(PAT_ACK);
  return true;
}

// ======================= 11. BLYNK =======================
BLYNK_CONNECTED() { blynkJustConnected = true; }
BLYNK_WRITE(VP_NORMAL) { float v = param.asFloat(); if (v >= 15 && v <= 200) setNormal(v); }
BLYNK_WRITE(VP_SETBTN) { if (param.asInt() == 1) calibrateFromCurrent(); }
BLYNK_WRITE(VP_LIMIT)  { int v = param.asInt(); if (v >= 1 && v <= 600) { limitMin = v; saveCfg(); } }
BLYNK_WRITE(VP_REPLAY) { if (param.asInt() == 1) requestReplay(); }

String statusText() {
  if (!sessActive) return "Idle - nobody at desk";
  if (onBreak) return "On break";
  if (blocked) return "Sensor blocked / object in front";
  String s;
  switch (stableZone) {
    case Z_CLOSE: s = "TOO CLOSE"; break;
    case Z_MID:   s = "A bit close"; break;
    case Z_GOOD:  s = "Good distance"; break;
    default:      s = "Leaving..."; break;
  }
  uint32_t c = continuousMs(millis()), lim = (uint32_t)limitMin * 60000UL;
  if (c > lim) s += " | BREAK NOW (+" + String((c - lim) / 60000UL) + "m)";
  return s;
}
void sendRecToBlynk(const SessionRec& r) {
  String s = describeName(r) + "\t";                                   // #12  23 Oct 14:05
  s += "Study \t" + String(r.studyS / 60) + "m,\t br \t" + String(r.breaks) + " \t";
  s += "Cl \t" + String(r.closeCnt) + ",\t over \t" + String(r.overS / 60) + "m\n";
  Blynk.virtualWrite(VP_TABLE, s);
}

void syncPending() {                                 // push sessions logged while offline
  if (!Blynk.connected() || !fsOk || tot.lastSynced >= tot.lastId) return;
  File f = LittleFS.open("/sessions.csv", "r");
  if (!f) return;
  int sent = 0; SessionRec last; bool any = false;
  while (f.available() && sent < 4) {
    String l = f.readStringUntil('\n');
    SessionRec r;
    if (!parseRec(l, r) || r.id <= tot.lastSynced) continue;
    sendRecToBlynk(r); tot.lastSynced = r.id; last = r; any = true; sent++;
  }
  f.close();
    if (any) {
    saveTotals();
    if (tot.lastSynced >= tot.lastId) {
      if (!replaying) Blynk.logEvent("session_end", describeName(last) + " - " + describeValue(last));
      replaying = false;
    }
  }
}
void requestReplay() {
  uint32_t from = (tot.lastId > REPLAY_COUNT) ? tot.lastId - REPLAY_COUNT : 0;
  if (from < tot.lastSynced) tot.lastSynced = from;    // only ever move backwards
  Blynk.virtualWrite(VP_TABLE, "clr");                 // clears the terminal
  Blynk.virtualWrite(VP_TABLE, "Last sessions:\n");
  replaying = true;
}
void pushSettings() {
  Blynk.virtualWrite(VP_NORMAL, normalCm);
  Blynk.virtualWrite(VP_LIMIT, (int)limitMin);
  Blynk.virtualWrite(VP_INFO, String("IP ") + WiFi.localIP().toString());
  requestReplay();
}
void pushLive(uint32_t now) {
  float disp = distFilt >= 400.0f ? 0.0f : distFilt;
  String st = statusText();
  Blynk.virtualWrite(VP_DIST, disp);
  if (st != lastStatusSent) { Blynk.virtualWrite(VP_STATUS, st); lastStatusSent = st; }
  Blynk.virtualWrite(VP_CONT, continuousMs(now) / 60000.0f);
  Blynk.virtualWrite(VP_SESS, currentStudyMs(now) / 60000.0f);
  Blynk.virtualWrite(VP_TOTAL, (tot.studyS + currentStudyMs(now) / 1000UL) / 60.0f);
  Blynk.virtualWrite(VP_COUNT, (int)(tot.sessions + (sessActive ? 1 : 0)));
  Blynk.virtualWrite(VP_CLOSE, (int)(tot.closeCnt + sessClose));
}

// ======================= 12. WI-FI + LOCAL PORTAL =======================
void startAP() {
  if (apOn) return;
  WiFi.mode(WIFI_AP_STA);
  WiFi.softAP(AP_SSID, strlen(AP_PASS) >= 8 ? AP_PASS : nullptr);
  dns.setErrorReplyCode(DNSReplyCode::NoError);
  dns.start(53, "*", WiFi.softAPIP());
  apOn = true; apLastClientMs = millis();
  Serial.printf("[wifi] setup hotspot '%s' on %s\n", AP_SSID, WiFi.softAPIP().toString().c_str());
}
void stopAP() {
  if (!apOn) return;
  dns.stop(); WiFi.softAPdisconnect(true); WiFi.mode(WIFI_STA);
  apOn = false;
  Serial.println("[wifi] setup hotspot off");
}
void beginWifi() {
  String ss[2], pp[2]; int n = 0;
  if (cfgSsid.length()) { ss[n] = cfgSsid; pp[n] = cfgPass; n++; }
  if (strlen(DEFAULT_SSID) && String(DEFAULT_SSID) != cfgSsid) { ss[n] = DEFAULT_SSID; pp[n] = DEFAULT_PASS; n++; }
  if (n == 0) return;
  int i = wifiTryIdx++ % n;
  Serial.printf("[wifi] connecting to '%s'\n", ss[i].c_str());
  WiFi.begin(ss[i].c_str(), pp[i].c_str());
}
void handleWifi(uint32_t now) {
  bool up = (WiFi.status() == WL_CONNECTED);
  if (up && !wifiWasUp) {
    Serial.printf("[wifi] connected, IP %s\n", WiFi.localIP().toString().c_str());
    configTime(TZ_OFFSET_S, 0, "pool.ntp.org", "time.google.com");
    haveTriedBlynk = false; apLastClientMs = now;
  }
  if (!up && wifiWasUp) wifiDownSince = now;
  wifiWasUp = up;
  if (!up) {
    if (forceConnect || now - lastWifiTry >= 30000UL) { forceConnect = false; lastWifiTry = now; beginWifi(); }
    if (!apOn && now - wifiDownSince >= 20000UL) startAP();   // offline: offer the setup hotspot
  } else if (apOn) {
    if (WiFi.softAPgetStationNum() > 0) apLastClientMs = now;
    else if (now - apLastClientMs > 120000UL) stopAP();
  }
}
void tryBlynk(uint32_t now) {
  if (WiFi.status() != WL_CONNECTED) return;
  if (haveTriedBlynk && now - lastBlynkTry < 30000UL) return;
  haveTriedBlynk = true; lastBlynkTry = now;
  Blynk.connect(2000);
}

String esc(const String& s) {
  String o;
  for (size_t i = 0; i < s.length(); i++) {
    char c = s[i];
    if (c == '<') o += "&lt;"; else if (c == '>') o += "&gt;";
    else if (c == '&') o += "&amp;"; else if (c == '"') o += "&quot;"; else o += c;
  }
  return o;
}
void redirectHome() {
  server.sendHeader("Location", "/", true);
  server.send(302, "text/plain", "");
}
void handleRoot() {
  uint32_t now = millis();
  int n = WiFi.scanComplete();
  if (n == WIFI_SCAN_FAILED) WiFi.scanNetworks(true);
  String h; h.reserve(4000);
  h += F("<!doctype html><html><head><meta charset=utf-8><meta name=viewport content='width=device-width,initial-scale=1'>"
         "<title>Study Desk Guard</title><style>body{font-family:sans-serif;max-width:460px;margin:auto;padding:12px}"
         "input,button{font-size:16px;padding:8px;margin:4px 0;width:100%;box-sizing:border-box}"
         "fieldset{margin:12px 0;border-radius:8px}</style></head><body><h2>Study Desk Guard</h2>");
  h += "<fieldset><legend>Status</legend>";
  h += "Now: <b>" + esc(statusText()) + "</b><br>Distance: " + (distFilt >= 400 ? String("--") : String(distFilt, 1)) + " cm<br>";
  h += "Normal distance: " + String(normalCm, 1) + " cm | Limit: " + String(limitMin) + " min<br>";
  h += "Sessions: " + String(tot.sessions) + " | Total study: " + String(tot.studyS / 60) + " min<br>";
  h += "Wi-Fi: " + String(WiFi.status() == WL_CONNECTED ? ("connected to " + esc(WiFi.SSID())) : String("not connected"));
  h += " | Blynk: " + String(Blynk.connected() ? "online" : "offline") + "</fieldset>";

  h += F("<fieldset><legend>Wi-Fi</legend><form method=POST action=/wifi><input name=ssid list=nets placeholder='Network name' required><datalist id=nets>");
  if (n > 0) for (int i = 0; i < n; i++) h += "<option value=\"" + esc(WiFi.SSID(i)) + "\">";
  h += F("</datalist><input name=pass type=password placeholder='Password'><button>Save &amp; connect</button></form>"
         "<a href=/scan>Rescan networks</a></fieldset>");

  h += F("<fieldset><legend>Settings</legend><form method=POST action=/calib><button>Set current distance as normal</button></form>"
         "<form method=POST action=/set>Study limit (min)<input name=limit type=number min=1 max=600 value=");
  h += String(limitMin) + F("><button>Save limit</button></form></fieldset>");

  h += "<fieldset><legend>Recent sessions</legend>";
  if (fsOk && LittleFS.exists("/sessions.csv")) {
    String ring[6]; int cnt = 0;
    File f = LittleFS.open("/sessions.csv", "r");
    while (f && f.available()) {
      String l = f.readStringUntil('\n'); SessionRec r;
      if (parseRec(l, r)) { ring[cnt % 6] = describeName(r) + "<br>&nbsp;&nbsp;" + describeValue(r); cnt++; }
    }
    if (f) f.close();
    if (cnt == 0) h += "None yet";
    for (int i = 0; i < (cnt < 6 ? cnt : 6); i++) h += ring[(cnt - 1 - i) % 6] + "<br>";
  } else h += "None yet";
  h += "</fieldset></body></html>";
  server.send(200, "text/html", h);
}
void handleSaveWifi() {
  cfgSsid = server.arg("ssid"); cfgPass = server.arg("pass");
  saveCfg(); wifiTryIdx = 0; forceConnect = true;
  WiFi.disconnect();
  server.send(200, "text/html", F("<html><body style='font-family:sans-serif'><h3>Saved.</h3>Connecting to the network now. "
                                  "If it works, the hotspot switches off after a couple of minutes.<br><a href=/>Back</a></body></html>"));
}
void handleCalib() { calibrateFromCurrent(); redirectHome(); }
void handleSet() {
  int v = server.arg("limit").toInt();
  if (v >= 1 && v <= 600) { limitMin = v; saveCfg(); if (Blynk.connected()) Blynk.virtualWrite(VP_LIMIT, v); }
  redirectHome();
}
void handleScan() { WiFi.scanDelete(); WiFi.scanNetworks(true); redirectHome(); }
void handleNotFound() {
  if (apOn) {                                        // captive-portal behaviour
    server.sendHeader("Location", String("http://") + WiFi.softAPIP().toString() + "/", true);
    server.send(302, "text/plain", "");
  } else server.send(404, "text/plain", "Not found");
}

// ======================= 13. SETUP / LOOP =======================
void setup() {
  Serial.begin(115200);
  pinMode(PIN_TRIG, OUTPUT); pinMode(PIN_ECHO, INPUT);
  pinMode(PIN_RED, OUTPUT); pinMode(PIN_GREEN, OUTPUT); pinMode(PIN_BUZZ, OUTPUT);
  pinMode(PIN_BTN, INPUT_PULLUP);
  setOut(false, false, false);

  fsOk = LittleFS.begin();
  if (!fsOk) Serial.println("[fs] LittleFS failed - running without storage (set Flash Size with FS!)");
  loadCfg(); loadTotals(); recalcThresholds();
  recoverInterruptedSession();

  WiFi.persistent(false);
  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  wifiDownSince = millis();
  server.on("/", HTTP_GET, handleRoot);
  server.on("/wifi", HTTP_POST, handleSaveWifi);
  server.on("/calib", HTTP_POST, handleCalib);
  server.on("/set", HTTP_POST, handleSet);
  server.on("/scan", HTTP_GET, handleScan);
  server.onNotFound(handleNotFound);
  server.begin();
  Blynk.config(BLYNK_AUTH_TOKEN);

  nextOfflineBeep = millis() + OFFLINE_GRACE_MS;
  lastTick = millis();
  PLAY(PAT_BOOT);
  Serial.printf("[boot] normal %.1f cm, limit %u min, %lu sessions stored\n",
                normalCm, (unsigned)limitMin, (unsigned long)tot.sessions);
}

void loop() {
  uint32_t now = millis();

  handleWifi(now);
  server.handleClient();
  if (apOn) dns.processNextRequest();

  if (Blynk.connected()) Blynk.run(); else tryBlynk(now);
  if (blynkJustConnected) { blynkJustConnected = false; pushSettings(); lastSync = 0; }

  if (now - lastSample >= SAMPLE_MS) {
    lastSample = now;
    sampleSensor();
    if (winCount >= 5) { updateStationary(now); updateState(now); }
  }
  runAlerts(now);
  updatePattern(now);

  if (Blynk.connected()) {
    if (now - lastPush >= 2000UL) { lastPush = now; pushLive(now); }
    if (now - lastSync >= 1500UL) { lastSync = now; syncPending(); }
  }
  if (sessActive && now - lastCheckpoint >= 60000UL) { lastCheckpoint = now; checkpointSession(now); }

  // hold the FLASH button (D3) for 2 s to open the setup hotspot manually
  static uint32_t btnSince = 0; static bool btnHandled = false;
  if (digitalRead(PIN_BTN) == LOW) {
    if (btnSince == 0) btnSince = now;
    else if (!btnHandled && now - btnSince > 2000) { btnHandled = true; startAP(); PLAY(PAT_ACK); }
  } else { btnSince = 0; btnHandled = false; }

  yield();
}
