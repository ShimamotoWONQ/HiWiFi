// WiFi Mapper — Processing
// SERIAL_PORT を実機に合わせること (printArray(Serial.list()) で確認)

import processing.serial.*;
import processing.data.JSONObject;
import processing.data.JSONArray;

final String SERIAL_PORT  = "/dev/cu.usbmodem48CA435E00AC2";
final int    BAUD          = 115200;
final int    WIN_SIZE      = 800;
final float  METERS_TO_PX  = 40.0f;
final float  MIN_METERS_TO_PX = 12.0f;
final float  MAX_METERS_TO_PX = 160.0f;
final float  ZOOM_STEP = 1.20f;
final float  CAMERA_PAN_SPEED = 2.0f;  // m/s in world coordinates
final float  WALK_SPEED    = 0.8f;
final float  MAX_RADIUS_M  = 12.0f;
final boolean SIMULATE_INPUT = false;

final float ACCEL_DEADBAND        = 0.12f;
final float ACCEL_LPF_ALPHA       = 0.25f;
final float ACCEL_GAIN            = 0.35f;
final float MOVING_BLEND          = 0.09f;
final float MOVING_DAMPING_PER_S  = 0.94f;
final float STILL_DAMPING_PER_S   = 0.10f;
final float MAX_SPEED             = 1.8f;
final int   IMU_STALE_MS          = 700;
final int   SCAN_PREDICT_MS       = 3000;
final int   ACCEL_HOLD_MS         = 350;

final float AP_DISTANCE_BLEND = 0.20f;

final float MOVE_THRESHOLD_MS2 = 0.35f;
final float FIFO_CORRECT_ALPHA = 0.25f;

final int   TRILATERATION_MIN_APS   = 3;
final int   TRILATERATION_MIN_HITS  = 3;
final float TRILATERATION_BLEND     = 0.15f;
final float TRILATERATION_MAX_ERR_M = 5.0f;

final float[] RINGS = { 1, 3, 5, 10 };

class Snapshot {
  float x, y, heading, vx, vy;
  float filtAx, filtAy, lastGz, lastAx, lastAy;
  int moving;
}

class AP {
  String ssid, bssid;
  int rssi, channel;
  float worldX, worldY;
  float dist;
  long lastSeen;
  int hitCount = 1;

  AP(String ssid, String bssid, int rssi, int channel, float userX, float userY) {
    this.ssid = ssid;
    this.bssid = bssid;
    this.rssi = rssi;
    this.channel = channel;
    this.dist = rssiToDistance(rssi);
    float angle = random(TWO_PI);
    this.worldX = userX + cos(angle) * dist;
    this.worldY = userY + sin(angle) * dist;
    this.lastSeen = millis();
  }

  void update(String newSsid, int newRssi, int newChannel, float userX, float userY) {
    ssid = newSsid;
    rssi = newRssi;
    channel = newChannel;
    dist = rssiToDistance(newRssi);
    lastSeen = millis();

    float dx = worldX - userX;
    float dy = worldY - userY;
    float len = sqrt(dx * dx + dy * dy);
    if (len < 0.001f) {
      float angle = random(TWO_PI);
      dx = cos(angle);
      dy = sin(angle);
      len = 1.0f;
    }

    float targetX = userX + (dx / len) * dist;
    float targetY = userY + (dy / len) * dist;
    worldX = lerp(worldX, targetX, AP_DISTANCE_BLEND);
    worldY = lerp(worldY, targetY, AP_DISTANCE_BLEND);
    hitCount++;
  }

  float screenX() { return toScreenX(worldX); }
  float screenY() { return toScreenY(worldY); }
  float dotSize() { return constrain(map(rssi, -90, -30, 6, 18), 5, 22); }

  float alpha() {
    float age = (millis() - lastSeen) / 1000.0f;
    return map(constrain(age, 0, 10), 0, 10, 255, 60);
  }
}

class ScanObservation {
  String ssid, bssid;
  int rssi, channel;
  float dist;

  ScanObservation(String ssid, String bssid, int rssi, int channel) {
    this.ssid = ssid;
    this.bssid = bssid;
    this.rssi = rssi;
    this.channel = channel;
    this.dist = rssiToDistance(rssi);
  }
}

class ScanCapture {
  int slot;
  float userX, userY;
  long capturedAt;
  HashMap<String, ScanObservation> observations = new HashMap<String, ScanObservation>();

  ScanCapture(int slot, float userX, float userY) {
    this.slot = slot;
    this.userX = userX;
    this.userY = userY;
    this.capturedAt = millis();
  }

  void add(ScanObservation obs) {
    observations.put(obs.bssid, obs);
  }

  int count() {
    return observations.size();
  }
}

class MotionState {
  float userX = 0, userY = 0;
  float heading = 0;
  float vx = 0, vy = 0;
  float lastGz = 0, lastAx = 0, lastAy = 0;
  float filtAx = 0, filtAy = 0;
  int moving = 0;
  long lastImuMs = 0, lastArduinoMs = 0;
  boolean scanActive = false;

  Snapshot snap;
  boolean correcting = false;
  float correctX, correctY;

  // serialEvent writes gz deltas; draw() consumes via consumeGzAccum()
  private float gzAccum = 0;
  private long lastArduinoMsPrev = 0;

  synchronized void addGzDelta(float gz, long arduinoMs) {
    if (lastArduinoMsPrev > 0) {
      float dt = (arduinoMs - lastArduinoMsPrev) / 1000.0f;
      if (dt > 0 && dt < 0.5f) gzAccum += gz * dt;
    }
    lastArduinoMsPrev = arduinoMs;
  }

  synchronized float consumeGzAccum() {
    float v = gzAccum;
    gzAccum = 0;
    return v;
  }

  void applyImu(JSONObject json) {
    float gz = json.getFloat("gz", 0.0f);
    long arduinoMs = json.getInt("t_ms", 0);
    addGzDelta(gz, arduinoMs);
    lastGz = gz;
    lastAx = json.getFloat("ax", 0.0f);
    lastAy = json.getFloat("ay", 0.0f);
    moving = json.getInt("mv", 0);
    lastArduinoMs = arduinoMs;
    lastImuMs = millis();
  }

  void saveSnapshot() {
    snap = new Snapshot();
    snap.x = userX; snap.y = userY; snap.heading = heading;
    snap.vx = vx; snap.vy = vy;
    snap.filtAx = filtAx; snap.filtAy = filtAy;
    snap.lastGz = lastGz; snap.lastAx = lastAx; snap.lastAy = lastAy;
    snap.moving = moving;
  }

  void restoreSnapshot() {
    userX = snap.x; userY = snap.y; heading = snap.heading;
    vx = snap.vx; vy = snap.vy;
    filtAx = snap.filtAx; filtAy = snap.filtAy;
    lastGz = snap.lastGz; lastAx = snap.lastAx; lastAy = snap.lastAy;
    moving = snap.moving;
    synchronized(this) { gzAccum = 0; }
  }

  void setFifoSample(float ax, float ay, float gz) {
    lastAx = ax; lastAy = ay; lastGz = gz;
    float mag = sqrt(ax * ax + ay * ay);
    moving = (mag > MOVE_THRESHOLD_MS2) ? 1 : 0;
    lastImuMs = millis();
  }

  void beginCorrection(float x, float y) {
    correcting = true;
    correctX = x;
    correctY = y;
  }

  void update(float dt) {
    long now = millis();
    int age = (lastImuMs == 0) ? 999999 : (int)(now - lastImuMs);
    boolean canPredict = age < (scanActive ? SCAN_PREDICT_MS : IMU_STALE_MS);
    boolean holdAccel  = age < ACCEL_HOLD_MS;

    if (canPredict) {
      heading = wrapAngle(heading + consumeGzAccum());
    }

    if (holdAccel) {
      float ca = cos(heading), sa = sin(heading);
      float worldAx = applyDeadband(ca * lastAx - sa * lastAy);
      float worldAy = applyDeadband(sa * lastAx + ca * lastAy);
      filtAx = lerp(filtAx, worldAx, ACCEL_LPF_ALPHA);
      filtAy = lerp(filtAy, worldAy, ACCEL_LPF_ALPHA);
    } else {
      filtAx = lerp(filtAx, 0.0f, 0.08f);
      filtAy = lerp(filtAy, 0.0f, 0.08f);
    }

    if (canPredict && moving == 1) {
      vx += filtAx * ACCEL_GAIN * dt;
      vy += filtAy * ACCEL_GAIN * dt;

      float forwardAccel = filtAx * cos(heading) + filtAy * sin(heading);
      if (forwardAccel > ACCEL_DEADBAND) {
        vx = lerp(vx, cos(heading) * WALK_SPEED, MOVING_BLEND);
        vy = lerp(vy, sin(heading) * WALK_SPEED, MOVING_BLEND);
      }
      dampVelocity(MOVING_DAMPING_PER_S, dt);
    } else {
      dampVelocity(STILL_DAMPING_PER_S, dt);
    }

    capVelocity();
    userX += vx * dt;
    userY += vy * dt;

    if (correcting) {
      float dx = correctX - userX;
      float dy = correctY - userY;
      if (abs(dx) < 0.01f && abs(dy) < 0.01f) {
        userX = correctX; userY = correctY; correcting = false;
      } else {
        userX = lerp(userX, correctX, FIFO_CORRECT_ALPHA);
        userY = lerp(userY, correctY, FIFO_CORRECT_ALPHA);
        correctX += vx * dt;  // 補正ターゲットも物理に合わせてシフト
        correctY += vy * dt;
      }
    }
  }

  void resetPose() {
    userX = 0; userY = 0; heading = 0;
    vx = 0; vy = 0;
    filtAx = 0; filtAy = 0;
    correcting = false;
    synchronized(this) { gzAccum = 0; lastArduinoMsPrev = 0; }
  }

  float speed() { return sqrt(vx * vx + vy * vy); }

  void dampVelocity(float dampingPerSecond, float dt) {
    float k = pow(dampingPerSecond, dt);
    vx *= k; vy *= k;
  }

  void capVelocity() {
    float s = speed();
    if (s > MAX_SPEED) { vx = vx / s * MAX_SPEED; vy = vy / s * MAX_SPEED; }
  }
}

Serial port;
HashMap<String, AP> apMap = new HashMap<String, AP>();
ArrayList<PVector> path = new ArrayList<PVector>();
MotionState motion = new MotionState();

ArrayList<JSONObject> pendingAps = new ArrayList<JSONObject>();
volatile JSONObject pendingFifoBurst = null;

ScanCapture scan1 = null;
ScanCapture scan2 = null;
boolean captureArmed = false;
int nextCaptureSlot = 1;

float cameraX = 0, cameraY = 0;
float cameraOffsetX = 0, cameraOffsetY = 0;
float metersToPx = METERS_TO_PX;
long lastFrameMs = 0;
boolean debugOverlay = true;
long lastSimMs = 0, simStartMs = 0;
boolean panUp = false, panDown = false, panLeft = false, panRight = false;

volatile int    rawLineCount    = 0;
volatile long   lastRawLineMs   = 0;
volatile String lastRawLine     = "";
volatile int    parseErrorCount = 0;
int             bytesAvailable  = 0;  // draw() スレッドのみ更新

void setup() {
  size(800, 800);
  pixelDensity(displayDensity());
  smooth();
  textFont(createFont("Monospaced", 11));
  lastFrameMs = millis();
  simStartMs  = millis();

  try {
    port = new Serial(this, SERIAL_PORT, BAUD);
    port.bufferUntil('\n');
  } catch (Exception e) {
    println("シリアルポートに接続できません: " + SERIAL_PORT);
    println("利用可能なポート:");
    printArray(Serial.list());
  }
}

void serialEvent(Serial p) {
  String line = p.readStringUntil('\n');
  if (line == null || line.trim().isEmpty()) return;
  String trimmed = line.trim();
  rawLineCount++;
  lastRawLineMs = millis();
  lastRawLine = trimmed.length() > 80 ? trimmed.substring(0, 80) + "…" : trimmed;
  parseLine(trimmed);
}

void parseLine(String line) {
  if (line.length() < 2 || line.charAt(0) != '{' || line.charAt(line.length()-1) != '}') {
    parseErrorCount++;
    return;  // framing error — drop without logging (can flood console at 100 Hz)
  }
  try {
    JSONObject json = JSONObject.parse(line);
    String type = json.getString("t", "");

    if (type.equals("i")) {
      motion.applyImu(json);
    } else if (type.equals("scan")) {
      String state = json.getString("state", "");
      if (state.equals("begin")) {
        motion.saveSnapshot();
        synchronized(pendingAps) { pendingAps.clear(); }
        motion.scanActive = true;
      } else {
        motion.scanActive = false;
        boolean hasPending;
        synchronized(pendingAps) { hasPending = !pendingAps.isEmpty(); }
        if (hasPending && pendingFifoBurst == null) applyPendingAps();  // fallback: no "fb" packet
      }
    } else if (type.equals("s")) {
      synchronized(pendingAps) { pendingAps.add(json); }
    } else if (type.equals("fb")) {
      pendingFifoBurst = json;
    }
  } catch (Exception e) {
    parseErrorCount++;
    println("JSON parse failed: " + line);
  }
}

void draw() {
  long now = millis();
  float dt = constrain((now - lastFrameMs) / 1000.0f, 0.0f, 0.08f);
  lastFrameMs = now;

  if (SIMULATE_INPUT) feedSimulatedData(now);
  if (port != null) bytesAvailable = port.available();

  if (pendingFifoBurst != null) {
    applyFifoBurst(pendingFifoBurst);
    pendingFifoBurst = null;
  }

  motion.update(dt);
  updateCameraPan(dt);
  recordPath();
  cameraX = motion.userX + cameraOffsetX;
  cameraY = motion.userY + cameraOffsetY;

  background(0);
  drawRings();
  drawPath();
  drawAPs();
  drawScanCaptures();
  drawUser();
  drawInfo();
}

void applyFifoBurst(JSONObject json) {
  if (motion.snap == null) return;  // no snapshot yet — "begin" never received
  int n    = json.getInt("n", 0);
  int dtMs = json.getInt("dt", 40);
  int ov   = json.getInt("ov", 0);
  JSONArray d = json.getJSONArray("d");
  if (d == null) return;

  float dt = dtMs / 1000.0f;

  motion.restoreSnapshot();
  boolean prevScanActive = motion.scanActive;
  motion.scanActive = false;

  for (int i = 0; i < n; i++) {
    int base = i * 3;
    motion.setFifoSample(d.getFloat(base), d.getFloat(base + 1), d.getFloat(base + 2));
    motion.update(dt);
  }

  if (ov == 1) {
    long totalMs   = (long)json.getInt("te", 0) - (long)json.getInt("ts", 0);
    long coveredMs = (long)n * dtMs;
    int extraSteps = (int)max(0, (totalMs - coveredMs) / dtMs);
    for (int i = 0; i < extraSteps; i++) {
      motion.lastImuMs = millis();  // keep canPredict=true during extrapolation
      motion.update(dt);
    }
  }

  // heading/velocity applied immediately; only position is lerp-corrected
  motion.beginCorrection(motion.userX, motion.userY);

  motion.scanActive = prevScanActive;
  applyPendingAps();
}

void applyPendingAps() {
  ArrayList<JSONObject> batch;
  synchronized(pendingAps) {
    batch = new ArrayList<JSONObject>(pendingAps);
    pendingAps.clear();
  }
  synchronized(apMap) {
    for (JSONObject apJson : batch) {
      JSONArray aps = apJson.getJSONArray("a");
      if (aps == null) continue;
      for (int i = 0; i < aps.size(); i++) {
        JSONObject ap = aps.getJSONObject(i);
        String bssid = ap.getString("b");
        String ssid  = ap.getString("n", "??");
        int rssi = ap.getInt("r");
        int ch   = ap.getInt("c");
        if (apMap.containsKey(bssid)) {
          apMap.get(bssid).update(ssid, rssi, ch, motion.userX, motion.userY);
        } else {
          apMap.put(bssid, new AP(ssid, bssid, rssi, ch, motion.userX, motion.userY));
        }
      }
    }
  }
  maybeCaptureScan(batch);
  tryTrilaterate();
}

void maybeCaptureScan(ArrayList<JSONObject> batch) {
  if (!captureArmed || batch.isEmpty()) return;

  ScanCapture capture = new ScanCapture(nextCaptureSlot, motion.userX, motion.userY);
  for (JSONObject apJson : batch) {
    JSONArray aps = apJson.getJSONArray("a");
    if (aps == null) continue;
    for (int i = 0; i < aps.size(); i++) {
      JSONObject ap = aps.getJSONObject(i);
      String bssid = ap.getString("b", "");
      if (bssid.isEmpty()) continue;
      String ssid = ap.getString("n", "??");
      int rssi = ap.getInt("r");
      int ch = ap.getInt("c");
      capture.add(new ScanObservation(ssid, bssid, rssi, ch));
    }
  }

  if (capture.count() == 0) return;

  if (nextCaptureSlot == 1) {
    scan1 = capture;
    scan2 = null;
    nextCaptureSlot = 2;
  } else {
    scan2 = capture;
    nextCaptureSlot = 1;
  }
  captureArmed = false;
}

PVector trilaterate(ArrayList<AP> anchors) {
  AP ref = anchors.get(0);
  float x0 = ref.worldX, y0 = ref.worldY, d0 = ref.dist;
  float ATA00=0, ATA01=0, ATA11=0, ATb0=0, ATb1=0;
  for (int i = 1; i < anchors.size(); i++) {
    AP ap = anchors.get(i);
    float a0 = 2*(ap.worldX - x0);
    float a1 = 2*(ap.worldY - y0);
    float bi = ap.worldX*ap.worldX - x0*x0
              + ap.worldY*ap.worldY - y0*y0
              - ap.dist*ap.dist + d0*d0;
    ATA00 += a0*a0; ATA01 += a0*a1; ATA11 += a1*a1;
    ATb0  += a0*bi; ATb1  += a1*bi;
  }
  float det = ATA00*ATA11 - ATA01*ATA01;
  if (abs(det) < 0.001f) return null;
  return new PVector((ATA11*ATb0 - ATA01*ATb1) / det,
                     (ATA00*ATb1 - ATA01*ATb0) / det);
}

void tryTrilaterate() {
  ArrayList<AP> anchors = new ArrayList<AP>();
  synchronized(apMap) {
    for (AP ap : apMap.values()) {
      if (ap.hitCount >= TRILATERATION_MIN_HITS) anchors.add(ap);
    }
  }
  if (anchors.size() < TRILATERATION_MIN_APS) return;

  PVector result = trilaterate(anchors);
  if (result == null) return;
  if (dist(motion.userX, motion.userY, result.x, result.y) > TRILATERATION_MAX_ERR_M) return;

  motion.userX = lerp(motion.userX, result.x, TRILATERATION_BLEND);
  motion.userY = lerp(motion.userY, result.y, TRILATERATION_BLEND);
}

void drawRings() {
  float cx = toScreenX(motion.userX);
  float cy = toScreenY(motion.userY);
  noFill();
  for (float r : RINGS) {
    float px = r * metersToPx;
    float alpha = map(r, 1, 10, 120, 40);
    stroke(0, 160, 0, alpha);
    strokeWeight(1);
    ellipse(cx, cy, px * 2, px * 2);

    fill(0, 120, 0, alpha);
    noStroke();
    textSize(10);
    text(nf(r, 0, 0) + "m", cx + px + 3, cy);
  }
}

void drawPath() {
  noFill();
  stroke(0, 110, 0, 150);
  strokeWeight(1);
  beginShape();
  for (PVector p : path) {
    vertex(toScreenX(p.x), toScreenY(p.y));
  }
  endShape();

  float px = toScreenX(0);
  float py = toScreenY(0);
  stroke(0, 80, 0, 130);
  strokeWeight(1);
  line(px - 5, py, px + 5, py);
  line(px, py - 5, px, py + 5);
}

void drawAPs() {
  synchronized(apMap) {
    for (AP ap : apMap.values()) {
      if (dist(ap.worldX, ap.worldY, motion.userX, motion.userY) > MAX_RADIUS_M) continue;
      float x = ap.screenX();
      float y = ap.screenY();
      if (x < -80 || x > width + 80 || y < -80 || y > height + 80) continue;

      float a = ap.alpha();
      float s = ap.dotSize();

      noStroke();
      fill(0, 255, 0, a * 0.5f);
      ellipse(x, y, s * 2, s * 2);
      fill(0, 255, 80, a);
      ellipse(x, y, s, s);

      fill(0, 200, 0, a);
      textSize(10);
      String label = ap.ssid.isEmpty() ? "[hidden]" : ap.ssid;
      text(label, x + s / 2 + 3, y - 2);
      fill(0, 140, 0, a);
      text(ap.rssi + "dBm  " + nf(dist(ap.worldX, ap.worldY, motion.userX, motion.userY), 0, 1) + "m", x + s / 2 + 3, y + 10);
    }
  }
}

void drawScanCaptures() {
  if (scan1 == null && scan2 == null) return;

  int col1 = color(0, 190, 255);
  int col2 = color(255, 130, 0);
  int colHit = color(255, 255, 255);
  if (scan1 != null) drawScanCapture(scan1, col1, "P1");
  if (scan2 != null) drawScanCapture(scan2, col2, "P2");

  if (scan1 == null || scan2 == null) return;

  int idx = 0;
  for (String bssid : scan1.observations.keySet()) {
    if (!scan2.observations.containsKey(bssid)) continue;

    ScanObservation a = scan1.observations.get(bssid);
    ScanObservation b = scan2.observations.get(bssid);
    PVector[] hits = circleIntersections(scan1.userX, scan1.userY, a.dist,
                                         scan2.userX, scan2.userY, b.dist);
    String label = displaySsid(a.ssid) + " " + shortBssid(bssid);

    if (hits == null) {
      float mx = (scan1.userX + scan2.userX) * 0.5f;
      float my = (scan1.userY + scan2.userY) * 0.5f;
      fill(255, 80, 80, 200);
      noStroke();
      textSize(10);
      text(label + " no hit", toScreenX(mx) + 8, toScreenY(my) + 12 + idx * 12);
      idx++;
      continue;
    }

    for (int i = 0; i < hits.length; i++) {
      float x = toScreenX(hits[i].x);
      float y = toScreenY(hits[i].y);
      stroke(colHit, 230);
      strokeWeight(2.0f);
      line(x - 6, y - 6, x + 6, y + 6);
      line(x - 6, y + 6, x + 6, y - 6);
    }

    fill(255, 230);
    noStroke();
    textSize(10);
    text(label, toScreenX(hits[0].x) + 10, toScreenY(hits[0].y) - 4);
    text(a.rssi + "/" + b.rssi + "dBm  " + nf(a.dist, 0, 1) + "/" + nf(b.dist, 0, 1) + "m",
         toScreenX(hits[0].x) + 10, toScreenY(hits[0].y) + 8);
  }
}

void drawScanCapture(ScanCapture capture, int col, String label) {
  float cx = toScreenX(capture.userX);
  float cy = toScreenY(capture.userY);

  stroke(col, 55);
  strokeWeight(1);
  noFill();
  for (ScanObservation obs : capture.observations.values()) {
    float px = obs.dist * metersToPx;
    ellipse(cx, cy, px * 2, px * 2);
  }

  stroke(col, 230);
  strokeWeight(2);
  fill(0, 210);
  ellipse(cx, cy, 18, 18);
  fill(col, 240);
  noStroke();
  ellipse(cx, cy, 8, 8);
  textSize(11);
  text(label + " " + capture.count() + "AP", cx + 12, cy - 8);
}

void drawUser() {
  float cx = toScreenX(motion.userX);
  float cy = toScreenY(motion.userY);

  float arrowLen = 28;
  float arrowX = cx + cos(motion.heading) * arrowLen;
  float arrowY = cy + sin(motion.heading) * arrowLen;
  stroke(255, 255, 255, 200);
  strokeWeight(2);
  line(cx, cy, arrowX, arrowY);

  float tipAngle = atan2(arrowY - cy, arrowX - cx);
  fill(255);
  noStroke();
  pushMatrix();
  translate(arrowX, arrowY);
  rotate(tipAngle);
  triangle(6, 0, -4, -4, -4, 4);
  popMatrix();

  fill(255);
  noStroke();
  ellipse(cx, cy, 8, 8);

  if (motion.moving == 1) {
    stroke(255, 200, 0, 180);
    strokeWeight(1.5f);
    noFill();
    ellipse(cx, cy, 20, 20);
  }
}

void drawInfo() {
  int apCount;
  synchronized(apMap) { apCount = apMap.size(); }

  fill(0, 180, 0, 220);
  noStroke();
  textSize(11);
  text("AP: " + apCount, 12, 18);
  text("HDG: " + nf(normalizedDegrees(motion.heading), 0, 1) + " deg", 12, 32);
  text(motion.moving == 1 ? "MOVING" : "STILL", 12, 46);
  text("SCAN: " + (motion.scanActive ? "ON" : "OFF"), 12, 60);

  if (debugOverlay) {
    int age = (motion.lastImuMs == 0) ? -1 : (int)(millis() - motion.lastImuMs);
    text("POS: " + nf(motion.userX, 0, 2) + "," + nf(motion.userY, 0, 2) + "m", 12, 78);
    text("SPD: " + nf(motion.speed(), 0, 2) + "m/s", 12, 92);
    text("IMU age: " + age + "ms", 12, 106);
    text("FIFO: " + (motion.correcting ? "corr" : "ok"), 12, 120);
    int anchorCount = 0;
    synchronized(apMap) {
      for (AP ap : apMap.values()) if (ap.hitCount >= TRILATERATION_MIN_HITS) anchorCount++;
    }
    text("TRIL anchors: " + anchorCount + "/" + apMap.size(), 12, 134);
    text("CAPTURE: " + captureStatus(), 12, 148);
    text("ZOOM: " + nf(metersToPx / METERS_TO_PX, 0, 2) + "x", 12, 162);

    int rawAge = (lastRawLineMs == 0) ? -1 : (int)(millis() - lastRawLineMs);
    boolean portOpen = (port != null);
    fill(portOpen ? (rawAge >= 0 && rawAge < 2000 ? color(0, 200, 255) : color(255, 160, 0)) : color(255, 80, 80), 220);
    text("SERIAL: " + (portOpen ? "open" : "closed") + "  rx=" + rawLineCount + "  err=" + parseErrorCount + "  buf=" + bytesAvailable, 12, 180);
    if (rawAge >= 0) {
      text("last rx: " + rawAge + "ms ago", 12, 194);
      text(lastRawLine, 12, 208);
    }
  }

  if (port == null && !SIMULATE_INPUT) {
    fill(255, 80, 80, 220);
    textSize(12);
    text("シリアル未接続: " + SERIAL_PORT, WIN_SIZE / 2 - 160, WIN_SIZE - 54);
    textSize(11);
    String[] ports = Serial.list();
    text("利用可能ポート (" + ports.length + "):", WIN_SIZE / 2 - 160, WIN_SIZE - 38);
    String portList = "";
    for (int i = 0; i < min(ports.length, 4); i++) portList += (i > 0 ? " | " : "") + ports[i];
    text(portList.isEmpty() ? "(なし)" : portList, WIN_SIZE / 2 - 160, WIN_SIZE - 22);
  }
}

float toScreenX(float worldX) {
  return WIN_SIZE / 2.0f + (worldX - cameraX) * metersToPx;
}

float toScreenY(float worldY) {
  return WIN_SIZE / 2.0f + (worldY - cameraY) * metersToPx;
}

float applyDeadband(float v) {
  return abs(v) < ACCEL_DEADBAND ? 0 : v;
}

float wrapAngle(float a) {
  while (a > PI) a -= TWO_PI;
  while (a < -PI) a += TWO_PI;
  return a;
}

float normalizedDegrees(float rad) {
  float deg = degrees(rad) % 360.0f;
  return deg < 0 ? deg + 360.0f : deg;
}

// Free-space path loss: d = 10^((A - rssi) / (10*n)), A=-59dBm@1m, n=2.5
float rssiToDistance(int rssi) {
  return pow(10.0f, (-59.0f - rssi) / 25.0f);
}

PVector[] circleIntersections(float x0, float y0, float r0, float x1, float y1, float r1) {
  float dx = x1 - x0;
  float dy = y1 - y0;
  float d = sqrt(dx * dx + dy * dy);
  if (d < 0.001f) return null;
  if (d > r0 + r1) return null;
  if (d < abs(r0 - r1)) return null;

  float a = (r0 * r0 - r1 * r1 + d * d) / (2.0f * d);
  float h2 = r0 * r0 - a * a;
  if (h2 < -0.001f) return null;
  float h = sqrt(max(0.0f, h2));

  float xm = x0 + a * dx / d;
  float ym = y0 + a * dy / d;
  float rx = -dy * h / d;
  float ry = dx * h / d;

  return new PVector[] {
    new PVector(xm + rx, ym + ry),
    new PVector(xm - rx, ym - ry)
  };
}

String shortBssid(String bssid) {
  return bssid.length() <= 5 ? bssid : bssid.substring(bssid.length() - 5);
}

String displaySsid(String ssid) {
  return ssid == null || ssid.isEmpty() ? "[hidden]" : ssid;
}

String captureStatus() {
  String state = captureArmed ? ("armed P" + nextCaptureSlot) : "idle";
  int n1 = scan1 == null ? 0 : scan1.count();
  int n2 = scan2 == null ? 0 : scan2.count();
  return state + "  P1=" + n1 + " P2=" + n2;
}

void updateCameraPan(float dt) {
  float step = CAMERA_PAN_SPEED * dt;
  if (panUp) cameraOffsetY += step;
  if (panDown) cameraOffsetY -= step;
  if (panLeft) cameraOffsetX += step;
  if (panRight) cameraOffsetX -= step;
}

void zoomMap(float factor) {
  metersToPx = constrain(metersToPx * factor, MIN_METERS_TO_PX, MAX_METERS_TO_PX);
}

void recordPath() {
  if (path.isEmpty()) {
    path.add(new PVector(motion.userX, motion.userY));
    return;
  }
  PVector last = path.get(path.size() - 1);
  if (dist(last.x, last.y, motion.userX, motion.userY) >= 0.08f) {
    path.add(new PVector(motion.userX, motion.userY));
    if (path.size() > 500) path.remove(0);
  }
}

void feedSimulatedData(long now) {
  if (now - lastSimMs < 50) return;
  lastSimMs = now;
  float t = (now - simStartMs) / 1000.0f;

  if ((int)t % 4 == 2) {
    if (!motion.scanActive) parseLine("{\"t\":\"scan\",\"state\":\"begin\",\"t_ms\":" + now + "}");
  } else if (motion.scanActive) {
    parseLine("{\"t\":\"scan\",\"state\":\"end\",\"t_ms\":" + now + ",\"n\":2}");
    parseLine("{\"t\":\"s\",\"a\":[{\"n\":\"Lab\",\"b\":\"00:11:22:33:44:55\",\"r\":-55,\"c\":6},{\"n\":\"Phone\",\"b\":\"AA:BB:CC:DD:EE:FF\",\"r\":-67,\"c\":11}]}");
  }

  if (!motion.scanActive) {
    float gz = sin(t * 0.7f) * 0.25f;
    float ax = 0.25f + sin(t * 2.0f) * 0.12f;
    parseLine("{\"t\":\"i\",\"t_ms\":" + now + ",\"gz\":" + gz + ",\"ax\":" + ax + ",\"ay\":0.02,\"mv\":1}");
  }
}

void armNextScanCapture() {
  if (scan1 != null && scan2 != null) {
    scan1 = null;
    scan2 = null;
    nextCaptureSlot = 1;
  } else if (scan1 != null) {
    nextCaptureSlot = 2;
  } else {
    nextCaptureSlot = 1;
  }
  captureArmed = true;
}

// WASD: マップ移動 / <>: ズーム / R: 姿勢リセット / C: AP/2点観測クリア / G: デバッグ表示 / Space: 次スキャンを2点観測に記録
void keyPressed() {
  if (key == 'w' || key == 'W') {
    panUp = true;
  } else if (key == 's' || key == 'S') {
    panDown = true;
  } else if (key == 'a' || key == 'A') {
    panLeft = true;
  } else if (key == 'd' || key == 'D') {
    panRight = true;
  } else if (key == 'r' || key == 'R') {
    motion.resetPose();
    path.clear();
  } else if (key == 'c' || key == 'C') {
    synchronized(apMap) { apMap.clear(); }
    scan1 = null;
    scan2 = null;
    captureArmed = false;
    nextCaptureSlot = 1;
  } else if (key == 'g' || key == 'G') {
    debugOverlay = !debugOverlay;
  } else if (key == ' ') {
    armNextScanCapture();
  } else if (key == '<' || key == ',') {
    zoomMap(1.0f / ZOOM_STEP);
  } else if (key == '>' || key == '.') {
    zoomMap(ZOOM_STEP);
  }
}

void keyReleased() {
  if (key == 'w' || key == 'W') {
    panUp = false;
  } else if (key == 's' || key == 'S') {
    panDown = false;
  } else if (key == 'a' || key == 'A') {
    panLeft = false;
  } else if (key == 'd' || key == 'D') {
    panRight = false;
  }
}
