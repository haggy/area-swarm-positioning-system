#include <SPI.h>
#include <LoRa.h>
#include <cstring>
#include <vector>
#include <iomanip>

typedef struct Point {
  float x;
  float y;
} Point;

typedef struct BeaconMsg {
    uint16_t id;
    Point location;
    int rssi;
} BeaconMsg;

typedef struct NodeConfiguration {
    uint16_t mode;
    uint16_t id;
    Point location;
    float dist_est; // Estimated distance from the beacon
    float weight;   // 1 / dist^2 (inverse square law)
} NodeConfiguration;


constexpr uint8_t MODE_BEACON = 1;
constexpr uint8_t MODE_RECEIVER = 1 << 2;
constexpr Point DEFAULT_LOC = { .x = 0, .y = 0 };
constexpr int ONE_SEC_MS = 1000;
constexpr int MAX_ITER = 20;
constexpr int TX_POWER_1M = -60; // dBm
constexpr float N_EXPONENT = 2.5;    // Path loss exponent

// Primary MUTABLE map of nodes (mostly for beacon access)
// The ID needs to always match the array idx
NodeConfiguration NODES[] = {
    {.mode = MODE_BEACON, .id = 0, .location = { .x = 5, .y = 5}, .dist_est = 0.0, .weight = 1.0},
    {.mode = MODE_BEACON, .id = 1, .location = { .x = 0, .y = 0}, .dist_est = 0.0, .weight = 1.0},
    {.mode = MODE_BEACON, .id = 2, .location = { .x = 0, .y = 10}, .dist_est = 0.0, .weight = 1.0},
    {.mode = MODE_BEACON, .id = 3, .location = { .x = 10, .y = 10}, .dist_est = 0.0, .weight = 1.0},
    {.mode = MODE_RECEIVER, .id = 4, .location = { .x = 0, .y = 0}, .dist_est = 0.0, .weight = 1.0}
};

// "Map" of node ID to latest message state
BeaconMsg LATEST_MSGS[] = {
  {.id = 0, .location = { .x = 0, .y = 0 }, .rssi = 0},
  {.id = 0, .location = { .x = 0, .y = 0 }, .rssi = 0},
  {.id = 0, .location = { .x = 0, .y = 0 }, .rssi = 0},
  {.id = 0, .location = { .x = 0, .y = 0 }, .rssi = 0}
};

// Keep track of last known (estimated) position
Point lastEstimatedPos = { .x = 0, .y = 0 };

// =================================
// Set the system configuration here!!
uint16_t DEPLOYED_NODE_ID = 4;
constexpr int NUM_BEACONS = 3;
int DISABLE_WEIGHTS = 1;
NodeConfiguration NODE_CONF = NODES[DEPLOYED_NODE_ID];
// =================================

// Tracks last uptime value
unsigned long previousUptime = 0;


bool isMode(const NodeConfiguration nc, const uint8_t expectedMode) {
  return nc.mode & expectedMode;
}

void initSerial(const NodeConfiguration nc) {
  if (isMode(nc, MODE_BEACON)) {
    return;
  }

  Serial.begin(9600);
  while (!Serial);
  Serial.println("Serial started BAUDRATE=9600");
}

void initLoraModule(const NodeConfiguration nc) {
  Serial.println("Init Lora comms");
  Serial.println("Starting LoRa module...");
  if (!LoRa.begin(915E6)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }

  Serial.println("Lora comms init complete");
}

void LoRa_sendMessage(const uint8_t *buf, size_t size) {
  LoRa.beginPacket(true); 
  LoRa.write(buf, size);
  LoRa.endPacket();
}

void setup() {
  initSerial(NODE_CONF);

  if (isMode(NODE_CONF, MODE_BEACON)) {
    Serial.println("--Initializing Beacon--");
  } else {
    Serial.println("--Initializing Receiver--");
  }

  initLoraModule(NODE_CONF);
}

void checkLightCycle() {
  unsigned long currentUptime = millis();
  
  if (currentUptime >= previousUptime + ONE_SEC_MS / 5) {
    toggleLED();
    previousUptime = currentUptime;
  }
}

void toggleLED() {
  pinMode(LED_BUILTIN, OUTPUT);
  int output = digitalRead(LED_BUILTIN);
  pinMode(LED_BUILTIN, INPUT);
  digitalWrite(LED_BUILTIN, !output);
}

void updateLatestMsgState(const BeaconMsg msg) {
  auto & currState = LATEST_MSGS[msg.id];
  currState.id = msg.id;
  currState.location.x = msg.location.x;
  currState.location.y = msg.location.y;
  currState.rssi = msg.rssi;
}

bool validBeaconMessage(const BeaconMsg msg) {
  if (msg.id > NUM_BEACONS - 1) {
    return false;
  }

  return true;
}

Point getCurrentLocation(const NodeConfiguration nc) {
  if (isMode(nc, MODE_BEACON)) {
    return nc.location;
  }

  int psize = LoRa.parsePacket(sizeof(BeaconMsg));
  if (psize) {
    uint8_t buf[sizeof(BeaconMsg)];
    BeaconMsg* msg = readBeaconMsg(buf, sizeof(BeaconMsg));
    msg->rssi = LoRa.packetRssi();

    if (!validBeaconMessage(*msg)) {
      return DEFAULT_LOC;
    }

    updateLatestMsgState(*msg);

    auto & beacon = NODES[msg->id];
    updateBeaconDistance(*msg, beacon);

    estimatePosition(lastEstimatedPos);

    Serial.print("ID="); Serial.print(LATEST_MSGS[msg->id].id);
    Serial.print(" LOC=("); Serial.print(LATEST_MSGS[msg->id].location.x);
    Serial.print(", "); Serial.print(LATEST_MSGS[msg->id].location.y); Serial.print(")");
    Serial.print(" RSSI="); Serial.print(LATEST_MSGS[msg->id].rssi);
    Serial.print(" DE="); Serial.print(NODES[msg->id].dist_est);
    Serial.print(" W="); Serial.print(NODES[msg->id].weight);
    Serial.print(" EP=("); Serial.print(lastEstimatedPos.x);
    Serial.print(", "); Serial.print(lastEstimatedPos.y); Serial.print(")");
    Serial.println();
    
    if (!sufficientDataInitComplete()) {
      Serial.println("--- Data has not been received from all beacons. Skipping distance calculation ---");
    }
  }

  return DEFAULT_LOC;
}

// Euclidean distance between two points
float getDist(Point p1, Point p2) {
  
  // double powX = pow(p1.x - p2.x, 2);
  // double powY = pow(p1.y - p2.y, 2);
  // double dist = sqrt(powX + powY);
  // Serial.print("-- IN getDist: ");
  // Serial.print("p1=("); Serial.print(p1.x); Serial.print(","); Serial.print(p1.y); Serial.print(")");
  // Serial.print("p2=("); Serial.print(p2.x); Serial.print(","); Serial.print(p2.y); Serial.print(")");
  // Serial.print("powX="); Serial.print(powX);
  // Serial.print(" powY="); Serial.print(powY);
  // Serial.print(" dist="); Serial.print(dist);
  // Serial.println();

  return sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2));
}

float rssi2dist(float rssi) {
    // d = 10 ^ ((Tx - RSSI) / (10*n))
    float exponent = (TX_POWER_1M - rssi) / (10.0 * N_EXPONENT);
    return abs(static_cast<float>(std::pow(10.0, exponent)));
}

void updateBeaconDistance(const BeaconMsg msg, NodeConfiguration &beacon) {
    // Convert back to distance
    beacon.dist_est = rssi2dist(msg.rssi);

    // Calculate Weight (Inverse Square Law)
    // We add a small epsilon to avoid div by zero if dist is 0
    beacon.weight = (DISABLE_WEIGHTS ? 1.0 : 1.0 / static_cast<float>(std::pow(beacon.dist_est, 2)));
}

void estimatePosition(Point &est_pos) {
  // Serial.print("--- IN EP:");
  // Serial.print(" est_pos.x="); Serial.print(est_pos.x); Serial.print(" est_pos.y="); Serial.print(est_pos.y);
  // Serial.println();
    for (int iter = 0; iter < MAX_ITER; ++iter) {
        // We need to solve the Normal Equations: (H^T * W * H) * correction = H^T * W * dP
        // Let A = H^T * W * H  (2x2 matrix)
        // Let b = H^T * W * dP (2x1 vector)

        float A[2][2] = {{0.0, 0.0}, {0.0, 0.0}};
        float b[2] = {0.0, 0.0};

        for (int i = 0; i < NUM_BEACONS; ++i) {
            auto beacon = NODES[i];
            // Serial.print(" beacon.id="); Serial.print(beacon.id); 
            // Serial.print(" beacon.location=("); Serial.print(beacon.location.x); Serial.print(","); Serial.print(beacon.location.y); Serial.print(")");
            // Serial.print(" beacon.dist_est="); Serial.print(beacon.dist_est); 

            float geo_dist = getDist(est_pos, beacon.location);
            float residual = beacon.dist_est - geo_dist; // dP
            float w = beacon.weight;

            // Serial.print(" geo_dist="); Serial.print(geo_dist);
            // Serial.print(" residual="); Serial.print(residual);
            // Serial.print(" w="); Serial.print(w);
            // Serial.println();

            // Jacobian Row elements (H_i)
            // H_x = (x - bx) / dist
            // H_y = (y - by) / dist
            if (geo_dist < 1e-5) geo_dist = 1e-5; // Prevent div by zero
            float h_x = (est_pos.x - beacon.location.x) / geo_dist;
            float h_y = (est_pos.y - beacon.location.y) / geo_dist;

            // Accumulate into A = H^T * W * H
            // A_00 += w * h_x * h_x
            A[0][0] += w * h_x * h_x;
            A[0][1] += w * h_x * h_y;
            A[1][0] += w * h_y * h_x; // Symmetric
            A[1][1] += w * h_y * h_y;

            // Accumulate into b = H^T * W * dP
            b[0] += w * h_x * residual;
            b[1] += w * h_y * residual;
        }

        // Solve 2x2 system A * x = b using Cramer's Rule / Determinant
        float det = A[0][0] * A[1][1] - A[0][1] * A[1][0];

        // Serial.print("["); Serial.print(A[0][0]); Serial.print(" "); Serial.print(A[0][1]); Serial.println();
        // Serial.print(A[1][0]); Serial.print(" "); Serial.print(A[1][1]); Serial.println("]");
        // Serial.print(" DET="); Serial.print(det); 
        // Serial.println();
        if (abs(det) < 1e-9) {
            Serial.println(" EEEEEEEE Singular matrix, stopping EEEEEEEEE ");
            break;
        }

        float invDet = 1.0 / det;

        // Inverse of 2x2:
        // [ d  -b ]
        // [ -c  a ] * (1/det)
        float corr_x = invDet * (A[1][1] * b[0] - A[0][1] * b[1]);
        float corr_y = invDet * (-A[1][0] * b[0] + A[0][0] * b[1]);

        // Update Position
        est_pos.x += corr_x;
        est_pos.y += corr_y;

        // Serial.print("Iteration: "); Serial.print(iter);
        // Serial.print(" corr_x="); Serial.print(corr_x);
        // Serial.print(" corr_y="); Serial.print(corr_y);
        // Serial.print(" est_pos.x="); Serial.print(est_pos.x);
        // Serial.print(" est_pos.y="); Serial.print(est_pos.y);
        // Serial.println();

        // Convergence Check
        if (std::sqrt(corr_x*corr_x + corr_y*corr_y) < 1e-4) {
            break;
        }
    }
}

/**
  Returns true if we have received at least one location message from each beacon
  An unset RSSI value (0) is considered to be the empty value as it's unlikely to ever
  be set (TODO: Need to verify that this statement is true...)
**/
bool sufficientDataInitComplete() {
  for(size_t i = 0; i < NUM_BEACONS; i++) {
    auto latestMsg = LATEST_MSGS[i];
    if (latestMsg.rssi == 0) {
      // id = 0 is a default (empty) value
      return false;
    }
  }
  return true;
}

void sendBeaconMsg(const BeaconMsg msg) {
  uint8_t buf[sizeof(msg)];
  std::memcpy(buf, &msg, sizeof(msg));
  LoRa_sendMessage(buf, sizeof(buf));
  Serial.print("ID="); Serial.print(msg.id);
  Serial.print(" SIZE="); Serial.print(sizeof(buf)); Serial.print("bytes");
  Serial.println(" -- Beacon Message Sent");
}

BeaconMsg* readBeaconMsg(uint8_t *buf, size_t bufSize) {
  int idx = 0;
  while (LoRa.available()) {
    buf[idx++] = (uint8_t)LoRa.read();
  }

  return reinterpret_cast<BeaconMsg*>(buf);
}

int executeLoopIteration(const NodeConfiguration nc) {
  if (isMode(nc, MODE_BEACON)) {
    return executeBeaconIteration(nc);
  } else {
    return executeReceiverIteration(nc);
  }
}

int executeBeaconIteration(const NodeConfiguration nc) {
  Serial.print("DEVID="); Serial.print(nc.id); Serial.println(" Executing Beacon Loop");
  // BeaconMsg msg = { .id = nc.id, .location = nc.location, .rssi = 0 };
  BeaconMsg msg;
  msg.id = nc.id;
  msg.location = nc.location;
  msg.rssi = 0;
  sendBeaconMsg(msg);
  return 0;
}

int executeReceiverIteration(const NodeConfiguration nc) {
  Point loc = getCurrentLocation(nc);
  checkLightCycle();
  return 0;
}

void loop() {
  /*
    For beacons, they need to transmit a message containing the following information:
      - Beacon ID
      - Coordinates
    
    Receivers read incoming data packets from beacons and calculate position based on the 
    beacon position and RSSI value
  */

  executeLoopIteration(NODE_CONF);
  if (isMode(NODE_CONF, MODE_BEACON)) {
    delay(1000);
  }
}
