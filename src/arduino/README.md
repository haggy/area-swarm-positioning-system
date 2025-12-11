
# Localized Positioning System PoC

This is a proof of concept implementation of a localized, Received Signal Strength Indicator (RSSI) based positioning system using LoRa (Long Range) radio modules . The system functions by defining nodes as either **Beacons** or **Receivers** :
* **Beacons:** Fixed nodes with known coordinates that periodically broadcast their identity and location .
* **Receivers:** Mobile nodes that listen for beacon signals, estimate their distance from each beacon using a log-distance path loss model based on signal strength (RSSI) , and calculate their own $(x, y)$ position relative to the network .

The position calculation utilizes a **Weighted Least Squares (WLS)** iterative algorithm (specifically a Gauss-Newton approach) to minimize the error between estimated signal distances and geometric distances .

---

### **2. Data Structures Overview**
The system relies on three primary structures to manage state, configuration, and messaging.

* **`struct Point`**
    A simple coordinate structure used for both fixed beacon locations and the estimated mobile receiver location.
    * **Fields:** `float x`, `float y` .

* **`struct BeaconMsg`**
    The packet structure transmitted over the air by beacons.
    * **Fields:**
        * `uint16_t id`: The unique identifier of the beacon .
        * `Point location`: The hardcoded physical coordinates of the beacon .
        * `int rssi`: Signal strength (filled by the receiver upon arrival, not sent by the beacon) .

* **`struct NodeConfiguration`**
    The primary configuration object for every device in the network.
    * **Fields:**
        * `uint16_t mode`: Bitmask determining if the device is a `MODE_BEACON` or `MODE_RECEIVER` .
        * `uint16_t id`: Unique identifier matching the `NODES` array index .
        * `Point location`: The known (if beacon) or estimated (if receiver) location .
        * `float dist_est`: The current estimated distance from this specific node (derived from RSSI) .
        * `float weight`: The weight used in the least squares calculation, defined as $\frac{1}{dist^2}$ .

---

### **3. Function Overview**
The following functions control the hardware and execution logic.

#### **Initialization & Hardware**
* **`setup()`**: The entry point. Initializes Serial and LoRa modules based on the node's configured mode .
* **`initSerial(const NodeConfiguration nc)`**: Starts the serial monitor, but only if the device is a Receiver (Beacons run silently) .
* **`initLoraModule(const NodeConfiguration nc)`**: Initializes the LoRa radio at 915MHz .
* **`checkLightCycle()` / `toggleLED()`**: Manages a non-blocking LED blink to indicate system uptime/heartbeat .

#### **Core Logic Loop**
* **`loop()`**: The main cycle. Checks the node mode and delegates execution to either the beacon or receiver loop function .
* **`executeLoopIteration(const NodeConfiguration nc)`**: Router function that calls the specific iteration function based on the mode flag .
* **`executeBeaconIteration(const NodeConfiguration nc)`**: Constructs a `BeaconMsg` with the node's ID and location, then broadcasts it .
* **`executeReceiverIteration(const NodeConfiguration nc)`**: The main receiver workflow. Calls `getCurrentLocation` to process incoming packets and update position estimates .

#### **Comms & Parsing**
* **`LoRa_sendMessage(const uint8_t *buf, size_t size)`**: Wraps the LoRa library calls to transmit a raw byte buffer .
* **`sendBeaconMsg(const BeaconMsg msg)`**: Serializes a message struct into a byte buffer and transmits it .
* **`readBeaconMsg(uint8_t *buf, size_t bufSize)`**: Reads available bytes from the LoRa stream into a buffer and casts it to a `BeaconMsg` pointer .
* **`validBeaconMessage(const BeaconMsg msg)`**: Validates that the received beacon ID is within the expected range of the `NODES` array .

#### **Math & Positioning**
* **`rssi2dist(float rssi)`**: Converts Received Signal Strength Indicator (RSSI) to a distance in meters using the formula $d = 10^{\frac{TX - RSSI}{10n}}$ .
* **`getDist(Point p1, Point p2)`**: Calculates the Euclidean distance between two points: $\sqrt{(x_1-x_2)^2 + (y_1-y_2)^2}$ .
* **`updateBeaconDistance(const BeaconMsg msg, NodeConfiguration &beacon)`**: Updates a specific beacon's distance estimate based on the latest packet and recalculates its weight for the solver .
* **`estimatePosition(Point &est_pos)`**: The core solver. It performs a Weighted Least Squares optimization (up to `MAX_ITER` times) to refine the receiver's coordinates ($x, y$) based on the estimated distances to all beacons . It constructs and solves the normal equations $A\delta = b$ via Cramer's Rule .
* **`getCurrentLocation(const NodeConfiguration nc)`**: The high-level receiver controller. It checks for new packets, updates distance estimates, runs the `estimatePosition` solver, and prints telemetry to Serial .
