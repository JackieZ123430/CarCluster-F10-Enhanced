// ####################################################################################################################
//
// CarCluster-F10-Enhanced - BMW F10-only optimized build
// Original project: https://github.com/r00li/CarCluster
// Enhanced project: https://github.com/JackieZ123430/CarCluster-F10-Enhanced
// Better_CAN protocol: https://github.com/JackieZ123430/Better_CAN
//
// Retained inputs: Better_CAN/BeamNG UDP, Forza UDP, SimHub serial JSON, Wi-Fi dashboard.
// Other vehicle implementations have been removed from this build.
//
// ####################################################################################################################

// ----------------------------- Hardware configuration ---------------------------------------------------------------
#define SPI_CS_PIN 5
#define CAN_INT 2

#define MAXIMUM_RPM 7500
#define RPM_CORRECTION_FACTOR 1.0f
#define MAXIMUM_SPEED 260
#define SPEED_CORRECTION_FACTOR 1.0f
#define MINIMUM_COOLANT_TEMPERATURE 50
#define MAXIMUM_COOLANT_TEMPERATURE 150

#define WIFI_ENABLED 1
#define WIFI_WEB_DASHBOARD_UPDATE_INTERVAL 1000
#define WIFI_CONFIG_PORTAL_ACCESS_POINT_NAME "CarCluster-F10"
#define WIFI_CONFIG_PORTAL_ACCESS_POINT_PASSWORD "carcluster"
#define WIFI_CONFIG_PORTAL_TIMEOUT 180

#define MAX_SERIAL_MESSAGE_LENGTH 250
#define SERIAL_BAUD_RATE 921600

#define WIFI_FORZA_UDP_PORT 1101
#define WIFI_BEAM_UDP_PORT 4444
#define WIFI_WEB_DASHBOARD_PORT 80

// ----------------------------- Libraries and project modules --------------------------------------------------------
#include <SPI.h>

#include "src/Libs/ArduinoJson/ArduinoJson.h"
#include "src/Libs/MCP_CAN/mcp_can.h"
#include "src/Games/GameSimulation.h"
#include "src/Games/SimhubGame.h"
#include "src/Clusters/BMW_F/BMWFSeriesCluster.h"

MCP_CAN CAN(SPI_CS_PIN);
BMWFSeriesCluster cluster(CAN);

ClusterConfiguration defaultClusterConfig = BMWFSeriesCluster::clusterConfig();
ClusterConfiguration clusterConfig = ClusterConfiguration::updatedFromDefaults(
    defaultClusterConfig,
    SPEED_CORRECTION_FACTOR,
    RPM_CORRECTION_FACTOR,
    MAXIMUM_RPM,
    MAXIMUM_SPEED,
    MINIMUM_COOLANT_TEMPERATURE,
    MAXIMUM_COOLANT_TEMPERATURE);

GameState game(clusterConfig);
SimhubGame simhubGame(game);

#if WIFI_ENABLED == 1
#include "src/Other/WifiFunctions.h"
#include "src/Other/WebDashboard.h"
#include "src/Other/mongoose/mongoose.h"
#include "src/Other/mongoose/mongoose_glue.h"
#include "src/Games/ForzaHorizonGame.h"
#include "src/Games/BeamNGGame.h"

WifiFunctions wifiFunctions;
WebDashboard webDashboard(game, WIFI_WEB_DASHBOARD_UPDATE_INTERVAL);
ForzaHorizonGame forzaHorizonGame(game, WIFI_FORZA_UDP_PORT);
BeamNGGame beamNGGame(game, WIFI_BEAM_UDP_PORT);

void webDashboardGetState(struct state* data) {
  webDashboard.getState(data);
}

void webDashBoardSetState(struct state* data) {
  webDashboard.setState(data);
}

bool webDashboardCheckSteeringButtonPressed(void) {
  return false;
}

void webDashboardSetSteeringButtonPressed(struct mg_str params) {
  webDashboard.steeringWheelAction(params);
}
#endif

JsonDocument serialDocument;

void initializeCan();
void readSerialJson();
void drainCanReceiveBuffer();

void initializeCan() {
  while (CAN.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) != CAN_OK) {
    Serial.println("[CAN] MCP2515 initialization failed; retrying");
    delay(250);
  }

  CAN.setMode(MCP_NORMAL);
  Serial.println("[CAN] MCP2515 ready at 500 kbit/s");
}

void setup() {
  pinMode(SPI_CS_PIN, OUTPUT);
  pinMode(CAN_INT, INPUT);

  Serial.begin(SERIAL_BAUD_RATE);
  delay(300);
  Serial.println("Starting CarCluster-F10-Enhanced optimized build");

  initializeCan();
  simhubGame.begin();

#if WIFI_ENABLED == 1
  wifiFunctions.begin(
      WIFI_CONFIG_PORTAL_ACCESS_POINT_NAME,
      WIFI_CONFIG_PORTAL_ACCESS_POINT_PASSWORD,
      WIFI_CONFIG_PORTAL_TIMEOUT);

  mongoose_init();
  mg_log_set(MG_LL_ERROR);
  mongoose_set_http_handlers("state", webDashboardGetState, webDashBoardSetState);
  mongoose_set_http_handlers(
      "steering_button_pressed",
      webDashboardCheckSteeringButtonPressed,
      webDashboardSetSteeringButtonPressed);

  forzaHorizonGame.begin();
  beamNGGame.begin();
#endif
}

void loop() {
  cluster.updateWithGame(game);
  readSerialJson();
  drainCanReceiveBuffer();

#if WIFI_ENABLED == 1
  webDashboard.update();
  mongoose_poll();

  static unsigned long lastWifiReconnectAttempt = 0;
  if (WiFi.status() != WL_CONNECTED && millis() - lastWifiReconnectAttempt >= 10000) {
    lastWifiReconnectAttempt = millis();
    WiFi.reconnect();
  }
#endif

  // Yield to the ESP32 Wi-Fi/UDP tasks and avoid a 100% busy loop.
  delay(1);
}

void readSerialJson() {
  static char message[MAX_SERIAL_MESSAGE_LENGTH];
  static size_t messagePosition = 0;
  static bool droppingOversizeMessage = false;

  while (Serial.available() > 0) {
    const char incoming = static_cast<char>(Serial.read());

    if (incoming == '\r') continue;

    if (incoming != '\n') {
      if (!droppingOversizeMessage && messagePosition < sizeof(message) - 1) {
        message[messagePosition++] = incoming;
      } else {
        droppingOversizeMessage = true;
      }
      continue;
    }

    if (droppingOversizeMessage) {
      Serial.println("[Serial] ignored oversized JSON message");
      messagePosition = 0;
      droppingOversizeMessage = false;
      continue;
    }

    message[messagePosition] = '\0';
    messagePosition = 0;

    if (message[0] == '\0') continue;

    const DeserializationError error = deserializeJson(serialDocument, message);
    if (error) {
      Serial.print("[Serial] JSON error: ");
      Serial.println(error.c_str());
      continue;
    }

    const uint8_t action = serialDocument["action"] | 255;

    if (action == 0) {
      const uint32_t address = serialDocument["address"] | 0;
      if (address > 0x7FF) {
        Serial.println("[Serial] ignored invalid standard CAN ID");
        continue;
      }

      uint8_t payload[8] = {
          static_cast<uint8_t>(serialDocument["p1"] | 0),
          static_cast<uint8_t>(serialDocument["p2"] | 0),
          static_cast<uint8_t>(serialDocument["p3"] | 0),
          static_cast<uint8_t>(serialDocument["p4"] | 0),
          static_cast<uint8_t>(serialDocument["p5"] | 0),
          static_cast<uint8_t>(serialDocument["p6"] | 0),
          static_cast<uint8_t>(serialDocument["p7"] | 0),
          static_cast<uint8_t>(serialDocument["p8"] | 0)};

      CAN.sendMsgBuf(address, 0, 8, payload);
    } else if (action == 10) {
      simhubGame.decodeSerialData(serialDocument);
    }
  }
}

void drainCanReceiveBuffer() {
  // This F10-only build does not use inbound CAN data, but the MCP2515 RX buffers
  // still need to be drained to avoid overflow and a permanently asserted INT pin.
  uint8_t drained = 0;
  while (!digitalRead(CAN_INT) && drained < 8) {
    unsigned long rxId = 0;
    uint8_t length = 0;
    uint8_t payload[8] = {};
    CAN.readMsgBuf(&rxId, &length, payload);
    drained++;
  }
}
