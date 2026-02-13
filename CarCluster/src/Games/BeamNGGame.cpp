// ####################################################################################################################
// 
// Code part of CarCluster project by Andrej Rolih. See .ino file more details
// 
// ####################################################################################################################

#include "BeamNGGame.h"

BeamNGGame::BeamNGGame(GameState& game, int port): Game(game) {
  this->port = port;
}

void BeamNGGame::begin() {

  // ===== Protocol switch (can later be made runtime) =====
  static bool useCustomProtocol = true;   // true = BMW custom, false = original OutGauge

  // Start UDP listener
  if (!beamUdp.listen(4444)) {
    return;
  }

  beamUdp.onPacket([this](AsyncUDPPacket packet) {

    // ===== DEBUG: confirm UDP reception =====
    Serial.print("[UDP] packet received, size=");
    Serial.println(packet.length());

    // ---- Reset volatile indicators each frame ----
    gameState.leftTurningIndicator = false;
    gameState.rightTurningIndicator = false;
    gameState.highBeam = false;
    gameState.absLight = false;
    gameState.handbrake = false;

    if (useCustomProtocol) {

      struct __attribute__((packed)) BMWPacket {
        uint32_t time;
        float speedKmh;
        float rpm;
        char gearLetter[2];

        uint8_t ignition;
        uint8_t engineRunning;

        uint8_t doorFL;
        uint8_t doorFR;
        uint8_t doorRL;
        uint8_t doorRR;

        uint8_t parkingBrake;

        // Original ABS/ESC/TCS
        uint8_t absAvailable;
        uint8_t absActive;
        uint8_t escAvailable;
        uint8_t escActive;
        uint8_t tcsAvailable;
        uint8_t tcsActive;

        // Extended stability
        uint8_t abs;
        uint8_t isABSBrakeActive;
        uint8_t hasABS;
        uint8_t hasESC;
        uint8_t hasTCS;
        uint8_t esc;
        uint8_t tcs;
        uint8_t isTCBrakeActive;
        uint8_t isYCBrakeActive;

        // Lighting
        uint8_t highBeam;
        uint8_t lowBeam;
        uint8_t fog;
        uint8_t signalL;
        uint8_t signalR;
        uint8_t hazard;
        uint8_t brakelights;
        uint8_t battery;
        uint8_t oil;
        uint8_t checkengine;
        uint8_t lowfuel;

        // Cruise
        uint8_t cruiseControlActive;
        float   cruiseControlTarget;

        float fuel;
        float waterTemp;
        float oilTemp;
      };

      if (packet.length() < sizeof(BMWPacket)) return;

      BMWPacket data;
      memcpy(&data, packet.data(), sizeof(BMWPacket));

      if (data.gearLetter[0] == 'R') {
        gameState.gear = GearState_Auto_R;
      } else if (data.gearLetter[0] == 'N') {
        gameState.gear = GearState_Auto_N;
      } else if (data.gearLetter[0] == 'P') {
        gameState.gear = GearState_Auto_P;
      } else {
        gameState.gear = GearState_Auto_D;
      }

      static float speedFiltered = 0.0f;
      static float rpmFiltered   = 0.0f;

      speedFiltered = speedFiltered * 0.80f + data.speedKmh * 0.20f;
      rpmFiltered   = rpmFiltered   * 0.85f + data.rpm      * 0.15f;

      gameState.speed = speedFiltered;
      gameState.rpm   = rpmFiltered;

      // Temperature (use oil temp instead of water temp)
      gameState.coolantTemperature = data.oilTemp;

      // Fuel (BeamNG fuel is 0.0 ~ 1.0, convert to 0 ~ 100)
      gameState.fuelQuantity = (int)(data.fuel * 100.0f);

      gameState.highBeam  = data.highBeam;
      gameState.mainLights            = data.lowBeam;
      gameState.absLight             = data.absActive;
      gameState.handbrake            = data.parkingBrake;
      // Ignition logic:
      // 0 = off
      // 1 = accessory AND ignition ON
      gameState.ignition = (data.ignition != 0);      // ===== 300ms indicator hold logic =====
      unsigned long now = millis();
      static const unsigned long kOffTimeoutMs = 300;
      static unsigned long lastLeftSeen = 0;
      static unsigned long lastRightSeen = 0;

      if (data.signalL) {
        lastLeftSeen = now;
      }

      if (data.signalR) {
        lastRightSeen = now;
      }

      gameState.leftTurningIndicator  = (now - lastLeftSeen  <= kOffTimeoutMs);
      gameState.rightTurningIndicator = (now - lastRightSeen <= kOffTimeoutMs);

      gameState.batteryLight = data.battery;
      gameState.engineLight  = data.checkengine;
      gameState.lowFuelLight = data.lowfuel;

      // Cruise control mapping (custom BMW protocol)
      gameState.cruiseControlActive = (data.cruiseControlActive != 0);
      gameState.cruiseControlTarget = data.cruiseControlTarget;

      // You may later map these into CAN logic:
      // data.lowBeam
      // data.fog
      // data.cruiseControlActive
      // data.cruiseControlTarget
      // data.oil

    }
    else {

      if (packet.length() < 64) return;

      char dataBuff[4];

      uint8_t beamGear = *(packet.data() + 10);

      if (beamGear == 0) {
        gameState.gear = GearState_Auto_R;
      } 
      else if (beamGear == 1) {
        gameState.gear = GearState_Auto_N;
      } 
      else if (beamGear >= 2 && beamGear <= 10) {
        gameState.gear = GearState_Auto_D;
      } 
      else {
        gameState.gear = GearState_Auto_P;
      }

      static float speedFiltered = 0.0f;

      memcpy(dataBuff, (packet.data() + 12), 4);
      float speedRaw = *((float*)dataBuff);
      speedRaw = speedRaw * 3.6f;

      speedFiltered = speedFiltered * 0.80f + speedRaw * 0.20f;
      gameState.speed = speedFiltered;

      static float rpmFiltered = 0.0f;

      memcpy(dataBuff, (packet.data() + 16), 4);
      float rpmRaw = *((float*)dataBuff);

      rpmFiltered = rpmFiltered * 0.85f + rpmRaw * 0.15f;
      gameState.rpm = rpmFiltered;

      memcpy(dataBuff, (packet.data() + 24), 4);
      gameState.coolantTemperature = *((float*)dataBuff);

      memcpy(dataBuff, (packet.data() + 44), 4);
      int lights = *((int*)dataBuff);

      bool rawRight = ((lights & 0x0040) != 0);
      bool rawLeft  = ((lights & 0x0020) != 0);

      unsigned long now = millis();
      static const unsigned long kOffTimeoutMs = 300;
      static unsigned long lastLeftSeen = 0;
      static unsigned long lastRightSeen = 0;

      if (rawLeft)  lastLeftSeen  = now;
      if (rawRight) lastRightSeen = now;

      gameState.leftTurningIndicator  = (now - lastLeftSeen  <= kOffTimeoutMs);
      gameState.rightTurningIndicator = (now - lastRightSeen <= kOffTimeoutMs);

      gameState.highBeam     = ((lights & 0x0002) != 0);
      gameState.batteryLight = ((lights & 0x0200) != 0);
      gameState.absLight     = ((lights & 0x0400) != 0);
      gameState.handbrake    = ((lights & 0x0004) != 0);
      gameState.offroadLight = ((lights & 0x0010) != 0);
    }

  });
}