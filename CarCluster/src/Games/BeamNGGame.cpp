// ####################################################################################################################
// Better_CAN / BeamNG UDP integration for BMW F10
// Protocol source: https://github.com/JackieZ123430/Better_CAN
// Project: https://github.com/JackieZ123430/CarCluster-F10-Enhanced
// ####################################################################################################################

#include "BeamNGGame.h"
#include "BetterCANProtocol.h"

#include <math.h>
#include <string.h>

#ifndef BETTER_CAN_DEBUG
#define BETTER_CAN_DEBUG 0
#endif

namespace {

float finiteOr(float value, float fallback) {
  return isfinite(value) ? value : fallback;
}

float clampFloat(float value, float minimum, float maximum) {
  if (value < minimum) return minimum;
  if (value > maximum) return maximum;
  return value;
}

uint8_t mapBetterCanDriveMode(uint8_t mode) {
  // Better_CAN: 1 Comfort, 2 Sport/Race, 3 Sport+, 4 Off, 5 2WD, 6 Drift.
  // BMW F10 cluster: 2 Comfort, 4 Sport, 5 Sport+, 6 DSC off.
  switch (mode) {
    case 1: return 2;
    case 2: return 4;
    case 3: return 5;
    case 4:
    case 5:
    case 6: return 6;
    default: return 2;
  }
}

GearState decodeGear(char gearLetter, uint8_t gearIndex) {
  switch (gearLetter) {
    case 'P': return GearState_Auto_P;
    case 'R': return GearState_Auto_R;
    case 'N': return GearState_Auto_N;
    case 'S': return GearState_Auto_S;
    case 'M':
      if (gearIndex >= 1 && gearIndex <= 8) {
        return static_cast<GearState>(GearState_Manual_1 + gearIndex - 1);
      }
      return GearState_Manual_1;
    case 'D':
    default: return GearState_Auto_D;
  }
}

bool packetShowsEngineFault(const BetterCANPacket& data) {
  return data.checkengine ||
         data.engineImpactDamage ||
         data.radiatorLeak ||
         data.oilpanLeak ||
         data.oilRadiatorLeak ||
         data.exhaustBroken ||
         data.mainEngineBroken ||
         data.gearboxBroken ||
         data.engineDisabled ||
         data.engineLockedUp ||
         data.engineReducedTorque ||
         data.coolantOverheating ||
         data.oilOverheating;
}

}  // namespace

BeamNGGame::BeamNGGame(GameState& game, uint16_t port) : Game(game), port(port) {}

void BeamNGGame::begin() {
  if (!beamUdp.listen(port)) {
    Serial.printf("[Better_CAN] UDP listen failed on port %u\n", port);
    return;
  }

  Serial.printf("[Better_CAN] UDP listening on port %u, packet size %u\n",
                port,
                static_cast<unsigned>(sizeof(BetterCANPacket)));

  beamUdp.onPacket([this](AsyncUDPPacket packet) {
    if (packet.length() != sizeof(BetterCANPacket)) {
#if BETTER_CAN_DEBUG
      static uint32_t lastSizeWarning = 0;
      const uint32_t now = millis();
      if (now - lastSizeWarning >= 1000) {
        Serial.printf("[Better_CAN] ignored packet size %u, expected %u\n",
                      static_cast<unsigned>(packet.length()),
                      static_cast<unsigned>(sizeof(BetterCANPacket)));
        lastSizeWarning = now;
      }
#endif
      return;
    }

    BetterCANPacket data{};
    memcpy(&data, packet.data(), sizeof(data));

    gameState.time = data.time;
    gameState.speed = static_cast<int>(roundf(clampFloat(finiteOr(data.speedKmh, 0.0f), 0.0f, 400.0f)));
    gameState.rpm = static_cast<int>(roundf(clampFloat(finiteOr(data.rpm, 0.0f), 0.0f, 12000.0f)));

    gameState.gearLetter = data.gearLetter;
    gameState.gearIndex = data.gearIndex <= 8 ? data.gearIndex : 0;
    gameState.gear = decodeGear(data.gearLetter, gameState.gearIndex);

    gameState.ignition = data.ignition != 0;
    gameState.engineRunning = data.engineRunning != 0;

    gameState.doorFL = data.doorFL != 0;
    gameState.doorFR = data.doorFR != 0;
    gameState.doorRL = data.doorRL != 0;
    gameState.doorRR = data.doorRR != 0;
    gameState.trunkOpen = data.trunkOpen != 0;
    gameState.hoodOpen = data.hoodOpen != 0;
    gameState.doorOpen = gameState.doorFL || gameState.doorFR || gameState.doorRL ||
                         gameState.doorRR || gameState.trunkOpen || gameState.hoodOpen;

    gameState.handbrake = data.parkingBrake != 0;
    gameState.absLight = data.absActive != 0 || data.isABSBrakeActive != 0;
    gameState.escActive = data.escActive != 0 || data.isYCBrakeActive != 0;
    gameState.tcsActive = data.tcsActive != 0 || data.isTCBrakeActive != 0;
    gameState.hasESC = data.hasESC != 0 || data.escAvailable != 0;
    gameState.hasTCS = data.hasTCS != 0 || data.tcsAvailable != 0;
    gameState.offroadLight = gameState.escActive || gameState.tcsActive;

    gameState.driveMode = mapBetterCanDriveMode(data.driveMode);
    gameState.escDisabled = gameState.driveMode == 6;

    gameState.highBeam = data.highBeam != 0;
    gameState.mainLights = data.lowBeam != 0 || gameState.highBeam;
    gameState.frontFogLight = data.fog != 0;
    gameState.leftTurningIndicator = data.signalL != 0;
    gameState.rightTurningIndicator = data.signalR != 0;
    gameState.turningIndicatorsBlinking = data.signalL != 0 || data.signalR != 0 || data.hazard != 0;
    gameState.brakeLights = data.brakelights != 0;

    gameState.batteryLight = data.battery != 0;
    gameState.oilLight = data.oil != 0 || data.oilLevelCritical != 0 || data.starvedOfOil != 0;
    gameState.engineLight = packetShowsEngineFault(data);

    gameState.cruiseControlActive = data.cruiseControlActive != 0;
    gameState.cruiseControlTarget = clampFloat(finiteOr(data.cruiseControlTarget, 0.0f), 0.0f, 400.0f);

    gameState.fuelQuantity = clampFloat(finiteOr(data.fuel, 0.0f), 0.0f, 100.0f);
    gameState.lowFuelLight = data.lowfuel != 0 || gameState.fuelQuantity <= 10.0f;
    gameState.coolantTemperature = static_cast<int>(roundf(clampFloat(finiteOr(data.waterTemp, 0.0f), -50.0f, 250.0f)));
    gameState.oilTemperature = static_cast<int>(roundf(clampFloat(finiteOr(data.oilTemp, 0.0f), -50.0f, 250.0f)));

    gameState.tireDefFL = data.tireDefFL != 0;
    gameState.tireDefFR = data.tireDefFR != 0;
    gameState.tireDefRL = data.tireDefRL != 0;
    gameState.tireDefRR = data.tireDefRR != 0;
  });
}
