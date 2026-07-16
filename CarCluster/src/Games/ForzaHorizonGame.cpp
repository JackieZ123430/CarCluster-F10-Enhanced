// ####################################################################################################################
// Forza UDP integration retained for the BMW F10 build
// Original integration: https://github.com/r00li/CarCluster
// ####################################################################################################################

#include "ForzaHorizonGame.h"

#include <math.h>
#include <string.h>

namespace {

float readFloat(const uint8_t* data, size_t offset) {
  float value = 0.0f;
  memcpy(&value, data + offset, sizeof(value));
  return isfinite(value) ? value : 0.0f;
}

uint8_t readByte(const uint8_t* data, size_t offset) {
  return data[offset];
}

}  // namespace

ForzaHorizonGame::ForzaHorizonGame(GameState& game, uint16_t port)
    : Game(game), port(port) {}

void ForzaHorizonGame::begin() {
  if (!forzaUdp.listen(port)) {
    Serial.printf("[Forza] UDP listen failed on port %u\n", port);
    return;
  }

  Serial.printf("[Forza] UDP listening on port %u\n", port);

  forzaUdp.onPacket([this](AsyncUDPPacket packet) {
    if (packet.length() != 324 && packet.length() != 331) return;

    const uint8_t* bytes = packet.data();
    const bool motorsport2023 = packet.length() == 331;

    const float maximumRpm = readFloat(bytes, 8);
    const float currentRpm = readFloat(bytes, 16);
    const float speedMps = readFloat(bytes, motorsport2023 ? 244 : 256);

    gameState.time = millis();
    gameState.ignition = maximumRpm > 0.0f;
    gameState.engineRunning = currentRpm > 50.0f;
    gameState.rpm = static_cast<int>(currentRpm);

    if (maximumRpm > gameState.configuration.maximumRPMValue && maximumRpm > 0.0f) {
      gameState.rpm = static_cast<int>(
          currentRpm * gameState.configuration.maximumRPMValue / maximumRpm);
    }

    gameState.speed = static_cast<int>(speedMps * 3.6f);
    if (gameState.speed < 0) gameState.speed = 0;

    const uint8_t forzaGear = readByte(bytes, motorsport2023 ? 307 : 319);
    gameState.gearIndex = 0;

    if (!gameState.ignition) {
      gameState.gear = GearState_Auto_P;
      gameState.gearLetter = 'P';
      gameState.doorOpen = true;  // menu/not driving indication retained from the original project
    } else if (forzaGear == 0) {
      gameState.gear = GearState_Auto_R;
      gameState.gearLetter = 'R';
      gameState.doorOpen = false;
    } else if (forzaGear == 1) {
      gameState.gear = GearState_Auto_N;
      gameState.gearLetter = 'N';
      gameState.doorOpen = false;
    } else {
      gameState.gear = GearState_Auto_D;
      gameState.gearLetter = 'D';
      gameState.gearIndex = forzaGear > 1 ? static_cast<uint8_t>(forzaGear - 1) : 0;
      if (gameState.gearIndex > 8) gameState.gearIndex = 8;
      gameState.doorOpen = false;
    }

    gameState.doorFL = gameState.doorOpen;
    gameState.doorFR = false;
    gameState.doorRL = false;
    gameState.doorRR = false;

    const size_t handbrakeOffset = motorsport2023 ? 306 : 318;
    gameState.handbrake = readByte(bytes, handbrakeOffset) != 0;
  });
}
