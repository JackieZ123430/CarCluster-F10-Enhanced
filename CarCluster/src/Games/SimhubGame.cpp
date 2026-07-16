// ####################################################################################################################
// SimHub serial JSON integration retained for the BMW F10 build
// ####################################################################################################################

#include "SimhubGame.h"

SimhubGame::SimhubGame(GameState& game) : Game(game) {}

void SimhubGame::begin() {}

void SimhubGame::decodeSerialData(JsonDocument& doc) {
  gameState.time = millis();
  gameState.rpm = doc["rpm"] | 0;
  gameState.engineRunning = gameState.rpm > 50;
  gameState.ignition = (doc["run"] | 0) != 0 || gameState.engineRunning;

  const int simGear = doc["gea"].as<int>();
  gameState.gearIndex = 0;

  if (simGear > 0) {
    gameState.gear = GearState_Auto_D;
    gameState.gearLetter = 'D';
    gameState.gearIndex = simGear > 8 ? 8 : static_cast<uint8_t>(simGear);
  } else if (simGear == 0) {
    gameState.gear = GearState_Auto_N;
    gameState.gearLetter = 'N';
  } else {
    gameState.gear = GearState_Auto_R;
    gameState.gearLetter = 'R';
  }

  gameState.speed = doc["spe"] | 0;
  gameState.leftTurningIndicator = (doc["lft"] | 0) != 0;
  gameState.rightTurningIndicator = (doc["rit"] | 0) != 0;
  gameState.turningIndicatorsBlinking =
      gameState.leftTurningIndicator || gameState.rightTurningIndicator;

  gameState.oilTemperature = doc["oit"] | 90;
  gameState.coolantTemperature = gameState.oilTemperature;

  gameState.doorOpen = (doc["pau"] | 0) != 0 || (doc["run"] | 0) == 0;
  gameState.doorFL = gameState.doorOpen;
  gameState.doorFR = false;
  gameState.doorRL = false;
  gameState.doorRR = false;

  gameState.fuelQuantity = doc["fue"] | 0;
  if (gameState.fuelQuantity < 0.0f) gameState.fuelQuantity = 0.0f;
  if (gameState.fuelQuantity > 100.0f) gameState.fuelQuantity = 100.0f;
  gameState.lowFuelLight = gameState.fuelQuantity <= 10.0f;

  gameState.handbrake = (doc["hnb"] | 0) != 0;
  gameState.absLight = (doc["abs"] | 0) != 0;
  gameState.offroadLight = (doc["tra"] | 0) != 0;
}
