// ####################################################################################################################
// SimHub serial JSON integration retained for the BMW F10 build
// ####################################################################################################################

#ifndef SIMHUB_GAME_H
#define SIMHUB_GAME_H

#include "Arduino.h"
#include "../Libs/ArduinoJson/ArduinoJson.h"
#include "GameSimulation.h"

class SimhubGame : public Game {
 public:
  explicit SimhubGame(GameState& game);
  void begin() override;
  void decodeSerialData(JsonDocument& doc);
};

#endif
