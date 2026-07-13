// ####################################################################################################################
// Forza UDP integration retained for the BMW F10 build
// ####################################################################################################################

#ifndef FORZA_HORIZON_GAME_H
#define FORZA_HORIZON_GAME_H

#include "Arduino.h"
#include "AsyncUDP.h"
#include "GameSimulation.h"

class ForzaHorizonGame : public Game {
 public:
  ForzaHorizonGame(GameState& game, uint16_t port);
  void begin() override;

 private:
  uint16_t port;
  AsyncUDP forzaUdp;
};

#endif
