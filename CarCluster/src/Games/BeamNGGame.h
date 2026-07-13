// ####################################################################################################################
// Better_CAN / BeamNG UDP integration for BMW F10
// https://github.com/JackieZ123430/Better_CAN
// ####################################################################################################################

#ifndef BEAM_NG_GAME_H
#define BEAM_NG_GAME_H

#include "Arduino.h"
#include "AsyncUDP.h"
#include "GameSimulation.h"

class BeamNGGame : public Game {
 public:
  BeamNGGame(GameState& game, uint16_t port);
  void begin() override;

 private:
  uint16_t port;
  AsyncUDP beamUdp;
};

#endif
