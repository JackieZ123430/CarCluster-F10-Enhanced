// ####################################################################################################################
// 
// Code part of CarCluster project by Andrej Rolih. See .ino file more details
// 
// ####################################################################################################################

#ifndef BEAM_NG_GAME
#define BEAM_NG_GAME

#include "Arduino.h"
#include "AsyncUDP.h" // For game integration (system library part of ESP core)

#include "GameSimulation.h"

class BeamNGGame: public Game {
  public:
    BeamNGGame(GameState& game, int port);
    void begin();

  private:
    int port;
    AsyncUDP beamUdp;

    // Protocol control
    bool useCustomProtocol = true;

    // Ignition state tracking (edge detection)
    bool lastIgnitionState = false;

    // ===== Custom trigger states (cluster logic) =====
    bool overSpeed120Trigger = false;
    bool parkingBrakeTrigger = false;
    bool can41Trigger = false;

    // Engine start timing (for delayed CAN messages)
    unsigned long engineStartTimestamp = 0;
    bool engineWasRunning = false;
};

#endif