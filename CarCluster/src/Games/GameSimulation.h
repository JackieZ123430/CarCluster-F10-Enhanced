// ####################################################################################################################
//
// CarCluster-F10-Enhanced - F10-only optimized game state
// Original project: https://github.com/r00li/CarCluster
// Enhanced project: https://github.com/JackieZ123430/CarCluster-F10-Enhanced
// Better_CAN protocol: https://github.com/JackieZ123430/Better_CAN
//
// ####################################################################################################################

#ifndef GAME_SIMULATION_H
#define GAME_SIMULATION_H

#include "Arduino.h"

struct ClusterConfiguration {
  float speedCorrectionFactor = 1.00f;
  float rpmCorrectionFactor = 1.00f;
  int maximumRPMValue = 7500;
  int maximumSpeedValue = 260;
  int minimumCoolantTemperature = 50;
  int maximumCoolantTemperature = 150;

  static ClusterConfiguration updatedFromDefaults(
      ClusterConfiguration current,
      float speedCorrectionFactor,
      float rpmCorrectionFactor,
      int maximumRPMValue,
      int maximumSpeedValue,
      int minimumCoolantTemperature,
      int maximumCoolantTemperature) {
    ClusterConfiguration result = current;

    result.speedCorrectionFactor = speedCorrectionFactor;
    result.rpmCorrectionFactor = rpmCorrectionFactor;

    if (maximumRPMValue > 0) result.maximumRPMValue = maximumRPMValue;
    if (maximumSpeedValue > 0) result.maximumSpeedValue = maximumSpeedValue;
    if (minimumCoolantTemperature > 0) result.minimumCoolantTemperature = minimumCoolantTemperature;
    if (maximumCoolantTemperature > 0) result.maximumCoolantTemperature = maximumCoolantTemperature;

    return result;
  }
};

enum GearState {
  GearState_Manual_1 = 1,
  GearState_Manual_2 = 2,
  GearState_Manual_3 = 3,
  GearState_Manual_4 = 4,
  GearState_Manual_5 = 5,
  GearState_Manual_6 = 6,
  GearState_Manual_7 = 7,
  GearState_Manual_8 = 8,
  GearState_Manual_9 = 9,
  GearState_Manual_10 = 10,
  GearState_Auto_P = 11,
  GearState_Auto_R = 12,
  GearState_Auto_N = 13,
  GearState_Auto_D = 14,
  GearState_Auto_S = 15
};

class GameState {
 public:
  explicit GameState(ClusterConfiguration configuration) : configuration(configuration) {}

  ClusterConfiguration configuration;

  // Primary telemetry
  int speed = 0;                          // km/h
  int rpm = 0;
  GearState gear = GearState_Auto_P;
  char gearLetter = 'P';
  uint8_t gearIndex = 0;
  uint8_t backlightBrightness = 100;
  int coolantTemperature = 90;            // C
  int oilTemperature = 90;                // C
  bool ignition = false;
  bool engineRunning = false;
  float fuelQuantity = 20.0f;             // 0-100 percent
  int outdoorTemperature = 20;
  unsigned long time = 0;                 // simulation time in ms

  // Lamps and body state
  bool leftTurningIndicator = false;
  bool rightTurningIndicator = false;
  bool turningIndicatorsBlinking = false;
  bool mainLights = false;
  bool brakeLights = false;
  bool handbrake = false;
  bool rearFogLight = false;
  bool frontFogLight = false;
  bool highBeam = false;

  bool doorOpen = false;
  bool doorFL = false;
  bool doorFR = false;
  bool doorRL = false;
  bool doorRR = false;
  bool trunkOpen = false;
  bool hoodOpen = false;

  bool offroadLight = false;               // transient ESC/TCS intervention lamp
  uint8_t driveMode = 2;                   // F10: 1 Traction, 2 Comfort, 4 Sport, 5 Sport+, 6 DSC off, 7 Eco Pro
  bool absLight = false;
  bool batteryLight = false;
  bool oilLight = false;
  bool engineLight = false;
  bool lowFuelLight = false;

  bool cruiseControlActive = false;
  float cruiseControlTarget = 0.0f;
  bool escActive = false;
  bool escDisabled = false;
  bool hasESC = true;
  bool tcsActive = false;
  bool hasTCS = true;

  // TPMS
  bool tireDefFL = false;
  bool tireDefFR = false;
  bool tireDefRL = false;
  bool tireDefRR = false;

  bool isAnyTyreDeflated() const {
    return tireDefFL || tireDefFR || tireDefRL || tireDefRR;
  }

  // Web dashboard actions
  int buttonEventToProcess = 0;
  uint8_t alertId = 0;
  bool alertStart = false;
  bool alertClear = false;
};

class Game {
 public:
  virtual ~Game() {}
  virtual void begin() = 0;

 protected:
  explicit Game(GameState& game) : gameState(game) {}
  GameState& gameState;
};

#endif
