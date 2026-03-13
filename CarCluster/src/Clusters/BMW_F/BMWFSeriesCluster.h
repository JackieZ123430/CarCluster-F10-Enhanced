// ####################################################################################################################
// 
// Code part of CarCluster project by Andrej Rolih. See .ino file more details
// 
// ####################################################################################################################

#ifndef F_SERIES_DASH
#define F_SERIES_DASH

#include "../../Libs/MultiMap/MultiMap.h" // For fuel level calculation - supports non linear mapping found on BMW clusters ( https://github.com/RobTillaart/MultiMap )
#include "../../Libs/MCP_CAN/mcp_can.h" // CAN Bus Shield Compatibility Library ( https://github.com/coryjfowler/MCP_CAN_lib )

#include "CRC8.h"
#include "../Cluster.h"

#define lo8(x) (uint8_t)((x) & 0xFF)
#define hi8(x) (uint8_t)(((x)>>8) & 0xFF)

class BMWFSeriesCluster: public Cluster {

  // Do not ever send 0x380 ID - that is VIN number

  public:
  static ClusterConfiguration clusterConfig(bool isCarMini) {
    ClusterConfiguration config;
    config.minimumCoolantTemperature = 50;
    config.maximumCoolantTemperature = 150;
    config.maximumSpeedValue = 260;

    if (isCarMini) {
      config.maximumRPMValue = 7000;
    } else {
      config.maximumRPMValue = 6000;
    }

    return config;
  }

  BMWFSeriesCluster(MCP_CAN& CAN, bool isCarMini);
  void updateWithGame(GameState& game);
  void updateLanguageAndUnits();

  uint8_t inFuelRange[3] = {};
  uint8_t outFuelRange[3] = {};

  private:
  MCP_CAN &CAN; 
  CRC8 crc8Calculator;

  unsigned long dashboardUpdateTime100 = 20;
  unsigned long dashboardUpdateTime1000 = 1000;
  unsigned long lastDashboardUpdateTime = 0; // Timer for the fast updated variables
  unsigned long lastDashboardUpdateTime1000ms = 0; // Timer for slow updated variables

  uint8_t schedulerIndex = 0; // Round-robin CAN scheduler index

  uint8_t counter4Bit = 0;
  uint8_t accCounter = 0;
  uint8_t count = 0;
  uint16_t distanceTravelledCounter = 0;
  bool isCarMini = false;

  void sendIgnitionStatus(bool ignition);
  void sendSpeed(int speed);
  void sendRPM(int rpm, int manualGear);
  void sendAutomaticTransmission(GearState gear, uint8_t gearIndex);
  void sendBasicDriveInfo(GameState& game, int engineTemperature);
  void sendParkBrake(bool handbrakeActive);
  void sendFuel(float fuelRatio, uint8_t inFuelRange[], uint8_t outFuelRange[], bool isCarMini);
  void sendDistanceTravelled(int speed);
  void sendBlinkers(bool leftTurningIndicator, bool rightTurningIndicator);
  void sendLights(bool mainLights, bool highBeam, bool rearFogLight, bool frontFogLight);
  void sendBacklightBrightness(uint8_t brightness);
  void sendAlerts(GameState& game, bool offroad, bool handbrake, bool isCarMini);
  void sendSteeringWheelButton(int buttonEvent);
  void sendDriveMode(uint8_t driveMode);
  void sendAcc();
  void sendFixedLIM();
  void sendOutsideTemperature(int temperature);
  void sendTime(uint8_t hours, uint8_t minutes);

  // Maps P/R/N/D/S/M only (no gear number). Gear number handled separately via gearIndex.
  uint8_t mapGenericGearToLocalGear(GearState inputGear);
  int mapSpeed(GameState& game);
  int mapRPM(GameState& game);
  int mapCoolantTemperature(GameState& game);

  void sendBackbone2C5();
  void sendBackbone393();
  void sendGateway2CA();
  void sendClusterSync1B3();

  int autoHighBeam = 0;   // 0 = off, 1 = on
  int autoStartStop = 0;  // 0 = off, 1 = on

  float mapRPMValueForFuelModel(int speed);

  bool escActive;
  bool escDisabled;
  bool hasESC;
};

#endif
